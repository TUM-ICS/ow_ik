#include <ow_ik/utilities.h>
#include <ow_ik/ik_solver_nullspace.h>
#include <ros/assert.h>

namespace ow_ik
{

  IkSolverNullspace::IkSolverNullspace(
      const KDL::Chain &chain,
      KDL::ChainFkSolverPos_recursive &fk_solver,
      KDL::ChainJntToJacSolver &j_solver,
      const SolverSettings &hyp) : chain(chain),
                                   fk_solver_(fk_solver),
                                   j_solver_(j_solver),
                                   hyp_(hyp)
  {
    // set defaults
    dof_ = chain.getNrOfJoints();
    I_ = ow::MatrixX::Identity(dof_, dof_);
    W_sqrt_inv_ = ow::MatrixX::Identity(dof_, dof_);
    q_ll_.setConstant(-10);
    q_ul_.setConstant(10);

    // set gains
    K_task_ = hyp_.k_task * ow::MatrixX::Identity(6, 6);
    K_null_ = hyp_.k_null * ow::MatrixX::Identity(dof_, dof_);
  }

  void IkSolverNullspace::setSettings(const SolverSettings &hyp)
  {
    hyp_ = hyp;
  }

  void IkSolverNullspace::setJointLimits(
      const ow::JointPositionX &q_ll,
      const ow::JointPositionX &q_ul)
  {
    bool is_valid = ((q_ll.size() == dof_) && (q_ul.size() == dof_));
    ROS_ASSERT_MSG(is_valid,
                   "Invalid vector size: q_ll.size=%d, q_ul.size=%d but needs"
                   "to be size_task=%d and size_null=%d",
                   (int)q_ll.size(), (int)q_ul.size(), dof_, dof_);

    q_ll_ = q_ll;
    q_ul_ = q_ul;
  }

  void IkSolverNullspace::setGains(
      const ow::VectorX &k_task,
      const ow::VectorX &k_null)
  {
    bool is_valid = ((k_task.size() == dof_) && (k_null.size() == dof_));
    ROS_ASSERT_MSG(is_valid,
                   "Invalid vector size: k_task.size=%d, k_null.size=%d but needs"
                   "to be size_task=6 and size_null=%d",
                   (int)k_task.size(), (int)k_null.size(), dof_);

    K_task_ = k_task.asDiagonal();
    K_null_ = k_null.asDiagonal();
  }

  void IkSolverNullspace::setWeights(
      const ow::VectorX &weights)
  {
    bool is_valid = (weights.size() == dof_);
    ROS_ASSERT_MSG(is_valid,
                   "Invalid vector size: weights.size=%d but needs to be dof=%d",
                   (int)weights.size(), dof_);

    W_sqrt_inv_ = weights.array().rsqrt().matrix().asDiagonal();
  }

  bool IkSolverNullspace::CartToJnt(
      ow::JointPositionX &q,
      const ow::JointPositionX &q_0,
      const ow::JointPositionX &q_init,
      const ow::HomogeneousTransformation &T_target)
  {
    ow::MatrixX J;         // Jacobian
    ow::MatrixX Jw;        // weighted jacobian
    ow::MatrixX Jw_pinv;   // weighted pseudo inverse
    ow::MatrixX N;         // Nullspace
    ow::VectorX s_inv;     // singular values
    ow::Scalar lambda_adj; // adjusted singularity damping

    ow::JointPositionX q_prev; // joints
    ow::JointVelocityX qP;     // joint velo
    ow::JointPositionX Eq;     // joint error

    ow::CartesianVector EX;          // cartesian error
    ow::HomogeneousTransformation T; // cartesian transf

    // initalize
    q = q_init;
    q_prev = q_init;
    s_inv.setZero(dof_);

    size_t k = 0;
    bool converged = false;
    while (!converged && k < hyp_.max_iter)
    {
      // forward kinematics
      computeFK(T, q);

      // check cartesian error
      EX = ow::cartesianError(T_target, T);

      // weighted jacobian Matrix
      computeJacobian(J, q);
      Jw = W_sqrt_inv_ * J;

      // compute svd
      Eigen::JacobiSVD<ow::MatrixX> svd;
      svd = Jw.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

      // weigted damped pseudo inverse of J
      const Eigen::JacobiSVD<ow::MatrixX>::SingularValuesType &s =
          svd.singularValues();

      // adjust damping based on error (Sugihara)
      lambda_adj = 
        EX.linear().squaredNorm() + EX.angular().squaredNorm() / (4 * M_PI * M_PI);

      for (size_t i = 0; i < dof_; ++i)
      {
        s_inv[i] = s[i] / (s[i] * s[i] + hyp_.lambda + 0.2 * lambda_adj);
      }
      Jw_pinv = 
        W_sqrt_inv_ * svd.matrixV() * s_inv.asDiagonal() * svd.matrixU().adjoint();

      // Nullspace
      N = I_ - Jw_pinv * J;

      // cartesian velocity
      qP = Jw_pinv * K_task_ * EX + K_null_ * N * (q_0 - q);

      // integrate
      q += hyp_.alpha * qP;

      // address constrains
      for (size_t i = 0; i < dof_; ++i)
      {
        if (q[i] > q_ul_[i])
          q[i] = q_ul_[i];
        if (q[i] < q_ll_[i])
          q[i] = q_ll_[i];
      }

      // check joint angles convergence
      Eq = q - q_prev;
      q_prev = q;

      if (EX.linear().squaredNorm() < hyp_.pos_tol &&
          EX.angular().squaredNorm() < hyp_.ori_tol &&
          Eq.squaredNorm() < hyp_.step_tol)
      {
        converged = true;
      }
      k++;
    }
    return converged;
  }

  int IkSolverNullspace::computeFK(
      ow::HomogeneousTransformation &T,
      const ow::JointPositionX &q)
  {
    KDL::Frame T_kdl;
    KDL::JntArray q_kdl;

    q_kdl.data = q;
    if (fk_solver_.JntToCart(q_kdl, T_kdl))
    {
      return -1;
    }
    T = convert(T, T_kdl);
    return 0;
  }

  int IkSolverNullspace::computeJacobian(
      ow::MatrixX &J,
      const ow::JointPositionX &q)
  {
    KDL::JntArray q_kdl;
    KDL::Jacobian J_kdl(dof_);

    q_kdl.data = q;
    if (j_solver_.JntToJac(q_kdl, J_kdl))
    {
      return -1;
    }
    J = J_kdl.data;
    return 0;
  }

} // namespace ow_ik
