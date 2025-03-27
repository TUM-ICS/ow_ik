/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#include <ow_ik/robot_limb.h>
#include <ow_ik/utilities.h>

namespace ow_ik
{

  RobotLimb::RobotLimb(
      const KDL::Chain &chain,
      JointStateRef joint_state,
      const ow::JointStateX &ll_joint_state,
      const ow::JointStateX &ul_joint_state,
      const std::string &base_fame,
      const std::string &name) : chain_(chain),
                                 joint_state_(joint_state),
                                 ll_joint_state_(ll_joint_state),
                                 ul_joint_state_(ul_joint_state),
                                 base_frame_(base_fame),
                                 name_(name),
                                 fk_solver_(chain_),
                                 j_solver_(chain_),
                                 ik_solver_v_(chain_),
                                 ik_solver_p_(chain_, fk_solver_, ik_solver_v_),
                                 id_solver_(chain_, KDL::Vector(0, 0, -OW_GRAVITY)),
                                 ik_solver_nullspace_(chain_, fk_solver_, j_solver_),
                                 Tee_b_(ow::HomogeneousTransformation::Identity())
  {
    dof_ = chain_.getNrOfJoints();
    ee_frame_ = chain_.getSegment(dof_ - 1).getName();

    // set default zero position
    q_0_ = ow::JointPositionX::Zero(dof_);

    ik_solver_nullspace_.setJointLimits(
        ll_joint_state_.pos(),
        ul_joint_state_.pos());
  }

  bool RobotLimb::initialize(const ow::Parameter &parameter)
  {
    // extract the solver settings and update solver
    SolverSettings hyp;
    parameter.get("solver/lambda", hyp.lambda);
    parameter.get("solver/alpha", hyp.alpha);
    parameter.get("solver/beta", hyp.beta);
    parameter.get("solver/pos_tol", hyp.pos_tol);
    parameter.get("solver/ori_tol", hyp.ori_tol);
    parameter.get("solver/step_tol", hyp.step_tol);
    parameter.get("solver/max_iter", hyp.max_iter);
    parameter.get("solver/k_task", hyp.k_task);
    parameter.get("solver/k_null", hyp.k_null);
    ik_solver_nullspace_.setSettings(hyp);

    return true;
  }

  size_t RobotLimb::dof() const
  {
    return dof_;
  }

  std::string RobotLimb::baseFrame() const
  {
    return base_frame_;
  }

  std::string RobotLimb::eeFrame() const
  {
    return ee_frame_;
  }

  std::string RobotLimb::name() const
  {
    return name_;
  }

  bool RobotLimb::setNullspacePosture(const ow::JointPositionX &q_0)
  {
    if (!checkDimension(q_0))
    {
      return false;
    }
    q_0_ = q_0;
    return true;
  }

  ow::JointPositionX RobotLimb::nullSpacePosture() const
  {
    return q_0_;
  }

  const ow::HomogeneousTransformation &RobotLimb::forwardKinematics(
      const CJointStateRef &joint_state)
  {
    KDL::Frame T;
    KDL::JntArray q(dof_);

    q.data = joint_state.pos();
    if (fk_solver_.JntToCart(q, T))
    {
      ROS_ERROR("Failed to compute forward kinematics in '%s'.", name_.c_str());
      return Tee_b_;
    }
    return convert(Tee_b_, T);
  }

  bool RobotLimb::inverseKinematics(
      const CJointStateRef &joint_state,
      const ow::JointPositionX &q_0,
      const ow::HomogeneousTransformation &Tee_b)
  {
    ow::JointPositionX q;
    if (!ik_solver_nullspace_.CartToJnt(q, q_0, joint_state.pos(), Tee_b))
    {
      ROS_ERROR("Failed to compute inverse kinematics in '%s'.", name_.c_str());
      joint_state_.pos() = q;
      return false;
    }
    joint_state_.pos() = q;
    return true;
  }

  bool RobotLimb::inverseKinematics(
      const CJointStateRef &joint_state,
      const ow::HomogeneousTransformation &Tee_b)
  {
    return inverseKinematics(joint_state, q_0_, Tee_b);
  }

  bool RobotLimb::inverseDynamics(
      const CJointStateRef &joint_state,
      const RobotLimb::Wrenches &wrenches)
  {
    KDL::JntArray q(dof_);
    KDL::JntArray qP(dof_);
    KDL::JntArray qPP(dof_);
    KDL::JntArray tau(dof_);
    KDL::Wrenches ws;
    KDL::Wrench w;

    q.data = joint_state.pos();
    qP.data = joint_state.vel();
    qPP.data = joint_state.acc();

    ws.reserve(wrenches.size());
    for (int i = 0; i < wrenches.size(); ++i)
    {
      w = convert(w, wrenches[i]);
      ws.push_back(w);
    }

    if (id_solver_.CartToJnt(q, qP, qPP, ws, tau) != KDL::SolverI::E_NOERROR)
    {
      ROS_ERROR("Failed to compute inverse dynamics in '%s'.", name_.c_str());
      return false;
    }

    joint_state_.tau() = tau.data;
    return true;
  }

  bool RobotLimb::checkDimension(const ow::JointPositionX &q)
  {
    if (q.size() != dof_)
    {
      ROS_ERROR("Wrong vector dimension in '%s'.", name_.c_str());
      return false;
    }
    return true;
  }

}; // namespace ow_ik
