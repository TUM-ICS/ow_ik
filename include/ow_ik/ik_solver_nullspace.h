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

#ifndef OPEN_WALKER_IK_SOLVER_NULLSPACE_H
#define OPEN_WALKER_IK_SOLVER_NULLSPACE_H

#include <ow_core/common/type_not_assignable.h>

#include <ow_core/math.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace ow_ik
{

  /*!
  * \brief The SolverSettings class.
  *
  * Custom IK Solver Settings.
  * Contains all the parameters that are independent of joint vector size.
  */
  class SolverSettings
  {
  public:
    /*!
    * \brief The Default Solver Settings
    */
    static SolverSettings Default()
    {
      SolverSettings v;
      v.lambda = 1e-3;
      v.alpha = 0.007;
      v.beta = 0.0;
      v.pos_tol = 0.00001;
      v.ori_tol = 0.00001;
      v.step_tol = 1e-7;
      v.max_iter = 100;
      v.k_task = 150;
      v.k_null = 50;
      return v;
    }

  public:
    ow::Scalar lambda;   //!< dampening factor
    ow::Scalar alpha;    //!< learning rate
    ow::Scalar beta;     //!< transient coeff
    ow::Scalar pos_tol;  //!< position tolerance
    ow::Scalar ori_tol;  //!< orientation tolerance
    ow::Scalar step_tol; //!< minimum posible change in q
    int max_iter;        //!< maximum number of iterations

    ow::Scalar k_task; //!< taskspace gain (same for all axis)
    ow::Scalar k_null; //!< nullspace gain (same for all joints)
  };

  /*!
  * \brief The IkSolverNullspace class.
  *
  * A custom IK Solver.
  * 
  * Inverse Kinematics solver using iterative least squares with SVD.
  * The forward kinematics and jacobian matrices are computed by KDL.
  */
  class IkSolverNullspace
  {
    OW_TYPE_NOT_ASSIGNABLE(IkSolverNullspace)

  public:
    typedef ow_core::JointRef<ow::JointPosition::Base> JointPositionRef;

  protected:
    int dof_;                                    //!< Number of limb degrees of freedorm.
    const KDL::Chain &chain;                     //!< kdl body chain
    KDL::ChainFkSolverPos_recursive &fk_solver_; //!< forward kinematics solver
    KDL::ChainJntToJacSolver &j_solver_;         //!< jacobian matrix solver
    SolverSettings hyp_;                         //!< hyper parameters

    ow::MatrixX I_;           //!< Identity.
    ow::MatrixX W_sqrt_inv_;  //!< Inverse sqrt weight matrix.
    ow::MatrixX K_task_;      //!< Task gain.
    ow::MatrixX K_null_;      //!< Nullspace gain.
    ow::JointPositionX q_ll_; //!< Lower joint limits.
    ow::JointPositionX q_ul_; //!< Upper joint limits.

  public:
    /*!
    * \brief Constructor
    */
    IkSolverNullspace(
        const KDL::Chain &chain,
        KDL::ChainFkSolverPos_recursive &fk_solver,
        KDL::ChainJntToJacSolver &j_solver,
        const SolverSettings &hyp_ = SolverSettings::Default());

    /*!
    * \brief Settings.
    * 
    * Setting used by ik solver
    */
    void setSettings(const SolverSettings &hyp);

    /*!
    * \brief Set Joint limits.
    *
    * Joint Limit Vectors considered by the IK Solver
    *
    * \param q_ll
    *    Lower joint limmits.
    *
    * \param q_ul
    *    Upper joint limmits.
    */
    void setJointLimits(
        const ow::JointPositionX &q_ll,
        const ow::JointPositionX &q_ul);

    /*!
    * \brief Change the task and nullspace gains
    *
    * \param k_task
    *    Task space gain vector.
    *
    * \param k_null
    *    Null space gain vector. Important for redundant legs.
    */
    void setGains(
        const ow::VectorX &k_task,
        const ow::VectorX &k_null);

    /*!
    * \brief Change the joint weights
    */
    void setWeights(
        const ow::VectorX &weights);

    /*!
    * \brief Compute the inverse kinematics
    * 
    * \param q
    *    Resutling Joint position
    * 
    * \param q_0
    *    Nullspace Joint position
    * 
    * \param q_init
    *    Inital Joint position guess for the solver
    * 
    * \param q_init
    *    Target Transformation w.r.t to the kdl chain base frame
    */
    bool CartToJnt(
        ow::JointPositionX &q,
        const ow::JointPositionX &q_0,
        const ow::JointPositionX &q_init,
        const ow::HomogeneousTransformation &T_target);

  protected:
    /*!
     * \brief Compute the forward kinematics
     * 
     * Extracts the Homogeneous transformation of end link w.r.t
     * the kdl chain base
     */
    int computeFK(
        ow::HomogeneousTransformation &T,
        const ow::JointPositionX &q);

    /*!
    * \brief Compute the jacobian matrix
    */
    int computeJacobian(
        ow::MatrixX &J,
        const ow::JointPositionX &q);
  };

} // namespace ow_ik

#endif // OPEN_WALKER_IK_SOLVER_NULLSPACE_H