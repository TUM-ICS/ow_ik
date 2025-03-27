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

#ifndef OPEN_WALKER_IK_SOLVER_LIMB_H
#define OPEN_WALKER_IK_SOLVER_LIMB_H

#include <ow_core/types.h>
#include <ow_core/common/type_not_assignable.h>
#include <ow_core/common/parameter.h>

// kdl
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

// ik solver
#include <ow_ik/ik_solver_nullspace.h>

namespace ow_ik
{

  /*!
  * \brief The RobotLimb class
  *
  * This class calculates the kinematics and dynamics of a RobotLimb.
  * It takes as an input a reference to a sub-vector of the complete body state
  * and modifies this vector upon function calls.
  * 
  * Mutliple Robotlimbs presend in the robot_model.h to build the complete robot.
  */
  class RobotLimb
  {
    OW_TYPE_NOT_ASSIGNABLE(RobotLimb)

  public:
    typedef ow_core::JointStateRef<const ow::JointState> CJointStateRef;
    typedef ow_core::JointStateRef<ow::JointState> JointStateRef;
    typedef std::vector<ow::Wrench> Wrenches;

  protected:
    std::string name_;       //!< identifier
    std::string base_frame_; //!< base frame name
    std::string ee_frame_;   //!< end frame name
    size_t dof_;             //!< degrees of freedom

    KDL::Chain chain_;                          //!< kdl body chain
    KDL::ChainFkSolverPos_recursive fk_solver_; //!< forward kinematics solver
    KDL::ChainJntToJacSolver j_solver_;         //!< jacobian solver
    KDL::ChainIkSolverVel_pinv ik_solver_v_;    //!< inverse kinematics solver
    KDL::ChainIkSolverPos_NR ik_solver_p_;      //!< inverse kinematics solver
    KDL::ChainIdSolver_RNE id_solver_;          //!< inverse dynamcis solver

    IkSolverNullspace ik_solver_nullspace_; //!< ik solver with nullspace

    JointStateRef joint_state_;      //!< resulting joint state
    ow::JointStateX ll_joint_state_; //!< lower limits joint state
    ow::JointStateX ul_joint_state_; //!< upper limits joint state

    ow::JointPositionX q_0_; //!< zero posture joint state

    ow::HomogeneousTransformation Tee_b_; //!< End Effector Transformation

  public:
    /*!
   * \brief Constructor
   */
    RobotLimb(
        const KDL::Chain &chain,
        JointStateRef joint_state,
        const ow::JointStateX &ll_joint_state,
        const ow::JointStateX &ul_joint_state,
        const std::string &base_fame,
        const std::string &name);

    /*!
    * \brief Initialize the limb
    */
    bool initialize(const ow::Parameter &parameter);

    /*!
    * \brief Number of dof.
    */
    size_t dof() const;

    /*!
    * \brief Name of base frame.
    */
    std::string baseFrame() const;

    /*!
    * \brief Name of end efector frame.
    */
    std::string eeFrame() const;

    /*!
    * \brief Name of the limb.
    */
    std::string name() const;

    /*!
    * \brief Set the nullspace posture used if non 
    * is provided by the user.
    */
    bool setNullspacePosture(const ow::JointPositionX &q_0);

    /*!
    * \brief Get the nullspace posture.
    */
    ow::JointPositionX nullSpacePosture() const;

    /*!
    * \brief Compute the forward kinematics
    * 
    * \return The homogeneous position of end frame.
    */
    const ow::HomogeneousTransformation &forwardKinematics(
        const CJointStateRef &joint_state);

    /*!
    * \brief Compute the inverse kinematics
    * 
    * \param joint_state
    *    Const Reference to the current jointstate of the robot.
    * 
    * \param q_0
    *    Desired joint nullspace posture.
    *   
    * \param Tee_b
    *    Target Transformation of end effector frame w.r.t limb base frame.
    */
    bool inverseKinematics(
        const CJointStateRef &joint_state,
        const ow::JointPositionX &q_0,
        const ow::HomogeneousTransformation &Tee_b);

    /*!
    * \brief Compute the inverse kinematics
    * 
    * \param joint_state
    *    Const Reference to the current jointstate of the robot.
    *   
    * \param Tee_b
    *    Target Transformation of end frame w.r.t start frame.
    */
    bool inverseKinematics(
        const CJointStateRef &joint_state,
        const ow::HomogeneousTransformation &Tee_b);

    /*!
    * \brief Compute the inverse dynamics
    * 
    * \param joint_state
    *    Const Reference to the current jointstate of the robot.
    *   
    * \param wrenches
    *    Desired wrenches applied at each link frame of the kinematic chain.
    */
    bool inverseDynamics(
        const CJointStateRef &joint_state,
        const Wrenches &wrenches);

  protected:
    /*!
    * \brief Check if the vector dimension is correct
    */
    bool checkDimension(const ow::JointPositionX &q);
  };

} // namespace ow_ik

#endif // OPEN_WALKER_IK_SOLVER_H
