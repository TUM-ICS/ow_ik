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

#ifndef OPEN_WALKER_IK_ROBOT_MODEL_H
#define OPEN_WALKER_IK_ROBOT_MODEL_H

#include <ros/console.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>

#include <ow_rbdl/rbdl.h>
#include <ow_rbdl/urdfreader/urdfreader.h>

#include <memory>

#include <ow_ik/robot_limb.h>

namespace ow_ik
{

  /*!
  * \brief The RobotModel class
  *
  * This class calculates the kinematics and dynamics of the Robot or 
  * individual limbs.
  * It loads the information from a given urdf file and a user description
  * of the robots limbs.
  */
  class RobotModel
  {
  public:
    typedef std::shared_ptr<RobotLimb> RobotLimbPtr;

    /*!
    * \brief Stores the information of a Robot limb
    * 
    * \note limb is using a pointer because of KDL solvers not copyable
    */
    struct RobotLimbDescription
    {
      RobotLimbPtr limb_; //!< pointer to the limb
      size_t idx_;        //!< index within the complete robot joint vecotr
      size_t size_;       //!< size (dof) of the limb join state
    };

  protected:
    bool is_initalized_; //!< initalized flag

    urdf::Model urdf_model_;    //!< robot urdf model
    KDL::Tree tree_;            //!< robot kdl tree
    ow_rbdl::Model rbdl_model_; //!< robot rbdl model

    std::vector<RobotLimbDescription> limbs_; //!< robot limbs

    ow::JointState q_;                        //!< resulting jointstate

    ow_ik::SolverSettings solver_settings_;   //!< common solver settings

  public:
    /*!
    * \brief Constructor
    */
    RobotModel();

    /*!
    * \brief Initialize RobotModel from URDF description.
    *
    * \param robot_description
    *    URDF robot robot description from parameter server.
    * 
    * \param Parameter
    *    The configuration parameters   
    * 
    * \return
    *    true on success.
    */
    bool initialize(const std::string &robot_description);

    /*!
    * \brief Add a limb to the robot
    * 
    * Adds new limb to robot with a given limb id.
    * 
    * \param limb_name
    *    limb name only used for identification
    * 
    * \param base_link_name
    *    base_link name of the limb on robot model
    *    This is the start of the limb kinematic chain
    * 
    * \param end_link_name
    *    end effector link name of the limb
    *    This is the end of the limb kinematic chain
    * 
    * \return
    *    returns the limb_id assigned by the robot model.
    *    Use this when calling subsequent function.
    */
    size_t addLimb(
        const std::string &limb_name,
        const std::string &base_link_name,
        const std::string &end_link_name,
        const ow::Parameter &parameter);

    /*!
    * \brief Number of limbs in robot.
    */
    size_t numLimbs() const;

    /*!
    * \brief Number of dof in limb
    */
    size_t dofLimb(size_t id) const;

    /*!
    * \brief The position of the limb in the complete joint vector
    */
    size_t jointIndexLimb(size_t id) const;

    /*!
    * \brief Compute the forward kinematics of given limb
    * 
    * \return The homogeneous position of end frame.
    */
    const ow::HomogeneousTransformation &forwardKinematics(
        const ow::JointState &q,
        size_t id);

    /*!
    * \brief Compute the inverse kinematics.
    * 
    * \param q
    *    Const Reference to the current jointstate of the robot.
    * 
    * \param q_0
    *    The nullspace joint position of the limb
    *   
    * \param Tee_b
    *    Target Transformation of end frame w.r.t start frame.
    */
    bool inverseKinematics(
        const ow::JointState &q,
        const ow::JointPositionX &q_0,
        const ow::HomogeneousTransformation &Tee_b,
        size_t id);

    /*!
    * \brief Compute the inverse kinematics.
    * 
    * \param q
    *    Const Reference to the current jointstate of the robot.
    * 
    * \param q_0
    *    The nullspace joint position of the limb
    *   
    * \param Tee_b
    *    Target Transformation of end frame w.r.t start frame.
    */
    bool inverseKinematics(
        const ow::JointState &q,
        const ow::HomogeneousTransformation &Tee_b,
        size_t id);

    /*!
    * \brief Compute the inverse dynamics
    * 
    * \param q
    *    Const Reference to the current jointstate of the robot.
    *   
    * \param wrenches
    *    Desired wrenches applied at each link frame of the kinematic chain.
    */
    bool inverseDynamics(
        const ow::JointState &q,
        const RobotLimb::Wrenches &wrenches,
        size_t id);

    /*!
    * \brief jointState
    *    Return the internal jointstate
    */
    const ow::JointState &jointState() const;

  protected:
    /*!
    * \brief load joint state limits urdf
    * 
    * load the joint limits of a given kinematic chain
    * from a urdf file
    */
    bool loadJointStateLimits(
        const urdf::Model &model,
        const KDL::Chain &chain,
        ow::JointStateX &ll_q,
        ow::JointStateX &ul_q) const;

    /*!
    * \brief check if limb id is valid.
    */
    bool checkLimbId(size_t id) const;
  };

} // namespace ow_ik

#endif
