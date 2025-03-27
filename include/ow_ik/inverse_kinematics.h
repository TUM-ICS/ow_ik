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

#ifndef OPEN_WALKER_IK_H
#define OPEN_WALKER_IK_H

#include <ow_core/interfaces/i_inverse_kinematics.h>
#include <ow_ik/robot_model.h>

/*!
 * \brief Open Walker inverse kinematics module namespace. These classes
 * implement the inverse kinematics of the legs for the walking controllers.
 */
namespace ow_ik
{

/*!
 * \brief The InverseKinematics class
 *
 * This class implements the inverse kinematics module of the
 * openwalker framework. It computes the required joint angles of both legs
 * given the input the commanded cartesian position of the com and both feets.
 */
class InverseKinematics : 
    public ow::IInverseKinematics
{
public:
  typedef ow::IInverseKinematics Base;

protected:
  ow::Parameter parameter_;         //!< Configuration of this module
  ow_ik::RobotModel robot_;         //!< Robot model for ik computation
  std::vector<int> limb_ids_;       //!< Access ids for robot limbs

public:
  /*!
  * \brief InverseKinematics Default constructor.
  */
  InverseKinematics();

  // destructor
  virtual ~InverseKinematics();

  /*!
   * \brief Update joint state of the robot.
   *
   * \param q
   *    Current JointState.
   *
   * \param X_l_w
   *    Left foot w.r.t world.
   *
   * \param X_r_w
   *    Left foot w.r.t world.
   *
   * \param X_com_w
   *    Center of mass w.r.t world
   * 
   * \param X_com_hip
   *    Center of mass w.r.t hip
   */
  void update(
      ow::Flags& flags,
      const ow::JointState& q,
      const ow::CartesianState& X_l_w,
      const ow::CartesianState& X_r_w,
      const ow::CartesianState& X_com_w,
      const ow::CartesianState& X_com_hip = ow::CartesianState::Zero());

  /*!
   * \brief Access function.
   *
   * \return
   *    JointState of the robot computed by IK.
   */
  virtual const ow::JointState& q() const;

  /*!
   * \brief get the foot joint index
   */
  size_t jointIndex(const ow::FootId& ref_foot) const;

protected:
  /*!
   * \brief Initialization of InverseKinematics module
   */
  virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh);

  /*!
   * \brief Update legs kinematics from the CartesianStates of the CoM and the
   *    feet.
   *
   * \param T_odom_w
   *    HomogeneousTransformation from odometry (reference foot) frame to world
   *    frame.
   *
   * \param T_foot_w
   *    HomogeneousTransformation from other foot frame to world frame.
   *
   * \param T_hip_w
   *    HomogeneousTransformation from hip frame to world frame.
   *
   * \param T_odom_hip
   *    HomogeneousTransformation from odometry (reference foot) frame to world
   *    frame.
   *
   * \param T_foot_hip
   *    HomogeneousTransformation from other foot frame to world hip frame.
   */
  void updateLegTargets(
      const ow::HomogeneousTransformation& T_odom_w,
      const ow::HomogeneousTransformation& T_foot_w,
      const ow::HomogeneousTransformation& T_hip_w,
      ow::HomogeneousTransformation& T_odom_hip,
      ow::HomogeneousTransformation& T_foot_hip);
};

}

#endif
