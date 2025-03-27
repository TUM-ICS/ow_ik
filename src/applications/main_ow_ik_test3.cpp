#include <iostream>
#include <ctime>

#include <ow_ik/robot_model.h>
#include <ros/ros.h>

int main()
{
  //----------------------------------------------------------------------------
  // file settings
  std::string dir_path = "/home/simon/ros/workspaces/tmp/tests/";
  std::string urdf_file = "reemc_full_ft_hey5.urdf";

  // solver settings
  ow_ik::SolverSettings hyp;
  hyp.lambda = 1e-3;
  hyp.alpha = 0.007;
  hyp.beta = 0.0;
  hyp.pos_tol = 0.00001;
  hyp.ori_tol = 0.00001;
  hyp.step_tol = 1e-7;
  hyp.max_iter = 100;
  hyp.k_task = 150;
  hyp.k_null = 50;

  // robot model
  ow_ik::RobotModel robot_model;

  // init
  std::string path = dir_path + urdf_file; 
  if (!robot_model.initialize(path))
  {
    ROS_ERROR("Failled to contruct robot model");
    return -1;
  }

  std::vector<int> ids(4, -1);

  // add limbs
  ids[0] = robot_model.addLimb(
    "leg_left", "base_link", "left_sole_link", hyp);
  if(ids[0] < 0)
  {
    ROS_ERROR("Failled to contruct limb 1");
    return -1;
  }

  ids[1] = robot_model.addLimb(
    "leg_right", "base_link", "right_sole_link", hyp);
  if(ids[1] < 0)
  {
    ROS_ERROR("Failled to contruct limb 2");
    return -1;
  }

  ids[2] = robot_model.addLimb(
    "arm_left", "torso_2_link", "hand_left_palm_link", hyp);
  if(ids[2] < 0)
  {
    ROS_ERROR("Failled to contruct limb 3");
    return -1;
  }

  ids[3] = robot_model.addLimb(
    "arm_right", "torso_2_link", "hand_right_palm_link", hyp);
  if(ids[3] < 0)
  {
    ROS_ERROR("Failled to contruct limb 4");
    return -1;
  }

  //----------------------------------------------------------------------------

  // current complete robot state vector (n=18)
  ow::JointState cur_joint_state;
  cur_joint_state.setZero();

  // simulate some target
  ow::HomogeneousTransformation T_target;
  T_target.matrix() <<
  -0.598801,  0.135186,  0.789406,  0.524985,
  -0.361291, -0.925259, -0.115606, -0.336034,
   0.714777,  -0.35443,  0.602888,  0.451977,
          0,         0,         0,         1;

  //----------------------------------------------------------------------------

  std::cout << "before joint_state=\n" << robot_model.jointState().toString() << std::endl;

  // compute the inverse kinematics 2
  for(int i = 0; i < robot_model.numLimbs(); ++i)
  {
    ow::JointPositionX q_0 = ow::JointPositionX::Zero(robot_model.dofLimb(i));

    robot_model.inverseKinematics(cur_joint_state, q_0, T_target, ids[i]);
  }
  std::cout << "after joint_state=\n" << robot_model.jointState().toString() << std::endl;;


  return 0;
};