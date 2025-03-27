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
  hyp.k_null = 10;

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

  // simulate some target
  ow::HomogeneousTransformation T_hip_w = ow::HomogeneousTransformation::Identity();
  T_hip_w.pos() << 0.5, 0.0, 0.0;
  ow::HomogeneousTransformation T_rl_w = ow::HomogeneousTransformation::Identity();
  T_rl_w.pos() << 0.5, -0.075, -0.7;
  ow::HomogeneousTransformation T_ll_w = ow::HomogeneousTransformation::Identity();
  T_ll_w.pos() << 0.5, 0.075, -0.7;

  std::cout << "T_hip_w=\n" << T_hip_w.toString() << std::endl;
  std::cout << "T_rl_w=\n" << T_rl_w.toString() << std::endl;
  std::cout << "T_ll_w=\n" << T_ll_w.toString() << std::endl;

  ow::HomogeneousTransformation T_rl_hip = T_hip_w.inverse()*T_rl_w;
  ow::HomogeneousTransformation T_ll_hip = T_hip_w.inverse()*T_ll_w;

  std::cout << "T_rl_hip=\n" << T_rl_hip.toString() << std::endl;
  std::cout << "T_ll_hip=\n" << T_ll_hip.toString() << std::endl;

  //----------------------------------------------------------------------------

  // current complete robot state vector
  ow::JointState cur_joint_state;
  cur_joint_state.setZero();

  // compute the ik solution for both feet
  std::vector<ow::HomogeneousTransformation> targets = {
    T_ll_hip,
    T_rl_hip};

  // for each foot
  for(int i = 0; i < robot_model.numLimbs()-2; ++i)
  {
    // set inital guess to zero
    ow::JointPositionX q_0 = ow::JointPositionX::Zero(robot_model.dofLimb(i));

    // compute
    robot_model.inverseKinematics(cur_joint_state, q_0, targets[i], ids[i]);
  }
  cur_joint_state = robot_model.jointState();

  // check by computing the forward kinematics  
  std::vector<ow::HomogeneousTransformation> soluations(targets.size());
  for(int i = 0; i < robot_model.numLimbs()-2; ++i)
  {
    soluations[i] = robot_model.forwardKinematics(cur_joint_state, ids[i]);
  }

  // 8.60882e-20  5.22982e-18    -0.748883      1.49777    -0.748879 -1.78256e-17

  // compair:
  for(int i = 0; i < soluations.size(); ++i)
  {
    std::cout << "target=\n" << targets[i].toString() 
      << "\nsolution=\n" << soluations[i].toString() << std::endl;
  }

  return 0;
};