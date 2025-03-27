#include <iostream>
#include <ctime>

// urdf (only for joint limits...)
#include <urdf/model.h>

// kdl
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

// ow
#include <ow_core/types.h>
#include <ow_ik/utilities.h>

#include <ros/ros.h>

int main()
{
  // test settings
  std::string dir_path = "/home/simon/ros/workspaces/ow/data/";
  std::string urdf_file = "ur5_robot.urdf";
  std::string base_link_name = "ur5_base_link";
  std::string end_link_name = "ur5_ee_link";

  // variables
  urdf::Model urdf_model;
  KDL::Tree kdl_tree;
  KDL::Chain kdl_chain;
  int dof_chain;

  ow::VectorX q, qP, qPP, q_effort; 

  std::string path = dir_path + urdf_file;  

  // load urdf model
  if(!urdf_model.initFile(path)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }

  // construct tree
  if(!kdl_parser::treeFromFile(path, kdl_tree))
  {
    ROS_ERROR("Failled to contruct kdl tree");
    return -1;
  }

  //----------------------------------------------------------------------------

  // extract chain
  kdl_tree.getChain(base_link_name, end_link_name, kdl_chain);

  // get dof of chain
  dof_chain = kdl_chain.getNrOfJoints();
  std::cout << "dot_chain=" << dof_chain << std::endl;

  // setup the state
  q.setZero(dof_chain);
  qP.setZero(dof_chain);
  qPP.setZero(dof_chain);
  q_effort.setZero(dof_chain);

  //----------------------------------------------------------------------------
  // compute the forward kinematics
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

  /*KDL::Frame T_kdl;
  KDL::JntArray q_kdl(dof_chain);

  // joint values
  q << 
     2.8201029300689697, 
    -2.0355775356292725, 
    -1.4348351955413818, 
    -0.676140546798706, 
    -1.0098475217819214, 
     0.16580627858638763;
  q_kdl.data = q;

  // fk
  if(fk_solver.JntToCart(q_kdl, T_kdl)) 
  {
    ROS_ERROR("Failled to compute fk");
    return -1;
  }

  ow::HomogeneousTransformation T;
  T = ow_ik::convert(T, T_kdl);
  std::cout << "T_ef=\n" << T.toString() << std::endl;

  //----------------------------------------------------------------------------
  // compute the Jacobian Matrix
  KDL::ChainJntToJacSolver j_solver(kdl_chain);

  KDL::Jacobian J_kdl(dof_chain);

  if(j_solver.JntToJac(q_kdl,J_kdl))
  {
    ROS_ERROR("Failled to compute J");
    return -1;
  }

  ow::MatrixX J = J_kdl.data;
  std::cout << "J_ef=\n" << J << std::endl; */

  //----------------------------------------------------------------------------
  // compute the Inverse kinematics
  KDL::ChainIkSolverVel_pinv ik_solver_v(kdl_chain);
  KDL::ChainIkSolverPos_NR ik_solver_p(kdl_chain, fk_solver, ik_solver_v);

  KDL::Frame T_kdl;
  ow::HomogeneousTransformation T;
  T.matrix() <<
  -0.598801,  0.135186,  0.789406,  0.524985,
  -0.361291, -0.925259, -0.115606, -0.336034,
   0.714777,  -0.35443,  0.602888,  0.451977,
          0,         0,         0,         1;

  // set target
  ow::HomogeneousTransformation T_target;
  T_target = T;
  T_kdl = ow_ik::convert(T_kdl, T_target);

  // inital joint state
  KDL::JntArray q_init_kdl(dof_chain);
  q_init_kdl.data.setZero();

  // solution
  KDL::JntArray q_sol_kdl(dof_chain);
  q_sol_kdl.data.setZero();

  std::cout << "target=\n" << T_target.toString() << std::endl;

  // solver
  if(ik_solver_p.CartToJnt(q_init_kdl, T_kdl, q_sol_kdl) != KDL::SolverI::E_NOERROR)
  {
    ROS_ERROR("Failled to compute ik");
    return -1;
  }
  std::cout << "q_ik=" << q_sol_kdl.data.transpose() << std::endl;

  // test
  KDL::Frame T_sol_kdl;
  if(fk_solver.JntToCart(q_sol_kdl, T_sol_kdl)) 
  {
    ROS_ERROR("Failled to compute fk");
    return -1;
  }

  ow::HomogeneousTransformation T_sol;
  T_sol = ow_ik::convert(T_sol, T_sol_kdl);
  std::cout << "T_target=\n" << T_target.toString() << std::endl;
  std::cout << "T_sol=\n" << T_sol.toString() << std::endl;
  
  //----------------------------------------------------------------------------
  /* compute the Inverse dynamics
  KDL::ChainIdSolver_RNE id_solver(kdl_chain, KDL::Vector(0, 0, -9.81));

  KDL::JntArray q_(dof_chain);
  KDL::JntArray q_dot_(dof_chain);
  KDL::JntArray q_dotdot_(dof_chain);
  KDL::JntArray torques(dof_chain);
  KDL::Wrenches ws_;

  ws_.reserve(dof_chain);
  for(int i = 0; i < dof_chain; ++i) 
  {
    ws_.push_back(KDL::Wrench());
  }

  q_.data.setZero();
  q_dot_.data.setZero();
  q_dotdot_.data.setZero();

  if(id_solver.CartToJnt(q_, q_dot_, q_dotdot_, ws_, torques) != KDL::SolverI::E_NOERROR)
  {
    ROS_ERROR("Failled to compute id");
    return -1;
  }
  std::cout << "tau=" << torques.data << std::endl;*/

  return 0;
}
