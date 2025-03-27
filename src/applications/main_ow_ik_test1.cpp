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
#include <ow_core/math.h>
#include <ow_ik/utilities.h>

//------------------------------------------------------------------------------

ow::Rotation3 exponential_map(ow::Scalar theta, const ow::Vector3& a)
{
  static const ow::Rotation3 I = ow::Rotation3::Identity();
  ow::Matrix3 a_hat = ow::hat(a);
  ow::Rotation3 R;
  R = I + a_hat*std::sin(theta) + (a*a.transpose() - I)*(ow::Scalar(1) - std::cos(theta));  
  return R;
}

ow::AngularPosition exponential_map_quad(ow::Scalar theta, const ow::Vector3& a)
{
  ow::Scalar theta_norm = std::abs(theta);
  if(theta_norm <= std::numeric_limits<ow::Scalar>::epsilon())
  {
    return ow::AngularPosition::Identity();
  }
  ow::Scalar th = ow::Scalar(0.5)*theta;
  ow::AngularPosition Q;
  Q.coeffs() << std::sin(th)*a, std::cos(th);
  return Q;
}

ow::Matrix6 adjointTransformation(const ow::Rotation3& R, const ow::Vector3 t)
{
  ow::Matrix6 Ad;
  Ad << R, ow::hat(t)*R, ow::Matrix3::Zero(), R;
  return Ad;
}

//------------------------------------------------------------------------------

class Chain
{
public:
  Chain() : 
    dof_(0),
    num_links_(0)
  {
  }

  void clear()
  {
    dof_ = 0;
    num_links_ = 0;
    joints_.clear();
    X_joints_parent_.clear();
    axis_joints.clear();
  }

  size_t dof_;
  size_t num_links_;
  std::vector<urdf::JointConstSharedPtr> joints_;
  std::vector<ow::CartesianPosition> X_joints_parent_;
  std::vector<ow::Vector3> axis_joints;
};

// load a kinematic chain from the urdf file 
Chain loadChain(
  urdf::Model& model,
  std::string& end_effector_link,
  std::string& base_link)
{
  bool success = true;
  Chain chain;
  std::vector<urdf::LinkConstSharedPtr> links;
  std::vector<urdf::JointConstSharedPtr> joints;
  ow::AngularPosition Q;
  ow::Vector3 a;

  // starting from the end effector link, load the chain
  links.push_back(model.getLink(end_effector_link));
  while(links.back()->name != base_link)
  {
    // save revolute joints
    joints.push_back(links.back()->parent_joint);

    // save the axis of rotation if not a fixed joint
    if(links.back()->parent_joint->type != urdf::Joint::FIXED)
    {
      chain.axis_joints.push_back( ow::Vector3(
        joints.back()->axis.x,
        joints.back()->axis.y,
        joints.back()->axis.z));
      chain.dof_++;
    }
    else
    {
      chain.axis_joints.push_back(ow::Vector3::Zero());
    }

    // save the joint to link transformation
    chain.X_joints_parent_.push_back( ow::CartesianPosition(
      joints.back()->parent_to_joint_origin_transform.position.x,
      joints.back()->parent_to_joint_origin_transform.position.y,
      joints.back()->parent_to_joint_origin_transform.position.z,
      joints.back()->parent_to_joint_origin_transform.rotation.w,
      joints.back()->parent_to_joint_origin_transform.rotation.x,
      joints.back()->parent_to_joint_origin_transform.rotation.y,
      joints.back()->parent_to_joint_origin_transform.rotation.z));

    // iterate tree
    links.push_back(links.back()->getParent());
    if(links.back() == nullptr)
    {
      success = false;
      break;
    }
  }

  if(!success)
  {
    chain.clear();
  }
  std::reverse(joints.begin(), joints.end());
  std::reverse(chain.axis_joints.begin(), chain.axis_joints.end());
  std::reverse(chain.X_joints_parent_.begin(), chain.X_joints_parent_.end());
  chain.joints_ = joints;
  chain.num_links_ = chain.joints_.size();
  return chain;
}

//------------------------------------------------------------------------------

// compute the forward kinematics for SE(3)
std::vector<ow::HomogeneousTransformation> forwardKinematicsSE3(
  const Chain& chain,
  const ow::JointPositionX& q)
{
  ow::HomogeneousTransformation Hi;
  std::vector<ow::HomogeneousTransformation> Hi_0_stack(chain.num_links_+1);
  Hi_0_stack[0] = ow::HomogeneousTransformation::Identity();

  for(size_t i = 0; i < chain.num_links_; ++i)
  {
    // get the transformaiton in SE(3)
    Hi.orientation() = chain.X_joints_parent_[i].angular().toRotationMatrix();
    Hi.position() = chain.X_joints_parent_[i].linear();

    // joint can rotate, add rotation based on q[i]
    if(chain.joints_[i]->type == urdf::Joint::REVOLUTE)
    {      
      // add the joint rotation
      Hi.orientation() = Hi.orientation()*exponential_map(q[i], chain.axis_joints[i]);
    }
    else if(chain.joints_[i]->type == urdf::Joint::PRISMATIC)
    {
      // add the joint translation
      Hi.position() = Hi.position() + q[i]*chain.axis_joints[i];
    }

    // compute the base transformation
    Hi_0_stack[i+1] = Hi_0_stack[i]*Hi;
  }
  return Hi_0_stack;
}

// compute the forward kinematics using the Quaternion group
// compute the forward kinematics for SE(3)
std::vector<ow::CartesianPosition> forwardKinematicsQ(
  const Chain& chain,
  const ow::JointPositionX& q)
{
  ow::CartesianPosition Xi;
  std::vector<ow::CartesianPosition> Xi_0_stack(chain.num_links_+1);
  Xi_0_stack[0] = ow::CartesianPosition::Identity();

  for(size_t i = 0; i < chain.num_links_; ++i)
  {
    // get the transformaiton in SE(3)
    Xi = chain.X_joints_parent_[i];

    // joint can rotate, add rotation based on q[i]
    if(chain.joints_[i]->type == urdf::Joint::REVOLUTE)
    {      
      // add the joint rotation
      Xi.angular() = Xi.angular()*ow::AngularPosition::ExpMap(q[i]*chain.axis_joints[i]);
    }
    else if(chain.joints_[i]->type == urdf::Joint::PRISMATIC)
    {
      // add the joint translation
      Xi.linear() = Xi.linear() + q[i]*chain.axis_joints[i];
    }

    // compute the base transformation
    Xi_0_stack[i+1] = Xi_0_stack[i]*Xi;
  }
  return Xi_0_stack;
}

//------------------------------------------------------------------------------

// compute the forward kinematics using the Quaternion group
// compute the forward kinematics for SE(3)
ow::MatrixX JacobianSE3(
  const Chain& chain,
  const ow::JointPositionX& q)
{
  ow::MatrixX J = ow::MatrixX::Zero(6, chain.dof_);
  ow::Vector3 ai_0;

  // compute the forward kinematics
  std::vector<ow::HomogeneousTransformation> Ti_0_stack = 
    forwardKinematicsSE3(chain, q);

  size_t k = 0;
  for(size_t i = 0; i < chain.num_links_; ++i) // HERE IS AN ERROR 
  {
    if(chain.joints_[i]->type == urdf::Joint::FIXED)
      continue;

    ai_0 = Ti_0_stack[i+1].orientation()*chain.axis_joints[i];

    // joint can rotate, add rotation based on q[i]
    if(chain.joints_[i]->type == urdf::Joint::REVOLUTE)
    {
      J.block(0, k, 3, 1) = ai_0.cross(Ti_0_stack.back().pos() - Ti_0_stack[i+1].pos());
      J.block(3, k, 3, 1) = ai_0;
    }
    else if(chain.joints_[i]->type == urdf::Joint::PRISMATIC)
    {
      J.block(0, k, 3, 1) = ai_0;
    }
    k++;
  }
  return J;
}

//------------------------------------------------------------------------------

// compute the forward kinematics using the Quaternion group
// compute the forward kinematics for SE(3)
ow::MatrixX JacobianQ(
  const Chain& chain,
  const ow::JointPositionX& q)
{
  ow::MatrixX J = ow::MatrixX::Zero(6, chain.dof_);
  ow::Vector3 ai_0;

  // compute the forward kinematics
  std::vector<ow::CartesianPosition> Xi_0_stack = 
    forwardKinematicsQ(chain, q);

  size_t k = 0;
  for(size_t i = 0; i < chain.num_links_; ++i) // HERE IS AN ERROR 
  {
    if(chain.joints_[i]->type == urdf::Joint::FIXED)
      continue;

    ai_0 = Xi_0_stack[i+1].angular()*chain.axis_joints[i];

    // joint can rotate, add rotation based on q[i]
    if(chain.joints_[i]->type == urdf::Joint::REVOLUTE)
    {
      J.block(0, k, 3, 1) = ai_0.cross(Xi_0_stack.back().pos() - Xi_0_stack[i+1].pos());
      J.block(3, k, 3, 1) = ai_0;
    }
    else if(chain.joints_[i]->type == urdf::Joint::PRISMATIC)
    {
      J.block(0, k, 3, 1) = ai_0;
    }
    k++;
  }
  return J;
}

int main()
{
  // test settings
  std::string dir_path = "/home/simon/ros/workspaces/ow/data/reemc_full_ft_hey5_no_float.urdf";

  // variables
  urdf::Model urdf_model;
  std::string path = "/home/simon/ros/workspaces/ow/data/reemc_full_ft_hey5_no_float.urdf";

  std::string base_link = "base_link";
  std::string right_sole_link = "right_sole_link";
  std::string left_sole_link = "left_sole_link";

  // load urdf model
  if(!urdf_model.initFile(path)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }

  // extract the foot chain between left_sole_link and base_link
  Chain chain_left = 
    loadChain(urdf_model, left_sole_link, base_link);
  
  Chain chain_right = 
    loadChain(urdf_model, right_sole_link, base_link);

  ow::JointPositionX q_left = M_PI*ow::JointPositionX::Random(chain_left.dof_);
  q_left << -0.385328,0.0101702,-0.488859,0.87345,-0.384451,-0.0101694;

  ow::HomogeneousTransformation T_l_b = forwardKinematicsSE3(chain_left, q_left).back();

  ow::CartesianPosition X_l_b = forwardKinematicsQ(chain_left, q_left).back();

  ow::JointPositionX q_right = ow::JointPositionX::Zero(chain_right.dof_);
  ow::HomogeneousTransformation T_r_b = forwardKinematicsSE3(chain_right, q_right).back();

  ow::MatrixX Jee_0 = JacobianSE3(chain_left, q_left);
  ow::MatrixX Jee_0_2 = JacobianQ(chain_left, q_left);

  // compair solution with kdl
  KDL::Tree tree_kdl;
  kdl_parser::treeFromFile(path, tree_kdl);

  KDL::Chain chain_kdl;
  tree_kdl.getChain(base_link, left_sole_link, chain_kdl);

  KDL::ChainFkSolverPos_recursive fk_solver(chain_kdl);
  KDL::Frame T_kdl;
  KDL::JntArray q_kdl(chain_left.dof_);
  q_kdl.data = q_left;

  fk_solver.JntToCart(q_kdl, T_kdl);
  ow::HomogeneousTransformation T_ref;
  T_ref = ow_ik::convert(T_ref, T_kdl);

  std::cout << "Joint val=" << q_left.toString() << std::endl;
  std::cout << "KDL: SOLUTION =\n" << T_ref.toString() << std::endl;
  std::cout << "HAND: SE(3)=\n" << T_l_b.toString() << std::endl;
  ow::HomogeneousTransformation T_l_b_sec(X_l_b);
  std::cout << "HAND: Q=\n" << T_l_b_sec.toString() << std::endl << std::endl;
  std::cout << "HAND: Q=\n" << X_l_b.toString() << std::endl << std::endl;

  //----------------------------------------------------------------------------
  KDL::ChainJntToJacSolver jac_solver(chain_kdl);

  KDL::Jacobian jac(chain_left.dof_);
  jac_solver.JntToJac(q_kdl, jac);

  ow::MatrixX J_ref = ow::MatrixX::Zero(6, chain_left.dof_);
  J_ref = jac.data;

  std::cout << "KDL:  Jee_0=\n" << J_ref << std::endl;
  std::cout << "Hand SE(3): Jee_0=\n" << Jee_0 << std::endl;
  std::cout << "Hand Q: Jee_0=\n" << Jee_0_2 << std::endl;

  //----------------------------------------------------------------------------
  std::random_device rd;
  ow::AngularPosition Q = ow::AngularPosition::Random(rd);

  ow::Rotation3 R = ow::Rotation3::Random(rd);

  std::cout << "Random=" << Q.toString() << std::endl;
  std::cout << "Random=\n" << R.toString() << std::endl;


  return 0;
}
