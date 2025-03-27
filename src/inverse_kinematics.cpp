#include <ow_ik/inverse_kinematics.h>

namespace ow_ik
{

InverseKinematics::InverseKinematics() :
  Base("inverse_kinematics")
{
  limb_ids_.resize(2, -1);
}

InverseKinematics::~InverseKinematics()
{
}

bool InverseKinematics::init(const ow::Parameter& parameter,
                             ros::NodeHandle& nh)
{
  // load module parameter
  parameter_.add<std::string>("body/arms/left/ee");
  parameter_.add<std::string>("body/arms/left/name");
  parameter_.add<std::string>("body/arms/left/base");
  parameter_.add<std::string>("body/arms/right/base");
  parameter_.add<std::string>("body/arms/right/ee");
  parameter_.add<std::string>("body/arms/right/name");
  parameter_.add<std::string>("body/legs/left/base");
  parameter_.add<std::string>("body/legs/left/ee");
  parameter_.add<std::string>("body/legs/left/name");
  parameter_.add<std::string>("body/legs/right/base");
  parameter_.add<std::string>("body/legs/right/ee");
  parameter_.add<std::string>("body/legs/right/name");
  parameter_.add<ow::Scalar>("solver/lambda", 1e-3);
  parameter_.add<ow::Scalar>("solver/alpha", 0.007);
  parameter_.add<ow::Scalar>("solver/beta", 0.0);
  parameter_.add<ow::Scalar>("solver/pos_tol", 0.00001);
  parameter_.add<ow::Scalar>("solver/ori_tol", 0.00001);
  parameter_.add<ow::Scalar>("solver/step_tol", 1e-7);
  parameter_.add<int>("solver/max_iter", 100);
  parameter_.add<ow::Scalar>("solver/k_task", 150);
  parameter_.add<ow::Scalar>("solver/k_null", 50);

  if(!parameter_.load(nh, "kinematics"))
  {
    ROS_ERROR("%s::initialize: Config loading failed.",
              Base::name().c_str());
    return false;
  }

  // initalize the robot
  if(!robot_.initialize(parameter.get<std::string>("robot_description")))
  {
    ROS_ERROR("%s::initialize: Robot initializing failed.",
              Base::name().c_str());
    return false;
  }

  // left foot
  limb_ids_[ow::FootId::LEFT] = robot_.addLimb(
        parameter_.get<std::string>("body/legs/left/name"),
        parameter_.get<std::string>("body/legs/left/base"),
        parameter_.get<std::string>("body/legs/left/ee"),
        parameter_);
  if(limb_ids_[ow::FootId::LEFT] < 0)
  {
    ROS_ERROR("robot initialization limb");
    return false;
  }

  // right foot
  limb_ids_[ow::FootId::RIGHT] = robot_.addLimb(
        parameter_.get<std::string>("body/legs/right/name"),
        parameter_.get<std::string>("body/legs/right/base"),
        parameter_.get<std::string>("body/legs/right/ee"),
        parameter_);
  if(limb_ids_[ow::FootId::RIGHT] < 0)
  {
    ROS_ERROR("robot initialization limb");
    return false;
  }
  return true;
}

void InverseKinematics::update(
    ow::Flags& flags,
    const ow::JointState& q,
    const ow::CartesianState& X_l_w,
    const ow::CartesianState& X_r_w,
    const ow::CartesianState& X_com_w,
    const ow::CartesianState& X_com_hip)
{
  // target transformations
  ow::HomogeneousTransformation T_r_hip;
  ow::HomogeneousTransformation T_l_hip;

  // current transformations
  ow::HomogeneousTransformation T_l_w(X_l_w.pos());
  ow::HomogeneousTransformation T_r_w(X_r_w.pos());

  // compute the hip wrt world
  ow::HomogeneousTransformation T_hip_w;
  T_hip_w = X_com_w.pos()*X_com_hip.pos().inverse();

  // update the target transformations
  if(flags.supportFoot() == ow::FootId::LEFT)
  {
    // using left as odom
    updateLegTargets(T_l_w, T_r_w, T_hip_w, T_l_hip, T_r_hip);
  }
  else
  {
    // using right as odom
    updateLegTargets(T_r_w, T_l_w, T_hip_w, T_r_hip, T_l_hip);
  }

  // compute ik solution of both feed starting from hip frame
  robot_.inverseKinematics(q, T_r_hip, limb_ids_[ow::FootId::RIGHT]);
  robot_.inverseKinematics(q, T_l_hip, limb_ids_[ow::FootId::LEFT]);
}

void InverseKinematics::updateLegTargets(
    const ow::HomogeneousTransformation& T_odom_w,
    const ow::HomogeneousTransformation& T_swing_w,
    const ow::HomogeneousTransformation& T_fb_w,
    ow::HomogeneousTransformation& T_odom_fb,
    ow::HomogeneousTransformation& T_swing_fb)
{
  ow::HomogeneousTransformation T_odom_inv;
  ow::HomogeneousTransformation T_swing_odom;
  ow::HomogeneousTransformation T_fb_odom;

  T_odom_inv = T_odom_w.inverse();

  // moving foot and fb w.r.t odom frame
  T_swing_odom = T_odom_inv*T_swing_w;
  T_fb_odom = T_odom_inv*T_fb_w;

  // odom frame w.r.t fb (=reference foot)
  T_odom_fb = T_fb_odom.inverse();

  // swing foot w.r.t fb
  T_swing_fb = T_odom_fb*T_swing_odom;
}

size_t InverseKinematics::jointIndex(const ow::FootId& ref_foot) const
{
  return robot_.jointIndexLimb(limb_ids_[ref_foot]);
}

const ow::JointState& InverseKinematics::q() const
{
  return robot_.jointState();
}

}
