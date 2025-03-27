#include <ow_ik/robot_model.h>

namespace ow_ik
{

  RobotModel::RobotModel() : is_initalized_(false),
                             q_(ow::JointState::Zero())
  {
  }

  bool RobotModel::initialize(const std::string &robot_description)
  {
    // load urdf model
    if (!urdf_model_.initString(robot_description))
    {
      ROS_ERROR("RobotModel::initialize: Failed to parse urdf file");
      return false;
    }

    // construct tree
    if (!kdl_parser::treeFromString(robot_description, tree_))
    {
      ROS_ERROR("RobotModel::initialize: Failed to contruct kdl tree");
      return false;
    }

    // construct rbdl
    if (!ow_rbdl::Addons::URDFReadFromString(
            robot_description.c_str(), &rbdl_model_, false, false))
    {
      ROS_ERROR("RobotModel::initialize: Failed to contruct rbdl model");
      return false;
    }

    is_initalized_ = true;
    return true;
  }

  size_t RobotModel::addLimb(
      const std::string &limb_name,
      const std::string &base_link_name,
      const std::string &end_link_name,
      const ow::Parameter &parameter)
  {
    if (!is_initalized_)
    {
      ROS_ERROR("RobotModel::addLimb: Not initialized");
      return -1;
    }

    // extract sub chain from complete robot
    KDL::Chain chain;
    if (!tree_.getChain(base_link_name, end_link_name, chain))
    {
      ROS_ERROR("RobotModel::addLimb: Failed to extract KDL chain");
      return -1;
    }
    RobotModel::RobotLimbDescription limb;

    // extract position in complete robot joint vector
    // which corresponds to the id of the first kdl segment - 1
    limb.idx_ = rbdl_model_.GetBodyId(chain.getSegment(0).getName().c_str()) - 1;

    // size of limb joint vector
    limb.size_ = chain.getNrOfJoints();

    // extract joint limits
    ow::JointStateX ll_q, ul_q;
    if (!loadJointStateLimits(
            urdf_model_,
            chain,
            ll_q,
            ul_q))
    {
      ROS_ERROR("RobotModel::addLimb: Failed to extract joint limits");
      return -1;
    }

    // create limb
    limb.limb_.reset(new ow_ik::RobotLimb(
        chain,
        q_.ref(limb.idx_, limb.size_),
        ll_q,
        ul_q,
        base_link_name,
        limb_name));

    // init limb
    limb.limb_->initialize(parameter);

    // add use position in vector as id
    size_t id = limbs_.size();
    limbs_.push_back(limb);
    return id;
  }

  size_t RobotModel::numLimbs() const
  {
    return limbs_.size();
  }

  size_t RobotModel::dofLimb(size_t id) const
  {
    if (!checkLimbId(id))
    {
      return 0;
    }
    return limbs_[id].size_;
  }

  size_t RobotModel::jointIndexLimb(size_t id) const
  {
    if (!checkLimbId(id))
    {
      return 0;
    }
    return limbs_[id].idx_;
  }

  const ow::HomogeneousTransformation &RobotModel::forwardKinematics(
      const ow::JointState &q,
      size_t id)
  {
    if (!checkLimbId(id))
    {
      static const ow::HomogeneousTransformation tmp = ow::HomogeneousTransformation::Identity();
      return tmp;
    }
    return limbs_[id].limb_->forwardKinematics(
        q.ref(limbs_[id].idx_, limbs_[id].size_));
  }

  bool RobotModel::inverseKinematics(
      const ow::JointState &q,
      const ow::JointPositionX &q_0,
      const ow::HomogeneousTransformation &Tee_b,
      size_t id)
  {
    if (!checkLimbId(id))
    {
      return false;
    }
    return limbs_[id].limb_->inverseKinematics(
        q.ref(limbs_[id].idx_, limbs_[id].size_),
        q_0,
        Tee_b);
  }

  bool RobotModel::inverseKinematics(
      const ow::JointState &q,
      const ow::HomogeneousTransformation &Tee_b,
      size_t id)
  {
    if (!checkLimbId(id))
    {
      return false;
    }
    return limbs_[id].limb_->inverseKinematics(
        q.ref(limbs_[id].idx_, limbs_[id].size_), 
        Tee_b);
  }

  bool RobotModel::inverseDynamics(
      const ow::JointState &q,
      const RobotLimb::Wrenches &wrenches,
      size_t id)
  {
    if (!checkLimbId(id))
    {
      return false;
    }
    return limbs_[id].limb_->inverseDynamics(
        q.ref(limbs_[id].idx_, limbs_[id].size_),
        wrenches);
  }

  const ow::JointState &RobotModel::jointState() const
  {
    return q_;
  }

  bool RobotModel::loadJointStateLimits(
      const urdf::Model &model,
      const KDL::Chain &chain,
      ow::JointStateX &ll_q,
      ow::JointStateX &ul_q) const
  {
    ll_q.setZero(chain.getNrOfJoints());
    ul_q.setZero(chain.getNrOfJoints());

    // set to some defaults limits
    ll_q.pos().setConstant(-2.0 * M_PI);
    ul_q.pos().setConstant(2.0 * M_PI);
    ll_q.vel().setConstant(-100);
    ul_q.vel().setConstant(100);
    ll_q.tau().setConstant(-1000);
    ul_q.tau().setConstant(1000);

    std::string name;
    for (int idx = 0; idx < chain.getNrOfJoints(); ++idx)
    {
      // joint name
      name = chain.getSegment(idx).getJoint().getName();

      urdf::JointLimitsSharedPtr limits =
          model.getJoint(name)->limits;

      // extract limits, if they exsist within the urdf file
      if (limits)
      {
        if (limits->upper || limits->lower)
        {
          ll_q.pos()[idx] = limits->lower;
          ul_q.pos()[idx] = limits->upper;
        }
        if (limits->velocity)
        {
          ll_q.vel()[idx] = -limits->velocity;
          ul_q.vel()[idx] = limits->velocity;
        }
        if (limits->effort)
        {
          ll_q.tau()[idx] = -limits->effort;
          ul_q.tau()[idx] = limits->effort;
        }
      }
    }
    return true;
  }

  bool RobotModel::checkLimbId(size_t id) const
  {
    if (id >= limbs_.size())
    {
      ROS_ERROR("Wrong limb id=%ld >= size=%ld", id, limbs_.size());
      return false;
    }
    return true;
  }

} // namespace ow_ik
