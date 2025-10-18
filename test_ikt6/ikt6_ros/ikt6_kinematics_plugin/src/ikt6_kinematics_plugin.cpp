#include <ikt6_ros/ikt6_kinematics_plugin/ikt6_kinematics_plugin.h>
#include <class_loader/class_loader.hpp>

#include <tf2_eigen/tf2_eigen.h>

// register IKT6Kinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(ikt6_kinematics_plugin::IKT6KinematicsPlugin, kinematics::KinematicsBase)

namespace ikt6_kinematics_plugin
{
IKT6KinematicsPlugin::IKT6KinematicsPlugin() : active_(false)
{
}

bool IKT6KinematicsPlugin::initialize(
    const moveit::core::RobotModel& robot_model,
    const std::string& group_name,
    const std::string& base_frame,
    const std::vector<std::string>& tip_frames,
    double search_discretization)
{
  ROS_ERROR_NAMED("ikt6", "initialize");
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

  ROS_ERROR_NAMED("ikt6", "robot_model_.getName() = %s", robot_model.getName().c_str());
  ROS_ERROR_NAMED("ikt6", "robot_model_.isEmpty() = %d", robot_model.isEmpty());
  ROS_ERROR_NAMED("ikt6", "group_name_ = '%s'", group_name.c_str());

  joint_model_group_ = robot_model.getJointModelGroup(group_name);

  if (!joint_model_group_) {
    ROS_ERROR_NAMED("ikt6", "No joint_model_group_");
    return false;
  }

  dimension_ = joint_model_group_->getActiveJointModels().size() + joint_model_group_->getMimicJointModels().size();


  for (std::size_t i = 0; i < joint_model_group_->getJointModels().size(); ++i)
  {
    if (joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE ||
        joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)
    {
      solver_info_.joint_names.push_back(joint_model_group_->getJointModelNames()[i]);
      const std::vector<moveit_msgs::JointLimits>& jvec = joint_model_group_->getJointModels()[i]->getVariableBoundsMsg();
      solver_info_.limits.insert(solver_info_.limits.end(), jvec.begin(), jvec.end());
    }
  }

  Isometry3d base = Isometry3d::Identity();
  Isometry3d tool = Isometry3d::Identity();

  double s_limit_max = joint_model_group_->getActiveJointModels().at(0)->getVariableBoundsMsg().front().max_position;
  double s_limit_min = joint_model_group_->getActiveJointModels().at(0)->getVariableBoundsMsg().front().min_position;

  double l_limit_max = joint_model_group_->getActiveJointModels().at(1)->getVariableBoundsMsg().front().max_position;
  double l_limit_min = joint_model_group_->getActiveJointModels().at(1)->getVariableBoundsMsg().front().min_position;

  double u_limit_max = joint_model_group_->getActiveJointModels().at(3)->getVariableBoundsMsg().front().max_position;
  double u_limit_min = joint_model_group_->getActiveJointModels().at(3)->getVariableBoundsMsg().front().min_position;

  double r_limit_max = joint_model_group_->getActiveJointModels().at(4)->getVariableBoundsMsg().front().max_position;
  double r_limit_min = joint_model_group_->getActiveJointModels().at(4)->getVariableBoundsMsg().front().min_position;

  double b_limit_max = joint_model_group_->getActiveJointModels().at(5)->getVariableBoundsMsg().front().max_position;
  double b_limit_min = joint_model_group_->getActiveJointModels().at(5)->getVariableBoundsMsg().front().min_position;

  double t_limit_max = joint_model_group_->getActiveJointModels().at(6)->getVariableBoundsMsg().front().max_position;
  double t_limit_min = joint_model_group_->getActiveJointModels().at(6)->getVariableBoundsMsg().front().min_position;

  Vector6d lengths_eigen;
  Vector6d offsets_eigen;
  Vector6d directions_eigen;
  Vector6d limits_max_eigen;
  Vector6d limits_min_eigen;

   // TODO: Get it from configuration file
  lengths_eigen <<   0.340,     0.0,     0.4,   0.0,   0.4,   0.126;     // Lenghts
  offsets_eigen <<       0,       0,       0,     0,     0,     0;     // Joint offsets in Rad
  directions_eigen <<    1,       1,      -1,    -1,    1,      1;     // Joint directions
  limits_max_eigen <<  s_limit_max, l_limit_max, u_limit_max, r_limit_max, b_limit_max, t_limit_max;
  limits_min_eigen <<  s_limit_min, l_limit_min, u_limit_min, r_limit_min, b_limit_min, t_limit_min;

  ROS_ERROR_STREAM_NAMED("ikt6", "IKT6 Robot Parameters"
          << "\n lengths:    " << lengths_eigen.transpose()
          << "\n offsets:    " << offsets_eigen.transpose()
          << "\n directions: " << directions_eigen.transpose()
          << "\n limits_max: " << limits_max_eigen.transpose()
          << "\n limits_min: " << limits_min_eigen.transpose());



  robot_ = ikt6_robot_init(
      "ikt6_kinematics_plugin",
      lengths_eigen,
      offsets_eigen,
      directions_eigen,
      limits_max_eigen,
      limits_min_eigen,
      base,
      tool);

  active_ = true;
  ROS_INFO_NAMED("ikt6", "kinematics initialized");
  return true;
}

bool IKT6KinematicsPlugin::getPositionIK(
    const geometry_msgs::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    std::vector<double>& solution,
    moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(
      ik_pose,
      ik_seed_state,
      default_timeout_,
      solution,
      solution_callback,
      error_code,
      consistency_limits,
      options);
}

bool IKT6KinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double timeout,
    std::vector<double>& solution,
    moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(
      ik_pose,
      ik_seed_state,
      timeout,
      solution,
      solution_callback,
      error_code,
      consistency_limits,
      options);
}

bool IKT6KinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double timeout,
    const std::vector<double>& consistency_limits,
    std::vector<double>& solution,
    moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(
      ik_pose,
      ik_seed_state,
      timeout,
      solution,
      solution_callback,
      error_code,
      consistency_limits,
      options);
}

bool IKT6KinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double timeout,
    std::vector<double>& solution,
    const IKCallbackFn& solution_callback,
    moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(
      ik_pose,
      ik_seed_state,
      timeout,
      solution,
      solution_callback,
      error_code,
      consistency_limits,
      options);
}

bool IKT6KinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double timeout,
    const std::vector<double>& consistency_limits,
    std::vector<double>& solution,
    const IKCallbackFn& solution_callback,
    moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(
      ik_pose,
      ik_seed_state,
      timeout,
      solution,
      solution_callback,
      error_code,
      consistency_limits,
      options);
}

bool IKT6KinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double timeout,
    std::vector<double>& solution,
    const IKCallbackFn& solution_callback,
    moveit_msgs::MoveItErrorCodes& error_code,
    const std::vector<double>& consistency_limits,
    const kinematics::KinematicsQueryOptions& options) const
{
  ros::WallTime n1 = ros::WallTime::now();
  if (!active_)
  {
    ROS_ERROR_NAMED("ikt6", "kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  Isometry3d pose_T;
  Eigen::fromMsg(ik_pose, pose_T);


  ROS_ERROR_STREAM_NAMED("ikt6", "Query: \n" << pose_T.matrix());


  Matrix<double, 6, Dynamic> J = ikt6_ikt(&robot_, pose_T);

  ROS_ERROR_STREAM_NAMED("ikt6", "Solutions: \n" << J);

  // Select from solutions
  Vector6d ik_seed_state_;
  ik_seed_state_ << ik_seed_state[0], ik_seed_state[1], ik_seed_state[3], ik_seed_state[4], ik_seed_state[5], ik_seed_state[6];

  // selected solution as a Eigen vector
  Vector6d solution_;

  double min_val = std::numeric_limits<float>::infinity();
  Eigen::Index min_i = -1;
  for (Eigen::Index i = 0; i < J.cols(); i++) {
    if (!J.col(i).hasNaN()) {

      // Calculate solution reference value based on selection strategy
      double val;
      val = (J.col(i) - ik_seed_state_).array().pow(2).sum() / 6;

      // better solution found
      if (val < min_val) {
        min_val = val;
        min_i = i;
      }
    }
  }

  if (min_i < 0) {
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  solution_ = J.col(min_i);
  solution.push_back(solution_[0]);
  solution.push_back(solution_[1]);
  solution.push_back(0.0);
  solution.push_back(solution_[2]);
  solution.push_back(solution_[3]);
  solution.push_back(solution_[4]);
  solution.push_back(solution_[5]);

  return true;
}


bool IKT6KinematicsPlugin::getPositionFK(
    const std::vector<std::string>& link_names,
    const std::vector<double>& joint_angles,
    std::vector<geometry_msgs::Pose>& poses) const
{
  if (!active_)
  {
    ROS_ERROR_NAMED("ikt6", "kinematics not active");
    return false;
  }

  ROS_ERROR_NAMED("ikt6", "forward kinematics not implemented");
  return false;
}


const std::vector<std::string>& IKT6KinematicsPlugin::getJointNames() const
{
  return solver_info_.joint_names;
}

const std::vector<std::string>& IKT6KinematicsPlugin::getLinkNames() const
{
  return solver_info_.link_names;
}

}  // namespace
