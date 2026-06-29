#include <neupan_service/neupan_local_planner_ros.h>

#include <pluginlib/class_list_macros.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace neupan_service
{

NeuPANLocalPlannerROS::NeuPANLocalPlannerROS()
  : initialized_(false)
  , goal_reached_(false)
  , use_tf_pose_(true)
  , reset_on_set_plan_(true)
  , publish_plans_(true)
  , wait_for_services_on_initialize_(false)
  , stop_returns_failure_(false)
  , wait_for_service_timeout_(2.0)
  , transform_timeout_(0.2)
  , tf_(nullptr)
  , costmap_ros_(nullptr)
{
}

void NeuPANLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                                       costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("NeuPANLocalPlannerROS has already been initialized");
    return;
  }

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  nh_ = ros::NodeHandle();
  private_nh_ = ros::NodeHandle("~/" + name);

  private_nh_.param("set_path_service", set_path_service_,
                    std::string("/neupan_service/set_path"));
  private_nh_.param("compute_velocity_service", compute_velocity_service_,
                    std::string("/neupan_service/compute_velocity"));
  private_nh_.param("wait_for_service_timeout", wait_for_service_timeout_, 2.0);
  private_nh_.param("transform_timeout", transform_timeout_, 0.2);
  private_nh_.param("use_tf_pose", use_tf_pose_, true);
  private_nh_.param("reset_on_set_plan", reset_on_set_plan_, true);
  private_nh_.param("publish_plans", publish_plans_, true);
  private_nh_.param("wait_for_services_on_initialize", wait_for_services_on_initialize_, false);
  private_nh_.param("stop_returns_failure", stop_returns_failure_, false);

  std::string default_path_frame = "map";
  if (costmap_ros_ != nullptr)
  {
    default_path_frame = costmap_ros_->getGlobalFrameID();
  }
  private_nh_.param("path_frame", path_frame_, default_path_frame);

  set_path_client_ = nh_.serviceClient<neupan_service::SetPath>(set_path_service_);
  compute_velocity_client_ =
      nh_.serviceClient<neupan_service::ComputeVelocity>(compute_velocity_service_);

  if (publish_plans_)
  {
    local_plan_pub_ = private_nh_.advertise<nav_msgs::Path>("local_plan", 1);
    reference_plan_pub_ = private_nh_.advertise<nav_msgs::Path>("reference_plan", 1);
  }

  if (wait_for_services_on_initialize_)
  {
    waitForService(set_path_client_, set_path_service_);
    waitForService(compute_velocity_client_, compute_velocity_service_);
  }

  initialized_ = true;
  ROS_INFO("Initialized NeuPAN local planner plugin with services [%s] and [%s]",
           set_path_service_.c_str(), compute_velocity_service_.c_str());
}

bool NeuPANLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("NeuPANLocalPlannerROS has not been initialized");
    return false;
  }

  if (plan.empty())
  {
    ROS_WARN("NeuPANLocalPlannerROS received an empty global plan");
    return false;
  }

  nav_msgs::Path path;
  try
  {
    path = makePathMessage(plan);
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN("Failed to transform global plan for NeuPAN: %s", ex.what());
    return false;
  }

  if (!waitForService(set_path_client_, set_path_service_))
  {
    return false;
  }

  neupan_service::SetPath srv;
  srv.request.path = path;
  srv.request.reset = reset_on_set_plan_;

  if (!set_path_client_.call(srv))
  {
    ROS_WARN("Failed to call NeuPAN set_path service [%s]", set_path_service_.c_str());
    return false;
  }

  if (!srv.response.success)
  {
    ROS_WARN("NeuPAN rejected global plan: %s", srv.response.message.c_str());
    return false;
  }

  current_plan_ = path;
  goal_reached_ = false;
  return true;
}

bool NeuPANLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  cmd_vel = geometry_msgs::Twist();

  if (!initialized_)
  {
    ROS_ERROR("NeuPANLocalPlannerROS has not been initialized");
    return false;
  }

  if (current_plan_.poses.empty())
  {
    ROS_WARN_THROTTLE(1.0, "NeuPAN local planner is waiting for a global plan");
    return false;
  }

  if (!waitForService(compute_velocity_client_, compute_velocity_service_))
  {
    return false;
  }

  neupan_service::ComputeVelocity srv;
  srv.request.use_tf_pose = use_tf_pose_;
  if (!use_tf_pose_)
  {
    if (costmap_ros_ == nullptr || !costmap_ros_->getRobotPose(srv.request.robot_pose))
    {
      ROS_WARN_THROTTLE(1.0, "Failed to get robot pose from costmap");
      return false;
    }
  }

  if (!compute_velocity_client_.call(srv))
  {
    ROS_WARN_THROTTLE(1.0, "Failed to call NeuPAN compute_velocity service [%s]",
                      compute_velocity_service_.c_str());
    return false;
  }

  if (!srv.response.success)
  {
    ROS_WARN_THROTTLE(1.0, "NeuPAN failed to compute velocity: %s",
                      srv.response.message.c_str());
    return false;
  }

  cmd_vel = srv.response.cmd_vel;
  goal_reached_ = srv.response.arrive;

  if (publish_plans_)
  {
    local_plan_pub_.publish(srv.response.local_plan);
    reference_plan_pub_.publish(srv.response.reference_plan);
  }

  if (srv.response.stop && stop_returns_failure_)
  {
    cmd_vel = geometry_msgs::Twist();
    ROS_WARN_THROTTLE(1.0, "NeuPAN reported stop; returning local planner failure");
    return false;
  }

  return true;
}

bool NeuPANLocalPlannerROS::isGoalReached()
{
  return initialized_ && goal_reached_;
}

nav_msgs::Path NeuPANLocalPlannerROS::makePathMessage(
    const std::vector<geometry_msgs::PoseStamped>& plan) const
{
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = path_frame_;
  path.poses.reserve(plan.size());

  for (const auto& pose : plan)
  {
    geometry_msgs::PoseStamped transformed_pose;
    transformPoseToPathFrame(pose, transformed_pose);
    transformed_pose.header.stamp = path.header.stamp;
    transformed_pose.header.frame_id = path.header.frame_id;
    path.poses.push_back(transformed_pose);
  }

  return path;
}

bool NeuPANLocalPlannerROS::transformPoseToPathFrame(
    const geometry_msgs::PoseStamped& input,
    geometry_msgs::PoseStamped& output) const
{
  geometry_msgs::PoseStamped stamped = input;
  if (stamped.header.frame_id.empty())
  {
    stamped.header.frame_id = path_frame_;
  }

  if (stamped.header.frame_id == path_frame_)
  {
    output = stamped;
    return true;
  }

  if (tf_ == nullptr)
  {
    throw tf2::TransformException("TF buffer is not available");
  }

  stamped.header.stamp = ros::Time(0);
  output = tf_->transform(stamped, path_frame_, ros::Duration(transform_timeout_));
  return true;
}

bool NeuPANLocalPlannerROS::waitForService(ros::ServiceClient& client,
                                           const std::string& service_name)
{
  if (wait_for_service_timeout_ < 0.0)
  {
    client.waitForExistence();
    return true;
  }

  const bool available = wait_for_service_timeout_ == 0.0
                             ? client.exists()
                             : client.waitForExistence(
                                   ros::Duration(wait_for_service_timeout_));
  if (!available)
  {
    ROS_WARN_THROTTLE(1.0, "Waiting for NeuPAN service [%s]", service_name.c_str());
  }
  return available;
}

}  // namespace neupan_service

PLUGINLIB_EXPORT_CLASS(neupan_service::NeuPANLocalPlannerROS, nav_core::BaseLocalPlanner)
