#ifndef NEUPAN_SERVICE_NEUPAN_LOCAL_PLANNER_ROS_H_
#define NEUPAN_SERVICE_NEUPAN_LOCAL_PLANNER_ROS_H_

#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

#include <neupan_service/ComputeVelocity.h>
#include <neupan_service/SetPath.h>

namespace neupan_service
{

class NeuPANLocalPlannerROS : public nav_core::BaseLocalPlanner
{
public:
  NeuPANLocalPlannerROS();

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

  bool isGoalReached() override;

private:
  nav_msgs::Path makePathMessage(const std::vector<geometry_msgs::PoseStamped>& plan) const;
  bool transformPoseToPathFrame(const geometry_msgs::PoseStamped& input,
                                geometry_msgs::PoseStamped& output) const;
  bool waitForService(ros::ServiceClient& client, const std::string& service_name);

  bool initialized_;
  bool goal_reached_;
  bool use_tf_pose_;
  bool reset_on_set_plan_;
  bool publish_plans_;
  bool wait_for_services_on_initialize_;
  bool stop_returns_failure_;

  double wait_for_service_timeout_;
  double transform_timeout_;

  std::string path_frame_;
  std::string set_path_service_;
  std::string compute_velocity_service_;

  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::ServiceClient set_path_client_;
  ros::ServiceClient compute_velocity_client_;
  ros::Publisher local_plan_pub_;
  ros::Publisher reference_plan_pub_;

  nav_msgs::Path current_plan_;
};

}  // namespace neupan_service

#endif  // NEUPAN_SERVICE_NEUPAN_LOCAL_PLANNER_ROS_H_
