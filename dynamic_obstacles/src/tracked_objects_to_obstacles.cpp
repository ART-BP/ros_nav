#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <dynamic_obstacles/TrackedObject.h>
#include <dynamic_obstacles/TrackedObjectArray.h>
#include <geometry_msgs/Point32.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <uuid_msgs/UniqueID.h>

namespace
{

constexpr double kEpsilon = 1e-6;

bool isFinite(double value)
{
  return std::isfinite(value);
}

bool hasValidPosition(const geometry_msgs::Point& position)
{
  return isFinite(position.x) && isFinite(position.y) && isFinite(position.z);
}

tf2::Quaternion normalizedQuaternion(const geometry_msgs::Quaternion& msg)
{
  tf2::Quaternion quaternion(msg.x, msg.y, msg.z, msg.w);
  const double length2 = quaternion.length2();
  if (!isFinite(length2) || length2 < kEpsilon)
  {
    return tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
  }
  quaternion.normalize();
  return quaternion;
}

geometry_msgs::Quaternion normalizedQuaternionMsg(const geometry_msgs::Quaternion& msg)
{
  const tf2::Quaternion quaternion = normalizedQuaternion(msg);
  geometry_msgs::Quaternion normalized;
  normalized.x = quaternion.x();
  normalized.y = quaternion.y();
  normalized.z = quaternion.z();
  normalized.w = quaternion.w();
  return normalized;
}

geometry_msgs::Point32 makePoint32(double x, double y, double z = 0.0)
{
  geometry_msgs::Point32 point;
  point.x = static_cast<float>(x);
  point.y = static_cast<float>(y);
  point.z = static_cast<float>(z);
  return point;
}

geometry_msgs::Point32 transformLocalPoint(
    const geometry_msgs::Point& origin,
    const tf2::Matrix3x3& rotation,
    double local_x,
    double local_y,
    double local_z = 0.0)
{
  const tf2::Vector3 local(local_x, local_y, local_z);
  const tf2::Vector3 world = rotation * local;
  return makePoint32(origin.x + world.x(), origin.y + world.y(), origin.z + world.z());
}

bool sameXY(const geometry_msgs::Point32& lhs, const geometry_msgs::Point32& rhs)
{
  return std::fabs(lhs.x - rhs.x) < kEpsilon && std::fabs(lhs.y - rhs.y) < kEpsilon;
}

int64_t idFromUuid(const uuid_msgs::UniqueID& object_id)
{
  uint64_t hash = 1469598103934665603ULL;
  for (const uint8_t byte : object_id.uuid)
  {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return static_cast<int64_t>(hash & 0x7fffffffffffffffULL);
}

}  // namespace

class TrackedObjectsToObstacles
{
public:
  TrackedObjectsToObstacles()
      : private_nh_("~")
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/tracked_objects");
    private_nh_.param<std::string>("output_topic", output_topic_, "/move_base/TebLocalPlannerROS/obstacles");
    private_nh_.param("default_radius", default_radius_, 0.3);
    private_nh_.param("drop_duplicate_closing_vertex", drop_duplicate_closing_vertex_, true);
    if (!isFinite(default_radius_) || default_radius_ <= kEpsilon)
    {
      ROS_WARN("~default_radius must be positive; using 0.3 m");
      default_radius_ = 0.3;
    }

    obstacles_pub_ = nh_.advertise<costmap_converter::ObstacleArrayMsg>(output_topic_, 1);
    tracked_objects_sub_ = nh_.subscribe(input_topic_, 10, &TrackedObjectsToObstacles::trackedObjectsCallback, this);

    ROS_INFO_STREAM("Converting dynamic_obstacles/TrackedObjectArray from " << input_topic_
                    << " to costmap_converter/ObstacleArrayMsg on " << output_topic_);
  }

private:
  void trackedObjectsCallback(const dynamic_obstacles::TrackedObjectArray::ConstPtr& msg)
  {
    costmap_converter::ObstacleArrayMsg output;
    output.header = msg->header;
    output.obstacles.reserve(msg->objects.size());

    for (const auto& tracked_object : msg->objects)
    {
      costmap_converter::ObstacleMsg obstacle;
      if (toObstacleMsg(tracked_object, msg->header, obstacle))
      {
        output.obstacles.push_back(obstacle);
      }
    }

    obstacles_pub_.publish(output);
  }

  bool toObstacleMsg(
      const dynamic_obstacles::TrackedObject& tracked_object,
      const std_msgs::Header& header,
      costmap_converter::ObstacleMsg& obstacle) const
  {
    const auto& pose = tracked_object.pose.pose;
    if (!hasValidPosition(pose.position))
    {
      ROS_WARN_THROTTLE(1.0, "Skipping tracked object with invalid position");
      return false;
    }

    obstacle.header = header;
    obstacle.id = idFromUuid(tracked_object.object_id);
    obstacle.orientation = normalizedQuaternionMsg(pose.orientation);
    obstacle.velocities = tracked_object.twist;

    if (!tracked_object.has_linear_velocity)
    {
      obstacle.velocities.twist.linear.x = 0.0;
      obstacle.velocities.twist.linear.y = 0.0;
      obstacle.velocities.twist.linear.z = 0.0;
    }
    if (!tracked_object.has_angular_velocity)
    {
      obstacle.velocities.twist.angular.x = 0.0;
      obstacle.velocities.twist.angular.y = 0.0;
      obstacle.velocities.twist.angular.z = 0.0;
    }

    switch (tracked_object.shape_type)
    {
      case dynamic_obstacles::TrackedObject::SHAPE_BOUNDING_BOX:
        return fillBoundingBox(tracked_object, obstacle);
      case dynamic_obstacles::TrackedObject::SHAPE_CYLINDER:
        return fillCylinder(tracked_object, obstacle);
      case dynamic_obstacles::TrackedObject::SHAPE_POLYGON:
        return fillPolygon(tracked_object, obstacle);
      case dynamic_obstacles::TrackedObject::SHAPE_UNKNOWN:
      default:
        return fillFallbackPoint(tracked_object, obstacle);
    }
  }

  bool fillBoundingBox(
      const dynamic_obstacles::TrackedObject& tracked_object,
      costmap_converter::ObstacleMsg& obstacle) const
  {
    const double length = tracked_object.dimensions.x;
    const double width = tracked_object.dimensions.y;
    if (!isFinite(length) || !isFinite(width) || length <= kEpsilon || width <= kEpsilon)
    {
      ROS_WARN_THROTTLE(1.0, "Bounding box obstacle has invalid dimensions; publishing fallback circle");
      return fillFallbackPoint(tracked_object, obstacle);
    }

    const auto& position = tracked_object.pose.pose.position;
    const auto rotation = tf2::Matrix3x3(normalizedQuaternion(tracked_object.pose.pose.orientation));
    const double half_length = 0.5 * length;
    const double half_width = 0.5 * width;

    obstacle.radius = 0.0;
    obstacle.polygon.points.clear();
    obstacle.polygon.points.push_back(transformLocalPoint(position, rotation, -half_length, -half_width));
    obstacle.polygon.points.push_back(transformLocalPoint(position, rotation, half_length, -half_width));
    obstacle.polygon.points.push_back(transformLocalPoint(position, rotation, half_length, half_width));
    obstacle.polygon.points.push_back(transformLocalPoint(position, rotation, -half_length, half_width));
    return true;
  }

  bool fillCylinder(
      const dynamic_obstacles::TrackedObject& tracked_object,
      costmap_converter::ObstacleMsg& obstacle) const
  {
    const auto& position = tracked_object.pose.pose.position;
    const double diameter = std::max(tracked_object.dimensions.x, tracked_object.dimensions.y);
    const double radius = isFinite(diameter) && diameter > kEpsilon ? 0.5 * diameter : default_radius_;

    obstacle.radius = radius;
    obstacle.polygon.points.clear();
    obstacle.polygon.points.push_back(makePoint32(position.x, position.y, position.z));
    return true;
  }

  bool fillPolygon(
      const dynamic_obstacles::TrackedObject& tracked_object,
      costmap_converter::ObstacleMsg& obstacle) const
  {
    if (tracked_object.footprint.points.size() < 3)
    {
      ROS_WARN_THROTTLE(1.0, "Polygon obstacle has fewer than 3 vertices; publishing fallback circle");
      return fillFallbackPoint(tracked_object, obstacle);
    }

    const auto& position = tracked_object.pose.pose.position;
    const auto rotation = tf2::Matrix3x3(normalizedQuaternion(tracked_object.pose.pose.orientation));

    obstacle.radius = 0.0;
    obstacle.polygon.points.clear();
    obstacle.polygon.points.reserve(tracked_object.footprint.points.size());

    for (const auto& local_point : tracked_object.footprint.points)
    {
      obstacle.polygon.points.push_back(
          transformLocalPoint(position, rotation, local_point.x, local_point.y, local_point.z));
    }

    if (drop_duplicate_closing_vertex_ && obstacle.polygon.points.size() > 3 &&
        sameXY(obstacle.polygon.points.front(), obstacle.polygon.points.back()))
    {
      obstacle.polygon.points.pop_back();
    }

    return obstacle.polygon.points.size() >= 3;
  }

  bool fillFallbackPoint(
      const dynamic_obstacles::TrackedObject& tracked_object,
      costmap_converter::ObstacleMsg& obstacle) const
  {
    const auto& position = tracked_object.pose.pose.position;
    obstacle.radius = default_radius_;
    obstacle.polygon.points.clear();
    obstacle.polygon.points.push_back(makePoint32(position.x, position.y, position.z));
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber tracked_objects_sub_;
  ros::Publisher obstacles_pub_;
  std::string input_topic_;
  std::string output_topic_;
  double default_radius_;
  bool drop_duplicate_closing_vertex_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracked_objects_to_obstacles");
  TrackedObjectsToObstacles node;
  ros::spin();
  return 0;
}
