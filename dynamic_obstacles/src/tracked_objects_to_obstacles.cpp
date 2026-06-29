#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <dynamic_obstacles/TrackedObject.h>
#include <dynamic_obstacles/TrackedObjectArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <uuid_msgs/UniqueID.h>
#include <visualization_msgs/MarkerArray.h>

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

geometry_msgs::Point makePoint(double x, double y, double z = 0.0)
{
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
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

double validOrDefault(double value, double default_value)
{
  return isFinite(value) && value > kEpsilon ? value : default_value;
}

bool hasUsableLinearVelocity(const dynamic_obstacles::TrackedObject& tracked_object)
{
  const auto& velocity = tracked_object.twist.twist.linear;
  return tracked_object.has_linear_velocity && isFinite(velocity.x) && isFinite(velocity.y) &&
         isFinite(velocity.z) && std::hypot(velocity.x, velocity.y) > kEpsilon;
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
    private_nh_.param<std::string>("marker_topic", marker_topic_, "/tracked_objects_to_obstacles/markers");
    private_nh_.param("default_radius", default_radius_, 0.3);
    private_nh_.param("default_height", default_height_, 0.6);
    private_nh_.param("publish_markers", publish_markers_, true);
    private_nh_.param("marker_lifetime", marker_lifetime_, 0.5);
    private_nh_.param("velocity_marker_seconds", velocity_marker_seconds_, 1.5);
    private_nh_.param("prediction_horizon", prediction_horizon_, 5.0);
    private_nh_.param("prediction_samples", prediction_samples_, 30);
    private_nh_.param("drop_duplicate_closing_vertex", drop_duplicate_closing_vertex_, true);
    private_nh_.param("min_velocity", min_velocity_, 0.1);
    
    if (!isFinite(default_radius_) || default_radius_ <= kEpsilon)
    {
      ROS_WARN("~default_radius must be positive; using 0.3 m");
      default_radius_ = 0.3;
    }
    if (!isFinite(default_height_) || default_height_ <= kEpsilon)
    {
      ROS_WARN("~default_height must be positive; using 0.6 m");
      default_height_ = 0.6;
    }
    if (!isFinite(marker_lifetime_) || marker_lifetime_ < 0.0)
    {
      ROS_WARN("~marker_lifetime must be non-negative; using 0.5 s");
      marker_lifetime_ = 0.5;
    }
    if (!isFinite(velocity_marker_seconds_) || velocity_marker_seconds_ < 0.0)
    {
      ROS_WARN("~velocity_marker_seconds must be non-negative; using 1.5 s");
      velocity_marker_seconds_ = 1.5;
    }
    if (!isFinite(prediction_horizon_) || prediction_horizon_ < 0.0)
    {
      ROS_WARN("~prediction_horizon must be non-negative; using 5.0 s");
      prediction_horizon_ = 5.0;
    }
    if (prediction_samples_ < 1)
    {
      ROS_WARN("~prediction_samples must be >= 1; using 30");
      prediction_samples_ = 30;
    }
    if (!isFinite(min_velocity_) || min_velocity_ < 0.0)
    {
      ROS_WARN("~min_velocity must be non-negative; using 0.1 m/s");
      min_velocity_ = 0.1;
    }

    obstacles_pub_ = nh_.advertise<costmap_converter::ObstacleArrayMsg>(output_topic_, 1);
    if (publish_markers_)
    {
      markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);
    }
    tracked_objects_sub_ = nh_.subscribe(input_topic_, 10, &TrackedObjectsToObstacles::trackedObjectsCallback, this);

    ROS_INFO_STREAM("Converting dynamic_obstacles/TrackedObjectArray from " << input_topic_
                    << " to costmap_converter/ObstacleArrayMsg on " << output_topic_);
    if (publish_markers_)
    {
      ROS_INFO_STREAM("Publishing tracked object visualization markers on " << marker_topic_);
    }
  }

private:
  void trackedObjectsCallback(const dynamic_obstacles::TrackedObjectArray::ConstPtr& msg)
  {
    costmap_converter::ObstacleArrayMsg output;
    output.header = msg->header;
    output.obstacles.reserve(msg->objects.size());

    visualization_msgs::MarkerArray markers;
    if (publish_markers_)
    {
      appendDeleteAllMarker(msg->header, markers);
    }

    for (const auto& tracked_object : msg->objects)
    {
      costmap_converter::ObstacleMsg obstacle;
      if (toObstacleMsg(tracked_object, msg->header, obstacle))
      {
        output.obstacles.push_back(obstacle);
        if (publish_markers_)
        {
          appendVisualizationMarkers(tracked_object, obstacle, msg->header, markers);
        }
      }
    }

    obstacles_pub_.publish(output);
    if (publish_markers_)
    {
      markers_pub_.publish(markers);
    }
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

    if (!tracked_object.has_linear_velocity || (tracked_object.twist.twist.linear.x < min_velocity_ && tracked_object.twist.twist.linear.y < min_velocity_))
    {
      ROS_INFO_THROTTLE(1.0, "Skipping tracked object with zero velocity");
      return false;
    }

    obstacle.header = header;
    obstacle.id = idFromUuid(tracked_object.object_id);
    obstacle.orientation = normalizedQuaternionMsg(pose.orientation);
    obstacle.velocities = tracked_object.twist;

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

  void appendDeleteAllMarker(
      const std_msgs::Header& header,
      visualization_msgs::MarkerArray& markers) const
  {
    visualization_msgs::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);
  }

  void appendVisualizationMarkers(
      const dynamic_obstacles::TrackedObject& tracked_object,
      const costmap_converter::ObstacleMsg& obstacle,
      const std_msgs::Header& header,
      visualization_msgs::MarkerArray& markers) const
  {
    markers.markers.push_back(buildBodyMarker(tracked_object, obstacle, header));

    if (hasUsableLinearVelocity(tracked_object))
    {
      markers.markers.push_back(buildVelocityMarker(tracked_object, obstacle, header));
      if (prediction_horizon_ > kEpsilon)
      {
        markers.markers.push_back(buildPredictionMarker(tracked_object, obstacle, header));
      }
    }
  }

  visualization_msgs::Marker buildBodyMarker(
      const dynamic_obstacles::TrackedObject& tracked_object,
      const costmap_converter::ObstacleMsg& obstacle,
      const std_msgs::Header& header) const
  {
    visualization_msgs::Marker marker;
    configureMarker(marker, header, markerNamespace(obstacle.id, "body"), visualization_msgs::Marker::CYLINDER);

    const auto& pose = tracked_object.pose.pose;
    const double height = objectHeight(tracked_object);
    marker.pose.position = pose.position;
    marker.pose.position.z += height * 0.5;
    marker.scale.x = default_radius_ * 2.0;
    marker.scale.y = default_radius_ * 2.0;
    marker.scale.z = height;

    switch (tracked_object.shape_type)
    {
      case dynamic_obstacles::TrackedObject::SHAPE_BOUNDING_BOX:
        configureMarker(marker, header, markerNamespace(obstacle.id, "body"), visualization_msgs::Marker::CUBE);
        marker.pose.position = pose.position;
        marker.pose.position.z += height * 0.5;
        marker.pose.orientation = normalizedQuaternionMsg(pose.orientation);
        marker.scale.x = validOrDefault(tracked_object.dimensions.x, default_radius_ * 2.0);
        marker.scale.y = validOrDefault(tracked_object.dimensions.y, default_radius_ * 2.0);
        marker.scale.z = height;
        break;

      case dynamic_obstacles::TrackedObject::SHAPE_CYLINDER:
      {
        const double diameter = validOrDefault(
            std::max(tracked_object.dimensions.x, tracked_object.dimensions.y),
            default_radius_ * 2.0);
        marker.scale.x = diameter;
        marker.scale.y = diameter;
        marker.scale.z = height;
        break;
      }

      case dynamic_obstacles::TrackedObject::SHAPE_POLYGON:
      {
        std::vector<geometry_msgs::Point32> local_vertices = normalizedFootprint(tracked_object);
        if (local_vertices.size() >= 3)
        {
          configureMarker(marker, header, markerNamespace(obstacle.id, "body"), visualization_msgs::Marker::TRIANGLE_LIST);
          marker.pose.position = pose.position;
          marker.pose.orientation = normalizedQuaternionMsg(pose.orientation);
          marker.scale.x = 1.0;
          marker.scale.y = 1.0;
          marker.scale.z = 1.0;
          marker.points = prismTriangles(local_vertices, height);
        }
        break;
      }

      case dynamic_obstacles::TrackedObject::SHAPE_UNKNOWN:
      default:
        break;
    }

    marker.color = colorForObstacle(obstacle.id, 0.75);
    return marker;
  }

  visualization_msgs::Marker buildVelocityMarker(
      const dynamic_obstacles::TrackedObject& tracked_object,
      const costmap_converter::ObstacleMsg& obstacle,
      const std_msgs::Header& header) const
  {
    visualization_msgs::Marker marker;
    configureMarker(marker, header, markerNamespace(obstacle.id, "velocity"), visualization_msgs::Marker::ARROW);
    marker.scale.x = 0.06;
    marker.scale.y = 0.14;
    marker.scale.z = 0.18;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    const auto& position = tracked_object.pose.pose.position;
    const auto& velocity = tracked_object.twist.twist.linear;
    const double z = position.z + objectHeight(tracked_object) + 0.08;
    marker.points.push_back(makePoint(position.x, position.y, z));
    marker.points.push_back(makePoint(
        position.x + velocity.x * velocity_marker_seconds_,
        position.y + velocity.y * velocity_marker_seconds_,
        z));
    return marker;
  }

  visualization_msgs::Marker buildPredictionMarker(
      const dynamic_obstacles::TrackedObject& tracked_object,
      const costmap_converter::ObstacleMsg& obstacle,
      const std_msgs::Header& header) const
  {
    visualization_msgs::Marker marker;
    configureMarker(marker, header, markerNamespace(obstacle.id, "prediction"), visualization_msgs::Marker::LINE_STRIP);
    marker.scale.x = 0.07;
    marker.color.r = 1.0;
    marker.color.g = 0.85;
    marker.color.b = 0.0;
    marker.color.a = 0.95;

    const auto& position = tracked_object.pose.pose.position;
    const auto& velocity = tracked_object.twist.twist.linear;
    marker.points.reserve(static_cast<size_t>(prediction_samples_ + 1));
    for (int index = 0; index <= prediction_samples_; ++index)
    {
      const double t = prediction_horizon_ * static_cast<double>(index) /
                       static_cast<double>(prediction_samples_);
      marker.points.push_back(makePoint(
          position.x + velocity.x * t,
          position.y + velocity.y * t,
          position.z + 0.06));
    }
    return marker;
  }

  void configureMarker(
      visualization_msgs::Marker& marker,
      const std_msgs::Header& header,
      const std::string& marker_namespace,
      int marker_type) const
  {
    marker.header = header;
    marker.ns = marker_namespace;
    marker.id = 0;
    marker.type = marker_type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(marker_lifetime_);
  }

  std::string markerNamespace(int64_t obstacle_id, const std::string& suffix) const
  {
    std::ostringstream stream;
    stream << "tracked_object_" << obstacle_id << "/" << suffix;
    return stream.str();
  }

  std_msgs::ColorRGBA colorForObstacle(int64_t obstacle_id, double alpha) const
  {
    static const std::array<std::array<double, 3>, 5> colors = {{
        {{0.95, 0.15, 0.10}},
        {{0.10, 0.55, 1.00}},
        {{0.20, 0.80, 0.25}},
        {{0.85, 0.30, 0.90}},
        {{1.00, 0.65, 0.10}},
    }};

    const auto& selected = colors[static_cast<size_t>(obstacle_id) % colors.size()];
    std_msgs::ColorRGBA color;
    color.r = selected[0];
    color.g = selected[1];
    color.b = selected[2];
    color.a = alpha;
    return color;
  }

  double objectHeight(const dynamic_obstacles::TrackedObject& tracked_object) const
  {
    return validOrDefault(tracked_object.dimensions.z, default_height_);
  }

  std::vector<geometry_msgs::Point32> normalizedFootprint(
      const dynamic_obstacles::TrackedObject& tracked_object) const
  {
    std::vector<geometry_msgs::Point32> vertices = tracked_object.footprint.points;
    if (drop_duplicate_closing_vertex_ && vertices.size() > 3 && sameXY(vertices.front(), vertices.back()))
    {
      vertices.pop_back();
    }
    return vertices;
  }

  std::vector<geometry_msgs::Point> prismTriangles(
      const std::vector<geometry_msgs::Point32>& vertices,
      double height) const
  {
    std::vector<geometry_msgs::Point> points;
    if (vertices.size() < 3)
    {
      return points;
    }

    points.reserve((vertices.size() - 2) * 6 + vertices.size() * 6);
    for (size_t index = 1; index + 1 < vertices.size(); ++index)
    {
      points.push_back(makePoint(vertices[0].x, vertices[0].y, height));
      points.push_back(makePoint(vertices[index].x, vertices[index].y, height));
      points.push_back(makePoint(vertices[index + 1].x, vertices[index + 1].y, height));
      points.push_back(makePoint(vertices[0].x, vertices[0].y, 0.0));
      points.push_back(makePoint(vertices[index + 1].x, vertices[index + 1].y, 0.0));
      points.push_back(makePoint(vertices[index].x, vertices[index].y, 0.0));
    }

    for (size_t index = 0; index < vertices.size(); ++index)
    {
      const auto& start = vertices[index];
      const auto& end = vertices[(index + 1) % vertices.size()];
      points.push_back(makePoint(start.x, start.y, 0.0));
      points.push_back(makePoint(end.x, end.y, 0.0));
      points.push_back(makePoint(end.x, end.y, height));
      points.push_back(makePoint(start.x, start.y, 0.0));
      points.push_back(makePoint(end.x, end.y, height));
      points.push_back(makePoint(start.x, start.y, height));
    }

    return points;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber tracked_objects_sub_;
  ros::Publisher obstacles_pub_;
  ros::Publisher markers_pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  double default_radius_;
  double default_height_;
  bool publish_markers_;
  double marker_lifetime_;
  double velocity_marker_seconds_;
  double prediction_horizon_;
  int prediction_samples_;
  double min_velocity_;
  bool drop_duplicate_closing_vertex_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracked_objects_to_obstacles");
  TrackedObjectsToObstacles node;
  ros::spin();
  return 0;
}
