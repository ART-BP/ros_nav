#!/usr/bin/env python3

import math

import rospy
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point, Point32
from visualization_msgs.msg import Marker, MarkerArray


class DynamicObstaclePublisher:
    def __init__(self):
        self.output_topic = rospy.get_param(
            "~output_topic", "/move_base/TebLocalPlannerROS/obstacles"
        )
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/prediction_to_teb/markers"
        )
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.obstacle_id = rospy.get_param("~obstacle_id", 1)
        self.initial_x = rospy.get_param("~initial_x", 2.0)
        self.initial_y = rospy.get_param("~initial_y", -1.5)
        self.velocity_x = rospy.get_param("~velocity_x", 0.0)
        self.velocity_y = rospy.get_param("~velocity_y", 0.3)
        self.radius = rospy.get_param("~radius", 0.3)
        self.publish_rate = rospy.get_param("~publish_rate", 10.0)
        self.reset_period = rospy.get_param("~reset_period", 10.0)
        self.velocity_variance = rospy.get_param("~velocity_variance", 0.04)
        self.prediction_horizon = rospy.get_param("~prediction_horizon", 5.0)

        if self.publish_rate <= 0.0:
            raise ValueError("~publish_rate must be greater than zero")
        if self.radius < 0.0:
            raise ValueError("~radius must not be negative")

        self.publisher = rospy.Publisher(
            self.output_topic, ObstacleArrayMsg, queue_size=1
        )
        self.marker_publisher = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1
        )
        self.start_time = rospy.Time.now()
        rospy.on_shutdown(self.clear_obstacles)

        rospy.loginfo(
            "Publishing dynamic obstacle %d to %s in frame %s",
            self.obstacle_id,
            self.output_topic,
            self.frame_id,
        )
        rospy.loginfo("Publishing RViz obstacle markers to %s", self.marker_topic)

    def elapsed_time(self, now):
        elapsed = (now - self.start_time).to_sec()
        if self.reset_period > 0.0:
            elapsed %= self.reset_period
        return elapsed

    def build_message(self, now):
        elapsed = self.elapsed_time(now)
        obstacle = ObstacleMsg()
        obstacle.id = self.obstacle_id
        obstacle.radius = self.radius

        point = Point32()
        point.x = self.initial_x + self.velocity_x * elapsed
        point.y = self.initial_y + self.velocity_y * elapsed
        point.z = 0.0
        obstacle.polygon.points.append(point)

        speed = math.hypot(self.velocity_x, self.velocity_y)
        yaw = math.atan2(self.velocity_y, self.velocity_x) if speed > 1e-6 else 0.0
        obstacle.orientation.z = math.sin(yaw * 0.5)
        obstacle.orientation.w = math.cos(yaw * 0.5)

        obstacle.velocities.twist.linear.x = self.velocity_x
        obstacle.velocities.twist.linear.y = self.velocity_y
        obstacle.velocities.covariance[0] = self.velocity_variance
        obstacle.velocities.covariance[7] = self.velocity_variance

        message = ObstacleArrayMsg()
        message.header.stamp = now
        message.header.frame_id = self.frame_id
        message.obstacles.append(obstacle)
        return message

    def build_markers(self, obstacle_message):
        now = obstacle_message.header.stamp
        obstacle = obstacle_message.obstacles[0]
        position = obstacle.polygon.points[0]
        markers = MarkerArray()

        body = Marker()
        body.header = obstacle_message.header
        body.ns = "prediction_to_teb"
        body.id = 0
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x = position.x
        body.pose.position.y = position.y
        body.pose.position.z = 0.15
        body.pose.orientation.w = 1.0
        body.scale.x = max(2.0 * obstacle.radius, 0.05)
        body.scale.y = max(2.0 * obstacle.radius, 0.05)
        body.scale.z = 0.3
        body.color.r = 1.0
        body.color.g = 0.1
        body.color.b = 0.1
        body.color.a = 0.85
        body.lifetime = rospy.Duration(0.3)
        markers.markers.append(body)

        arrow = Marker()
        arrow.header = obstacle_message.header
        arrow.ns = "prediction_to_teb"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        arrow.scale.x = 0.08
        arrow.scale.y = 0.16
        arrow.scale.z = 0.2
        arrow.color.r = 1.0
        arrow.color.g = 1.0
        arrow.color.a = 1.0
        arrow.points = [
            Point(x=position.x, y=position.y, z=0.35),
            Point(
                x=position.x + self.velocity_x * 2.0,
                y=position.y + self.velocity_y * 2.0,
                z=0.35,
            ),
        ]
        arrow.lifetime = rospy.Duration(0.3)
        markers.markers.append(arrow)

        trajectory = Marker()
        trajectory.header.stamp = now
        trajectory.header.frame_id = self.frame_id
        trajectory.ns = "prediction_to_teb"
        trajectory.id = 2
        trajectory.type = Marker.LINE_STRIP
        trajectory.action = Marker.ADD
        trajectory.pose.orientation.w = 1.0
        trajectory.scale.x = 0.08
        trajectory.color.r = 1.0
        trajectory.color.g = 0.45
        trajectory.color.a = 0.9
        trajectory.lifetime = rospy.Duration(0.3)
        for step in range(21):
            prediction_time = self.prediction_horizon * step / 20.0
            trajectory.points.append(
                Point(
                    x=position.x + self.velocity_x * prediction_time,
                    y=position.y + self.velocity_y * prediction_time,
                    z=0.05,
                )
            )
        markers.markers.append(trajectory)

        return markers

    def clear_obstacles(self):
        message = ObstacleArrayMsg()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)
        marker = Marker()
        marker.header = message.header
        marker.action = Marker.DELETEALL
        self.marker_publisher.publish(MarkerArray(markers=[marker]))

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            message = self.build_message(rospy.Time.now())
            self.publisher.publish(message)
            self.marker_publisher.publish(self.build_markers(message))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("publish_dynamic_obstacle")
    try:
        DynamicObstaclePublisher().run()
    except (rospy.ROSInterruptException, ValueError) as error:
        rospy.logerr("%s", error)
