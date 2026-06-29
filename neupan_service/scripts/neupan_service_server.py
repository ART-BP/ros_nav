#!/usr/bin/env python

from math import atan2, cos, sin
import threading

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

from neupan import neupan
from neupan.util import get_transform
from neupan_service.srv import ComputeVelocity, ComputeVelocityResponse
from neupan_service.srv import SetPath, SetPathResponse


class NeuPANServiceServer:
    def __init__(self):
        rospy.init_node("neupan_service_server", anonymous=False)

        self.config_file = rospy.get_param("~config_file", None)
        if self.config_file is None:
            raise ValueError("~config_file is required")

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.lidar_frame = rospy.get_param("~lidar_frame", "")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.path_topic = rospy.get_param("~path_topic", "/initial_path")
        self.goal_topic = rospy.get_param("~goal_topic", "/neupan_goal")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.local_plan_topic = rospy.get_param("~local_plan_topic", "/neupan_plan")
        self.reference_plan_topic = rospy.get_param(
            "~reference_plan_topic", "/neupan_ref_state"
        )
        self.initial_path_topic = rospy.get_param(
            "~initial_path_topic", "/neupan_initial_path"
        )

        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.publish_cmd_vel = bool(rospy.get_param("~publish_cmd_vel", False))
        self.run_control_loop = bool(rospy.get_param("~run_control_loop", False))
        self.refresh_initial_path = bool(rospy.get_param("~refresh_initial_path", True))
        self.include_initial_path_direction = bool(
            rospy.get_param("~include_initial_path_direction", False)
        )
        self.flip_angle = bool(rospy.get_param("~flip_angle", False))

        scan_angle_range_param = rospy.get_param("~scan_angle_range", "-3.14 3.14")
        self.scan_angle_range = np.fromstring(
            scan_angle_range_param, dtype=np.float32, sep=" "
        )

        scan_range_param = rospy.get_param("~scan_range", "0.0 5.0")
        self.scan_range = np.fromstring(scan_range_param, dtype=np.float32, sep=" ")
        self.scan_downsample = int(rospy.get_param("~scan_downsample", 1))

        dune_checkpoint = rospy.get_param("~dune_checkpoint", None)
        self.neupan_planner = neupan.init_from_yaml(
            self.config_file, pan={"dune_checkpoint": dune_checkpoint}
        )

        self.listener = tf.TransformListener()
        self.lock = threading.RLock()
        self.robot_state = None
        self.obstacle_points = None
        self.last_cmd_vel = Twist()
        self.last_local_plan = Path()
        self.last_reference_plan = Path()
        self.stop = False
        self.arrive = False

        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.plan_pub = rospy.Publisher(self.local_plan_topic, Path, queue_size=10)
        self.ref_plan_pub = rospy.Publisher(
            self.reference_plan_topic, Path, queue_size=10
        )
        self.initial_path_pub = rospy.Publisher(
            self.initial_path_topic, Path, queue_size=10
        )

        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=1)

        self.set_path_srv = rospy.Service("~set_path", SetPath, self.handle_set_path)
        self.compute_velocity_srv = rospy.Service(
            "~compute_velocity", ComputeVelocity, self.handle_compute_velocity
        )

        rospy.loginfo("neupan_service_server ready")

    def spin(self):
        if not self.run_control_loop:
            rospy.spin()
            return

        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            response = self.compute_velocity(use_tf_pose=True)
            if response.success and self.publish_cmd_vel:
                self.cmd_vel_pub.publish(response.cmd_vel)
            rate.sleep()

    def handle_set_path(self, request):
        try:
            self.set_path(request.path, reset=request.reset)
            return SetPathResponse(True, "path accepted")
        except Exception as exc:
            rospy.logerr("failed to set path: %s", exc)
            return SetPathResponse(False, str(exc))

    def handle_compute_velocity(self, request):
        return self.compute_velocity(
            robot_pose=request.robot_pose, use_tf_pose=request.use_tf_pose
        )

    def compute_velocity(self, robot_pose=None, use_tf_pose=True):
        response = ComputeVelocityResponse()
        response.success = False
        response.arrive = False
        response.stop = False
        response.cmd_vel = Twist()
        response.local_plan = Path()
        response.reference_plan = Path()

        try:
            with self.lock:
                if use_tf_pose:
                    robot_state = self.lookup_robot_state()
                else:
                    robot_state = self.robot_state_from_pose(robot_pose)
                self.robot_state = robot_state

                if (
                    len(self.neupan_planner.waypoints) >= 1
                    and self.neupan_planner.initial_path is None
                ):
                    self.neupan_planner.set_initial_path_from_state(robot_state)

                if self.neupan_planner.initial_path is None:
                    response.message = "waiting for initial path"
                    return response

                self.initial_path_pub.publish(
                    self.path_msg_from_neupan_points(self.neupan_planner.initial_path)
                )

                action, info = self.neupan_planner(robot_state, self.obstacle_points)
                self.stop = bool(info["stop"])
                self.arrive = bool(info["arrive"])

                response.arrive = self.arrive
                response.stop = self.stop
                response.cmd_vel = self.twist_msg_from_action(action)
                response.local_plan = self.path_msg_from_neupan_points(
                    info["opt_state_list"]
                )
                response.reference_plan = self.path_msg_from_neupan_points(
                    info["ref_state_list"]
                )
                response.success = True
                response.message = "ok"

                self.last_cmd_vel = response.cmd_vel
                self.last_local_plan = response.local_plan
                self.last_reference_plan = response.reference_plan

                self.plan_pub.publish(response.local_plan)
                self.ref_plan_pub.publish(response.reference_plan)

                if self.publish_cmd_vel:
                    self.cmd_vel_pub.publish(response.cmd_vel)

                if self.stop:
                    rospy.logwarn_throttle(
                        0.5,
                        "neupan stop with min distance %s threshold %s",
                        self.neupan_planner.min_distance.detach().item(),
                        self.neupan_planner.collision_threshold,
                    )

                return response
        except Exception as exc:
            rospy.logerr_throttle(1.0, "compute velocity failed: %s", exc)
            response.message = str(exc)
            return response

    def scan_callback(self, scan_msg):
        with self.lock:
            if self.robot_state is None:
                try:
                    self.robot_state = self.lookup_robot_state()
                except Exception:
                    return

            ranges = np.array(scan_msg.ranges)
            angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
            if self.flip_angle:
                angles = np.flip(angles)

            points = []
            for index, distance in enumerate(ranges):
                angle = angles[index]
                if (
                    index % self.scan_downsample == 0
                    and self.scan_range[0] <= distance <= self.scan_range[1]
                    and self.scan_angle_range[0] < angle < self.scan_angle_range[1]
                ):
                    points.append(
                        np.array([[distance * cos(angle)], [distance * sin(angle)]])
                    )

            if not points:
                self.obstacle_points = None
                return

            point_array = np.hstack(points)
            scan_frame = self.lidar_frame or scan_msg.header.frame_id
            if not scan_frame:
                rospy.loginfo_throttle(1.0, "waiting for scan frame_id")
                self.obstacle_points = None
                return

            try:
                if scan_frame == self.map_frame:
                    self.obstacle_points = point_array
                else:
                    trans, rot = self.listener.lookupTransform(
                        self.map_frame, scan_frame, rospy.Time(0)
                    )
                    yaw = self.yaw_from_quat_list(rot)
                    trans_matrix, rot_matrix = get_transform(
                        np.c_[trans[0], trans[1], yaw].reshape(3, 1)
                    )
                    self.obstacle_points = rot_matrix @ point_array + trans_matrix
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                rospy.loginfo_throttle(
                    1.0,
                    "waiting for tf from %s to %s",
                    scan_frame,
                    self.map_frame,
                )

    def path_callback(self, path_msg):
        try:
            self.set_path(path_msg, reset=True)
            rospy.loginfo_throttle(0.5, "initial path updated from topic")
        except Exception as exc:
            rospy.logerr("failed to update path from topic: %s", exc)

    def goal_callback(self, goal_msg):
        try:
            with self.lock:
                robot_state = self.lookup_robot_state()
                goal = self.robot_state_from_pose(goal_msg)
                self.neupan_planner.update_initial_path_from_goal(robot_state, goal)
                self.neupan_planner.reset()
                rospy.loginfo("goal accepted: [%s, %s]", goal[0, 0], goal[1, 0])
        except Exception as exc:
            rospy.logerr("failed to set goal: %s", exc)

    def set_path(self, path_msg, reset=True):
        points = self.neupan_points_from_path(path_msg)
        if not points:
            raise ValueError("path is empty")

        with self.lock:
            if self.neupan_planner.initial_path is None or self.refresh_initial_path:
                self.neupan_planner.set_initial_path(points)
                if reset:
                    self.neupan_planner.reset()

    def lookup_robot_state(self):
        trans, rot = self.listener.lookupTransform(
            self.map_frame, self.base_frame, rospy.Time(0)
        )
        yaw = self.yaw_from_quat_list(rot)
        return np.array([trans[0], trans[1], yaw]).reshape(3, 1)

    def robot_state_from_pose(self, pose_msg):
        if pose_msg is None:
            raise ValueError("robot_pose is required when use_tf_pose is false")

        if pose_msg.header.frame_id and pose_msg.header.frame_id != self.map_frame:
            pose_msg = self.listener.transformPose(self.map_frame, pose_msg)

        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        theta = self.yaw_from_quat(pose_msg.pose.orientation)
        return np.array([x, y, theta]).reshape(3, 1)

    def neupan_points_from_path(self, path_msg):
        points = []
        for index, pose_stamped in enumerate(path_msg.poses):
            pose = pose_stamped.pose
            x = pose.position.x
            y = pose.position.y

            if self.include_initial_path_direction:
                theta = self.yaw_from_quat(pose.orientation)
            elif index + 1 < len(path_msg.poses):
                next_pose = path_msg.poses[index + 1].pose
                theta = atan2(next_pose.position.y - y, next_pose.position.x - x)
            elif points:
                theta = points[-1][2, 0]
            else:
                theta = self.yaw_from_quat(pose.orientation)

            points.append(np.array([x, y, theta, 1.0]).reshape(4, 1))

        return points

    def path_msg_from_neupan_points(self, point_list):
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = rospy.Time.now()

        for index, point in enumerate(point_list):
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = path_msg.header.stamp
            pose.header.seq = index
            pose.pose.position.x = point[0, 0]
            pose.pose.position.y = point[1, 0]
            pose.pose.orientation = self.quat_from_yaw(point[2, 0])
            path_msg.poses.append(pose)

        return path_msg

    def twist_msg_from_action(self, action):
        cmd_vel = Twist()
        if action is None or self.stop or self.arrive:
            return cmd_vel

        cmd_vel.linear.x = action[0, 0]
        cmd_vel.angular.z = action[1, 0]
        return cmd_vel

    @staticmethod
    def quat_from_yaw(yaw):
        quat = Quaternion()
        quat.z = sin(yaw / 2.0)
        quat.w = cos(yaw / 2.0)
        return quat

    @staticmethod
    def yaw_from_quat(quat):
        return atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.z * quat.z + quat.y * quat.y),
        )

    @staticmethod
    def yaw_from_quat_list(quat):
        return atan2(
            2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
            1.0 - 2.0 * (quat[2] * quat[2] + quat[1] * quat[1]),
        )


if __name__ == "__main__":
    server = NeuPANServiceServer()
    server.spin()
