#!/usr/bin/env python3

import bisect
import math

import rospy
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point, Point32
from visualization_msgs.msg import Marker, MarkerArray


EPSILON = 1e-9
DEFAULT_COLORS = (
    (0.95, 0.15, 0.10, 0.85),
    (0.10, 0.55, 1.00, 0.85),
    (0.20, 0.80, 0.25, 0.85),
    (0.85, 0.30, 0.90, 0.85),
    (1.00, 0.65, 0.10, 0.85),
)


def as_xy(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError("{} must contain [x, y]".format(name))
    return float(value[0]), float(value[1])


def as_size(value, name, dimensions):
    if isinstance(value, (int, float)):
        return tuple(float(value) for _ in range(dimensions))
    if not isinstance(value, (list, tuple)) or len(value) != dimensions:
        raise ValueError("{} must contain {} values".format(name, dimensions))
    return tuple(float(item) for item in value)


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def rotate_translate(point, position, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        position[0] + cosine * point[0] - sine * point[1],
        position[1] + sine * point[0] + cosine * point[1],
    )


def point32(x, y, z=0.0):
    result = Point32()
    result.x = x
    result.y = y
    result.z = z
    return result


def point(x, y, z=0.0):
    result = Point()
    result.x = x
    result.y = y
    result.z = z
    return result


class StationaryRoute:
    def __init__(self, position):
        self.position = position
        self.period = 0.0

    def state_at(self, _elapsed):
        return self.position, (0.0, 0.0)

    def cycle_points(self, _samples):
        return [self.position]


class PolylineRoute:
    def __init__(self, points, speed, closed=True, phase=0.0):
        if len(points) < 2:
            raise ValueError("waypoints route requires at least two points")
        if speed <= 0.0:
            raise ValueError("route speed must be greater than zero")

        route_points = list(points)
        if not closed:
            route_points.extend(reversed(route_points[1:-1]))

        self.points = route_points
        self.speed = speed
        self.phase = float(phase)
        self.segments = []
        self.cumulative_lengths = [0.0]

        for index, start in enumerate(self.points):
            end = self.points[(index + 1) % len(self.points)]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = math.hypot(dx, dy)
            if length <= EPSILON:
                continue
            self.segments.append((start, dx, dy, length))
            self.cumulative_lengths.append(self.cumulative_lengths[-1] + length)

        if not self.segments:
            raise ValueError("route waypoints must not all be identical")

        self.length = self.cumulative_lengths[-1]
        self.period = self.length / self.speed

    def state_at(self, elapsed):
        distance = ((elapsed / self.period) + self.phase) % 1.0 * self.length
        index = bisect.bisect_right(self.cumulative_lengths, distance) - 1
        index = min(index, len(self.segments) - 1)
        start, dx, dy, length = self.segments[index]
        ratio = (distance - self.cumulative_lengths[index]) / length
        position = (start[0] + ratio * dx, start[1] + ratio * dy)
        velocity = (self.speed * dx / length, self.speed * dy / length)
        return position, velocity

    def cycle_points(self, samples):
        return [
            self.state_at(self.period * index / float(samples))[0]
            for index in range(samples + 1)
        ]


class CircleRoute:
    def __init__(self, center, radius, speed, clockwise=False, phase=0.0):
        if radius <= 0.0:
            raise ValueError("circle route radius must be greater than zero")
        if speed <= 0.0:
            raise ValueError("route speed must be greater than zero")
        self.center = center
        self.radius = radius
        self.speed = speed
        self.direction = -1.0 if clockwise else 1.0
        self.phase = float(phase)
        self.period = 2.0 * math.pi * radius / speed

    def state_at(self, elapsed):
        angle = 2.0 * math.pi * self.phase
        angle += self.direction * self.speed * elapsed / self.radius
        position = (
            self.center[0] + self.radius * math.cos(angle),
            self.center[1] + self.radius * math.sin(angle),
        )
        velocity = (
            -self.direction * self.speed * math.sin(angle),
            self.direction * self.speed * math.cos(angle),
        )
        return position, velocity

    def cycle_points(self, samples):
        return [
            self.state_at(self.period * index / float(samples))[0]
            for index in range(samples + 1)
        ]


def build_route(config, name):
    route_type = str(config.get("type", "stationary")).lower()
    if route_type == "stationary":
        return StationaryRoute(as_xy(config.get("position", [0.0, 0.0]), name))

    speed = float(config.get("speed", 0.3))
    phase = float(config.get("phase", 0.0))
    if route_type == "square":
        center = as_xy(config.get("center", [0.0, 0.0]), name + ".center")
        width, height = as_size(config.get("size", [4.0, 4.0]), name + ".size", 2)
        if width <= 0.0 or height <= 0.0:
            raise ValueError(name + ".size values must be greater than zero")
        points = [
            (center[0] - width * 0.5, center[1] - height * 0.5),
            (center[0] + width * 0.5, center[1] - height * 0.5),
            (center[0] + width * 0.5, center[1] + height * 0.5),
            (center[0] - width * 0.5, center[1] + height * 0.5),
        ]
        if bool(config.get("clockwise", False)):
            points.reverse()
        return PolylineRoute(points, speed, closed=True, phase=phase)

    if route_type == "waypoints":
        raw_points = config.get("points", [])
        points = [
            as_xy(value, "{}.points[{}]".format(name, index))
            for index, value in enumerate(raw_points)
        ]
        return PolylineRoute(
            points, speed, closed=bool(config.get("closed", True)), phase=phase
        )

    if route_type == "circle":
        return CircleRoute(
            as_xy(config.get("center", [0.0, 0.0]), name + ".center"),
            float(config.get("radius", 2.0)),
            speed,
            clockwise=bool(config.get("clockwise", False)),
            phase=phase,
        )

    raise ValueError("unsupported route type '{}' in {}".format(route_type, name))


class DynamicObstacle:
    def __init__(self, config, index, velocity_variance):
        self.id = int(config.get("id", index + 1))
        self.name = str(config.get("name", "obstacle_{}".format(self.id)))
        self.shape = str(config.get("shape", "box")).lower()
        self.height = float(config.get("height", 0.6))
        self.radius = 0.0
        self.local_vertices = []
        self.marker_size = None
        self.velocity_variance = float(
            config.get("velocity_variance", velocity_variance)
        )
        self.yaw_mode = str(config.get("yaw_mode", "tangent")).lower()
        self.fixed_yaw = float(config.get("yaw", 0.0))
        self.color = tuple(
            float(value)
            for value in config.get("color", DEFAULT_COLORS[index % len(DEFAULT_COLORS)])
        )
        if len(self.color) != 4:
            raise ValueError("{}.color must contain [r, g, b, a]".format(self.name))
        if self.height <= 0.0:
            raise ValueError("{}.height must be greater than zero".format(self.name))
        if self.yaw_mode not in ("tangent", "fixed"):
            raise ValueError("{}.yaw_mode must be tangent or fixed".format(self.name))

        self._configure_shape(config)
        route_config = config.get("route", {})
        if not isinstance(route_config, dict):
            raise ValueError("{}.route must be a dictionary".format(self.name))
        if "position" in config and "position" not in route_config:
            route_config = dict(route_config)
            route_config["position"] = config["position"]
        self.route = build_route(route_config, self.name + ".route")

    def _configure_shape(self, config):
        if self.shape == "box":
            width, depth, height = as_size(
                config.get("size", [0.7, 0.7, self.height]),
                self.name + ".size",
                3,
            )
            if min(width, depth, height) <= 0.0:
                raise ValueError("{}.size values must be greater than zero".format(self.name))
            self.height = height
            self.marker_size = (width, depth, height)
            self.local_vertices = [
                (-width * 0.5, -depth * 0.5),
                (width * 0.5, -depth * 0.5),
                (width * 0.5, depth * 0.5),
                (-width * 0.5, depth * 0.5),
            ]
            return

        if self.shape in ("circle", "cylinder", "sphere"):
            self.radius = float(config.get("radius", 0.35))
            if self.radius <= 0.0:
                raise ValueError("{}.radius must be greater than zero".format(self.name))
            return

        if self.shape in ("polygon", "line"):
            raw_vertices = config.get("vertices", [])
            self.local_vertices = [
                as_xy(value, "{}.vertices[{}]".format(self.name, index))
                for index, value in enumerate(raw_vertices)
            ]
            required = 2 if self.shape == "line" else 3
            if len(self.local_vertices) < required:
                raise ValueError(
                    "{} requires at least {} vertices".format(self.name, required)
                )
            if self.shape == "line" and len(self.local_vertices) != 2:
                raise ValueError("{}.line requires exactly two vertices".format(self.name))
            return

        raise ValueError("unsupported shape '{}' in {}".format(self.shape, self.name))

    def state_at(self, elapsed):
        position, velocity = self.route.state_at(elapsed)
        if self.yaw_mode == "tangent" and math.hypot(*velocity) > EPSILON:
            yaw = math.atan2(velocity[1], velocity[0])
        else:
            yaw = self.fixed_yaw
        return position, velocity, yaw

    def build_message(self, elapsed, header):
        position, velocity, yaw = self.state_at(elapsed)
        obstacle = ObstacleMsg()
        obstacle.header = header
        obstacle.id = self.id
        obstacle.radius = self.radius

        if self.shape in ("circle", "cylinder", "sphere"):
            obstacle.polygon.points.append(point32(position[0], position[1]))
        else:
            for vertex in self.local_vertices:
                world_vertex = rotate_translate(vertex, position, yaw)
                obstacle.polygon.points.append(point32(world_vertex[0], world_vertex[1]))

        quaternion = quaternion_from_yaw(yaw)
        obstacle.orientation.x = quaternion[0]
        obstacle.orientation.y = quaternion[1]
        obstacle.orientation.z = quaternion[2]
        obstacle.orientation.w = quaternion[3]
        obstacle.velocities.twist.linear.x = velocity[0]
        obstacle.velocities.twist.linear.y = velocity[1]
        obstacle.velocities.covariance[0] = self.velocity_variance
        obstacle.velocities.covariance[7] = self.velocity_variance
        return obstacle, (position, velocity, yaw)


class DynamicObstaclePublisher:
    def __init__(self):
        self.output_topic = rospy.get_param(
            "~output_topic", "/move_base/TebLocalPlannerROS/obstacles"
        )
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/prediction_to_teb/markers"
        )
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.velocity_variance = float(rospy.get_param("~velocity_variance", 0.04))
        self.prediction_horizon = float(rospy.get_param("~prediction_horizon", 5.0))
        self.prediction_samples = int(rospy.get_param("~prediction_samples", 30))
        self.route_samples = int(rospy.get_param("~route_samples", 80))
        self.velocity_marker_seconds = float(
            rospy.get_param("~velocity_marker_seconds", 1.5)
        )

        if self.publish_rate <= 0.0:
            raise ValueError("~publish_rate must be greater than zero")
        if self.prediction_horizon < 0.0:
            raise ValueError("~prediction_horizon must not be negative")
        if self.prediction_samples < 1 or self.route_samples < 4:
            raise ValueError("~prediction_samples must be >= 1 and ~route_samples >= 4")

        configs = rospy.get_param("~obstacles", None)
        if configs is None:
            configs = [self.legacy_obstacle_config()]
            rospy.logwarn("No ~obstacles list found; using legacy single-obstacle parameters")
        if not isinstance(configs, list) or not configs:
            raise ValueError("~obstacles must be a non-empty list")

        self.obstacles = [
            DynamicObstacle(config, index, self.velocity_variance)
            for index, config in enumerate(configs)
        ]
        ids = [obstacle.id for obstacle in self.obstacles]
        if len(ids) != len(set(ids)):
            raise ValueError("obstacle IDs must be unique")

        self.publisher = rospy.Publisher(
            self.output_topic, ObstacleArrayMsg, queue_size=1
        )
        self.marker_publisher = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1
        )
        self.start_time = rospy.Time.now()
        rospy.on_shutdown(self.clear_obstacles)

        rospy.loginfo(
            "Publishing %d dynamic obstacles to %s in frame %s",
            len(self.obstacles),
            self.output_topic,
            self.frame_id,
        )
        for obstacle in self.obstacles:
            rospy.loginfo(
                "  id=%d name=%s shape=%s route_period=%.2fs",
                obstacle.id,
                obstacle.name,
                obstacle.shape,
                obstacle.route.period,
            )

    def legacy_obstacle_config(self):
        initial = (
            float(rospy.get_param("~initial_x", 2.0)),
            float(rospy.get_param("~initial_y", -1.5)),
        )
        velocity = (
            float(rospy.get_param("~velocity_x", 0.0)),
            float(rospy.get_param("~velocity_y", 0.3)),
        )
        speed = math.hypot(*velocity)
        reset_period = max(float(rospy.get_param("~reset_period", 10.0)), 1.0)
        if speed <= EPSILON:
            route = {"type": "stationary", "position": list(initial)}
        else:
            route = {
                "type": "waypoints",
                "closed": False,
                "speed": speed,
                "points": [
                    list(initial),
                    [
                        initial[0] + velocity[0] * reset_period,
                        initial[1] + velocity[1] * reset_period,
                    ],
                ],
            }
        return {
            "id": int(rospy.get_param("~obstacle_id", 1)),
            "shape": "circle",
            "radius": float(rospy.get_param("~radius", 0.3)),
            "route": route,
        }

    def elapsed_time(self, now):
        return max((now - self.start_time).to_sec(), 0.0)

    def build_message(self, now):
        elapsed = self.elapsed_time(now)
        message = ObstacleArrayMsg()
        message.header.stamp = now
        message.header.frame_id = self.frame_id
        states = []
        for obstacle in self.obstacles:
            obstacle_message, state = obstacle.build_message(elapsed, message.header)
            message.obstacles.append(obstacle_message)
            states.append(state)
        return message, states

    def configure_marker(self, marker, header, namespace, marker_id, marker_type):
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(2.5 / self.publish_rate)

    @staticmethod
    def set_color(marker, color):
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    def build_body_marker(self, obstacle, header, position, yaw):
        marker = Marker()
        namespace = "dynamic_obstacle_{}/body".format(obstacle.id)
        if obstacle.shape == "box":
            self.configure_marker(marker, header, namespace, 0, Marker.CUBE)
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = obstacle.height * 0.5
            marker.pose.orientation.z = math.sin(yaw * 0.5)
            marker.pose.orientation.w = math.cos(yaw * 0.5)
            marker.scale.x, marker.scale.y, marker.scale.z = obstacle.marker_size
        elif obstacle.shape in ("circle", "cylinder", "sphere"):
            marker_type = Marker.SPHERE if obstacle.shape == "sphere" else Marker.CYLINDER
            self.configure_marker(marker, header, namespace, 0, marker_type)
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = obstacle.height * 0.5
            marker.scale.x = obstacle.radius * 2.0
            marker.scale.y = obstacle.radius * 2.0
            marker.scale.z = obstacle.height
        elif obstacle.shape == "polygon":
            self.configure_marker(marker, header, namespace, 0, Marker.TRIANGLE_LIST)
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.orientation.z = math.sin(yaw * 0.5)
            marker.pose.orientation.w = math.cos(yaw * 0.5)
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.points = self.prism_triangles(obstacle.local_vertices, obstacle.height)
        else:
            self.configure_marker(marker, header, namespace, 0, Marker.LINE_STRIP)
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = obstacle.height * 0.5
            marker.pose.orientation.z = math.sin(yaw * 0.5)
            marker.pose.orientation.w = math.cos(yaw * 0.5)
            marker.scale.x = 0.08
            marker.points = [point(vertex[0], vertex[1]) for vertex in obstacle.local_vertices]

        self.set_color(marker, obstacle.color)
        return marker

    @staticmethod
    def prism_triangles(vertices, height):
        points = []
        for index in range(1, len(vertices) - 1):
            points.extend(
                [
                    point(vertices[0][0], vertices[0][1], height),
                    point(vertices[index][0], vertices[index][1], height),
                    point(vertices[index + 1][0], vertices[index + 1][1], height),
                    point(vertices[0][0], vertices[0][1], 0.0),
                    point(vertices[index + 1][0], vertices[index + 1][1], 0.0),
                    point(vertices[index][0], vertices[index][1], 0.0),
                ]
            )
        for index, start in enumerate(vertices):
            end = vertices[(index + 1) % len(vertices)]
            points.extend(
                [
                    point(start[0], start[1], 0.0),
                    point(end[0], end[1], 0.0),
                    point(end[0], end[1], height),
                    point(start[0], start[1], 0.0),
                    point(end[0], end[1], height),
                    point(start[0], start[1], height),
                ]
            )
        return points

    def build_markers(self, obstacle_message, states):
        elapsed = self.elapsed_time(obstacle_message.header.stamp)
        markers = MarkerArray()
        for obstacle, state in zip(self.obstacles, states):
            position, velocity, yaw = state
            markers.markers.append(
                self.build_body_marker(
                    obstacle, obstacle_message.header, position, yaw
                )
            )

            arrow = Marker()
            self.configure_marker(
                arrow,
                obstacle_message.header,
                "dynamic_obstacle_{}/velocity".format(obstacle.id),
                0,
                Marker.ARROW,
            )
            arrow.scale.x = 0.06
            arrow.scale.y = 0.14
            arrow.scale.z = 0.18
            arrow.color.r = 1.0
            arrow.color.g = 1.0
            arrow.color.a = 1.0
            arrow.points = [
                point(position[0], position[1], obstacle.height + 0.08),
                point(
                    position[0] + velocity[0] * self.velocity_marker_seconds,
                    position[1] + velocity[1] * self.velocity_marker_seconds,
                    obstacle.height + 0.08,
                ),
            ]
            markers.markers.append(arrow)

            prediction = Marker()
            self.configure_marker(
                prediction,
                obstacle_message.header,
                "dynamic_obstacle_{}/prediction".format(obstacle.id),
                0,
                Marker.LINE_STRIP,
            )
            prediction.scale.x = 0.07
            prediction.color.r = 1.0
            prediction.color.g = 0.85
            prediction.color.a = 0.95
            prediction.points = [
                point(*obstacle.route.state_at(
                    elapsed + self.prediction_horizon * index / self.prediction_samples
                )[0], z=0.06)
                for index in range(self.prediction_samples + 1)
            ]
            markers.markers.append(prediction)

            route = Marker()
            self.configure_marker(
                route,
                obstacle_message.header,
                "dynamic_obstacle_{}/route".format(obstacle.id),
                0,
                Marker.LINE_STRIP,
            )
            route.scale.x = 0.035
            route.color.r = obstacle.color[0]
            route.color.g = obstacle.color[1]
            route.color.b = obstacle.color[2]
            route.color.a = 0.45
            route.points = [
                point(route_point[0], route_point[1], 0.025)
                for route_point in obstacle.route.cycle_points(self.route_samples)
            ]
            markers.markers.append(route)

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
            message, states = self.build_message(rospy.Time.now())
            self.publisher.publish(message)
            self.marker_publisher.publish(self.build_markers(message, states))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("publish_dynamic_obstacle")
    try:
        DynamicObstaclePublisher().run()
    except (rospy.ROSInterruptException, ValueError, TypeError) as error:
        rospy.logerr("%s", error)
