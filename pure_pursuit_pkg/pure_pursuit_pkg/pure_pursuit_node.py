#!/usr/bin/env python3
"""
Pure pursuit waypoint follower.

On start-up the waypoint CSV is cleaned, resampled, smoothed and given a speed
profile (see pure_pursuit.py). On every pose message:
  1. nearest waypoint (windowed search ahead of the previous one),
  2. goal point at the lookahead distance L = lookahead_distance
     + lookahead_speed_gain * speed, found by circle / path intersection,
  3. goal point transformed into the vehicle frame,
  4. curvature gamma = 2 y / L^2, steering = atan(wheelbase * gamma),
     clipped to the steering limit,
  5. speed from the precomputed profile at the nearest waypoint,
  6. AckermannDriveStamped on /drive, waypoints and goal point as RViz
     markers.

Pose comes from /ego_racecar/odom in the simulator and from the particle
filter (/pf/pose/odom) on the car; set `odom_topic` accordingly.
"""

import os

from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

from pure_pursuit_pkg import pure_pursuit as pp


class PurePursuit(Node):
    """Implement Pure Pursuit on the car."""

    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Topics
        odom_topic = self.declare_parameter(
            'odom_topic', '/ego_racecar/odom').value
        drive_topic = self.declare_parameter('drive_topic', '/drive').value
        self.frame_id = self.declare_parameter('frame_id', 'map').value

        # Waypoints
        waypoints_path = self.declare_parameter('waypoints_path', '').value
        spacing = self.declare_parameter('waypoint_spacing', 0.05).value
        smoothing_window = self.declare_parameter(
            'smoothing_window', 9).value
        self.search_window = self.declare_parameter(
            'search_window', 100).value

        # Lookahead and steering
        self.lookahead_distance = self.declare_parameter(
            'lookahead_distance', 0.9).value
        self.lookahead_speed_gain = self.declare_parameter(
            'lookahead_speed_gain', 0.05).value
        self.wheelbase = self.declare_parameter('wheelbase', 0.3302).value
        self.max_steering = self.declare_parameter(
            'max_steering', 0.4189).value

        # Speed profile
        max_speed = self.declare_parameter('max_speed', 5.0).value
        min_speed = self.declare_parameter('min_speed', 1.5).value
        max_lateral_accel = self.declare_parameter(
            'max_lateral_accel', 5.0).value
        max_decel = self.declare_parameter('max_decel', 5.0).value
        self.speed_scale = self.declare_parameter('speed_scale', 1.0).value

        if not waypoints_path:
            waypoints_path = os.path.join(
                get_package_share_directory('pure_pursuit_pkg'),
                'waypoints', 'data.csv')
        raw = pp.load_waypoints(waypoints_path)
        self.path = pp.prepare_path(raw, spacing, smoothing_window)
        self.speeds = pp.speed_profile(
            self.path, max_speed, min_speed, max_lateral_accel, max_decel)
        self.get_logger().info(
            'Loaded %d waypoints from %s (%d after resampling), '
            'speed %.2f-%.2f m/s'
            % (len(raw), waypoints_path, len(self.path),
               self.speeds.min(), self.speeds.max()))

        self.current_idx = None
        self.current_speed = 0.0

        # Subs, Pubs
        self.pub_drive = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10)
        self.pub_env_viz = self.create_publisher(Marker, '/env_viz', 10)
        self.pub_dynamic_viz = self.create_publisher(
            Marker, '/dynamic_viz', 10)
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.pose_callback, 10)

        self.path_marker = self.make_marker(0, 0.1, (0.0, 0.0, 1.0))
        self.path_marker.points = [
            Point(x=float(x), y=float(y), z=0.0) for x, y in self.path]
        self.create_timer(1.0, self.publish_path)

    def pose_callback(self, odometry_info):
        pose = odometry_info.pose.pose
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        yaw = pp.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.current_speed = odometry_info.twist.twist.linear.x

        # Current waypoint: closest one, searched ahead of the last one.
        self.current_idx = pp.nearest_index(
            self.path, x, y, self.current_idx, self.search_window)

        # Goal point on the path at the lookahead distance.
        lookahead = (self.lookahead_distance
                     + self.lookahead_speed_gain * abs(self.current_speed))
        goal, _ = pp.lookahead_point(
            self.path, x, y, self.current_idx, lookahead)

        # Goal point in the vehicle frame, then curvature -> steering.
        local_x, local_y = pp.to_vehicle_frame(goal, x, y, yaw)
        angle = pp.steering_angle(
            local_x, local_y, self.wheelbase, self.max_steering)
        speed = self.speed_scale * float(self.speeds[self.current_idx])

        self.publish_drive(angle, speed)
        self.publish_goal(goal)

    def publish_drive(self, angle, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(angle)
        msg.drive.speed = float(speed)
        self.pub_drive.publish(msg)

    def make_marker(self, marker_id, scale, rgb):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = rgb
        return marker

    def publish_path(self):
        self.path_marker.header.stamp = self.get_clock().now().to_msg()
        self.pub_env_viz.publish(self.path_marker)

    def publish_goal(self, goal):
        marker = self.make_marker(1, 0.2, (1.0, 0.0, 0.0))
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.points.append(
            Point(x=float(goal[0]), y=float(goal[1]), z=0.0))
        self.pub_dynamic_viz.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    print('PurePursuit Initialized')
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
