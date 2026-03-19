#!/usr/bin/env python3
"""Left wall-following node using LiDAR for the Andino robot.

Subscribes to /scan (LaserScan) and publishes /cmd_vel (Twist).
Uses a PD controller to maintain a desired distance from the left wall.
Stops when inside the circle (>60% of LiDAR readings are valid/not inf).
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Declare parameters
        self.declare_parameter('desired_distance', 1.5)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('kd', 0.4)
        self.declare_parameter('front_obstacle_dist', 0.8)
        self.declare_parameter('front_slowdown_dist', 1.5)
        self.declare_parameter('max_wall_search_dist', 3.0)
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('inside_ratio_threshold', 0.6)
        self.declare_parameter('stop_confirmation_count', 15)

        # Read parameters
        self.desired_dist = self.get_parameter('desired_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.front_obs_dist = self.get_parameter('front_obstacle_dist').value
        self.front_slow_dist = self.get_parameter('front_slowdown_dist').value
        self.max_search_dist = self.get_parameter('max_wall_search_dist').value
        self.inside_ratio = self.get_parameter('inside_ratio_threshold').value
        self.stop_confirm_needed = self.get_parameter('stop_confirmation_count').value

        scan_topic = self.get_parameter('scan_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value

        # State
        self.prev_error = 0.0
        self.prev_time = None
        self.is_stopped = False
        self.stop_confirm_count = 0

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        self.get_logger().info(
            f'Wall follower started: desired_dist={self.desired_dist}m, '
            f'speed={self.forward_speed}m/s, kp={self.kp}, kd={self.kd}'
        )

    def _get_min_range_sector(self, msg: LaserScan, angle_start: float, angle_end: float) -> float:
        """Get minimum valid range in angular sector [angle_start, angle_end] in radians."""
        idx_start = int((angle_start - msg.angle_min) / msg.angle_increment)
        idx_end = int((angle_end - msg.angle_min) / msg.angle_increment)
        idx_start = max(0, min(idx_start, len(msg.ranges) - 1))
        idx_end = max(0, min(idx_end, len(msg.ranges) - 1))
        if idx_start > idx_end:
            idx_start, idx_end = idx_end, idx_start

        valid_ranges = []
        for i in range(idx_start, idx_end + 1):
            r = msg.ranges[i]
            if not (math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max):
                valid_ranges.append(r)

        return min(valid_ranges) if valid_ranges else float('inf')

    def _valid_reading_ratio(self, msg: LaserScan) -> float:
        """Return the fraction of LiDAR readings that are valid (not inf/nan)."""
        valid = 0
        for r in msg.ranges:
            if not (math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max):
                valid += 1
        return valid / len(msg.ranges) if msg.ranges else 0.0

    def scan_callback(self, msg: LaserScan):
        if self.is_stopped:
            self.cmd_pub.publish(Twist())
            return

        now = self.get_clock().now()
        cmd = Twist()

        # ── Check if inside the circle ──────────────────────────────
        # When >60% of lidar readings see a wall, we're inside → stop.
        ratio = self._valid_reading_ratio(msg)
        if ratio >= self.inside_ratio:
            self.stop_confirm_count += 1
            self.get_logger().info(
                f'INSIDE CHECK {self.stop_confirm_count}/{self.stop_confirm_needed} '
                f'valid={ratio:.0%}', throttle_duration_sec=0.5)
            if self.stop_confirm_count >= self.stop_confirm_needed:
                self.is_stopped = True
                self.get_logger().info(
                    f'STOPPED: Inside the circle ({ratio:.0%} valid readings).')
                self.cmd_pub.publish(Twist())
                return
        else:
            self.stop_confirm_count = 0

        # ── Measure key sectors ─────────────────────────────────────
        # NOTE: LiDAR is mounted rotated 180° (π) in the URDF.
        #   Scan angle   0   = robot BACK
        #   Scan angle  ±π   = robot FRONT
        #   Scan angle  -π/2 = robot LEFT
        #   Scan angle  +π/2 = robot RIGHT

        # Left wall: -120° to -60° (centered on -90° = robot's left)
        left_dist = self._get_min_range_sector(msg, math.radians(-120), math.radians(-60))
        # Slightly left-forward: -160° to -120° (anticipate curves)
        left_fwd_dist = self._get_min_range_sector(msg, math.radians(-160), math.radians(-120))
        # Front: two sectors near ±180° (robot forward)
        front_left = self._get_min_range_sector(msg, math.radians(135), math.radians(179))
        front_right = self._get_min_range_sector(msg, math.radians(-179), math.radians(-135))
        front_dist = min(front_left, front_right)

        # Best estimate of left wall distance
        wall_dist = min(left_dist, left_fwd_dist * 1.1)

        # ── State machine ───────────────────────────────────────────
        if front_dist < self.front_obs_dist:
            # FRONT OBSTACLE: stop and turn right sharply
            cmd.linear.x = 0.0
            cmd.angular.z = -1.0
            self.prev_error = 0.0
            self.get_logger().info(
                f'FRONT OBSTACLE {front_dist:.2f}m — turning right', throttle_duration_sec=0.5)

        elif wall_dist > self.max_search_dist:
            # NO WALL FOUND: drive forward and gently turn left to find one
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.3  # positive = turn left in ROS
            self.prev_error = 0.0
            self.get_logger().info(
                f'SEARCHING for left wall (left={left_dist:.2f}m)', throttle_duration_sec=1.0)

        else:
            # WALL FOLLOWING with PD control
            # error > 0 means too far from wall → turn left (positive angular.z)
            # error < 0 means too close → turn right (negative angular.z)
            error = wall_dist - self.desired_dist

            dt = 0.1
            if self.prev_time is not None:
                dt = max((now - self.prev_time).nanoseconds * 1e-9, 0.01)
            d_error = (error - self.prev_error) / dt

            self.prev_error = error

            angular_z = self.kp * error + self.kd * d_error
            angular_z = max(-1.0, min(1.0, angular_z))  # clamp

            # Slow down when turning hard or approaching front wall
            speed_scale = max(0.3, 1.0 - abs(angular_z) * 0.5)
            if front_dist < self.front_slow_dist:
                # Progressive slowdown as we approach a front wall
                front_scale = max(0.1, (front_dist - self.front_obs_dist) / (self.front_slow_dist - self.front_obs_dist))
                speed_scale *= front_scale
                # Also start turning right as we approach
                angular_z = min(angular_z, -0.3)
            cmd.linear.x = self.forward_speed * speed_scale
            cmd.angular.z = angular_z

            self.get_logger().info(
                f'FOLLOWING left={wall_dist:.2f}m err={error:+.2f} '
                f'ang={angular_z:+.2f} spd={cmd.linear.x:.2f}',
                throttle_duration_sec=1.0)

        self.prev_time = now
        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)
    node = WallFollowerNode()
    
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    
    finally:
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
