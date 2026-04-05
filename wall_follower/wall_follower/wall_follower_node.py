#!/usr/bin/env python3
"""Purely reactive left wall-following node using LiDAR for the Andino robot.

Subscribes to /scan (LaserScan) and publishes /cmd_vel (Twist).
Uses a PD controller to maintain a desired distance from the left wall.

Architecture: fully reactive — no state machine, no memory beyond PD
derivative term (prev_error, prev_time). Every scan produces a command
from scratch based only on current sensor readings.

Decision priority (per scan):
  1. Front obstacle  → stop + turn right
  2. Inside circle   → centroid-based centering / stop
  3. No wall visible → fixed linear + angular (search)
  4. Wall visible    → PD wall following

LiDAR is mounted rotated pi from base_link:
  scan 0      = robot BACK
  scan +-pi   = robot FRONT
  scan -pi/2  = robot LEFT
  scan +pi/2  = robot RIGHT
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('desired_distance', 1.0)
        self.declare_parameter('base_speed', 0.5)
        self.declare_parameter('forward_speed', 1.0)
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('kd', 0.4)
        self.declare_parameter('kv', 0.3)
        self.declare_parameter('front_obstacle_dist', 0.8)
        self.declare_parameter('max_wall_search_dist', 3.0)
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('inside_ratio_threshold', 0.6)
        self.declare_parameter('center_tolerance', 0.5)
        self.declare_parameter('centering_speed', 0.15)
        self.declare_parameter('search_linear_speed', 0.3)
        self.declare_parameter('search_angular_speed', 0.5)

        # Read parameters
        self.desired_dist     = self.get_parameter('desired_distance').value
        self.base_speed       = self.get_parameter('base_speed').value
        self.forward_speed    = self.get_parameter('forward_speed').value
        self.kp               = self.get_parameter('kp').value
        self.kd               = self.get_parameter('kd').value
        self.kv               = self.get_parameter('kv').value
        self.front_obs_dist   = self.get_parameter('front_obstacle_dist').value
        self.max_search_dist  = self.get_parameter('max_wall_search_dist').value
        self.inside_ratio     = self.get_parameter('inside_ratio_threshold').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        self.centering_speed  = self.get_parameter('centering_speed').value
        self.search_lin_spd   = self.get_parameter('search_linear_speed').value
        self.search_ang_spd   = self.get_parameter('search_angular_speed').value

        scan_topic = self.get_parameter('scan_topic').value
        cmd_topic  = self.get_parameter('cmd_vel_topic').value

        # ── Only PD memory (allowed by reactive architecture) ─────────
        self.prev_error = 0.0
        self.prev_time  = None

        # ── Publishers & Subscribers ──────────────────────────────────
        self.cmd_pub  = self.create_publisher(Twist, cmd_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10)

        self.get_logger().info(
            f'Wall follower (reactive) started: desired_dist={self.desired_dist}m, '
            f'speed={self.forward_speed}m/s, kp={self.kp}, kd={self.kd}')

    # ── Helpers (all per-scan, no state) ──────────────────────────────

    @staticmethod
    def _is_valid(r, msg):
        return not (math.isinf(r) or math.isnan(r)
                    or r < msg.range_min or r > msg.range_max)

    def _get_min_range_sector(self, msg, angle_start, angle_end):
        """Minimum valid range in [angle_start, angle_end] (radians)."""
        idx_s = int((angle_start - msg.angle_min) / msg.angle_increment)
        idx_e = int((angle_end   - msg.angle_min) / msg.angle_increment)
        idx_s = max(0, min(idx_s, len(msg.ranges) - 1))
        idx_e = max(0, min(idx_e, len(msg.ranges) - 1))
        if idx_s > idx_e:
            idx_s, idx_e = idx_e, idx_s
        vals = [msg.ranges[i] for i in range(idx_s, idx_e + 1)
                if self._is_valid(msg.ranges[i], msg)]
        return min(vals) if vals else float('inf')

    def _valid_reading_ratio(self, msg):
        """Fraction of readings that are valid (not inf/nan)."""
        valid = sum(1 for r in msg.ranges if self._is_valid(r, msg))
        return valid / len(msg.ranges) if msg.ranges else 0.0

    def _count_inf_sector(self, msg, angle_start, angle_end):
        """Count inf/out-of-range readings in angular sector [angle_start, angle_end]."""
        idx_s = int((angle_start - msg.angle_min) / msg.angle_increment)
        idx_e = int((angle_end   - msg.angle_min) / msg.angle_increment)
        idx_s = max(0, min(idx_s, len(msg.ranges) - 1))
        idx_e = max(0, min(idx_e, len(msg.ranges) - 1))
        if idx_s > idx_e:
            idx_s, idx_e = idx_e, idx_s
        return sum(
            1 for i in range(idx_s, idx_e + 1)
            if not self._is_valid(msg.ranges[i], msg))

    def _estimate_circle_center(self, msg):
        """Centroid of valid wall hits in robot frame. Returns (cx, cy, count)."""
        sx, sy, n = 0.0, 0.0, 0
        for i, r in enumerate(msg.ranges):
            if not self._is_valid(r, msg):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            # LiDAR rotated pi: negate to get base_link frame
            sx += -r * math.cos(angle)
            sy += -r * math.sin(angle)
            n += 1
        if n == 0:
            return 0.0, 0.0, 0
        return sx / n, sy / n, n

    # ── Main callback ─────────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now()
        cmd = Twist()

        # ── Measure key sectors ───────────────────────────────────────
        # Left wall: -120 to -60 deg (centered on -90 = robot left)
        left_dist = self._get_min_range_sector(
            msg, math.radians(-120), math.radians(-60))
        # Left-forward: -160 to -120 deg
        left_fwd_dist = self._get_min_range_sector(
            msg, math.radians(-160), math.radians(-120))
        # Front: near +-180 deg (robot forward)
        # Narrowed the field of view from 135 to 160 to avoid false positives at the entrance
        front_left  = self._get_min_range_sector(
            msg, math.radians(160), math.radians(179))
        front_right = self._get_min_range_sector(
            msg, math.radians(-179), math.radians(-160))
        front_dist = min(front_left, front_right)

        wall_dist = min(left_dist, left_fwd_dist * 1.1)
        valid_ratio = self._valid_reading_ratio(msg)

        # ── Priority 1: Front obstacle → turn right ──────────────────
        if front_dist < self.front_obs_dist:
            cmd.linear.x = 0.0
            cmd.angular.z = -1.0
            self.prev_error = 0.0
            self.get_logger().info(
                f'OBSTACLE front={front_dist:.2f}m — turning right',
                throttle_duration_sec=0.5)

        # ── Priority 2: Inside circle → centering & align to gap ──────
        elif valid_ratio > self.inside_ratio:
            # Measure 4 quadrant sectors
            q_front = min(
                self._get_min_range_sector(msg, math.radians(135), math.radians(179)),
                self._get_min_range_sector(msg, math.radians(-179), math.radians(-135)))
            q_left  = self._get_min_range_sector(msg, math.radians(-120), math.radians(-60))
            q_right = self._get_min_range_sector(msg, math.radians(60),   math.radians(120))
            q_back  = self._get_min_range_sector(msg, math.radians(-30),  math.radians(30))

            # Dist to circle center is rotation-invariant
            cx, cy, n_pts = self._estimate_circle_center(msg)
            dist_to_center = math.hypot(cx, cy) if n_pts > 0 else float('inf')

            # Count infs to see if gap is directly ahead
            left_inf  = self._count_inf_sector(msg, math.radians(-180), math.radians(-90))
            right_inf = self._count_inf_sector(msg, math.radians(90),   math.radians(180))
            total_inf = left_inf + right_inf
            margin    = max(2, int(0.3 * total_inf))  # less strict margin (30%)
            min_gap_infs = 3

            # Rotate until the immediate front sector is infinite, then check loose balance
            is_front_inf = math.isinf(front_dist)
            is_aligned_with_gap = is_front_inf and (total_inf >= min_gap_infs) and (abs(left_inf - right_inf) <= margin)

            # If cx < -0.15, the circle's centroid is behind the robot in its own frame.
            # Geometrically, if we are inside the circle but facing away from its center,
            # we MUST be facing an outward boundary (acting as a reactive "Exiting" lock!)
            is_facing_away_from_center = (cx < -0.15)
            
            # As the robot spins in place, polygonal noise causes dist_to_center to 
            # artificially fluctuate (e.g. from 0.50m to 0.53m). To prevent oscillating 
            # out of the align phase without using memory, we check if it is actively 
            # spinning around the center (cx has started dropping below the tolerance).
            is_spinning_at_center = (dist_to_center <= self.center_tolerance + 0.2) and (cx <= self.center_tolerance)

            # We trigger the align/exit phase if we are near the center of the room,
            # OR if we are actively exiting (facing away from the center).
            if dist_to_center <= self.center_tolerance or is_spinning_at_center or is_facing_away_from_center:
                if is_aligned_with_gap:
                    # DRIVE OUT
                    cmd.linear.x = self.centering_speed
                    cmd.angular.z = 0.0
                    self.get_logger().info('EXITING GAP', throttle_duration_sec=0.5)
                else:
                    # ALIGN (rotate towards gap)
                    cmd.linear.x = 0.0  # Explicitly hold linear velocity at 0 until aligned
                    if right_inf > left_inf:
                        cmd.angular.z = -0.4
                    else:
                        cmd.angular.z = 0.4
                    self.get_logger().info('ALIGNING TO GAP', throttle_duration_sec=0.5)
            else:
                # CENTERING (we are not centered, move away from closest wall)
                directions = {'front': q_front, 'left': q_left, 'right': q_right, 'back': q_back}
                valid_dirs = {k: v for k, v in directions.items() if not math.isinf(v)}
                
                if valid_dirs:
                    shortest = min(valid_dirs, key=lambda k: valid_dirs[k])
                    
                    slowdown_radius = 0.5
                    if dist_to_center < slowdown_radius:
                        speed = self.centering_speed * (dist_to_center / slowdown_radius)
                        speed = max(speed, 0.03)
                    else:
                        speed = self.centering_speed

                    if shortest == 'front':
                        cmd.linear.x = -speed
                    elif shortest == 'back':
                        cmd.linear.x = speed
                    elif shortest == 'left':
                        cmd.angular.z = -self.centering_speed
                        cmd.linear.x = speed * 0.3
                    elif shortest == 'right':
                        cmd.angular.z = self.centering_speed
                        cmd.linear.x = speed * 0.3
                        
                self.get_logger().info(f'CENTERING (dist={dist_to_center:.2f}m)', throttle_duration_sec=0.5)

            self.prev_error = 0.0

        # ── Priority 3: No wall → fixed search velocity ──────────────
        elif wall_dist > self.max_search_dist:
            cmd.linear.x = self.search_lin_spd
            cmd.angular.z = self.search_ang_spd
            self.prev_error = 0.0
            self.get_logger().info(
                f'SEARCH (no wall) lin={self.search_lin_spd:.2f} '
                f'ang={self.search_ang_spd:.2f}',
                throttle_duration_sec=1.0)

        # ── Priority 4: PD wall following ─────────────────────────────
        else:
            error = wall_dist - self.desired_dist

            dt = 0.1
            if self.prev_time is not None:
                dt = max((now - self.prev_time).nanoseconds * 1e-9, 0.01)
            d_error = (error - self.prev_error) / dt

            self.prev_error = error

            # PD controller: omega = Kp * e + Kd * de/dt
            angular_z = self.kp * error + self.kd * d_error
            angular_z = max(-1.0, min(1.0, angular_z))

            # Speed: v = v_base - Kv * |e|  (from T02c slide 55)
            speed = self.forward_speed - self.kv * abs(error)
            speed = max(self.base_speed, speed)

            # Slow down near front obstacles
            if front_dist < 1.5:
                front_scale = max(0.1,
                    (front_dist - self.front_obs_dist) /
                    (1.5 - self.front_obs_dist))
                speed *= front_scale
                angular_z = min(angular_z, -0.3)

            cmd.linear.x = speed
            cmd.angular.z = angular_z
            self.get_logger().info(
                f'FOLLOW left={wall_dist:.2f}m err={error:+.2f} '
                f'ang={angular_z:+.2f} spd={speed:.2f}',
                throttle_duration_sec=1.0)

        self.prev_time = now
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
