#!/usr/bin/env python3
"""Left wall-following node using LiDAR for the Andino robot.

Subscribes to /scan (LaserScan) and publishes /cmd_vel (Twist).
Uses a PD controller to maintain a desired distance from the left wall.

State machine:
  1. WALL_FOLLOWING  – PD left-wall following until inside the circle
  2. CENTERING       – move away from closest wall until centered
  3. ALIGNING        – rotate until exit gap is directly ahead
  4. DONE            – stopped, aligned with exit
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist



"""
DevNotes: I chose to move away from shortest distance instead of toward
longest distance, because that would cause problems with inf
"""

# ── Robot states ────────────────────────────────────────────────────
STATE_WALL_FOLLOWING = 'WALL_FOLLOWING'
STATE_CENTERING = 'CENTERING'
STATE_ALIGNING = 'ALIGNING'
STATE_DONE = 'DONE'


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Declare parameters — wall following
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

        # Declare parameters — centering & alignment
        self.declare_parameter('center_tolerance', 0.1)
        self.declare_parameter('centering_linear_speed', 0.18)
        self.declare_parameter('centering_angular_speed', 0.12)
        self.declare_parameter('align_angular_speed', 0.6)

        # Read parameters — wall following
        self.desired_dist = self.get_parameter('desired_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.front_obs_dist = self.get_parameter('front_obstacle_dist').value
        self.front_slow_dist = self.get_parameter('front_slowdown_dist').value
        self.max_search_dist = self.get_parameter('max_wall_search_dist').value
        self.inside_ratio = self.get_parameter('inside_ratio_threshold').value
        self.stop_confirm_needed = self.get_parameter('stop_confirmation_count').value

        # Read parameters — centering & alignment
        self.center_tolerance = self.get_parameter('center_tolerance').value
        self.centering_lin_spd = self.get_parameter('centering_linear_speed').value
        self.centering_ang_spd = self.get_parameter('centering_angular_speed').value
        self.align_ang_spd = self.get_parameter('align_angular_speed').value

        scan_topic = self.get_parameter('scan_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value

        # State
        self.state = STATE_WALL_FOLLOWING
        self.prev_error = 0.0
        self.prev_time = None
        self.stop_confirm_count = 0
        self.align_cumulative_yaw = 0.0   # track total rotation during alignment
        self.align_prev_time = None
        self.align_rotating = True        # True = rotating, False = driving out
        self.align_done_180 = False       # True once first 180° spin is complete
        self.has_been_outside = False     # must see open space before CENTERING allowed
        self.prev_centering_cmd = Twist()  # EMA smoothing state for centering

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        self.get_logger().info(
            f'Wall follower started: desired_dist={self.desired_dist}m, '
            f'speed={self.forward_speed}m/s, kp={self.kp}, kd={self.kd}'
        )

    # ── Helpers ──────────────────────────────────────────────────────

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

    def _count_inf_sector(self, msg: LaserScan, angle_start: float, angle_end: float) -> int:
        """Count inf/out-of-range readings in angular sector."""
        idx_start = int((angle_start - msg.angle_min) / msg.angle_increment)
        idx_end   = int((angle_end   - msg.angle_min) / msg.angle_increment)
        idx_start = max(0, min(idx_start, len(msg.ranges) - 1))
        idx_end   = max(0, min(idx_end,   len(msg.ranges) - 1))
        if idx_start > idx_end:
            idx_start, idx_end = idx_end, idx_start
        return sum(
            1 for i in range(idx_start, idx_end + 1)
            if math.isinf(msg.ranges[i]) or math.isnan(msg.ranges[i])
               or msg.ranges[i] > msg.range_max
        )

    def _estimate_circle_center(self, msg: LaserScan):
        """Estimate circle center from valid lidar wall hits.

        Returns (cx, cy) in robot base_link frame and the count of points used.
        The centroid of points on a circular wall approximates the circle center.
        """
        sum_x = 0.0
        sum_y = 0.0
        count = 0
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue
            scan_angle = msg.angle_min + i * msg.angle_increment
            # LiDAR is mounted rotated π from base_link
            sum_x += -r * math.cos(scan_angle)
            sum_y += -r * math.sin(scan_angle)
            count += 1
        if count == 0:
            return 0.0, 0.0, 0
        return sum_x / count, sum_y / count, count

    # ── Center & Align (Feature 2) ──────────────────────────────────

    def _center_and_align(self, msg: LaserScan) -> Twist:
        """Handle CENTERING and ALIGNING phases. Returns the Twist command.

        CENTERING: direction from old logic (move away from closest sector),
                   speed from new logic (ramp down by dist_to_circle_center),
                   EMA smoothing on output to suppress jitter.
        ALIGNING:  rotate in place until front_left and front_right both read inf
                   (exit gap is ahead). Error after 2 full rotations.
        """
        cmd = Twist()

        if self.state == STATE_CENTERING:
            # ── Measure 4 sectors (used for both exit check and direction) ──
            front_dist = min(
                self._get_min_range_sector(msg, math.radians(135), math.radians(179)),
                self._get_min_range_sector(msg, math.radians(-179), math.radians(-135))
            )
            left_dist  = self._get_min_range_sector(msg, math.radians(-120), math.radians(-60))
            right_dist = self._get_min_range_sector(msg, math.radians(60),   math.radians(120))
            back_dist  = self._get_min_range_sector(msg, math.radians(-30),  math.radians(30))

            valid_dists = [v for v in [front_dist, left_dist, right_dist, back_dist]
                           if not math.isinf(v)]

            # ── Centered check: max spread between valid sectors ────
            if valid_dists and (max(valid_dists) - min(valid_dists)) <= self.center_tolerance:
                self.state = STATE_ALIGNING
                self.align_cumulative_yaw = 0.0
                self.align_prev_time = None
                self.align_rotating = True
                self.align_done_180 = False
                self.prev_centering_cmd = Twist()
                self.get_logger().info(
                    f'CENTERED (spread={max(valid_dists) - min(valid_dists):.3f}m <= '
                    f'{self.center_tolerance}m). Entering ALIGNING phase.')
                return cmd

            # ── Speed: centroid distance ramps down near center ─────
            cx, cy, n_pts = self._estimate_circle_center(msg)
            dist_to_center = math.hypot(cx, cy) if n_pts > 0 else float('inf')

            slowdown_radius = 0.5
            if dist_to_center < slowdown_radius:
                speed = self.centering_lin_spd * (dist_to_center / slowdown_radius)
                speed = max(speed, 0.03)
            else:
                speed = self.centering_lin_spd

            # ── Direction: move away from closest sector ─────────────
            directions = {'front': front_dist, 'left': left_dist,
                          'right': right_dist, 'back': back_dist}
            valid_dirs = {k: v for k, v in directions.items() if not math.isinf(v)}

            if not valid_dirs:
                self.get_logger().warn('CENTERING: all lidar directions read inf, stopping.')
                return cmd

            shortest_dir = min(valid_dirs, key=lambda k: valid_dirs[k])

            raw = Twist()
            if shortest_dir == 'front':
                raw.linear.x = -speed
            elif shortest_dir == 'back':
                raw.linear.x = speed
            elif shortest_dir == 'left':
                raw.angular.z = -self.centering_ang_spd
                raw.linear.x  = speed * 0.3
            elif shortest_dir == 'right':
                raw.angular.z = self.centering_ang_spd
                raw.linear.x  = speed * 0.3

            # ── EMA smoothing (α=0.35) to suppress jitter ─────────
            alpha = 0.35
            cmd.linear.x  = alpha * raw.linear.x  + (1 - alpha) * self.prev_centering_cmd.linear.x
            cmd.angular.z = alpha * raw.angular.z + (1 - alpha) * self.prev_centering_cmd.angular.z
            self.prev_centering_cmd = cmd

            self.get_logger().info(
                f'CENTERING: F={front_dist:.2f} L={left_dist:.2f} '
                f'R={right_dist:.2f} B={back_dist:.2f} '
                f'| closest={shortest_dir} center_dist={dist_to_center:.3f}m '
                f'spd={cmd.linear.x:.3f} ang={cmd.angular.z:.3f}',
                throttle_duration_sec=0.5)

        elif self.state == STATE_ALIGNING:
            if self.align_rotating:
                now = self.get_clock().now()
                if self.align_prev_time is not None:
                    dt = max((now - self.align_prev_time).nanoseconds * 1e-9, 0.01)
                    self.align_cumulative_yaw += abs(self.align_ang_spd) * dt
                self.align_prev_time = now

                # Failsafe: max 2 full rotations total
                if self.align_cumulative_yaw >= 2 * 2 * math.pi:
                    self.get_logger().error(
                        f'ALIGNMENT FAILED: exceeded 2 rotations. Stopping.')
                    self.state = STATE_DONE
                    return cmd

                if not self.align_done_180:
                    # ── Sub-phase 1a: spin 180° unconditionally ──────
                    if self.align_cumulative_yaw >= math.radians(120):
                        self.align_done_180 = True
                        self.get_logger().info(
                            f'180° done. Starting balance check.')
                    cmd.angular.z = self.align_ang_spd
                    self.get_logger().info(
                        f'ALIGNING spin: {math.degrees(self.align_cumulative_yaw):.0f}°/120°',
                        throttle_duration_sec=0.5)
                else:
                    # ── Sub-phase 1b: rotate until L/R inf balanced ──
                    # LiDAR rotated 180°: front-left = scan -90°..-180°,
                    #                     front-right = scan +90°..+180°
                    left_inf  = self._count_inf_sector(
                        msg, math.radians(-180), math.radians(-90))
                    right_inf = self._count_inf_sector(
                        msg, math.radians(90),  math.radians(180))
                    total_inf = left_inf + right_inf
                    margin    = max(1, int(0.1 * total_inf))

                    if total_inf >= 5 and abs(left_inf - right_inf) <= margin:
                        self.align_rotating = False
                        self.get_logger().info(
                            f'Inf balanced: L={left_inf} R={right_inf} '
                            f'(margin={margin}) after '
                            f'{math.degrees(self.align_cumulative_yaw):.0f}°. Driving out.')
                        return cmd

                    # Rotate toward the side with more inf (gap is there)
                    if right_inf > left_inf:
                        cmd.angular.z = -self.align_ang_spd
                    else:
                        cmd.angular.z = self.align_ang_spd

                    self.get_logger().info(
                        f'ALIGNING balance: L={left_inf} R={right_inf} '
                        f'diff={abs(left_inf - right_inf)} margin={margin} '
                        f'yaw={math.degrees(self.align_cumulative_yaw):.0f}°',
                        throttle_duration_sec=0.5)

            else:
                # ── Phase 2: drive forward until exited circle ───────
                ratio = self._valid_reading_ratio(msg)
                if ratio < self.inside_ratio:
                    self.state = STATE_DONE
                    self.get_logger().info(
                        f'EXITED circle ({ratio:.0%} valid readings). DONE.')
                    return cmd

                cmd.linear.x = self.forward_speed
                self.get_logger().info(
                    f'ALIGNING: driving out, valid={ratio:.0%}',
                    throttle_duration_sec=0.5)

        return cmd

    # ── Main callback ───────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan):
        if self.state == STATE_DONE:
            self.cmd_pub.publish(Twist())
            return

        # ── CENTERING / ALIGNING phases ─────────────────────────────
        if self.state in (STATE_CENTERING, STATE_ALIGNING):
            cmd = self._center_and_align(msg)
            self.cmd_pub.publish(cmd)
            return

        # ── WALL_FOLLOWING phase ────────────────────────────────────
        now = self.get_clock().now()
        cmd = Twist()

        # ── Check if inside the circle ──────────────────────────────
        # When >60% of lidar readings see a wall, we're inside → centering.
        # Guard: robot must have seen open space first (has_been_outside),
        # so a corridor at startup never triggers CENTERING prematurely.
        ratio = self._valid_reading_ratio(msg)
        if ratio < self.inside_ratio:
            self.has_been_outside = True
            self.stop_confirm_count = 0
        elif self.has_been_outside:
            self.stop_confirm_count += 1
            self.get_logger().info(
                f'INSIDE CHECK {self.stop_confirm_count}/{self.stop_confirm_needed} '
                f'valid={ratio:.0%}', throttle_duration_sec=0.5)
            if self.stop_confirm_count >= self.stop_confirm_needed:
                self.state = STATE_CENTERING
                self.get_logger().info(
                    f'ENTERED CIRCLE ({ratio:.0%} valid readings). '
                    f'Starting CENTERING phase.')
                self.cmd_pub.publish(Twist())
                return

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

            # Boost when going straight, slow down when turning hard
            speed_scale = max(0.3, min(1.5, 1.5 - abs(angular_z)))
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