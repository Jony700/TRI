import os
import sys
import math
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# ── Wall geometry (must match assignment1.sdf) ──────────────────────────
# L-shape
L_VERT_CENTER = (-1.0, 2.982)
L_VERT_SIZE = (0.5, 5.5)
L_HORIZ_CENTER = (0.175, 0.0)
L_HORIZ_SIZE = (2.85, 0.7)

# Circle
CIRCLE_CENTER = (3.0, 2.0)
CIRCLE_R_INNER = 2.0
CIRCLE_R_OUTER = 3.0

# ── Spawn constraints ──────────────────────────────────────────────────
MIN_WALL_DIST = 0.5   # must be at least this far from any wall surface
MAX_WALL_DIST = 2.0   # must be within this distance of some wall
ROBOT_Z = 0.05        # ground level (just above floor)

# Bounding box for candidate sampling (encompasses everything + margin)
SAMPLE_X_MIN, SAMPLE_X_MAX = -4.0, 8.0
SAMPLE_Y_MIN, SAMPLE_Y_MAX = -3.0, 7.0

FIXED_SPAWN = 0  # Set to 1 to use fixed spawn, 0 for random spawn

def _box_distance(px, py, cx, cy, sx, sy):
    """Signed distance from point (px,py) to axis-aligned box centered at (cx,cy) with size (sx,sy).
       Negative = inside, positive = outside."""
    dx = abs(px - cx) - sx / 2.0
    dy = abs(py - cy) - sy / 2.0
    if dx > 0 and dy > 0:
        return math.sqrt(dx * dx + dy * dy)
    return max(dx, dy)


def _circle_ring_distance(px, py):
    """Signed distance from point to the circle ring wall.
       Negative = inside the ring wall, positive = outside it."""
    d = math.sqrt((px - CIRCLE_CENTER[0])**2 + (py - CIRCLE_CENTER[1])**2)
    # Inside the ring wall material itself
    if CIRCLE_R_INNER <= d <= CIRCLE_R_OUTER:
        return -min(d - CIRCLE_R_INNER, CIRCLE_R_OUTER - d)
    # Inside the hollow center
    if d < CIRCLE_R_INNER:
        return CIRCLE_R_INNER - d
    # Outside the ring
    return d - CIRCLE_R_OUTER


def min_wall_distance(px, py):
    """Returns minimum distance to any wall surface. Negative = inside a wall."""
    d_vert = _box_distance(px, py, *L_VERT_CENTER, *L_VERT_SIZE)
    d_horiz = _box_distance(px, py, *L_HORIZ_CENTER, *L_HORIZ_SIZE)
    d_ring = _circle_ring_distance(px, py)
    return min(d_vert, d_horiz, d_ring)


def closest_wall_distance(px, py):
    """Returns absolute distance to nearest wall surface (always positive for valid points)."""
    d_vert = abs(_box_distance(px, py, *L_VERT_CENTER, *L_VERT_SIZE))
    d_horiz = abs(_box_distance(px, py, *L_HORIZ_CENTER, *L_HORIZ_SIZE))
    d_ring = abs(_circle_ring_distance(px, py))
    return min(d_vert, d_horiz, d_ring)


def fixed_spawn():
    """Deterministic spawn near the head of the 5 (top of the L-wall).

    Places the robot just outside the top of the L vertical arm,
    facing right (yaw=0) so left-wall-following picks up the path.
    """
    return -2.5, 5.0, 0.0

def random_spawn():
    """Find a random valid spawn position via rejection sampling."""
    for _ in range(5000):
        x = random.uniform(SAMPLE_X_MIN, SAMPLE_X_MAX)
        y = random.uniform(SAMPLE_Y_MIN, SAMPLE_Y_MAX)

        # Must be outside EACH wall with clearance (reject if inside ANY)
        d_vert = _box_distance(x, y, *L_VERT_CENTER, *L_VERT_SIZE)
        d_horiz = _box_distance(x, y, *L_HORIZ_CENTER, *L_HORIZ_SIZE)
        d_ring = _circle_ring_distance(x, y)
        if d_vert < MIN_WALL_DIST or d_horiz < MIN_WALL_DIST or d_ring < MIN_WALL_DIST:
            continue
        # Must be close enough to at least sense walls
        d_closest = min(abs(d_vert), abs(d_horiz), abs(d_ring))
        if d_closest > MAX_WALL_DIST:
            continue
        # Valid!
        yaw = random.uniform(-math.pi, math.pi)
        return x, y, yaw

    # Fallback: safe spot outside L
    return -3.0, 3.0, 0.0


def generate_launch_description():
    pkg_andino_gz = get_package_share_directory('andino_gz')

    #use_fixed = os.environ.get('FIXED_SPAWN', '').strip()
    use_fixed = True if FIXED_SPAWN == 1 else False
    if use_fixed and use_fixed != '0':
        x, y, yaw = fixed_spawn()
        print(f"[assignment1] FIXED spawn at x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
    else:
        x, y, yaw = random_spawn()
        print(f"[assignment1] RANDOM spawn at x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

    # Inject into sys.argv so ParseMultiRobotPose picks it up
    robots_arg = f'robots:=andino={{x: {x:.3f}, y: {y:.3f}, z: {ROBOT_Z}, yaw: {yaw:.3f}}};'
    if not any('robots:=' in arg for arg in sys.argv):
        sys.argv.append(robots_arg)

    andino_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_andino_gz, 'launch', 'andino_gz.launch.py')
        ),
        launch_arguments={
            'world_name': 'assignment1.sdf',
        }.items()
    )

    return LaunchDescription([
        andino_simulation
    ])
