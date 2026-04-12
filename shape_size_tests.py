#!/usr/bin/env python3
"""Automated shape/size robot test runner for the wall-follower assignment.

Runs the wall-follower on the andino robot after "shapeshifting" it — the
stock cylinder body on ``base_link`` is replaced with a new primitive
(cylinder / square box / rectangle box / triangular prism mesh) at z ∈
[0.00, 0.10], and the wheels, motors, caster, LiDAR, and camera are moved
to match the new footprint. The stock rplidar and camera mount meshes are
stripped so the robot no longer looks like an andino-with-a-hat. The
diff-drive plugin's ``<wheel_separation>`` is updated so kinematics stay
consistent with the new wheel positions.

Test matrix: {cylinder, square, rectangle, triangle} × {small, medium, large}
= 12 runs. Same metrics as ``mission_time_tests.py``: mission time plus
per-run min/max/mean/|mean|/std of the signed wall-distance error.

Usage:
    cd ~/tri_ws
    source install/setup.bash
    python3 src/shape_size_tests.py

Output:
    shape_size_times.csv  — CSV table with all results
    Console table printed at the end

Prerequisites:
    - andino_gz rebuilt after the ``ROBOT_URDF_FILE`` / ``FIXED_SPAWN_COORDS``
      env-var additions (colcon build --packages-select andino_gz).
    - wall_follower package built.
"""

import csv
import math
import os
import re
import select
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from xml.dom import minidom

from ament_index_python.packages import get_package_share_directory
from xacro import process_file


# ── Config ──────────────────────────────────────────────────────────────
TIMEOUT = 180                        # seconds per run
SETTLE_TIME = 8                      # pause between runs
SPAWN_COORDS = "9.74,-3.17,2.49"     # 6 m radially out from seg_1 (end of the 5 on the right)
SUCCESS_MSG = "DONE! CENTERED AND PERFECTLY ALIGNED WITH EXIT"
FOLLOW_ERR_RE = re.compile(r'FOLLOW\b.*?err=([+-]?\d+\.\d+)')

# Wall-follower parameters reused across every test (baseline).
BASELINE = {
    'desired_distance':    1.0,
    'base_speed':          0.2,
    'forward_speed':       1.0,
    'kp':                  1.4,
    'ki':                  0.01,
    'kd':                  1.2,
    'kd_filter':           0.3,
    'max_accel':           0.8,
    'max_angular_vel':     1.5,
    'max_linear_vel':      1.0,
    'max_wheel_vel':       0.8,
    'front_obstacle_dist': 0.5,
}

# Body geometry (base_link frame). The new body *replaces* base_link's own
# cylinder visual+collision so it occupies the same vertical slot as the
# original andino body stack. The body is centered at z=0 (same as the
# stock cylinder whose inertial is at z=0) to avoid sim-vs-RViz mismatch.
# LiDAR is then moved to BODY_Z_TOP + clearance.
BODY_Z_BOTTOM = -0.01                # body extends below base_link origin (clear of 0.033m wheels)
BODY_Z_TOP    =  0.04                # body extends above base_link origin
BODY_HEIGHT   = BODY_Z_TOP - BODY_Z_BOTTOM  # 0.05m (matches stock andino height)
BODY_Z_CENTER = (BODY_Z_TOP + BODY_Z_BOTTOM) / 2.0  # 0.015

# Characteristic dimension per size bucket.
SIZES = {
    'small':  0.20,
    'medium': 0.35,
    'large':  0.50,
}

SHAPES = ['cylinder', 'square', 'rectangle', 'triangle']

# Temp dir for generated URDFs and STL meshes.
TMP_DIR = os.path.join(tempfile.gettempdir(), 'shape_robots')
os.makedirs(TMP_DIR, exist_ok=True)


# ── STL generator (for triangle meshes) ─────────────────────────────────
def write_triangle_stl(path, side, height):
    """Write an ASCII STL of an equilateral triangular prism.

    Apex points along +X, base parallel to Y axis, prism extruded along Z
    from 0 to ``height``. ``side`` is the edge length of the equilateral
    base triangle.
    """
    # Equilateral triangle vertices in XY (centroid at origin).
    # Circumradius R = side / sqrt(3) for an equilateral triangle.
    R = side / math.sqrt(3)
    # Vertex 0 on the +X axis; vertices 1,2 symmetric about X axis at -R/2 * 2 = -side/(2*sqrt(3))?
    # Use direct placement: apex at (+R, 0), base corners at (-R/2, ±side/2).
    v = [
        ( R,        0.0),
        (-R / 2.0,  side / 2.0),
        (-R / 2.0, -side / 2.0),
    ]
    z0, z1 = 0.0, height

    def facet(f, nx, ny, nz, p1, p2, p3):
        f.write(f"facet normal {nx:.6f} {ny:.6f} {nz:.6f}\n")
        f.write( "  outer loop\n")
        f.write(f"    vertex {p1[0]:.6f} {p1[1]:.6f} {p1[2]:.6f}\n")
        f.write(f"    vertex {p2[0]:.6f} {p2[1]:.6f} {p2[2]:.6f}\n")
        f.write(f"    vertex {p3[0]:.6f} {p3[1]:.6f} {p3[2]:.6f}\n")
        f.write( "  endloop\n")
        f.write( "endfacet\n")

    with open(path, 'w') as f:
        f.write("solid triangle_prism\n")
        # Bottom face (normal -Z); wind CCW when viewed from -Z.
        facet(f, 0, 0, -1,
              (v[0][0], v[0][1], z0),
              (v[2][0], v[2][1], z0),
              (v[1][0], v[1][1], z0))
        # Top face (normal +Z); wind CCW when viewed from +Z.
        facet(f, 0, 0, 1,
              (v[0][0], v[0][1], z1),
              (v[1][0], v[1][1], z1),
              (v[2][0], v[2][1], z1))
        # Three vertical side quads split into 2 triangles each.
        for i in range(3):
            a = v[i]
            b = v[(i + 1) % 3]
            ex, ey = b[0] - a[0], b[1] - a[1]
            # Outward normal (rotate edge -90° around +Z)
            nx, ny = ey, -ex
            nlen = math.hypot(nx, ny) or 1.0
            nx /= nlen
            ny /= nlen
            p_a0 = (a[0], a[1], z0)
            p_b0 = (b[0], b[1], z0)
            p_b1 = (b[0], b[1], z1)
            p_a1 = (a[0], a[1], z1)
            facet(f, nx, ny, 0, p_a0, p_b0, p_b1)
            facet(f, nx, ny, 0, p_a0, p_b1, p_a1)
        f.write("endsolid triangle_prism\n")


# ── Shape footprint extent helpers ──────────────────────────────────────
def body_extents(shape, dim):
    """(x_front, x_back, y_half) — signed extents of the body in base_link XY.

    ``x_front`` is the +X edge (camera goes just ahead of this), ``x_back``
    is the -X edge (caster goes just behind this), and ``y_half`` is the
    max half-width in Y (unused for wheel placement — see
    ``body_half_width_at_x`` for that).
    """
    if shape == 'cylinder':
        r = dim / 2.0
        return r, -r, r
    if shape == 'square':
        half = dim / 2.0
        return half, -half, half
    if shape == 'rectangle':
        half_x = dim / 2.0
        return half_x, -half_x, (dim * 0.5) / 2.0
    if shape == 'triangle':
        # Equilateral: apex at (+R, 0), base corners at (-R/2, ±side/2)
        R = dim / math.sqrt(3)
        return R, -R / 2.0, dim / 2.0
    raise ValueError(shape)


def body_half_width_at_x(shape, dim, x):
    """Half-width (Y extent) of the body at the given X slice, base_link frame.

    Used to place the wheels just outside the body at the wheel X position
    (x ≈ 0) so the robot looks right at any shape/size.
    """
    if shape == 'cylinder':
        r = dim / 2.0
        if abs(x) >= r:
            return 0.0
        return math.sqrt(r * r - x * x)
    if shape == 'square':
        half = dim / 2.0
        return half if abs(x) <= half else 0.0
    if shape == 'rectangle':
        half_x = dim / 2.0
        half_y = (dim * 0.5) / 2.0
        return half_y if abs(x) <= half_x else 0.0
    if shape == 'triangle':
        # Apex at +R, base edge at -R/2. Width is linear:
        #   at x = +R      → 0
        #   at x = -R/2    → side
        R = dim / math.sqrt(3)
        if x > R or x < -R / 2.0:
            return 0.0
        # Parameterize: t=0 at apex (+R), t=1 at base (-R/2)
        t = (R - x) / (1.5 * R)
        return 0.5 * dim * t
    raise ValueError(shape)


# ── Shape body geometry helpers ─────────────────────────────────────────
def make_body_geometry(shape, dim):
    """Return (inner_geometry_xml, z_origin_offset, params) for a given shape.

    ``inner_geometry_xml`` is the ``<geometry>…</geometry>`` block that goes
    inside a <visual>/<collision>. ``z_origin_offset`` is where the origin
    of that geometry should sit within base_link — cylinders and boxes are
    centered so they use BODY_Z_CENTER, the triangle prism mesh is
    extruded from z=0 upward so it uses BODY_Z_BOTTOM.
    """
    if shape == 'cylinder':
        radius = dim / 2.0
        geom = f'<geometry><cylinder radius="{radius:.4f}" length="{BODY_HEIGHT:.4f}"/></geometry>'
        return geom, BODY_Z_CENTER, {'radius': radius}

    if shape == 'square':
        sx = sy = dim
        geom = f'<geometry><box size="{sx:.4f} {sy:.4f} {BODY_HEIGHT:.4f}"/></geometry>'
        return geom, BODY_Z_CENTER, {'x': sx, 'y': sy}

    if shape == 'rectangle':
        sx = dim
        sy = dim * 0.5  # elongated along X, narrower along Y
        geom = f'<geometry><box size="{sx:.4f} {sy:.4f} {BODY_HEIGHT:.4f}"/></geometry>'
        return geom, BODY_Z_CENTER, {'x': sx, 'y': sy}

    if shape == 'triangle':
        side = dim
        mesh_file = os.path.join(TMP_DIR, f'triangle_{side:.3f}.stl')
        if not os.path.exists(mesh_file):
            write_triangle_stl(mesh_file, side, BODY_HEIGHT)
        geom = f'<geometry><mesh filename="file://{mesh_file}"/></geometry>'
        return geom, BODY_Z_BOTTOM, {'side': side, 'mesh': mesh_file}

    raise ValueError(f"Unknown shape: {shape}")


def make_body_xml(shape, dim):
    """Return (visual_fragment_xml, collision_fragment_xml, params).

    These fragments are designed to be parsed and grafted into
    ``<link name='base_link'>`` (replacing its stock cylinder geometry).
    """
    geom, z_off, params = make_body_geometry(shape, dim)

    visual = (
        '<visual name="shape_body_visual">'
        f'<origin xyz="0 0 {z_off:.4f}" rpy="0 0 0"/>'
        f'{geom}'
        '<material name="shape_body_mat"><color rgba="0.0 0.0 1.0 1.0"/></material>'
        '</visual>'
    )
    collision = (
        '<collision name="shape_body_collision">'
        f'<origin xyz="0 0 {z_off:.4f}" rpy="0 0 0"/>'
        f'{geom}'
        '</collision>'
    )

    # Shell mass scales with footprint area (~0.1 kg at small, ~0.6 kg at large).
    # The original base_link inertial stays, so this is just an advertised stat.
    params['mass_est'] = max(0.05, 0.8 * (dim ** 2) / (0.20 ** 2) * 0.1)
    return visual, collision, params


# ── DOM helpers ─────────────────────────────────────────────────────────
def _child_elements(node, tag):
    return [c for c in node.childNodes
            if c.nodeType == c.ELEMENT_NODE and c.tagName == tag]


def _strip_body_geometry(link):
    """Remove all <visual> and <collision> children from a link element."""
    for tag in ('visual', 'collision'):
        for child in _child_elements(link, tag):
            link.removeChild(child)


# ── Shapeshift: rewrite base_link's body with the new shape ─────────────
# Links whose visuals/collisions are stripped — everything that makes the
# robot look like the stock andino "with a hat" instead of like the target
# shape. All their <inertial> blocks (and the links themselves) are kept so
# joint chains and Gazebo sensor references still resolve.
_STRIP_LINKS = (
    'second_base_link',     # small cylinder above base_link (stock body stack)
    'left_motor',           # motor boxes that clip through a small shape
    'right_motor',
)

# Stock joint X offsets for the wheel axle and motor flanges (positive fwd).
_WHEEL_X   = 0.0
_MOTOR_X   = 0.004

# Stock andino wheel placement (from wheel.yaml / common_macros.urdf.xacro):
#   pos_y = (base_y_size/2) + y_offset + (wheel_length/2)
#         = body_half_width + (-0.0325) + 0.0125
#         = body_half_width - 0.02
# This net offset places wheels *inward* from the body edge.
_STOCK_WHEEL_NET_OFFSET = -0.02   # body_half_width + this = stock wheel_y
_MIN_WHEEL_CLEARANCE    =  0.005  # absolute minimum gap (wheels must not clip body)

_CASTER_MARGIN = 0.015   # how far behind the shape the caster sits
_CAMERA_MARGIN = 0.005   # how far ahead of the shape the (invisible) camera sensor sits
_LIDAR_CLEARANCE = 0.005 # LiDAR sits this far above BODY_Z_TOP


def _set_origin_xyz(joint, x, y, z):
    """Update the xyz attribute of a joint's <origin> child (rpy left alone)."""
    for child in joint.childNodes:
        if child.nodeType == child.ELEMENT_NODE and child.tagName == 'origin':
            child.setAttribute('xyz', f'{x:.4f} {y:.4f} {z:.4f}')
            return
    # No <origin>? create one.
    doc = joint.ownerDocument
    origin = doc.createElement('origin')
    origin.setAttribute('rpy', '0 0 0')
    origin.setAttribute('xyz', f'{x:.4f} {y:.4f} {z:.4f}')
    joint.appendChild(origin)


def _get_origin_xyz(joint):
    for child in joint.childNodes:
        if child.nodeType == child.ELEMENT_NODE and child.tagName == 'origin':
            xyz = child.getAttribute('xyz')
            if xyz:
                parts = xyz.split()
                if len(parts) == 3:
                    return tuple(float(p) for p in parts)
    return None


def _update_plugin_wheel_separation(doc, sep):
    """Rewrite all <wheel_separation> elements (gz-sim-diff-drive-system)."""
    for node in doc.getElementsByTagName('wheel_separation'):
        for child in list(node.childNodes):
            node.removeChild(child)
        node.appendChild(doc.createTextNode(f'{sep:.4f}'))


def shapeshift_urdf(urdf_str, shape, dim):
    """Replace the andino base body with the given shape and move hardware to match.

    Changes:
      - ``base_link`` — stock cylinder visual+collision stripped, new shape
        visual+collision appended. Its inertial is preserved so drivetrain
        physics still see the correct mass/inertia.
      - ``second_base_link``, ``left_motor``, ``right_motor``,
        ``rplidar_laser_link``, ``camera_link`` — visual+collision stripped
        so no stock hardware pokes out of the new body. Their links/joints/
        inertials remain so Gazebo sensor references still resolve.
      - wheel joints repositioned in Y so wheels sit just outside the new
        body at the wheel X slice, and the motor flanges follow them.
      - caster joints moved to the new back edge of the body.
      - rplidar_laser_joint moved to the top-center of the body so the
        invisible LiDAR sensor fires from the middle of whatever shape is
        active (instead of staying at its stock x=0.05 offset).
      - camera_joint moved to the new front edge of the body.
      - ``<wheel_separation>`` in the gz-sim-diff-drive-system plugin
        updated to match the new wheel Y so differential-drive kinematics
        stay consistent with the physical geometry.
    """
    doc = minidom.parseString(urdf_str)

    # ── New shape visual/collision ─────────────────────────────────────
    visual_xml, collision_xml, params = make_body_xml(shape, dim)
    visual_node    = minidom.parseString(visual_xml).documentElement
    collision_node = minidom.parseString(collision_xml).documentElement

    stripped = set()
    replaced = False
    for link in doc.getElementsByTagName('link'):
        name = link.getAttribute('name')
        if name == 'base_link':
            _strip_body_geometry(link)
            link.appendChild(visual_node)
            link.appendChild(collision_node)
            replaced = True
        elif name in _STRIP_LINKS:
            _strip_body_geometry(link)
            stripped.add(name)

    if not replaced:
        raise RuntimeError("Could not locate <link name='base_link'> in processed URDF.")

    # ── Reposition hardware to match new body footprint ────────────────
    x_front, x_back, y_half_max = body_extents(shape, dim)
    # Wheel Y: replicate the stock andino offset formula so that
    # cylinder/small (dim=0.20) gives exactly the stock 0.08 → sep 0.16.
    # For larger/wider shapes the wheels shift outward proportionally.
    # Clamp so wheels never clip inside the body.
    y_body_at_wheel = body_half_width_at_x(shape, dim, _WHEEL_X)
    if y_body_at_wheel <= 0.0:
        y_body_at_wheel = y_half_max
    new_wheel_y = y_body_at_wheel + _STOCK_WHEEL_NET_OFFSET
    # Safety: stock andino wheels overlap the body inwardly.
    # We only need to ensure the wheels don't cross the center line
    # or clip into each other on very narrow shapes.
    min_wheel_y = 0.05
    if new_wheel_y < min_wheel_y:
        new_wheel_y = min_wheel_y
    new_motor_y = new_wheel_y + 0.0175  # stock motor flange sits 0.0175m outboard of wheel
    new_caster_x = x_back - _CASTER_MARGIN
    new_camera_x = x_front + _CAMERA_MARGIN

    for joint in doc.getElementsByTagName('joint'):
        jname = joint.getAttribute('name')
        current = _get_origin_xyz(joint)
        if current is None:
            continue
        cx, cy, cz = current

        if jname == 'left_wheel_joint':
            _set_origin_xyz(joint, cx,  new_wheel_y, cz)
        elif jname == 'right_wheel_joint':
            _set_origin_xyz(joint, cx, -new_wheel_y, cz)
        elif jname == 'left_motor_joint':
            _set_origin_xyz(joint, cx,  new_motor_y, cz)
        elif jname == 'right_motor_joint':
            _set_origin_xyz(joint, cx, -new_motor_y, cz)
        elif jname == 'caster_base_joint':
            _set_origin_xyz(joint, new_caster_x, cy, cz)
        elif jname == 'caster_rotation_joint':
            # Keep the stock 0.008m offset between base and rotation pivot.
            _set_origin_xyz(joint, new_caster_x + 0.008, cy, cz)
        elif jname == 'rplidar_laser_joint':
            # Move LiDAR to top-center of the new body. Parent is
            # second_base_link (at (0,0,0) in base_link) so this is also the
            # position in base_link frame. rpy is left alone to preserve the
            # π flip that scan indexing depends on.
            _set_origin_xyz(joint, 0.0, 0.0, BODY_Z_TOP + _LIDAR_CLEARANCE)
        elif jname == 'camera_joint':
            _set_origin_xyz(joint, new_camera_x, 0.0, cz)

    # ── Keep diff-drive plugin kinematics consistent ───────────────────
    wheel_separation = 2.0 * new_wheel_y
    _update_plugin_wheel_separation(doc, wheel_separation)

    params['body_x_front']     = x_front
    params['body_x_back']      = x_back
    params['body_y_half']      = y_half_max
    params['wheel_y']          = new_wheel_y
    params['wheel_separation'] = wheel_separation
    params['caster_x']         = new_caster_x
    params['lidar_z']          = BODY_Z_TOP + _LIDAR_CLEARANCE

    return doc.toxml(), params, sorted(stripped)


# ── Full-robot URDF generator ───────────────────────────────────────────
def generate_shape_urdf(shape, size_name):
    """Process the andino xacro, shapeshift base_link, return (urdf_path, params)."""
    pkg_andino_gz = get_package_share_directory('andino_gz')
    pkg_andino_description = get_package_share_directory('andino_description')

    xacro_path = os.path.join(pkg_andino_gz, 'urdf', 'andino_gz.urdf.xacro')
    mappings = {'use_fixed_caster': 'false'}
    doc = process_file(xacro_path, mappings=mappings)
    urdf_str = doc.toprettyxml(indent='  ')
    # Same package→file path rewrite the stock launcher does.
    urdf_str = urdf_str.replace(
        'package://andino_description/', f'file://{pkg_andino_description}/'
    )

    dim = SIZES[size_name]
    urdf_str, params, stripped = shapeshift_urdf(urdf_str, shape, dim)
    params['dim'] = dim
    params['stripped_links'] = stripped

    out_path = os.path.join(TMP_DIR, f'andino_{shape}_{size_name}.urdf')
    with open(out_path, 'w') as f:
        f.write(urdf_str)

    return out_path, params


# ── Process helpers ─────────────────────────────────────────────────────
def build_param_args(overrides):
    params = {**BASELINE, **overrides}
    args = []
    for k, v in params.items():
        args.extend(['-p', f'{k}:={v}'])
    return args


def kill_process_tree(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=5)


# ── Single test run ─────────────────────────────────────────────────────
def run_test(shape, size_name):
    print(f"\n{'='*60}")
    print(f"  TEST: {shape} {size_name} (dim={SIZES[size_name]:.2f}m)")
    print(f"{'='*60}")

    # 1. Build the custom URDF file.
    urdf_path, params = generate_shape_urdf(shape, size_name)
    print(f"  URDF: {urdf_path}")
    _printable = {k: (f'{v:.3f}' if isinstance(v, float) else v)
                  for k, v in params.items() if k not in ('mesh', 'stripped_links')}
    print(f"  params: {_printable}")
    print(f"  stripped andino links: {params['stripped_links']}")

    # 2. Launch Gazebo + robot with the custom URDF (GUI + auto-run + RViz).
    sim_env = os.environ.copy()
    sim_env['EXTRA_GZ_ARGS']      = '-s -r'   # headless (-s) + auto-run (-r)
    sim_env['NO_RVIZ']            = 'True'    # enable RViz
    sim_env['SHAPE_TEST_URDF_FILE'] = urdf_path
    sim_env['FIXED_SPAWN_COORDS'] = SPAWN_COORDS

    sim_cmd = [
        'ros2', 'launch', 'andino_gz', 'assignment1.launch.py',
        'robots:=andino={x: 9.74, y: -3.17, z: 0.05, yaw: 2.49};'
    ]
    print("  Launching simulation (GUI + auto-run + RViz)…")
    sim_proc = subprocess.Popen(
        sim_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
        env=sim_env,
    )

    # Let Gazebo + the robot fully spawn.
    time.sleep(12)

    # 3. Launch wall_follower with baseline params.
    wf_cmd = [
        'ros2', 'run', 'wall_follower', 'wall_follower_node',
        '--ros-args',
    ] + build_param_args({})
    print("  Launching wall_follower…")
    wf_proc = subprocess.Popen(
        wf_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
        bufsize=1,
    )

    start_time = time.time()
    mission_time = None
    success = False
    errors = []

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed > TIMEOUT:
                print(f"  TIMEOUT after {TIMEOUT}s")
                break
            ready, _, _ = select.select([wf_proc.stdout], [], [], 1.0)
            if ready:
                line = wf_proc.stdout.readline()
                if not line:
                    print("  wall_follower ended unexpectedly")
                    break
                m = FOLLOW_ERR_RE.search(line)
                if m:
                    try:
                        errors.append(float(m.group(1)))
                    except ValueError:
                        pass
                if SUCCESS_MSG in line:
                    mission_time = time.time() - start_time
                    success = True
                    print(f"  MISSION COMPLETE! Time: {mission_time:.1f}s")
                    break
    except KeyboardInterrupt:
        print("\n  User interrupted.")
        raise
    finally:
        print("  Stopping wall_follower…")
        kill_process_tree(wf_proc)
        print("  Stopping simulation…")
        kill_process_tree(sim_proc)
        subprocess.run(['pkill', '-f', 'gz sim'],   capture_output=True)
        subprocess.run(['pkill', '-f', 'ruby.*gz'], capture_output=True)
        print(f"  Waiting {SETTLE_TIME}s for cleanup…")
        time.sleep(SETTLE_TIME)

    # 4. Stats.
    if errors:
        abs_errs = [abs(e) for e in errors]
        err_stats = {
            'err_n':        len(errors),
            'err_min':      min(errors),
            'err_max':      max(errors),
            'err_mean':     statistics.mean(errors),
            'err_abs_mean': statistics.mean(abs_errs),
            'err_std':      statistics.pstdev(errors) if len(errors) > 1 else 0.0,
        }
        print(f"  Wall-follow error (N={err_stats['err_n']}): "
              f"min={err_stats['err_min']:+.3f} max={err_stats['err_max']:+.3f} "
              f"mean={err_stats['err_mean']:+.3f} |mean|={err_stats['err_abs_mean']:.3f} "
              f"std={err_stats['err_std']:.3f}")
    else:
        err_stats = {'err_n': 0, 'err_min': None, 'err_max': None,
                     'err_mean': None, 'err_abs_mean': None, 'err_std': None}
        print("  Wall-follow error: no samples captured")

    return mission_time, success, err_stats, params


# ── Main ────────────────────────────────────────────────────────────────
def main():
    results = []
    csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'shape_size_times.csv')

    total = len(SHAPES) * len(SIZES)
    print("\n" + "=" * 74)
    print("  SHAPE × SIZE TEST RUNNER")
    print(f"  {total} tests ({len(SHAPES)} shapes × {len(SIZES)} sizes), {TIMEOUT}s timeout each")
    print(f"  Spawn: {SPAWN_COORDS}  (6m radially out from seg_1 — spiral → full-map loop)")
    print(f"  URDFs staged under: {TMP_DIR}")
    print(f"  Results → {csv_path}")
    print("=" * 74)

    i = 0
    for shape in SHAPES:
        for size_name in SIZES:
            i += 1
            print(f"\n  [{i}/{total}]", end='')
            try:
                mission_time, success, err_stats, params = run_test(shape, size_name)
            except KeyboardInterrupt:
                print("\n\nAborted by user. Saving partial results…")
                break

            def _fmt(v):
                return f"{v:+.3f}" if isinstance(v, (int, float)) else ""

            results.append({
                'shape':        shape,
                'size':         size_name,
                'dim':          f"{params['dim']:.2f}",
                'mass_est':     f"{params['mass_est']:.3f}",
                'footprint':    (f"r={params['radius']:.2f}" if shape == 'cylinder'
                                 else f"{params['x']:.2f}x{params['y']:.2f}" if shape in ('square', 'rectangle')
                                 else f"side={params['side']:.2f}"),
                'time_s':       f"{mission_time:.1f}" if mission_time else "FAIL",
                'success':      success,
                'err_n':        err_stats['err_n'],
                'err_min':      _fmt(err_stats['err_min']),
                'err_max':      _fmt(err_stats['err_max']),
                'err_mean':     _fmt(err_stats['err_mean']),
                'err_abs_mean': (f"{err_stats['err_abs_mean']:.3f}"
                                 if err_stats['err_abs_mean'] is not None else ""),
                'err_std':      (f"{err_stats['err_std']:.3f}"
                                 if err_stats['err_std'] is not None else ""),
            })
        else:
            continue
        break

    # ── Save CSV ────────────────────────────────────────────────────────
    if results:
        fieldnames = list(results[0].keys())
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)
        print(f"\nResults saved to {csv_path}")

    # ── Print table ─────────────────────────────────────────────────────
    print("\n" + "=" * 130)
    print(f"{'Shape':<10} {'Size':<7} {'Dim':<5} {'Footprint':<12} {'MassEst':<7} "
          f"{'Time(s)':<9} "
          f"{'errMin':<8} {'errMax':<8} {'errMean':<9} {'|err|':<7} {'errStd':<7} "
          f"{'Result':<6}")
    print("-" * 130)
    for r in results:
        status = "OK" if r['success'] else "FAIL"
        print(f"{r['shape']:<10} {r['size']:<7} {r['dim']:<5} {r['footprint']:<12} "
              f"{r['mass_est']:<7} {r['time_s']:<9} "
              f"{str(r['err_min']):<8} {str(r['err_max']):<8} "
              f"{str(r['err_mean']):<9} {str(r['err_abs_mean']):<7} "
              f"{str(r['err_std']):<7} {status:<6}")
    print("=" * 130)


if __name__ == '__main__':
    main()
