#!/usr/bin/env python3
"""Automated mission time test runner for the wall-follower assignment.

Launches the simulation with a FIXED spawn point and runs the wall_follower
node with different parameter configurations. Records the time from launch
to mission completion ("DONE! CENTERED AND PERFECTLY ALIGNED WITH EXIT").

Usage:
    python3 mission_time_tests.py

Output:
    mission_times.csv  — CSV table with all results
    Console table printed at the end

Prerequisites:
    - Gazebo and andino_gz must be installed
    - wall_follower package must be built
    - FIXED_SPAWN = 1 in assignment1.launch.py
    - Source your workspace: source install/setup.bash
"""

import subprocess
import signal
import time
import os
import sys
import csv
import re
import statistics
from datetime import datetime

# Regex to extract the signed error value from a throttled FOLLOW log line.
# Matches: "FOLLOW left=1.03m err=+0.03 inf=0% ang=-0.04 spd=0.71"
FOLLOW_ERR_RE = re.compile(r'FOLLOW\b.*?err=([+-]?\d+\.\d+)')

# ── Timeout per test (seconds) ────────────────────────────────────────
TIMEOUT = 360  # 3 minutes max per test

# ── Baseline parameters (the "default" configuration) ─────────────────
BASELINE = {
    'desired_distance': 1.0,
    'base_speed': 0.2,
    'forward_speed': 1.0,
    'kp': 1.4,
    'ki': 0.01,
    'kd': 1.2,
    'kd_filter': 0.3,
    'max_accel': 0.8,
    'max_angular_vel': 1.5,
    'max_linear_vel': 1.0,
    'max_wheel_vel': 0.8,
    'front_obstacle_dist': 0.5,
}

# ── Test matrix: each entry = (test_name, {param_overrides}) ──────────
# Vary one parameter at a time from baseline
TESTS = [
    # Baseline
    ("baseline", {}),

    # Vary Kp
    ("kp=0.8",  {'kp': 0.8}),
    ("kp=1.0",  {'kp': 1.0}),
    ("kp=1.8",  {'kp': 1.8}),
    ("kp=2.2",  {'kp': 2.2}),

    # Vary Kd
    ("kd=0.5",  {'kd': 0.5}),
    ("kd=0.8",  {'kd': 0.8}),
    ("kd=1.6",  {'kd': 1.6}),
    ("kd=2.0",  {'kd': 2.0}),

    # Vary Ki
    ("ki=0.0",   {'ki': 0.0}),
    ("ki=0.05",  {'ki': 0.05}),
    ("ki=0.1",   {'ki': 0.1}),

    # Vary forward speed
    ("vel=0.5",  {'forward_speed': 0.5}),
    ("vel=0.75", {'forward_speed': 0.75}),
    ("vel=1.5",  {'forward_speed': 1.5, 'max_linear_vel': 1.5}),

    # Vary max acceleration
    ("accel=0.3", {'max_accel': 0.3}),
    ("accel=0.5", {'max_accel': 0.5}),
    ("accel=1.5", {'max_accel': 1.5}),
    ("accel=3.0", {'max_accel': 3.0}),

    # Vary max angular velocity
    ("ang_vel=0.8", {'max_angular_vel': 0.8}),
    ("ang_vel=1.0", {'max_angular_vel': 1.0}),
    ("ang_vel=2.0", {'max_angular_vel': 2.0}),
    ("ang_vel=3.0", {'max_angular_vel': 3.0}),
]

SUCCESS_MSG = "DONE! CENTERED AND PERFECTLY ALIGNED WITH EXIT"
SETTLE_TIME = 8  # seconds to wait after killing before next test


def build_param_args(overrides):
    """Build ros2 run --ros-args parameter list."""
    params = {**BASELINE, **overrides}
    args = []
    for k, v in params.items():
        args.extend(['-p', f'{k}:={v}'])
    return args


def kill_process_tree(proc):
    """Send SIGINT then SIGKILL to process group."""
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


def run_test(test_name, overrides):
    """Run a single test. Returns (mission_time_seconds, success)."""
    print(f"\n{'='*60}")
    print(f"  TEST: {test_name}")
    print(f"  Overrides: {overrides if overrides else 'none (baseline)'}")
    print(f"{'='*60}")

    # 1. Launch Gazebo simulation (fixed spawn, GUI + auto-run + RViz)
    sim_env = os.environ.copy()
    sim_env['EXTRA_GZ_ARGS'] = '-r'      # auto-run (no play button); GUI stays on
    sim_env['NO_RVIZ'] = 'True'          # enable RViz (misnomer — this var is the literal rviz arg value)

    sim_cmd = [
        'ros2', 'launch', 'andino_gz', 'assignment1.launch.py',
        'robots:=andino={x: 9.74, y: -3.17, z: 0.05, yaw: 2.49};'
    ]
    print(f"  Launching simulation (GUI + auto-run + RViz)...")
    sim_proc = subprocess.Popen(
        sim_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
        env=sim_env
    )

    # Wait for Gazebo + robot to be ready
    time.sleep(12)

    # 2. Launch wall_follower with test parameters
    wf_cmd = [
        'ros2', 'run', 'wall_follower', 'wall_follower_node',
        '--ros-args',
    ] + build_param_args(overrides)

    print(f"  Launching wall_follower...")
    wf_proc = subprocess.Popen(
        wf_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
        bufsize=1
    )

    start_time = time.time()
    mission_time = None
    success = False
    errors = []  # signed wall-distance errors sampled during FOLLOW phase

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed > TIMEOUT:
                print(f"  TIMEOUT after {TIMEOUT}s")
                break

            # Read output line by line (non-blocking via timeout)
            try:
                # Use select-style reading
                import select
                ready, _, _ = select.select([wf_proc.stdout], [], [], 1.0)
                if ready:
                    line = wf_proc.stdout.readline()
                    if not line:
                        # Process ended
                        print(f"  Wall follower process ended unexpectedly")
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
            except Exception:
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n  User interrupted.")
        raise

    finally:
        # Kill wall_follower
        print(f"  Stopping wall_follower...")
        kill_process_tree(wf_proc)

        # Kill simulation
        print(f"  Stopping simulation...")
        kill_process_tree(sim_proc)

        # Also kill any lingering gz/gzserver processes
        subprocess.run(['pkill', '-f', 'gz sim'], capture_output=True)
        subprocess.run(['pkill', '-f', 'ruby.*gz'], capture_output=True)

        print(f"  Waiting {SETTLE_TIME}s for cleanup...")
        time.sleep(SETTLE_TIME)

    # Per-run error statistics (signed error = wall_dist - desired_dist, in meters)
    if errors:
        abs_errs = [abs(e) for e in errors]
        err_stats = {
            'err_n':    len(errors),
            'err_min':  min(errors),
            'err_max':  max(errors),
            'err_mean': statistics.mean(errors),
            'err_abs_mean': statistics.mean(abs_errs),
            'err_std':  statistics.pstdev(errors) if len(errors) > 1 else 0.0,
        }
        print(f"  Wall-follow error (N={err_stats['err_n']}): "
              f"min={err_stats['err_min']:+.3f} max={err_stats['err_max']:+.3f} "
              f"mean={err_stats['err_mean']:+.3f} |mean|={err_stats['err_abs_mean']:.3f} "
              f"std={err_stats['err_std']:.3f}")
    else:
        err_stats = {'err_n': 0, 'err_min': None, 'err_max': None,
                     'err_mean': None, 'err_abs_mean': None, 'err_std': None}
        print("  Wall-follow error: no samples captured")

    return mission_time, success, err_stats


def main():
    results = []
    csv_path = os.path.join(os.path.dirname(__file__), 'mission_times.csv')

    print("\n" + "="*70)
    print("  MISSION TIME TEST RUNNER")
    print(f"  {len(TESTS)} tests, {TIMEOUT}s timeout each")
    print(f"  Fixed spawn: x=9.74, y=-3.17, yaw=2.49  (6m radially out from seg_1 → spiral → full-map loop)")
    print(f"  Results will be saved to: {csv_path}")
    print("="*70)

    for i, (name, overrides) in enumerate(TESTS):
        print(f"\n  [{i+1}/{len(TESTS)}]", end="")
        try:
            mission_time, success, err_stats = run_test(name, overrides)
        except KeyboardInterrupt:
            print("\n\nAborted by user. Saving partial results...")
            break

        params = {**BASELINE, **overrides}

        def _fmt(v):
            return f"{v:+.3f}" if isinstance(v, (int, float)) else ""

        results.append({
            'test': name,
            'time_s': f"{mission_time:.1f}" if mission_time else "FAIL",
            'success': success,
            'kp': params['kp'],
            'ki': params['ki'],
            'kd': params['kd'],
            'forward_speed': params['forward_speed'],
            'max_accel': params['max_accel'],
            'max_angular_vel': params['max_angular_vel'],
            'max_linear_vel': params['max_linear_vel'],
            'err_n':        err_stats['err_n'],
            'err_min':      _fmt(err_stats['err_min']),
            'err_max':      _fmt(err_stats['err_max']),
            'err_mean':     _fmt(err_stats['err_mean']),
            'err_abs_mean': (f"{err_stats['err_abs_mean']:.3f}"
                             if err_stats['err_abs_mean'] is not None else ""),
            'err_std':      (f"{err_stats['err_std']:.3f}"
                             if err_stats['err_std'] is not None else ""),
        })

    # ── Save CSV ──────────────────────────────────────────────────────
    if results:
        fieldnames = list(results[0].keys())
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)
        print(f"\nResults saved to {csv_path}")

    # ── Print table ───────────────────────────────────────────────────
    print("\n" + "="*130)
    print(f"{'Test':<14} {'Time(s)':<9} {'Kp':<5} {'Ki':<5} {'Kd':<5} "
          f"{'Vel':<5} {'Accel':<6} {'AngVel':<6} "
          f"{'errMin':<8} {'errMax':<8} {'errMean':<9} {'|err|':<7} {'errStd':<7} "
          f"{'Result':<6}")
    print("-"*130)
    for r in results:
        status = "OK" if r['success'] else "FAIL"
        print(f"{r['test']:<14} {r['time_s']:<9} {r['kp']:<5} {r['ki']:<5} "
              f"{r['kd']:<5} {r['forward_speed']:<5} {r['max_accel']:<6} "
              f"{r['max_angular_vel']:<6} "
              f"{str(r['err_min']):<8} {str(r['err_max']):<8} "
              f"{str(r['err_mean']):<9} {str(r['err_abs_mean']):<7} "
              f"{str(r['err_std']):<7} {status:<6}")
    print("="*130)


if __name__ == '__main__':
    main()
