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
from datetime import datetime

# ── Timeout per test (seconds) ────────────────────────────────────────
TIMEOUT = 180  # 3 minutes max per test

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

    # 1. Launch Gazebo simulation (fixed spawn, headless, auto-run, no rviz)
    sim_env = os.environ.copy()
    sim_env['EXTRA_GZ_ARGS'] = '-s -r'   # server-only + auto-run (no GUI, no play button)
    sim_env['NO_RVIZ'] = 'False'          # disable RViz

    sim_cmd = [
        'ros2', 'launch', 'andino_gz', 'assignment1.launch.py'
    ]
    print(f"  Launching simulation (headless)...")
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

    return mission_time, success


def main():
    results = []
    csv_path = os.path.join(os.path.dirname(__file__), 'mission_times.csv')

    print("\n" + "="*70)
    print("  MISSION TIME TEST RUNNER")
    print(f"  {len(TESTS)} tests, {TIMEOUT}s timeout each")
    print(f"  Fixed spawn: x=-2.5, y=5.0, yaw=0.0")
    print(f"  Results will be saved to: {csv_path}")
    print("="*70)

    for i, (name, overrides) in enumerate(TESTS):
        print(f"\n  [{i+1}/{len(TESTS)}]", end="")
        try:
            mission_time, success = run_test(name, overrides)
        except KeyboardInterrupt:
            print("\n\nAborted by user. Saving partial results...")
            break

        params = {**BASELINE, **overrides}
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
    print("\n" + "="*90)
    print(f"{'Test':<16} {'Time(s)':<10} {'Kp':<6} {'Ki':<6} {'Kd':<6} "
          f"{'Vel':<6} {'Accel':<7} {'AngVel':<7} {'Result':<8}")
    print("-"*90)
    for r in results:
        status = "OK" if r['success'] else "FAIL"
        print(f"{r['test']:<16} {r['time_s']:<10} {r['kp']:<6} {r['ki']:<6} "
              f"{r['kd']:<6} {r['forward_speed']:<6} {r['max_accel']:<7} "
              f"{r['max_angular_vel']:<7} {status:<8}")
    print("="*90)


if __name__ == '__main__':
    main()
