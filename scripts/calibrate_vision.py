#!/usr/bin/env python3
"""
3-cube vision calibration.

Workflow:
  1. Place 3 cubes (distinguishable colors) anywhere in the workspace
  2. Run cube_detector_node.py (and servo.launch.py) in other terminals
  3. Run this script
  4. For each detected cube:
     - Use plan-execute in RViz to position gripper:
       * directly above cube center
       * gripper pointing straight down
       * roughly 10 cm above the cube top
     - Press SPACE in this terminal to record
  5. Script computes 2D affine transform + Z offset
  6. Saves to ~/vision_calibration.json

The pick_and_place_vision.py script will load this file and apply
the transform to all detected cube positions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import numpy as np
import json
import os
import sys
import termios
import tty
import select
import time

OUTPUT_FILE = os.path.expanduser('~/vision_calibration.json')
EE_FRAME = 'end_effector'
BASE_FRAME = 'base_link'

# Constants for Z calculation
TABLE_Z = 0.273
CUBE_HEIGHT = 0.020      # 20mm cubes
APPROACH_HEIGHT = 0.10   # 10cm above cube top during calibration
GRASP_OFFSET = 0.10      # EE frame is this far above gripper jaw tip

# Where EE should be when positioned correctly (jaw 10cm above cube top)
EXPECTED_EE_Z = TABLE_Z + CUBE_HEIGHT + APPROACH_HEIGHT + GRASP_OFFSET

# Stability requirements for detection
STABILITY_FRAMES = 5
STABILITY_TOL = 0.01


def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class Calibrator(Node):
    def __init__(self):
        super().__init__('vision_calibrator')
        self.create_subscription(String, '/detected_cubes_info', self._info_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.detection_history = []

    def _info_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.detection_history.append((time.time(), data))
            self.detection_history = [(t, d) for t, d in self.detection_history
                                       if time.time() - t < 2.0]
        except Exception:
            pass

    def get_stable_cubes(self):
        if len(self.detection_history) < STABILITY_FRAMES:
            return []
        recent = self.detection_history[-STABILITY_FRAMES:]
        by_color = {}
        for _, frame in recent:
            for cube in frame:
                col = cube['color']
                by_color.setdefault(col, []).append(cube)
        stable = []
        for col, obs in by_color.items():
            if len(obs) < STABILITY_FRAMES * 0.6: continue
            xs = [o['x'] for o in obs]
            ys = [o['y'] for o in obs]
            if (max(xs) - min(xs) > STABILITY_TOL or
                max(ys) - min(ys) > STABILITY_TOL):
                continue
            stable.append({'color': col,
                           'x': sum(xs)/len(xs),
                           'y': sum(ys)/len(ys)})
        return stable

    def get_ee_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, EE_FRAME, rclpy.time.Time(),
                Duration(seconds=1.0))
            t = tf.transform.translation
            return float(t.x), float(t.y), float(t.z)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None


def solve_affine_2d(vision_pts, arm_pts):
    """
    Solve [arm_x] = A * [vis_x] + b   for 2D
          [arm_y]       [vis_y]
    given 3+ correspondences. Returns A (2x2), b (2,).
    """
    n = len(vision_pts)
    if n < 3:
        raise ValueError("Need at least 3 points")
    # Build linear system: [vis_x vis_y 1 0 0 0; 0 0 0 vis_x vis_y 1] * params = [arm_x; arm_y]
    A_mat = np.zeros((2*n, 6))
    b_vec = np.zeros(2*n)
    for i, (vp, ap) in enumerate(zip(vision_pts, arm_pts)):
        A_mat[2*i]   = [vp[0], vp[1], 1, 0, 0, 0]
        A_mat[2*i+1] = [0, 0, 0, vp[0], vp[1], 1]
        b_vec[2*i]   = ap[0]
        b_vec[2*i+1] = ap[1]
    params, *_ = np.linalg.lstsq(A_mat, b_vec, rcond=None)
    A = np.array([[params[0], params[1]], [params[3], params[4]]])
    b = np.array([params[2], params[5]])
    return A, b


def main():
    rclpy.init()
    node = Calibrator()

    print("\n" + "=" * 55)
    print("  RA6A VISION CALIBRATION")
    print("=" * 55)
    print(f"  Cubes: {CUBE_HEIGHT*1000:.0f}mm  Approach: {APPROACH_HEIGHT*1000:.0f}mm above top")
    print(f"  Expected EE Z: {EXPECTED_EE_Z:.3f} m")
    print()
    print("  Place 3 cubes in the workspace, then for each:")
    print("    1. Plan-execute arm so gripper is above the cube,")
    print("       pointing down, ~10cm above cube top")
    print("    2. Press SPACE here to record")
    print("    3. Press Q anytime to abort")
    print("=" * 55)
    print()

    # Wait for cubes
    print("STEP 1: CAPTURE")
    print("  Make sure arm is OUT of camera view, all 3 cubes visible.")
    print("  Press SPACE to capture cube positions...")
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        k = get_key(0.05)
        if k == ' ': break
        if k.lower() == 'q':
            print("Aborted."); node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    # Capture for a couple seconds for stability
    print("  Capturing... (waiting for 3 stable cubes)")
    cubes = []
    while rclpy.ok():
        # Collect data for 1 second
        t0 = time.time()
        while time.time() - t0 < 1.0:
            rclpy.spin_once(node, timeout_sec=0.05)
            k = get_key(0.0)
            if k.lower() == 'q':
                print("\nAborted."); node.destroy_node(); rclpy.shutdown(); sys.exit(1)
        cubes = node.get_stable_cubes()
        print(f"  Stable cubes seen: {len(cubes)} ({[c['color'] for c in cubes]})  "
              f"[Q to abort]", end='\r')
        if len(cubes) >= 3:
            break
    print()

    cubes = cubes[:3]
    print(f"\n  CAPTURED — these positions are now FROZEN:")
    for c in cubes:
        print(f"    {c['color']}: ({c['x']:+.3f}, {c['y']:+.3f})")
    print("\n  You can now safely move the arm without affecting calibration.")
    print()
    print("STEP 2: POSITION ARM ABOVE EACH CUBE")

    # Calibrate each
    pairs = []
    for cube in cubes:
        col = cube['color']
        vx, vy = cube['x'], cube['y']
        print(f"  >> Position arm above {col.upper()} cube (vision: {vx:+.3f}, {vy:+.3f}), then press SPACE")
        print(f"     (Z height doesn't matter — only X,Y are recorded)")

        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            k = get_key(0.05)
            if k == ' ':
                ee = node.get_ee_pose()
                if ee is None:
                    print("    TF read failed, try again")
                    continue
                ax, ay, az = ee
                print(f"     Recorded XY: ({ax:+.3f}, {ay:+.3f})  [Z={az:+.3f} ignored]")
                pairs.append({
                    'color': col,
                    'vision': [vx, vy],
                    'arm':    [ax, ay, az],   # save Z for reference but won't use it
                })
                break
            if k.lower() == 'q':
                print("Aborted.")
                node.destroy_node()
                rclpy.shutdown()
                sys.exit(1)

    # Compute affine transform on XY (Z is hardcoded in pick_and_place_vision.py)
    vision_pts = [p['vision'] for p in pairs]
    arm_pts = [p['arm'][:2] for p in pairs]
    A, b = solve_affine_2d(vision_pts, arm_pts)

    # Z offset is HARDCODED in pick_and_place_vision.py — saving 0 here
    # so loaded calibrations don't override the hardcoded value.
    avg_z_offset = 0.0

    print()
    print("Calibration result:")
    print(f"  A = {A.tolist()}")
    print(f"  b = {b.tolist()}")
    print(f"  z_offset = HARDCODED in pick_and_place_vision.py (Z_OFFSET_FIXED)")

    # Verify: apply to vision points and compare to arm points
    print("\nVerification (vision → arm prediction vs actual):")
    total_err = 0
    for p in pairs:
        v = np.array(p['vision'])
        predicted = A @ v + b
        actual = np.array(p['arm'][:2])
        err = np.linalg.norm(predicted - actual)
        total_err += err
        print(f"  {p['color']}: predicted=({predicted[0]:+.3f},{predicted[1]:+.3f}) "
              f"actual=({actual[0]:+.3f},{actual[1]:+.3f})  err={err*1000:.1f}mm")
    print(f"  mean error: {total_err / len(pairs) * 1000:.1f}mm")

    # Save
    data = {
        'A': A.tolist(),
        'b': b.tolist(),
        'z_offset': float(avg_z_offset),
        'expected_ee_z': float(EXPECTED_EE_Z),
        'cube_height': float(CUBE_HEIGHT),
        'pairs': pairs,
    }
    with open(OUTPUT_FILE, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"\nSaved to {OUTPUT_FILE}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
