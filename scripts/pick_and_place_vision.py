#!/usr/bin/env python3
"""
RA6A vision-based pick and place.

Subscribes to /detected_cubes_info (from cube_detector_node.py),
reads ~/drop_positions.json (from save_box_positions.py),
picks each detected cube and drops it in its colored box.

Pick workflow per cube:
  1. APPROACH: above cube, gripper down, NO yaw (rotation deferred)
  2. ROTATE:   spin J6 in place to match cube angle (completes before descent)
  3. DESCEND:  straight down to grasp height
  4. GRASP
  5. LIFT
  6. RETRACT

Place workflow:
  Gripper is forced to forward-horizontal orientation (parallel to ground,
  pointing in +X from the arm's POV). The saved drop quaternion is ignored.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints,
                              PositionConstraint, OrientationConstraint,
                              JointConstraint)
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
import json
import os
import time
import math
import sys

DROP_FILE = os.path.expanduser('~/drop_positions.json')
CALIB_FILE = os.path.expanduser('~/vision_calibration.json')

# Top-down approach quaternion (gripper Z = world -Z, "pointing down")
APPROACH_DOWN_QUAT = (-0.7071068, 0.7071068, 0.0, 0.0)

# Forward-horizontal approach quaternion. In this URDF, base_link +X points
# BACKWARD and -X is "forward" (the direction the arm reaches). This is the
# measured home-pose orientation: gripper parallel to the ground, pointing
# along base_link -X (forward from the arm's POV).
APPROACH_FORWARD_QUAT = (0.5, -0.5, -0.5, 0.5)

# Heights (meters)
TABLE_Z = 0.273
CUBE_HEIGHT = 0.020
GRASP_OFFSET = 0.10
Z_OFFSET_FIXED = -0.055
GRASP_Z_ADJUST = 0.000

# Approach offsets
PRE_GRASP_HEIGHT = 0.10
LIFT_HEIGHT = 0.10
DROP_PREHEIGHT = 0.10

# Gripper angles
GRIP_OPEN = 90
GRIP_CLOSE = 0
GRIP_PAUSE = 0.5
INTER_CUBE_PAUSE = 3.0

# Speed
VEL_SCALE = 0.75
ACC_SCALE = 0.45

# Joint range constraints (radians)
J1_LIMIT = 1.57
J4_LIMIT = 1.57

HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Detection
DETECTION_TIMEOUT = 5.0
STABILITY_FRAMES = 5
STABILITY_TOL = 0.01


class PickPlace(Node):
    def __init__(self):
        super().__init__('pick_place_vision')

        self.gripper_pub = self.create_publisher(Int32, '/gripper_angle', 10)
        self.create_subscription(String, '/detected_cubes_info', self._info_cb, 10)
        self.move_client = ActionClient(self, MoveGroup, '/move_action')

        if not os.path.exists(DROP_FILE):
            self.get_logger().error(f"No {DROP_FILE} — run save_box_positions.py first")
            sys.exit(1)
        with open(DROP_FILE, 'r') as f:
            self.drop_positions = json.load(f)
        self.get_logger().info(f"Loaded drops for: {list(self.drop_positions.keys())}")

        self.calib_A = None
        self.calib_b = None
        self.calib_z_offset = 0.0
        if os.path.exists(CALIB_FILE):
            import numpy as np
            with open(CALIB_FILE, 'r') as f:
                cal = json.load(f)
            self.calib_A = np.array(cal['A'])
            self.calib_b = np.array(cal['b'])
            self.calib_z_offset = float(cal.get('z_offset', 0.0))
            self.get_logger().info(
                f"Loaded vision calibration from {CALIB_FILE} "
                f"(z_offset={self.calib_z_offset:+.4f})")
        else:
            self.get_logger().warn(
                f"No {CALIB_FILE} — using raw vision (run calibrate_vision.py first for accuracy)")

        self.detection_history = []

    def _info_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.detection_history.append((time.time(), data))
            cutoff = time.time() - 2.0
            self.detection_history = [(t, d) for t, d in self.detection_history if t > cutoff]
        except Exception as e:
            self.get_logger().warn(f"Bad info: {e}")

    def get_stable_cubes(self):
        """Return cubes that have been detected stably across recent frames."""
        if len(self.detection_history) < STABILITY_FRAMES:
            return []

        recent = self.detection_history[-STABILITY_FRAMES:]

        by_color = {}
        for _, frame in recent:
            for cube in frame:
                col = cube['color']
                if col not in by_color: by_color[col] = []
                by_color[col].append(cube)

        stable = []
        for col, observations in by_color.items():
            if len(observations) < STABILITY_FRAMES * 0.6:
                continue
            xs = [o['x'] for o in observations]
            ys = [o['y'] for o in observations]
            angs = [o['angle_deg'] for o in observations]
            if (max(xs) - min(xs) > STABILITY_TOL or
                max(ys) - min(ys) > STABILITY_TOL):
                continue
            # Circular mean for angle (handles ±45° boundary wrap from the
            # detector's 90°-symmetric normalization).
            sx = sum(math.cos(math.radians(2*a)) for a in angs) / len(angs)
            sy = sum(math.sin(math.radians(2*a)) for a in angs) / len(angs)
            mean_ang = math.degrees(math.atan2(sy, sx)) / 2.0
            stable.append({
                'color': col,
                'x': sum(xs)/len(xs),
                'y': sum(ys)/len(ys),
                'angle_deg': mean_ang,
            })
        return stable

    def gripper(self, angle):
        self.gripper_pub.publish(Int32(data=int(angle)))
        time.sleep(GRIP_PAUSE)

    def move_to_pose(self, x, y, z, qx, qy, qz, qw, timeout=30.0, strict=True):
        """Plan + execute to a pose. If strict=True, the quaternion's yaw
        (rotation around the gripper approach axis) is enforced tightly.
        If strict=False, any yaw is acceptable (use only when the wrist
        angle genuinely doesn't matter)."""
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server unavailable")
            return False

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'arm'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE

        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'end_effector'
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        # Goal position tolerance — was 5mm, tightened to 1mm so the planner
        # can't put the gripper a noticeable distance from the commanded pose.
        sphere.dimensions = [0.001]
        pc.constraint_region.primitives.append(sphere)
        from geometry_msgs.msg import Pose as PoseMsg
        pose = PoseMsg()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'end_effector'
        oc.orientation.x = qx
        oc.orientation.y = qy
        oc.orientation.z = qz
        oc.orientation.w = qw
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        # Strict yaw enforcement — without this, MoveIt is free to pick any
        # rotation around the gripper's approach axis, which causes:
        #   - wrist not matching cube angle on pick
        #   - "random horizontal" gripper roll on place
        oc.absolute_z_axis_tolerance = 0.05 if strict else 3.14
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)

        jc_j1 = JointConstraint()
        jc_j1.joint_name = 'j1'
        jc_j1.position = 0.0
        jc_j1.tolerance_above = J1_LIMIT
        jc_j1.tolerance_below = J1_LIMIT
        jc_j1.weight = 1.0
        c.joint_constraints.append(jc_j1)

        jc_j4 = JointConstraint()
        jc_j4.joint_name = 'j4'
        jc_j4.position = 0.0
        jc_j4.tolerance_above = J4_LIMIT
        jc_j4.tolerance_below = J4_LIMIT
        jc_j4.weight = 1.0
        c.joint_constraints.append(jc_j4)

        req.goal_constraints.append(c)

        path_c = Constraints()
        path_jc1 = JointConstraint()
        path_jc1.joint_name = 'j1'
        path_jc1.position = 0.0
        path_jc1.tolerance_above = J1_LIMIT
        path_jc1.tolerance_below = J1_LIMIT
        path_jc1.weight = 1.0
        path_c.joint_constraints.append(path_jc1)
        path_jc4 = JointConstraint()
        path_jc4.joint_name = 'j4'
        path_jc4.position = 0.0
        path_jc4.tolerance_above = J4_LIMIT
        path_jc4.tolerance_below = J4_LIMIT
        path_jc4.weight = 1.0
        path_c.joint_constraints.append(path_jc4)
        req.path_constraints = path_c

        goal.request = req

        future = self.move_client.send_goal_async(goal)
        t0 = time.time()
        while not future.done() and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        gh = future.result()
        if not gh or not gh.accepted:
            self.get_logger().error("Goal rejected")
            return False

        result_future = gh.get_result_async()
        deadline = time.time() + timeout
        while not result_future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result_future.done():
            self.get_logger().error("Move timed out")
            return False
        return True

    def quat_yaw(self, yaw_deg):
        """Apply yaw rotation around end-effector approach axis (gripper Z)."""
        yaw_rad = math.radians(yaw_deg)
        cz = math.cos(yaw_rad / 2.0)
        sz = math.sin(yaw_rad / 2.0)
        z_quat = (0.0, 0.0, sz, cz)
        x1, y1, z1, w1 = APPROACH_DOWN_QUAT
        x2, y2, z2, w2 = z_quat
        qx = w1*x2 + x1*w2 + y1*z2 - z1*y2
        qy = w1*y2 - x1*z2 + y1*w2 + z1*x2
        qz = w1*z2 + x1*y2 - y1*x2 + z1*w2
        qw = w1*w2 - x1*x2 - y1*y2 - z1*z2
        return qx, qy, qz, qw

    def go_home(self):
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            return False
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'arm'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE

        c = Constraints()
        joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        for name, target in zip(joint_names, HOME_JOINTS):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = target
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        goal.request = req

        future = self.move_client.send_goal_async(goal)
        t0 = time.time()
        while not future.done() and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        gh = future.result()
        if not gh or not gh.accepted:
            return False
        result_future = gh.get_result_async()
        deadline = time.time() + 15.0
        while not result_future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return result_future.done()

    def pick(self, cube):
        col = cube['color']
        # Apply vision calibration if available
        vx, vy = cube['x'], cube['y']
        if self.calib_A is not None:
            import numpy as np
            corrected = self.calib_A @ np.array([vx, vy]) + self.calib_b
            cx, cy = float(corrected[0]), float(corrected[1])
            self.get_logger().info(
                f"  vision=({vx:+.3f},{vy:+.3f}) → calibrated=({cx:+.3f},{cy:+.3f})")
        else:
            cx, cy = vx, vy

        grasp_z = TABLE_Z + CUBE_HEIGHT/2 + GRASP_OFFSET + Z_OFFSET_FIXED + GRASP_Z_ADJUST

        # Normalize cube angle to [-45, 45] for minimal gripper rotation
        ang = cube['angle_deg'] % 90
        if ang > 45: ang -= 90

        # Two orientations: down (no yaw) for approach, down+yaw for the rest.
        qx0, qy0, qz0, qw0 = APPROACH_DOWN_QUAT
        qx, qy, qz, qw = self.quat_yaw(ang)

        approach_z = grasp_z + PRE_GRASP_HEIGHT + 0.05  # extra 5cm safety

        self.get_logger().info(
            f"PICK {col}: pos=({cx:+.3f},{cy:+.3f}) angle={ang:+.1f}")

        # 1. APPROACH: above cube, gripper down, NO yaw yet
        self.get_logger().info(f"  [1/6] Approach (down, no yaw) at z={approach_z:.3f}")
        if not self.move_to_pose(cx, cy, approach_z, qx0, qy0, qz0, qw0):
            return False
        self.gripper(GRIP_OPEN)

        # 2. ROTATE in place: spin J6 to match cube angle BEFORE descending
        self.get_logger().info(f"  [2/6] Rotate wrist to {ang:+.1f}° (in place)")
        if not self.move_to_pose(cx, cy, approach_z, qx, qy, qz, qw):
            return False

        # 3. DESCEND straight down with rotated wrist
        self.get_logger().info(f"  [3/6] Descend to z={grasp_z:.3f}")
        if not self.move_to_pose(cx, cy, grasp_z, qx, qy, qz, qw):
            return False

        # 4. GRASP
        self.get_logger().info(f"  [4/6] Close gripper")
        self.gripper(GRIP_CLOSE)

        # 5. LIFT
        self.get_logger().info(f"  [5/6] Lift")
        if not self.move_to_pose(cx, cy, grasp_z + LIFT_HEIGHT,
                                 qx, qy, qz, qw):
            return False

        # 6. RETRACT
        self.get_logger().info(f"  [6/6] Retract")
        if not self.move_to_pose(cx, cy, approach_z, qx, qy, qz, qw):
            return False

        return True

    def place(self, color):
        if color not in self.drop_positions:
            self.get_logger().warn(f"No drop position for {color} — dropping at home")
            return False
        drop = self.drop_positions[color]
        dp = drop['pos']
        # Force gripper to forward-horizontal orientation; ignore saved quat.
        fx, fy, fz, fw = APPROACH_FORWARD_QUAT
        self.get_logger().info(
            f"PLACE {color}: pos=({dp[0]:+.3f},{dp[1]:+.3f},{dp[2]:+.3f}) "
            f"[gripper forward-horizontal]")

        # 1. Pre-drop (above drop)
        if not self.move_to_pose(dp[0], dp[1], dp[2] + DROP_PREHEIGHT,
                                 fx, fy, fz, fw):
            return False
        # 2. Drop position
        if not self.move_to_pose(dp[0], dp[1], dp[2],
                                 fx, fy, fz, fw):
            return False
        # 3. Open gripper
        self.gripper(GRIP_OPEN)
        # 4. Lift back up
        if not self.move_to_pose(dp[0], dp[1], dp[2] + DROP_PREHEIGHT,
                                 fx, fy, fz, fw):
            return False
        return True

    def run(self):
        self.get_logger().info("Continuous mode — Ctrl+C to stop")
        self.gripper(GRIP_OPEN)

        cycle = 0
        while rclpy.ok():
            cycle += 1
            self.get_logger().info(f"\n===== Cycle {cycle}: scanning for cubes =====")

            t0 = time.time()
            cubes = []
            while time.time() - t0 < DETECTION_TIMEOUT and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
            cubes = self.get_stable_cubes()

            if not cubes:
                self.get_logger().info("No stable cubes seen, retrying in 2s...")
                time.sleep(2.0)
                continue

            self.get_logger().info(f"Found {len(cubes)} stable cube(s)")

            for cube in cubes:
                if not rclpy.ok(): break
                if cube['color'] not in self.drop_positions:
                    self.get_logger().info(f"Skipping {cube['color']} (no drop position)")
                    continue
                if self.pick(cube):
                    self.place(cube['color'])
                else:
                    self.get_logger().error(f"Pick failed for {cube['color']}")
                    self.gripper(GRIP_OPEN)
                self.get_logger().info("Returning home...")
                self.go_home()
                self.get_logger().info(f"Pausing {INTER_CUBE_PAUSE}s before next cube...")
                time.sleep(INTER_CUBE_PAUSE)

            self.detection_history = []


def main():
    rclpy.init()
    node = PickPlace()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
