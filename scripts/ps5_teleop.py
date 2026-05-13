#!/usr/bin/env python3
"""
RA6A PS5 Teleop — AR4 style using joy_node + MoveIt Servo.

Terminal 1:  ros2 launch ra6a_moveit_config servo.launch.py
Terminal 2:  ros2 run joy joy_node
Terminal 3:  python3 ps5_teleop.py

Calibration:
  - Axis ZERO calibration (sticks/triggers) runs every time, since
    resting values drift slightly from session to session.
  - Button MAPPING is saved to ~/ps5_teleop_mapping.json after the
    first successful run and reused automatically thereafter. Press
    'r' + ENTER at the prompt to redo the mapping if needed, or
    delete the file.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg
import time
import threading
import math
import sys
import os
import json

JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
TWIST_TOPIC = '/servo_node/delta_twist_cmds'
PLANNING_FRAME = 'base_link'
JTC_ACTION = '/arm_controller/follow_joint_trajectory'

LIN_SCALE  = 0.1
ROT_SCALE  = 0.5
DEADZONE   = 0.1
GRIP_STEP  = 5
GRIP_START = 26

# Saved button mapping (axis-zero calibration is always re-run).
CONFIG_FILE = os.path.expanduser('~/ps5_teleop_mapping.json')


class PS5Teleop(Node):
    def __init__(self):
        super().__init__('ps5_teleop')

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        self.gripper_pub = self.create_publisher(Int32, '/gripper_angle', 10)
        self.jtc_client = ActionClient(self, FollowJointTrajectory, JTC_ACTION)

        self.latest_joy = None
        self.current_joints = [0.0] * 6
        self.mapping = {}
        self.axis_zero = []
        self.servo_angle = GRIP_START
        self.saved = []
        self.stopped = False
        self.busy = False
        self.prev_buttons = {}
        self.debug_counter = 0

    def _joy_cb(self, msg):
        self.latest_joy = msg

    def _js_cb(self, msg):
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_joints[i] = msg.position[idx]

    # ── Mapping persistence ──

    def _load_saved_mapping(self):
        """Try to load button mapping from CONFIG_FILE. Returns dict or None."""
        if not os.path.exists(CONFIG_FILE):
            return None
        try:
            with open(CONFIG_FILE, 'r') as f:
                d = json.load(f)
            # JSON has lists; convert back to tuples (keep None as-is)
            return {k: tuple(v) if v else None for k, v in d.items()}
        except Exception as e:
            print(f"  Failed to load {CONFIG_FILE}: {e}")
            return None

    def _save_mapping(self):
        try:
            d = {k: list(v) if v else None for k, v in self.mapping.items()}
            with open(CONFIG_FILE, 'w') as f:
                json.dump(d, f, indent=2)
            print(f"  Saved mapping to {CONFIG_FILE}")
        except Exception as e:
            print(f"  Failed to save mapping: {e}")

    # ── Start Servo ──

    def start_servo(self):
        self.get_logger().info("Starting MoveIt Servo...")
        cli = self.create_client(Trigger, '/servo_node/start_servo')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("start_servo service not found!")
            return False
        future = cli.call_async(Trigger.Request())
        t0 = time.time()
        while not future.done() and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        if future.done() and future.result().success:
            self.get_logger().info("MoveIt Servo started!")
            return True
        self.get_logger().error("Failed to start Servo")
        return False

    # ── Calibration ──

    def calibrate(self):
        print("\n" + "=" * 55)
        print("  RA6A PS5 CONTROLLER CALIBRATION")
        print("=" * 55)
        print("\n  Waiting for controller...")
        print("  (Make sure: ros2 run joy joy_node)")

        t0 = time.time()
        while self.latest_joy is None and time.time() - t0 < 30:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joy is None:
            print("  ERROR: No /joy messages! Run: ros2 run joy joy_node")
            return False

        na = len(self.latest_joy.axes)
        nb = len(self.latest_joy.buttons)
        print(f"  Found! Axes: {na}, Buttons: {nb}")

        # ── STEP 1: Always run axis-zero calibration (drift compensation) ──
        print("\n  STEP 1: Keep sticks centered, don't touch anything.")
        input("  Press ENTER... ")
        rclpy.spin_once(self, timeout_sec=0.1)
        self.axis_zero = list(self.latest_joy.axes) if self.latest_joy else [0.0]*na
        print(f"  Zeroed {na} axes.\n")

        # ── STEP 2: Try to reuse a saved button mapping ──
        saved = self._load_saved_mapping()
        if saved is not None:
            print(f"  Loaded button mapping from {CONFIG_FILE}")
            print("  Press 'r' + ENTER to remap, or just ENTER to use it.")
            choice = input("  > ").strip().lower()
            if choice != 'r':
                self.mapping = saved
                print("\n  Final mapping (loaded):")
                for k, v in self.mapping.items():
                    if v: print(f"    {k}: {v}")
                print("\n  Calibration complete (mapping reused).")
                print("=" * 55)
                return True
            print("  Remapping...\n")

        # ── STEP 2 (interactive): map every control ──
        print("  STEP 2: Map each control.\n")

        def spin():
            rclpy.spin_once(self, timeout_sec=0.02)

        def wait_release():
            time.sleep(0.3)
            while True:
                spin()
                if not self.latest_joy: break
                axes_ok = all(abs(self.latest_joy.axes[j] - self.axis_zero[j]) < 0.3
                              for j in range(na))
                btns_ok = not any(self.latest_joy.buttons)
                if axes_ok and btns_ok: break
            time.sleep(0.15)

        def detect_axis(prompt, timeout=15):
            print(f"  {prompt}")
            time.sleep(0.4)
            t0 = time.time()
            while time.time() - t0 < timeout:
                spin()
                if not self.latest_joy: continue
                for i in range(na):
                    v = self.latest_joy.axes[i] - self.axis_zero[i]
                    if abs(v) > 0.5:
                        d = 1 if v > 0 else -1
                        print(f"    -> Axis {i} ({'+'if d>0 else '-'})")
                        wait_release()
                        return ('axis', i, d)
            print("    -> TIMEOUT"); return None

        def detect_trigger(prompt, timeout=15):
            print(f"  {prompt}")
            time.sleep(0.4)
            t0 = time.time()
            while time.time() - t0 < timeout:
                spin()
                if not self.latest_joy: continue
                for i in range(na):
                    v = self.latest_joy.axes[i] - self.axis_zero[i]
                    if abs(v) > 0.5:
                        d = 1 if v > 0 else -1
                        print(f"    -> Trigger axis {i}")
                        wait_release()
                        return ('axis', i, d)
                for i in range(nb):
                    if self.latest_joy.buttons[i]:
                        print(f"    -> Button {i}")
                        wait_release()
                        return ('button', i, 1)
            print("    -> TIMEOUT"); return None

        def detect_button(prompt, timeout=15):
            print(f"  {prompt}")
            time.sleep(0.4)
            t0 = time.time()
            while time.time() - t0 < timeout:
                spin()
                if not self.latest_joy: continue
                for i in range(nb):
                    if self.latest_joy.buttons[i]:
                        print(f"    -> Button {i}")
                        wait_release()
                        return ('button', i, 1)
            print("    -> TIMEOUT"); return None

        m = self.mapping
        m.clear()
        m['ls_y'] = detect_axis("Move LEFT STICK UP...")
        m['ls_x'] = detect_axis("Move LEFT STICK RIGHT...")
        m['rs_y'] = detect_axis("Move RIGHT STICK UP...")
        m['rs_x'] = detect_axis("Move RIGHT STICK RIGHT...")
        m['l1']   = detect_trigger("Press L1 (up)...")
        m['l2']   = detect_trigger("Press L2 (down)...")
        m['r1']   = detect_trigger("Press R1 (orient +)...")
        m['r2']   = detect_trigger("Press R2 (orient -)...")
        m['triangle'] = detect_button("Press TRIANGLE (save)...")
        m['circle']   = detect_button("Press CIRCLE (execute)...")
        m['square']   = detect_button("Press SQUARE (stop)...")
        m['l3']       = detect_button("Press L3 (home)...")
        m['r3']       = detect_button("Press R3 (clear saves)...")
        m['dpad_up']  = detect_axis("Press D-PAD UP (gripper +)...")
        m['dpad_down'] = detect_axis("Press D-PAD DOWN (gripper -)...")

        # Persist the mapping for next runs
        self._save_mapping()

        # Print final mapping
        print("\n  Final mapping:")
        for k, v in m.items():
            if v: print(f"    {k}: {v}")
        print("\n  Calibration complete!")
        print("=" * 55)
        return True

    # ── Input reading ──

    def _read_axis(self, key):
        m = self.mapping.get(key)
        if not m or not self.latest_joy or m[0] != 'axis': return 0.0
        v = (self.latest_joy.axes[m[1]] - self.axis_zero[m[1]]) * m[2]
        if abs(v) < DEADZONE: return 0.0
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - DEADZONE) / (1.0 - DEADZONE)

    def _read_trigger(self, key):
        m = self.mapping.get(key)
        if not m or not self.latest_joy: return 0.0
        if m[0] == 'axis':
            v = (self.latest_joy.axes[m[1]] - self.axis_zero[m[1]]) * m[2]
            if abs(v) < DEADZONE: return 0.0
            return max(0.0, min(1.0, abs(v)))
        elif m[0] == 'button':
            return 1.0 if self.latest_joy.buttons[m[1]] else 0.0
        return 0.0

    def _btn_edge(self, key):
        m = self.mapping.get(key)
        if not m or not self.latest_joy: return False
        if m[0] == 'button':
            cur = bool(self.latest_joy.buttons[m[1]])
        elif m[0] == 'axis':
            raw = (self.latest_joy.axes[m[1]] - self.axis_zero[m[1]]) * m[2]
            cur = raw > 0.5
        else:
            cur = False
        prev = self.prev_buttons.get(key, False)
        self.prev_buttons[key] = cur
        return cur and not prev

    # ── Main loop ──

    def run_loop(self):
        if not self.latest_joy: return

        if self._btn_edge('square'):
            self.stopped = not self.stopped
            self.get_logger().info("STOPPED" if self.stopped else "Resumed")

        if self._btn_edge('triangle') and not self.busy:
            degs = [math.degrees(r) for r in self.current_joints]
            self.saved.append((list(self.current_joints), self.servo_angle))
            self.get_logger().info(
                f"Saved #{len(self.saved)}: [{', '.join(f'{d:.1f}' for d in degs)}] servo={self.servo_angle}")

        if self._btn_edge('l3') and not self.busy:
            threading.Thread(target=self._go_home, daemon=True).start()

        if self._btn_edge('r3'):
            self.saved.clear()
            self.get_logger().info("Saved positions cleared")

        if self._btn_edge('circle') and self.saved and not self.busy:
            threading.Thread(target=self._execute_saved, daemon=True).start()

        if self._btn_edge('dpad_up'):
            self.servo_angle = min(180, self.servo_angle + GRIP_STEP)
            self.gripper_pub.publish(Int32(data=self.servo_angle))
            self.get_logger().info(f"Servo: {self.servo_angle}")

        if self._btn_edge('dpad_down'):
            self.servo_angle = max(0, self.servo_angle - GRIP_STEP)
            self.gripper_pub.publish(Int32(data=self.servo_angle))
            self.get_logger().info(f"Servo: {self.servo_angle}")

        if not self.stopped and not self.busy:
            ls_y = self._read_axis('ls_y')
            ls_x = self._read_axis('ls_x')
            rs_y = self._read_axis('rs_y')
            rs_x = self._read_axis('rs_x')
            l1   = self._read_trigger('l1')
            l2   = self._read_trigger('l2')
            r1   = self._read_trigger('r1')
            r2   = self._read_trigger('r2')

            t = TwistStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = PLANNING_FRAME
            t.twist.linear.x  = ls_y * LIN_SCALE
            t.twist.linear.y  = ls_x * LIN_SCALE
            t.twist.linear.z  = (l1 - l2) * LIN_SCALE
            t.twist.angular.x = rs_x * ROT_SCALE
            t.twist.angular.y = rs_y * ROT_SCALE
            t.twist.angular.z = (r1 - r2) * ROT_SCALE

            self.twist_pub.publish(t)

            self.debug_counter += 1
            if self.debug_counter >= 50:
                self.debug_counter = 0
                if any(abs(v) > 0.01 for v in [ls_y,ls_x,rs_y,rs_x,l1,l2,r1,r2]):
                    self.get_logger().info(
                        f"LS({ls_y:.2f},{ls_x:.2f}) RS({rs_y:.2f},{rs_x:.2f}) "
                        f"L1={l1:.2f} L2={l2:.2f} R1={r1:.2f} R2={r2:.2f}")

    # ── JTC (always from threads) ──

    def _go_home(self):
        self.busy = True
        self.get_logger().info("Homing...")
        self._send_jtc([0.0] * 6, 4.0)
        self.get_logger().info("Home done.")
        self.busy = False

    def _execute_saved(self):
        self.busy = True
        self.get_logger().info(f"Executing {len(self.saved)} positions...")
        for i, (joints, grip) in enumerate(self.saved):
            self.get_logger().info(f"  [{i+1}/{len(self.saved)}]")
            self.servo_angle = grip
            self.gripper_pub.publish(Int32(data=grip))
            time.sleep(0.3)
            self._send_jtc(joints, 3.0)
            if i < len(self.saved) - 1:
                time.sleep(2.0)
        self.get_logger().info("Done.")
        self.busy = False

    def _send_jtc(self, positions, duration):
        if not self.jtc_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("JTC not available!")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.velocities = [0.0] * 6
        s = int(duration)
        ns = int((duration - s) * 1e9)
        pt.time_from_start = DurationMsg(sec=s, nanosec=ns)
        goal.trajectory.points = [pt]
        future = self.jtc_client.send_goal_async(goal)
        while not future.done(): time.sleep(0.05)
        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("JTC rejected!")
            return
        result_future = gh.get_result_async()
        deadline = time.time() + duration + 10
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.05)


def main():
    rclpy.init()
    node = PS5Teleop()
    node.start_servo()

    if not node.calibrate():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print("\n  CONTROLS:")
    print("  Left stick    : Cartesian forward/back, left/right")
    print("  L1 / L2       : Cartesian up / down")
    print("  Right stick   : orientation roll / pitch")
    print("  R1 / R2       : orientation yaw")
    print("  Triangle       : save position + servo angle")
    print("  Circle         : execute saved (2s between)")
    print("  Square         : toggle STOP")
    print("  L3             : home (all zeros)")
    print("  R3             : clear saved positions")
    print("  D-pad up/down  : servo angle +/- 5")
    print("\n  Running! Ctrl+C to quit.\n")

    node.create_timer(1.0 / 50.0, node.run_loop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    print("  Done.")


if __name__ == '__main__':
    main()
