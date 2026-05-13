#!/usr/bin/env python3
"""
Box position saver.

While running, listens for key presses. When you press a color key,
reads the current end-effector pose from TF and saves it to a JSON file.

Workflow:
  1. Run this script (keep it open)
  2. Use PS5 teleop (or plan-execute) to position the gripper EXACTLY
     above where you want to drop a colored cube
  3. Press the key for that color (r/g/b/y)
  4. Repeat for all colors
  5. Press Q to save and quit

Output: ~/drop_positions.json
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import json
import os
import sys
import termios
import tty
import select

OUTPUT_FILE = os.path.expanduser('~/drop_positions.json')
EE_FRAME = 'end_effector'
BASE_FRAME = 'base_link'

KEYS = {
    'r': 'red',
    'g': 'green',
    'b': 'blue',
    'y': 'yellow',
}


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if r:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class BoxSaver(Node):
    def __init__(self):
        super().__init__('box_saver')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.saved = {}
        # Load existing if any
        if os.path.exists(OUTPUT_FILE):
            with open(OUTPUT_FILE, 'r') as f:
                self.saved = json.load(f)
            self.get_logger().info(f"Loaded existing: {list(self.saved.keys())}")

    def get_ee_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, EE_FRAME,
                rclpy.time.Time(),
                Duration(seconds=1.0))
            t = tf.transform.translation
            q = tf.transform.rotation
            return {
                'pos':  [float(t.x), float(t.y), float(t.z)],
                'quat': [float(q.x), float(q.y), float(q.z), float(q.w)],
            }
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def save(self):
        with open(OUTPUT_FILE, 'w') as f:
            json.dump(self.saved, f, indent=2)
        self.get_logger().info(f"Saved to {OUTPUT_FILE}")


def main():
    rclpy.init()
    node = BoxSaver()

    print("\n" + "=" * 55)
    print("  BOX POSITION SAVER")
    print("=" * 55)
    print("  Move arm to drop pose using teleop, then:")
    print("    R = save RED box position")
    print("    G = save GREEN box position")
    print("    B = save BLUE box position")
    print("    Y = save YELLOW box position")
    print("    L = list saved positions")
    print("    D = delete a position")
    print("    Q = save & quit")
    print("=" * 55)
    print(f"  Output: {OUTPUT_FILE}\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            k = get_key().lower()
            if not k: continue

            if k == 'q':
                break

            if k == 'l':
                if not node.saved:
                    print("  No positions saved yet.")
                else:
                    print("  Saved:")
                    for color, p in node.saved.items():
                        pos = p['pos']
                        print(f"    {color}: ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})")
                continue

            if k == 'd':
                print("  Press color key to delete (r/g/b/y):", end=' ', flush=True)
                k2 = ''
                while not k2:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    k2 = get_key().lower()
                if k2 in KEYS and KEYS[k2] in node.saved:
                    del node.saved[KEYS[k2]]
                    print(f"\n  Deleted {KEYS[k2]}")
                else:
                    print(f"\n  Nothing to delete for '{k2}'")
                continue

            if k in KEYS:
                color = KEYS[k]
                pose = node.get_ee_pose()
                if pose:
                    node.saved[color] = pose
                    p = pose['pos']
                    print(f"  Saved {color}: pos=({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})")
                else:
                    print(f"  Failed to read EE pose. Is the arm running?")

    except KeyboardInterrupt:
        pass

    node.save()
    node.destroy_node()
    rclpy.shutdown()
    print("\nDone.")


if __name__ == '__main__':
    main()
