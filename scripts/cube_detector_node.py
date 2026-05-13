#!/usr/bin/env python3
"""
ROS2 cube detection node — hybrid ArUco + manual marker corners.

Loads ~/manual_marker_corners.json (saved by aruco_detect.py viewer's
calibration mode). For every frame, builds a pose from whichever markers
ArUco decoded this frame, falling back to the manual corners for any
markers it missed. Pose is therefore stable even when lighting kills the
auto-decoder, but still gets subpixel accuracy when the decoder works.

Set DEBUG_VIEW=True to also pop an OpenCV window with a live annotated
feed (markers, manual corners, workspace polygon, cube poses, status).

Topics:
  /detected_cubes        geometry_msgs/PoseArray
  /detected_cubes_info   std_msgs/String  (color/x/y/z/angle JSON)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
import cv2
import numpy as np
import yaml
import math
import json
import os
import sys
import time

# ---- Config ----
IPHONE_IP = '192.168.100.206'
URL = f'http://{IPHONE_IP}:4747/video'

CALIB_FILE = os.path.expanduser('~/camera_calibration.yaml')
MANUAL_CAL_FILE = os.path.expanduser('~/manual_marker_corners.json')
COLOR_FILE = os.path.expanduser('~/color_thresholds.json')
ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_SIZE_M = 0.050

MARKER_POSITIONS_BASE = {
    3: np.array([-0.370,  0.164, 0.273]),
    2: np.array([-0.370, -0.268, 0.273]),
    1: np.array([-0.190,  0.164, 0.273]),
    0: np.array([-0.190, -0.268, 0.273]),
}
TABLE_Z = 0.273

COLORS = {
    'red':   {'lower1': (0,   180, 130), 'upper1': (8,   255, 255),
              'lower2': (172, 180, 130), 'upper2': (180, 255, 255),
              'bgr':    (0, 0, 255)},
    'green': {'lower1': (40,  80,  50),  'upper1': (85,  255, 255),
              'bgr':    (0, 255, 0)},
    'blue':  {'lower1': (100, 100, 50),  'upper1': (130, 255, 255),
              'bgr':    (255, 0, 0)},
    'yellow':{'lower1': (22,  150, 150), 'upper1': (32,  255, 255),
              'bgr':    (0, 255, 255)},
}

MIN_CUBE_AREA   = 300
MAX_CUBE_AREA   = 15000
MIN_SQUARENESS  = 0.7
MIN_ASPECT      = 0.6
PUBLISH_RATE_HZ = 10

# Workspace = exact rectangle bounded by the four marker CENTERS.
WORKSPACE_X_MIN, WORKSPACE_X_MAX = -0.370, -0.190
WORKSPACE_Y_MIN, WORKSPACE_Y_MAX = -0.268, +0.164

# Low-pass filter on cube positions (per color)
EMA_ALPHA    = 0.4
CUBE_TIMEOUT = 0.5  # seconds without a sighting -> reset the filter

# Live annotated feed
DEBUG_VIEW = True
DEBUG_WINDOW = 'cube_detector'


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')

        with open(CALIB_FILE, 'r') as f:
            data = yaml.safe_load(f)
        self.K = np.array(data['camera_matrix'])
        self.D = np.array(data['distortion_coefficients'])

        # Load saved color thresholds (overrides defaults in COLORS).
        if os.path.exists(COLOR_FILE):
            try:
                with open(COLOR_FILE, 'r') as f:
                    d = json.load(f)
                for cname, cfg in d.items():
                    if cname not in COLORS: continue
                    for k in ('lower1', 'upper1', 'lower2', 'upper2'):
                        if k in cfg:
                            COLORS[cname][k] = tuple(cfg[k])
                self.get_logger().info(
                    f"Loaded color thresholds from {COLOR_FILE}")
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to load {COLOR_FILE}: {e}")

        self.aruco_dict   = cv2.aruco.Dictionary_get(ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.adaptiveThreshWinSizeMin   = 3
        self.aruco_params.adaptiveThreshWinSizeMax   = 53
        self.aruco_params.adaptiveThreshWinSizeStep  = 10
        self.aruco_params.cornerRefinementMethod     = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize    = 5
        self.aruco_params.cornerRefinementMinAccuracy = 0.05
        self.aruco_params.errorCorrectionRate        = 0.8
        self.aruco_params.polygonalApproxAccuracyRate = 0.05
        self.aruco_params.minOtsuStdDev              = 3.0
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

        # Manual marker corners (from viewer calibration)
        self.manual_corners = self._load_manual_corners()
        if self.manual_corners:
            self.get_logger().info(
                f"Loaded manual corners for markers: "
                f"{sorted(self.manual_corners.keys())}")
        else:
            self.get_logger().warn(
                f"No {MANUAL_CAL_FILE} — node depends entirely on auto "
                f"detection. Run aruco_detect.py and press 'k' to calibrate.")

        self.last_pose = None

        # Per-color cube EMA filter: {color: {'x','y','a','t'}}
        self.cube_filter = {}

        self.get_logger().info(f"Connecting to {URL}...")
        self.cap = cv2.VideoCapture(URL)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open stream!")
            sys.exit(1)

        self.pose_pub = self.create_publisher(PoseArray, '/detected_cubes', 10)
        self.info_pub = self.create_publisher(String, '/detected_cubes_info', 10)

        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.tick)
        self.get_logger().info("Detector running. Publishing /detected_cubes")

    # ---------- manual corner I/O ----------
    def _load_manual_corners(self):
        if not os.path.exists(MANUAL_CAL_FILE):
            return {}
        try:
            with open(MANUAL_CAL_FILE, 'r') as f:
                d = json.load(f)
            return {int(k): np.array(v, dtype=np.float32) for k, v in d.items()}
        except Exception as e:
            self.get_logger().warn(f"Failed to load manual corners: {e}")
            return {}

    # ---------- pose ----------
    def solve_pose_hybrid(self, detected_dict):
        """detected_dict: {id: 4x2 corner array}. Combines with manual."""
        obj_points, img_points = [], []
        s = MARKER_SIZE_M / 2.0
        local = np.array([[-s,  s, 0], [ s,  s, 0],
                          [ s, -s, 0], [-s, -s, 0]])
        used, n_det, n_man = set(), 0, 0
        for mid, c4 in detected_dict.items():
            if mid not in MARKER_POSITIONS_BASE: continue
            ctr = MARKER_POSITIONS_BASE[mid]
            for k in range(4):
                obj_points.append(ctr + local[k])
                img_points.append(c4[k])
            used.add(mid); n_det += 1
        for mid, c4 in self.manual_corners.items():
            if mid in used or mid not in MARKER_POSITIONS_BASE: continue
            ctr = MARKER_POSITIONS_BASE[mid]
            for k in range(4):
                obj_points.append(ctr + local[k])
                img_points.append(c4[k])
            n_man += 1
        if len(obj_points) < 4: return None
        obj_points = np.array(obj_points, dtype=np.float32)
        img_points = np.array(img_points, dtype=np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.K, self.D,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok: return None
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec.flatten(), n_det, n_man

    def pixel_to_table(self, px, py, R, t):
        u = cv2.undistortPoints(np.array([[[px, py]]], dtype=np.float32),
                                self.K, self.D, P=self.K)
        upx, upy = u[0, 0]
        ray_cam = np.linalg.inv(self.K) @ np.array([upx, upy, 1.0])
        ray_cam /= np.linalg.norm(ray_cam)
        ray_base = R.T @ ray_cam
        cam_pos = -R.T @ t
        if abs(ray_base[2]) < 1e-6: return None
        s = (TABLE_Z - cam_pos[2]) / ray_base[2]
        if s < 0: return None
        return cam_pos + s * ray_base

    # ---------- cube detection ----------
    def find_cubes(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        results, masks = [], {}
        for cname, cfg in COLORS.items():
            mask = cv2.inRange(hsv, np.array(cfg['lower1']), np.array(cfg['upper1']))
            if 'lower2' in cfg:
                m2 = cv2.inRange(hsv, np.array(cfg['lower2']), np.array(cfg['upper2']))
                mask = mask | m2
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
            masks[cname] = mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < MIN_CUBE_AREA or area > MAX_CUBE_AREA: continue
                rect = cv2.minAreaRect(c)
                (cx, cy), (w, h), _ = rect
                if w < 5 or h < 5: continue
                if min(w, h) / max(w, h) < MIN_ASPECT: continue
                if w * h < 1: continue
                if area / (w * h) < MIN_SQUARENESS: continue
                box = cv2.boxPoints(rect)
                results.append((cname, (cx, cy), box))
        return results, masks

    # ---------- low-pass filter ----------
    def filter_cube(self, color, x, y, ang):
        now = time.time()
        f = self.cube_filter.get(color)
        if f is None or now - f['t'] > CUBE_TIMEOUT:
            self.cube_filter[color] = {'x': x, 'y': y, 'a': ang, 't': now}
        else:
            f['x'] = (1 - EMA_ALPHA) * f['x'] + EMA_ALPHA * x
            f['y'] = (1 - EMA_ALPHA) * f['y'] + EMA_ALPHA * y
            # Circular EMA on 90°-symmetric angle
            pc = math.cos(math.radians(2 * f['a']))
            ps = math.sin(math.radians(2 * f['a']))
            nc = math.cos(math.radians(2 * ang))
            ns = math.sin(math.radians(2 * ang))
            fc = (1 - EMA_ALPHA) * pc + EMA_ALPHA * nc
            fs = (1 - EMA_ALPHA) * ps + EMA_ALPHA * ns
            f['a'] = math.degrees(math.atan2(fs, fc)) / 2.0
            f['t'] = now
        f = self.cube_filter[color]
        return f['x'], f['y'], f['a']

    # ---------- main loop ----------
    def tick(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame read failed", throttle_duration_sec=2.0)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_proc = self.clahe.apply(gray)

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray_proc, self.aruco_dict, parameters=self.aruco_params)
        n_markers = 0 if ids is None else len(ids)

        detected_dict = {}
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                detected_dict[int(mid)] = corners[i][0]

        # Hybrid pose
        R, t, n_det, n_man = None, None, 0, 0
        pose_status = "no data"
        if detected_dict or self.manual_corners:
            res = self.solve_pose_hybrid(detected_dict)
            if res is not None:
                R, t, n_det, n_man = res
                self.last_pose = (R, t)
                pose_status = f"hybrid det={n_det} man={n_man}"
            else:
                pose_status = "PnP failed"

        if R is None and self.last_pose is not None:
            R, t = self.last_pose
            pose_status = "cached"

        cubes, masks = self.find_cubes(frame)
        self.get_logger().info(
            f"Markers det={n_det} man={n_man}  Cubes={len(cubes)}",
            throttle_duration_sec=1.0)

        # Build PoseArray + info JSON, with EMA-filtered positions
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        info = []
        seen_colors = set()
        published_cubes = []  # for debug overlay

        if R is not None:
            for cname, (cx, cy), box in cubes:
                pt = self.pixel_to_table(cx, cy, R, t)
                if pt is None: continue
                bx, by, bz = float(pt[0]), float(pt[1]), float(pt[2])

                a = self.pixel_to_table(box[0][0], box[0][1], R, t)
                b = self.pixel_to_table(box[1][0], box[1][1], R, t)
                if a is not None and b is not None:
                    raw_ang = math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
                    raw_ang = ((raw_ang + 45.0) % 90.0) - 45.0
                else:
                    raw_ang = 0.0

                # EMA filter — only on the first cube of each color, since
                # the filter is keyed by color and assumes one per color.
                if cname not in seen_colors:
                    fx, fy, fa = self.filter_cube(cname, bx, by, raw_ang)
                    seen_colors.add(cname)
                else:
                    fx, fy, fa = bx, by, raw_ang

                if not (WORKSPACE_X_MIN <= fx <= WORKSPACE_X_MAX and
                        WORKSPACE_Y_MIN <= fy <= WORKSPACE_Y_MAX):
                    published_cubes.append((cname, (cx, cy), box, fx, fy, fa, False))
                    continue

                p = Pose()
                p.position.x = fx
                p.position.y = fy
                p.position.z = bz
                p.orientation.w = 1.0
                msg.poses.append(p)
                info.append({'color': cname,
                             'x': fx, 'y': fy, 'z': bz,
                             'angle_deg': float(fa)})
                published_cubes.append((cname, (cx, cy), box, fx, fy, fa, True))

        if msg.poses:
            self.pose_pub.publish(msg)
            self.info_pub.publish(String(data=json.dumps(info)))

        # Drop stale cubes from filter
        now = time.time()
        for c in list(self.cube_filter.keys()):
            if now - self.cube_filter[c]['t'] > CUBE_TIMEOUT:
                del self.cube_filter[c]

        # ---------- live debug feed ----------
        if DEBUG_VIEW:
            self._draw_debug(frame, gray, R, t, ids, corners, rejected,
                             published_cubes, pose_status, n_markers)

    # ---------- debug rendering ----------
    def _draw_debug(self, frame, gray, R, t, ids, corners, rejected,
                    published_cubes, pose_status, n_markers):
        disp = frame.copy()
        h, w = disp.shape[:2]

        # Workspace polygon
        if R is not None:
            rvec, _ = cv2.Rodrigues(R)
            pts = np.array([
                [WORKSPACE_X_MIN, WORKSPACE_Y_MIN, TABLE_Z],
                [WORKSPACE_X_MIN, WORKSPACE_Y_MAX, TABLE_Z],
                [WORKSPACE_X_MAX, WORKSPACE_Y_MAX, TABLE_Z],
                [WORKSPACE_X_MAX, WORKSPACE_Y_MIN, TABLE_Z],
            ], dtype=np.float32)
            ip, _ = cv2.projectPoints(pts, rvec, t, self.K, self.D)
            cv2.polylines(disp, [ip.reshape(-1, 2).astype(int)],
                          True, (0, 255, 255), 2)

        # Detected markers (green)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(disp, corners, ids, (0, 255, 0))

        # Manual corners (orange) — only for ones ArUco missed
        detected_ids = set() if ids is None else {int(i) for i in ids.flatten()}
        for mid, c4 in self.manual_corners.items():
            if mid in detected_ids: continue
            pts = c4.astype(int)
            cv2.polylines(disp, [pts], True, (0, 165, 255), 2)
            ctr_pix = pts.mean(axis=0).astype(int)
            cv2.putText(disp, f"M{mid}", tuple(ctr_pix),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

        # Cubes
        for cname, (cx, cy), box, bx, by, ba, in_ws in published_cubes:
            bgr = COLORS[cname]['bgr'] if in_ws else (140, 140, 140)
            cv2.drawContours(disp, [box.astype(int)], 0, bgr, 2)
            cv2.circle(disp, (int(cx), int(cy)), 4, bgr, -1)
            label = f"{cname} ({bx:+.2f},{by:+.2f}) {ba:+.0f}d"
            if not in_ws: label += " OOB"
            cv2.putText(disp, label, (int(cx) + 8, int(cy)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(disp, label, (int(cx) + 8, int(cy)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1)

        # Stats overlay (small)
        man_str = ','.join(str(i) for i in sorted(self.manual_corners.keys())) or '-'
        lines = [
            f"Mk det:{n_markers}",
            f"Pose:{pose_status}",
            f"Manual:[{man_str}]",
            f"Cubes pub:{sum(1 for c in published_cubes if c[6])}",
        ]
        for i, line in enumerate(lines):
            y = 14 + i * 16
            cv2.putText(disp, line, (8, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
            cv2.putText(disp, line, (8, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        cv2.imshow(DEBUG_WINDOW, disp)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    if DEBUG_VIEW:
        cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
