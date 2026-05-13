#!/usr/bin/env python3
"""
ArUco + cube detection diagnostic viewer.

Keys:
  q               quit
  c               toggle CLAHE
  m / r           toggle mask / threshold windows
  + / -           CLAHE clip limit
  s               save snapshot

  Pose:
  L / U           lock / unlock current pose
  k               calibrate ALL markers (16 clicks: M0 TL,TR,BR,BL ; M1 ; M2 ; M3)
  e then 0..3     redo a SINGLE marker (4 clicks for that marker only)
  K               clear manual calibration

  Color (red):
  1 / 2           S_min -/+ ; 3 / 4   V_min -/+

Mouse: left-click in main window samples HSV (or records corner in cal mode).
Manual cal saved to ~/manual_marker_corners.json and auto-loaded next run.
"""

import cv2
import numpy as np
import yaml
import math
import time
import sys
import os
import json

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
CUBE_HEIGHT = 0.020   # cube edge length, used to project silhouette center
                      # to mid-cube height instead of table level
ALL_MARKERS = [0, 1, 2, 3]
CORNER_NAMES = ['TL', 'TR', 'BR', 'BL']

COLORS = {
    'red':   {'lower1': [0,   180, 130], 'upper1': [8,   255, 255],
              'lower2': [172, 180, 130], 'upper2': [180, 255, 255],
              'bgr':    (0, 0, 255)},
    'green': {'lower1': [40,  80,  50],  'upper1': [85,  255, 255],
              'bgr':    (0, 255, 0)},
    'blue':  {'lower1': [100, 100, 50],  'upper1': [130, 255, 255],
              'bgr':    (255, 0, 0)},
    'yellow':{'lower1': [22,  150, 150], 'upper1': [32,  255, 255],
              'bgr':    (0, 255, 255)},
}

MIN_CUBE_AREA   = 300
MAX_CUBE_AREA   = 15000
MIN_SQUARENESS  = 0.7
MIN_ASPECT      = 0.6

# Workspace = exact rectangle bounded by the four marker CENTERS.
WORKSPACE_X_MIN, WORKSPACE_X_MAX = -0.370, -0.190
WORKSPACE_Y_MIN, WORKSPACE_Y_MAX = -0.268, +0.164

POSE_CACHE_FRAMES = 10

# Low-pass filter
EMA_ALPHA   = 0.4    # per-frame mixing weight for new measurement
CUBE_TIMEOUT = 0.5   # seconds — reset filter if cube unseen for this long


def setup_aruco():
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT)
    p = cv2.aruco.DetectorParameters_create()
    p.adaptiveThreshWinSizeMin   = 3
    p.adaptiveThreshWinSizeMax   = 53
    p.adaptiveThreshWinSizeStep  = 10
    p.cornerRefinementMethod     = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize    = 5
    p.cornerRefinementMinAccuracy = 0.05
    p.errorCorrectionRate        = 0.8
    p.polygonalApproxAccuracyRate = 0.05
    p.minOtsuStdDev              = 3.0
    return aruco_dict, p


def solve_pose_hybrid(detected_dict, manual_corners, K, D):
    obj_points, img_points = [], []
    s = MARKER_SIZE_M / 2.0
    local = np.array([[-s,  s, 0], [ s,  s, 0],
                      [ s, -s, 0], [-s, -s, 0]])
    used, n_det, n_man = set(), 0, 0
    for mid, c4 in detected_dict.items():
        if mid not in MARKER_POSITIONS_BASE:
            continue
        ctr = MARKER_POSITIONS_BASE[mid]
        for k in range(4):
            obj_points.append(ctr + local[k])
            img_points.append(c4[k])
        used.add(mid); n_det += 1
    for mid, c4 in manual_corners.items():
        if mid in used or mid not in MARKER_POSITIONS_BASE:
            continue
        ctr = MARKER_POSITIONS_BASE[mid]
        for k in range(4):
            obj_points.append(ctr + local[k])
            img_points.append(c4[k])
        n_man += 1
    if len(obj_points) < 4:
        return None
    obj_points = np.array(obj_points, dtype=np.float32)
    img_points = np.array(img_points, dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, K, D,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.flatten(), n_det, n_man


def pixel_to_table(px, py, K, D, R, t, z_plane=TABLE_Z):
    """Cast a camera ray through pixel (px, py) and intersect a horizontal
    plane at z = z_plane. Default is the table top. For cube xy detection
    pass z_plane = TABLE_Z + CUBE_HEIGHT/2 — the colored mask is the whole
    cube silhouette (top + visible sides) and its image-space center
    projects accurately onto mid-cube height, not the table."""
    u = cv2.undistortPoints(np.array([[[px, py]]], dtype=np.float32),
                            K, D, P=K)
    upx, upy = u[0, 0]
    ray_cam = np.linalg.inv(K) @ np.array([upx, upy, 1.0])
    ray_cam /= np.linalg.norm(ray_cam)
    ray_base = R.T @ ray_cam
    cam_pos  = -R.T @ t
    if abs(ray_base[2]) < 1e-6:
        return None
    s = (z_plane - cam_pos[2]) / ray_base[2]
    if s < 0:
        return None
    return cam_pos + s * ray_base


def find_cubes(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    results, masks = [], {}
    for cname, cfg in COLORS.items():
        mask = cv2.inRange(hsv, np.array(cfg['lower1']), np.array(cfg['upper1']))
        if 'lower2' in cfg:
            m2 = cv2.inRange(hsv, np.array(cfg['lower2']), np.array(cfg['upper2']))
            mask = mask | m2
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        masks[cname] = mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_CUBE_AREA or area > MAX_CUBE_AREA:
                continue
            rect = cv2.minAreaRect(c)
            (cx, cy), (w, h), _ = rect
            if w < 5 or h < 5: continue
            if min(w, h) / max(w, h) < MIN_ASPECT: continue
            if w * h < 1: continue
            if area / (w * h) < MIN_SQUARENESS: continue
            box = cv2.boxPoints(rect)
            results.append((cname, (cx, cy), box, COLORS[cname]['bgr']))
    return results, masks


def angle_ema(prev, new, alpha):
    """EMA on a 90°-symmetric angle (degrees) using circular statistics."""
    pc, ps = math.cos(math.radians(2*prev)), math.sin(math.radians(2*prev))
    nc, ns = math.cos(math.radians(2*new)),  math.sin(math.radians(2*new))
    fc = (1 - alpha) * pc + alpha * nc
    fs = (1 - alpha) * ps + alpha * ns
    return math.degrees(math.atan2(fs, fc)) / 2.0


def update_cube_filter(cube_filter, color, x, y, ang):
    now = time.time()
    f = cube_filter.get(color)
    if f is None or now - f['t'] > CUBE_TIMEOUT:
        cube_filter[color] = {'x': x, 'y': y, 'a': ang, 't': now}
    else:
        f['x'] = (1 - EMA_ALPHA) * f['x'] + EMA_ALPHA * x
        f['y'] = (1 - EMA_ALPHA) * f['y'] + EMA_ALPHA * y
        f['a'] = angle_ema(f['a'], ang, EMA_ALPHA)
        f['t'] = now
    f = cube_filter[color]
    return f['x'], f['y'], f['a']


def draw_workspace(disp, R, t, K, D):
    if R is None: return
    rvec, _ = cv2.Rodrigues(R)
    pts = np.array([
        [WORKSPACE_X_MIN, WORKSPACE_Y_MIN, TABLE_Z],
        [WORKSPACE_X_MIN, WORKSPACE_Y_MAX, TABLE_Z],
        [WORKSPACE_X_MAX, WORKSPACE_Y_MAX, TABLE_Z],
        [WORKSPACE_X_MAX, WORKSPACE_Y_MIN, TABLE_Z],
    ], dtype=np.float32)
    img_pts, _ = cv2.projectPoints(pts, rvec, t, K, D)
    img_pts = img_pts.reshape(-1, 2).astype(int)
    cv2.polylines(disp, [img_pts], True, (0, 255, 255), 2)


def adjust_color(target, ds=0, dv=0):
    """Adjust S_min / V_min for the named color and auto-save to disk so
    the values persist across runs AND get picked up by cube_detector_node."""
    cfg = COLORS[target]
    s = max(0, min(255, cfg['lower1'][1] + ds))
    v = max(0, min(255, cfg['lower1'][2] + dv))
    cfg['lower1'][1] = s; cfg['lower1'][2] = v
    if 'lower2' in cfg:
        cfg['lower2'][1] = s; cfg['lower2'][2] = v
    # Persist HSV bounds (skip the bgr display color)
    try:
        d = {}
        for c, c_cfg in COLORS.items():
            d[c] = {k: list(v) for k, v in c_cfg.items()
                    if k in ('lower1', 'upper1', 'lower2', 'upper2')}
        with open(COLOR_FILE, 'w') as f:
            json.dump(d, f, indent=2)
    except Exception as e:
        print(f"(failed to save {COLOR_FILE}: {e})")
    print(f"{target}: S_min={s}  V_min={v}")


def safe_destroy(name):
    try: cv2.destroyWindow(name)
    except cv2.error: pass


def save_manual_calibration(manual_corners):
    d = {str(k): v.tolist() for k, v in manual_corners.items()}
    with open(MANUAL_CAL_FILE, 'w') as f:
        json.dump(d, f, indent=2)
    print(f"Saved {MANUAL_CAL_FILE}")


def load_manual_calibration():
    if not os.path.exists(MANUAL_CAL_FILE): return {}
    try:
        with open(MANUAL_CAL_FILE, 'r') as f:
            d = json.load(f)
        return {int(k): np.array(v, dtype=np.float32) for k, v in d.items()}
    except Exception as e:
        print(f"Failed to load {MANUAL_CAL_FILE}: {e}")
        return {}


def on_mouse(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN: return
    if param.get('cal_mode'):
        # Raw click — no snap (cornerSubPix removed; it was misbehaving on
        # at least one marker, snapping to the bit pattern instead).
        param['cal_clicks'].append((float(x), float(y)))
        target = param.get('cal_target', [])
        idx = len(param['cal_clicks']) - 1
        if idx < len(target) * 4:
            mid = target[idx // 4]
            cn  = CORNER_NAMES[idx % 4]
            print(f"  marker {mid} {cn}: ({x}, {y})")
        return
    frame = param.get('frame')
    if frame is None: return
    h, w = frame.shape[:2]
    x0, y0 = max(0, x - 2), max(0, y - 2)
    x1, y1 = min(w, x + 3), min(h, y + 3)
    hsv_roi = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    h_avg = int(np.mean(hsv_roi[:, :, 0]))
    s_avg = int(np.mean(hsv_roi[:, :, 1]))
    v_avg = int(np.mean(hsv_roi[:, :, 2]))
    param['hsv'] = (h_avg, s_avg, v_avg)
    param['pos'] = (x, y)
    print(f"HSV at ({x:4d},{y:4d}):  H={h_avg:3d}  S={s_avg:3d}  V={v_avg:3d}")


def main():
    with open(CALIB_FILE, 'r') as f:
        data = yaml.safe_load(f)
    K = np.array(data['camera_matrix'])
    D = np.array(data['distortion_coefficients'])

    aruco_dict, aruco_params = setup_aruco()

    print(f"Connecting to {URL}...")
    cap = cv2.VideoCapture(URL)
    if not cap.isOpened():
        print("Failed to open stream"); sys.exit(1)

    manual_corners = load_manual_calibration()
    if manual_corners:
        print(f"Loaded manual corners for: {sorted(manual_corners.keys())}")

    # Load saved color thresholds, if any
    if os.path.exists(COLOR_FILE):
        try:
            with open(COLOR_FILE, 'r') as f:
                d = json.load(f)
            for cname, cfg in d.items():
                if cname not in COLORS: continue
                for k in ('lower1', 'upper1', 'lower2', 'upper2'):
                    if k in cfg:
                        COLORS[cname][k] = list(cfg[k])
            print(f"Loaded color thresholds from {COLOR_FILE}")
        except Exception as e:
            print(f"Failed to load {COLOR_FILE}: {e}")

    mouse_data = {
        'frame': None, 'hsv': None, 'pos': None,
        'cal_mode': False, 'cal_clicks': [], 'cal_target': [],
    }
    cv2.namedWindow('aruco_detect')
    cv2.setMouseCallback('aruco_detect', on_mouse, mouse_data)

    clip_limit = 3.0
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
    use_clahe = True
    show_mask = show_thresh = False
    mask_open = thresh_open = False

    last_pose, last_pose_age = None, 0
    pose_locked, locked_pose = False, None
    edit_mode = False
    cube_filter = {}
    tuning_color = 'red'   # which color the 1-4 keys adjust; switch with R/G/B/Y

    fps_t0, fps_count, fps = time.time(), 0, 0.0

    print("q c m r +/- s | L U | k=cal all  e+0..3=redo one  K=clear | 1-4 red HSV")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05); continue

        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_proc = clahe.apply(gray) if use_clahe else gray
        mouse_data['frame'] = frame
        mean_lum = float(np.mean(gray)); std_lum = float(np.std(gray))

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray_proc, aruco_dict, parameters=aruco_params)
        n_markers = 0 if ids is None else len(ids)
        n_rejected = len(rejected) if rejected is not None else 0

        detected_dict = {}
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                detected_dict[int(mid)] = corners[i][0]

        R, t, n_det, n_man = None, None, 0, 0
        pose_status = "no data"
        if detected_dict or manual_corners:
            res = solve_pose_hybrid(detected_dict, manual_corners, K, D)
            if res is not None:
                R, t, n_det, n_man = res
                last_pose = (R, t); last_pose_age = 0
                pose_status = f"hybrid det={n_det} man={n_man}"
            else:
                pose_status = "PnP failed"

        if R is None and last_pose and last_pose_age < POSE_CACHE_FRAMES:
            R, t = last_pose; last_pose_age += 1
            pose_status = f"cached {last_pose_age}/{POSE_CACHE_FRAMES}"

        if pose_locked and locked_pose is not None:
            R, t = locked_pose
            pose_status = "LOCKED"

        disp = frame.copy()
        draw_workspace(disp, R, t, K, D)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(disp, corners, ids, (0, 255, 0))
        if rejected is not None:
            for rj in rejected:
                pts = rj.reshape(-1, 2).astype(int)
                cv2.polylines(disp, [pts], True, (50, 50, 180), 1)

        for mid, c4 in manual_corners.items():
            if mid in detected_dict: continue
            pts = c4.astype(int)
            cv2.polylines(disp, [pts], True, (0, 165, 255), 2)
            ctr_pix = pts.mean(axis=0).astype(int)
            cv2.putText(disp, f"M{mid}", tuple(ctr_pix),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

        # Cubes (with EMA filter)
        cubes, masks = find_cubes(frame)
        seen_colors = set()
        for cname, (cx, cy), box, bgr in cubes:
            box_int = box.astype(int)
            cv2.drawContours(disp, [box_int], 0, bgr, 2)
            cv2.circle(disp, (int(cx), int(cy)), 4, bgr, -1)
            label = cname
            if R is not None:
                z_mid = TABLE_Z + CUBE_HEIGHT / 2
                pt = pixel_to_table(cx, cy, K, D, R, t, z_plane=z_mid)
                if pt is not None:
                    a = pixel_to_table(box[0][0], box[0][1], K, D, R, t, z_plane=z_mid)
                    b = pixel_to_table(box[1][0], box[1][1], K, D, R, t, z_plane=z_mid)
                    raw_ang = 0.0
                    if a is not None and b is not None:
                        raw_ang = math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
                        raw_ang = ((raw_ang + 45.0) % 90.0) - 45.0
                    if cname not in seen_colors:
                        fx, fy, fa = update_cube_filter(
                            cube_filter, cname, float(pt[0]), float(pt[1]), raw_ang)
                        seen_colors.add(cname)
                        in_ws = (WORKSPACE_X_MIN <= fx <= WORKSPACE_X_MAX and
                                 WORKSPACE_Y_MIN <= fy <= WORKSPACE_Y_MAX)
                        label = f"{cname} ({fx:+.2f},{fy:+.2f}) {fa:+.0f}d"
                    else:
                        in_ws = (WORKSPACE_X_MIN <= pt[0] <= WORKSPACE_X_MAX and
                                 WORKSPACE_Y_MIN <= pt[1] <= WORKSPACE_Y_MAX)
                        label = f"{cname} ({pt[0]:+.2f},{pt[1]:+.2f}) {raw_ang:+.0f}d"
                    if not in_ws:
                        label += " OOB"; bgr = (140, 140, 140)
            cv2.putText(disp, label, (int(cx) + 8, int(cy)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(disp, label, (int(cx) + 8, int(cy)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1)

        if mouse_data['hsv'] is not None and not mouse_data['cal_mode']:
            hh, ss, vv = mouse_data['hsv']
            px, py = mouse_data['pos']
            cv2.circle(disp, (px, py), 8, (255, 255, 255), 2)
            txt = f"H={hh} S={ss} V={vv}"
            cv2.putText(disp, txt, (px + 12, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(disp, txt, (px + 12, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        # Calibration UI
        if mouse_data['cal_mode']:
            target = mouse_data['cal_target']
            n_clicks = len(mouse_data['cal_clicks'])
            total = len(target) * 4
            for i, (cx, cy) in enumerate(mouse_data['cal_clicks']):
                mid = target[i // 4]; cn = CORNER_NAMES[i % 4]
                cv2.circle(disp, (int(cx), int(cy)), 5, (0, 200, 255), -1)
                cv2.putText(disp, f"M{mid}-{cn}", (int(cx) + 8, int(cy) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
                cv2.putText(disp, f"M{mid}-{cn}", (int(cx) + 8, int(cy) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
            if n_clicks < total:
                mid_n = target[n_clicks // 4]; cn_n = CORNER_NAMES[n_clicks % 4]
                banner = (f"CAL {n_clicks}/{total}  NEXT: marker {mid_n} corner "
                          f"{cn_n}  (CW from marker top-left)")
            else:
                banner = "CAL DONE — saving..."
            cv2.rectangle(disp, (0, h - 36), (w, h), (40, 40, 40), -1)
            cv2.putText(disp, banner, (10, h - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
            if n_clicks == total:
                clicks = mouse_data['cal_clicks']
                for i, mid in enumerate(target):
                    c4 = np.array(clicks[i*4:(i+1)*4], dtype=np.float32)
                    manual_corners[mid] = c4
                save_manual_calibration(manual_corners)
                mouse_data['cal_mode'] = False
                mouse_data['cal_clicks'] = []
                mouse_data['cal_target'] = []
                print("Calibration done.")

        if edit_mode:
            cv2.rectangle(disp, (0, h - 36), (w, h), (60, 0, 60), -1)
            cv2.putText(disp, "EDIT — press 0/1/2/3 to redo that marker, e to cancel",
                        (10, h - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 180, 255), 2)

        # FPS
        fps_count += 1
        if time.time() - fps_t0 > 0.5:
            fps = fps_count / (time.time() - fps_t0)
            fps_count = 0; fps_t0 = time.time()

        # Stats overlay — smaller font + tighter spacing
        tc = COLORS[tuning_color]
        tc_s = tc['lower1'][1]; tc_v = tc['lower1'][2]
        man_str = ','.join(str(i) for i in sorted(manual_corners.keys())) or '-'
        lines = [
            f"FPS:{fps:.1f}",
            f"Mk det:{n_markers} rej:{n_rejected}",
            f"Pose:{pose_status}",
            f"Manual:[{man_str}]",
            f"Cubes:{len(cubes)}",
            f"Lum:{mean_lum:.0f} ({std_lum:.0f})",
            f"CLAHE:{'ON c='+f'{clip_limit:.1f}' if use_clahe else 'OFF'}",
            f"tune {tuning_color} S/V:{tc_s}/{tc_v}",
        ]
        for i, line in enumerate(lines):
            y = 14 + i * 16
            cv2.putText(disp, line, (8, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
            cv2.putText(disp, line, (8, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        if mean_lum < 60:
            cv2.putText(disp, "DIM", (w - 60, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(disp, "DIM", (w - 60, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow("aruco_detect", disp)

        if show_thresh:
            thr = cv2.adaptiveThreshold(gray_proc, 255,
                                        cv2.ADAPTIVE_THRESH_MEAN_C,
                                        cv2.THRESH_BINARY_INV, 23, 7)
            cv2.imshow("threshold", thr); thresh_open = True
        elif thresh_open:
            safe_destroy("threshold"); thresh_open = False

        if show_mask:
            grid = []
            for cname in ['red', 'green', 'blue', 'yellow']:
                m = cv2.cvtColor(masks[cname], cv2.COLOR_GRAY2BGR)
                cv2.putText(m, cname, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            COLORS[cname]['bgr'], 2)
                grid.append(m)
            cv2.imshow("masks", np.vstack([np.hstack(grid[:2]),
                                           np.hstack(grid[2:])]))
            mask_open = True
        elif mask_open:
            safe_destroy("masks"); mask_open = False

        key = cv2.waitKey(1) & 0xFF
        if key == 255:
            continue

        # Edit-mode key handling FIRST so it overrides 0..3
        if edit_mode and key in (ord('0'), ord('1'), ord('2'), ord('3')):
            mid = key - ord('0')
            edit_mode = False
            mouse_data['cal_mode'] = True
            mouse_data['cal_clicks'] = []
            mouse_data['cal_target'] = [mid]
            print(f"Redo marker {mid} — click its 4 corners CW")
            continue
        if edit_mode and key == ord('e'):
            edit_mode = False
            print("Edit cancelled")
            continue

        if key == ord('q'): break
        elif key == ord('c'):
            use_clahe = not use_clahe
        elif key == ord('m'):
            show_mask = not show_mask
        elif key == ord('r'):
            show_thresh = not show_thresh
        elif key in (ord('+'), ord('=')):
            clip_limit = min(clip_limit + 0.5, 10.0)
            clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
        elif key == ord('-'):
            clip_limit = max(clip_limit - 0.5, 0.5)
            clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
        elif key == ord('1'): adjust_color(tuning_color, ds=-10)
        elif key == ord('2'): adjust_color(tuning_color, ds=+10)
        elif key == ord('3'): adjust_color(tuning_color, dv=-10)
        elif key == ord('4'): adjust_color(tuning_color, dv=+10)
        elif key == ord('R'): tuning_color = 'red';    print("Tuning: red")
        elif key == ord('G'): tuning_color = 'green';  print("Tuning: green")
        elif key == ord('B'): tuning_color = 'blue';   print("Tuning: blue")
        elif key == ord('Y'): tuning_color = 'yellow'; print("Tuning: yellow")
        elif key in (ord('l'), ord('L')):
            if R is not None:
                locked_pose = (R.copy(), t.copy()); pose_locked = True
                print("Pose LOCKED")
            else:
                print("No current pose to lock")
        elif key in (ord('u'), ord('U')):
            pose_locked = False; locked_pose = None
            print("Pose unlocked")
        elif key == ord('k'):
            mouse_data['cal_mode'] = True
            mouse_data['cal_clicks'] = []
            mouse_data['cal_target'] = list(ALL_MARKERS)
            print("Cal ALL: 16 clicks, M0 then M1 then M2 then M3")
        elif key == ord('e'):
            edit_mode = True
            print("Edit mode: press 0/1/2/3 to redo that marker, e to cancel")
        elif key == ord('K'):
            manual_corners.clear()
            if os.path.exists(MANUAL_CAL_FILE):
                os.remove(MANUAL_CAL_FILE)
            print("Manual calibration cleared")
        elif key == ord('s'):
            fname = f"aruco_snapshot_{int(time.time())}.png"
            cv2.imwrite(fname, disp)
            print(f"Saved {fname}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
