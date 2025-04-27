#!/usr/bin/env python3
"""Light-weight stereo + point-cloud viewer (fixed)."""

import os, sys, time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy
import rerun as rr

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from lib.camera import StereoCamera   # noqa: E402

# ───────────────── CONFIG ─────────────────
DOWNSAMPLE   = 0.5
CALIB_FILE   = os.path.join(os.path.dirname(__file__),
                            "..", "lib", "stereo_calibration_fisheye.yaml")

# SGBM params (identical to "big" file)
WINDOW_SIZE  = 9
MIN_DISP     = -32
NUM_DISP     = 144              # must be /16
UNIQUENESS   = 5
SPECKLE_WIN  = 100
SPECKLE_RANGE = 1
P1           = 8  * 3 * WINDOW_SIZE ** 2
P2           = 32 * 3 * WINDOW_SIZE ** 2

# ─────────── Helper: world → viewer ───────────
def _world_to_viewer(pts_world: np.ndarray) -> np.ndarray:
    x = pts_world[:, 0]          # right
    z = pts_world[:, 2]          # fwd
    y = pts_world[:, 1]          # up (positive down in world)
    return np.column_stack([x, z, -y]).astype(np.float32)

# ─────────── Load calibration (verbatim) ───────────
def load_calib_yaml(path: str, scale: float):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    mtx_l = fs.getNode("mtx_l").mat();  dist_l = fs.getNode("dist_l").mat()
    mtx_r = fs.getNode("mtx_r").mat();  dist_r = fs.getNode("dist_r").mat()
    R1 = fs.getNode("R1").mat();        R2 = fs.getNode("R2").mat()
    P1 = fs.getNode("P1").mat();        P2 = fs.getNode("P2").mat()
    Q  = fs.getNode("Q").mat().astype(np.float32)
    fs.release()
    # down-sample Q's translation entries
    for i in range(4):
        Q[i, 3] *= scale
    return mtx_l, dist_l, mtx_r, dist_r, R1, R2, P1, P2, Q

# ─────────────────── Main ────────────────────
def main():
    rr.init("tiny_depth_viewer")
    rr.connect_grpc("rerun+http://192.168.2.24:9876/proxy")

    cam = StereoCamera(0, scale=1.0)

    # Calibration
    (mtx_l, dist_l, mtx_r, dist_r,
     R1, R2, P1_cam, P2_cam, Q) = load_calib_yaml(CALIB_FILE, DOWNSAMPLE)

    baseline_m = abs(P2_cam[0, 3] / P2_cam[0, 0]) / 1000.0
    fx_ds      = P1_cam[0, 0] * DOWNSAMPLE

    # Rectification maps
    left_raw, _ = cam.get_stereo()
    h, w = left_raw.shape[:2]
    map1x, map1y = cv2.fisheye.initUndistortRectifyMap(mtx_l, dist_l, R1, P1_cam,
                                                      (w, h), cv2.CV_32FC1)
    map2x, map2y = cv2.fisheye.initUndistortRectifyMap(mtx_r, dist_r, R2, P2_cam,
                                                      (w, h), cv2.CV_32FC1)

    # SGBM matcher
    stereo = cv2.StereoSGBM_create(
        minDisparity=MIN_DISP,
        numDisparities=NUM_DISP,
        blockSize=WINDOW_SIZE,
        P1=P1,            # 8 * 3 * WINDOW_SIZE**2
        P2=P2,            # 32 * 3 * WINDOW_SIZE**2
        disp12MaxDiff=1,
        uniquenessRatio=UNIQUENESS,
        speckleWindowSize=SPECKLE_WIN,
        speckleRange=SPECKLE_RANGE,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )

    while True:
        left_raw, right_raw = cam.get_stereo()
        if left_raw is None:
            time.sleep(0.05)
            continue

        # Rectify → downsample
        left_rect  = cv2.remap(left_raw,  map1x, map1y, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right_raw, map2x, map2y, cv2.INTER_LINEAR)
        tgt_sz = (int(w * DOWNSAMPLE), int(h * DOWNSAMPLE))
        left_ds  = cv2.resize(left_rect,  tgt_sz, interpolation=cv2.INTER_AREA)
        right_ds = cv2.resize(right_rect, tgt_sz, interpolation=cv2.INTER_AREA)

        # Disparity (px)
        disp = stereo.compute(left_ds, right_ds).astype(np.float32) / 16.0

        # Valid pixels
        valid = disp > (MIN_DISP + 0.5)

        # Depth (m) – avoid div-by-zero near MIN_DISP
        denom = disp - MIN_DISP
        depth = np.zeros_like(disp)
        depth_mask = denom > 0.1
        depth[depth_mask] = fx_ds * baseline_m / denom[depth_mask]

        # Depth colouring (invert JET, 0.1–5 m)
        depth_clipped = np.clip(depth, 0.1, 5.0)
        depth_norm = ((depth_clipped - 0.1) / (5.0 - 0.1) * 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(255 - depth_norm, cv2.COLORMAP_JET)

        # Point cloud in camera frame
        pts_cam = cv2.reprojectImageTo3D(disp, Q) / 1000.0
        pts_cam = pts_cam[valid]
        cols    = cv2.cvtColor(left_ds, cv2.COLOR_BGR2RGB).reshape(-1, 3)[valid.ravel()]

        # Throw away far points (> 5 m) for clarity
        dist_m = np.linalg.norm(pts_cam, axis=1)
        keep   = dist_m < 5.0
        pts_cam, cols = pts_cam[keep], cols[keep]

        # Camera → robot (tilt & 1.5 m up)
        t_cam = np.array([0.0, -1.5, 0.0])           # +Y is down
        R_cam = R_scipy.from_euler('x', -36.0, degrees=True)
        pts_robot = R_cam.apply(pts_cam) + t_cam      # robot frame (y up)

        # Robot at origin → world
        pts_world = pts_robot

        # Viewer coords (X right, Y fwd, Z up)
        pts_view = _world_to_viewer(pts_world)

        # Sub-sample 4× before logging
        pts_view = pts_view[::4]
        cols     = cols[::4]

        if pts_view.size:
            rr.log("world/point_cloud", rr.Points3D(pts_view, colors=cols, radii=0.02))

        # Images to Rerun
        rr.set_time_seconds("time", time.time())
        rr.log("camera/disparity", rr.Image(disp))
        rr.log("camera/depth",     rr.Image(depth_color))
        time.sleep(0.1)

if __name__ == "__main__":
    main()
