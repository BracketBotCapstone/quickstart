import os
os.environ["RERUN"] = "1"
import cv2
import numpy as np
import time
from datetime import datetime
import rerun as rr
rr.init("interactive_calibrate", spawn=False)

# Path to save calibration YAML to the shared lib folder
CALIB_YAML_PATH = os.path.join(os.path.dirname(__file__), '..', 'lib', 'stereo_calibration_fisheye.yaml')

# --- HELPER: save calibration mats robustly to YAML ---
def write_calib_yaml(params: dict, filename: str = CALIB_YAML_PATH):
    """Save numpy matrices to a YAML file using cv2.FileStorage.

    Skips any entries that are None and ensures values are numpy arrays.
    """
    try:
        fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_WRITE)
        for k, v in params.items():
            if v is None:
                continue
            fs.write(k, np.asarray(v))
        fs.release()
        print(f"[INFO] Calibration parameters saved to {filename}")
    except Exception as e:
        print(f"[WARN] Could not save calibration file: {e}")

# --- CONFIGURATION ---
CHECKERBOARD = (9, 6)
SQUARE_SIZE = 22.5  # mm
IMG_DIR = './captures'
os.makedirs(IMG_DIR, exist_ok=True)

# --- RERUN TCP CONNECTION ---
rr.connect_grpc("rerun+http://<YOUR_COMPUTER_IP>:9876/proxy")
# --- STATE ---
captured_pairs = []  # List of (left, right, fname)

# Load existing images from captures folder
import glob
existing_files = glob.glob(os.path.join(IMG_DIR, '*_usb.jpg'))
for fname in existing_files:
    img = cv2.imread(fname)
    if img is not None:
        height, width = img.shape[:2]
        left = img[:, :width//2]
        right = img[:, width//2:]
        captured_pairs.append((left, right, fname))
        print(f"[INFO] Loaded existing image: {fname}")
if captured_pairs:
    print(f"[INFO] Loaded {len(captured_pairs)} existing image pairs for calibration.")

# --- Rectification maps placeholder ---
latest_maps = {'map1x': None, 'map1y': None, 'map2x': None, 'map2y': None}
# defer initial calibration until calibrate_and_rectify is defined
init_calib_pairs_loaded = len(captured_pairs) >= 5

# --- KEYBOARD POLLING (using select, no threads) ---
import sys
import select

# --- CALIBRATION UTILS ---
def calibrate_and_rectify(pairs):
    if len(pairs) < 5:
        return None, None, None, None

    # Prepare object points in fisheye format (N,1,3)
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 1, 3), np.float32)
    objp[:, 0, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints, imgpoints_left, imgpoints_right = [], [], []
    for left, right, _ in pairs:
        gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        ret_l, corners_l = cv2.findChessboardCorners(gray_left, CHECKERBOARD, cv2.CALIB_CB_FAST_CHECK)
        ret_r, corners_r = cv2.findChessboardCorners(gray_right, CHECKERBOARD, cv2.CALIB_CB_FAST_CHECK)
        if ret_l and ret_r:
            # Refine corners for better accuracy
            crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            corners_l = cv2.cornerSubPix(gray_left, corners_l, (3, 3), (-1, -1), crit)
            corners_r = cv2.cornerSubPix(gray_right, corners_r, (3, 3), (-1, -1), crit)
            objpoints.append(objp)
            imgpoints_left.append(corners_l)
            imgpoints_right.append(corners_r)

    if len(objpoints) < 5:
        return None, None, None, None

    # Initialize camera matrices & distortion coeffs
    K1 = np.zeros((3, 3))
    D1 = np.zeros((4, 1))
    K2 = np.zeros((3, 3))
    D2 = np.zeros((4, 1))

    image_size = gray_left.shape[::-1]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-6)
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW

    # OpenCV fisheye.stereoCalibrate has changed signature across versions.
    # It may return 7 or 9 values (… E, F).  Capture robustly.
    _stereo_out = cv2.fisheye.stereoCalibrate(
        objpoints,
        imgpoints_left,
        imgpoints_right,
        K1,
        D1,
        K2,
        D2,
        image_size,
        criteria=criteria,
        flags=flags,
    )

    rms = _stereo_out[0]
    K1, D1, K2, D2, R, T = _stereo_out[1:7]
    E = F = None
    if len(_stereo_out) >= 9:
        E, F = _stereo_out[7:9]

    # Alias pin‑hole variable names used elsewhere so the rest of the script and
    # downstream YAML consumers (stereo.py) remain untouched.
    mtx_l, dist_l = K1, D1
    mtx_r, dist_r = K2, D2

    print(f"[INFO] Fisheye stereo calibration RMS error: {rms:.3f}")

    # Rectify
    R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(K1, D1, K2, D2, image_size, R, T, flags=cv2.CALIB_ZERO_DISPARITY)

    map1x, map1y = cv2.fisheye.initUndistortRectifyMap(K1, D1, R1, P1, image_size, cv2.CV_16SC2)
    map2x, map2y = cv2.fisheye.initUndistortRectifyMap(K2, D2, R2, P2, image_size, cv2.CV_16SC2)

    return map1x, map1y, map2x, map2y, mtx_l, dist_l, mtx_r, dist_r, R, T, E, F, R1, R2, P1, P2, Q

def draw_epipolar_lines(img, color=(0,255,0), step=25):
    out = img.copy()
    for y in range(0, out.shape[0], step):
        cv2.line(out, (0, y), (out.shape[1], y), color, 1)
    return out

# --- Run initial calibration _after_ function definition ---
if init_calib_pairs_loaded:
    print("[INFO] Attempting initial calibration with existing images ...")
    calib_res = calibrate_and_rectify(captured_pairs)
    map1x, map1y, map2x, map2y = calib_res[:4]
    if map1x is not None:
        latest_maps['map1x'] = map1x
        latest_maps['map1y'] = map1y
        latest_maps['map2x'] = map2x
        latest_maps['map2y'] = map2y
        print("[INFO] Initial calibration successful – rectification ready.")
        # Collect params and write in one go
        calib_dict = {
            'mtx_l': calib_res[4],
            'dist_l': calib_res[5],
            'mtx_r': calib_res[6],
            'dist_r': calib_res[7],
            'R': calib_res[8],
            'T': calib_res[9],
            'E': calib_res[10],
            'F': calib_res[11],
            'R1': calib_res[12],
            'R2': calib_res[13],
            'P1': calib_res[14],
            'P2': calib_res[15],
            'Q': calib_res[16],
        }
        write_calib_yaml(calib_dict, CALIB_YAML_PATH)
    else:
        print("[INFO] Initial calibration failed or not enough valid pairs; capture more images.")
else:
    print("[INFO] Fewer than 5 image pairs present – capture more to calibrate.")

# --- MAIN LOOP ---

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
if not cap.isOpened():
    print("Could not open video device 0")
    exit(1)

print("Interactive stereo calibration. Press SPACE to capture, ESC to quit.")

last_log = 0
log_interval = 1.0  # 1s for rerun logging
print("Interactive stereo calibration. Type 'space'+Enter to capture, 'esc'+Enter to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    h, w = frame.shape[:2]
    left = frame[:, :w//2]
    right = frame[:, w//2:]
    gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    ret_l, corners_l = cv2.findChessboardCorners(gray_left, CHECKERBOARD, None)
    ret_r, corners_r = cv2.findChessboardCorners(gray_right, CHECKERBOARD, None)
    print(f"Checkerboard detection: left={'YES' if ret_l else 'NO'}, right={'YES' if ret_r else 'NO'}")
    vis_left = left.copy()
    vis_right = right.copy()
    if ret_l:
        cv2.drawChessboardCorners(vis_left, CHECKERBOARD, corners_l, ret_l)
    if ret_r:
        cv2.drawChessboardCorners(vis_right, CHECKERBOARD, corners_r, ret_r)
    # --- Live rectification preview ---
    if all(latest_maps[k] is not None for k in ['map1x', 'map1y', 'map2x', 'map2y']):
        rect_left = cv2.remap(left, latest_maps['map1x'], latest_maps['map1y'], cv2.INTER_LINEAR)
        rect_right = cv2.remap(right, latest_maps['map2x'], latest_maps['map2y'], cv2.INTER_LINEAR)
        rect_left_lines = draw_epipolar_lines(rect_left)
        rect_right_lines = draw_epipolar_lines(rect_right)
    else:
        rect_left_lines = None
        rect_right_lines = None
    # Downscale for rerun visualization only
    now = time.time()
    if now - last_log > log_interval:
        vis_left_small = cv2.resize(vis_left, (vis_left.shape[1]//4, vis_left.shape[0]//4))
        vis_right_small = cv2.resize(vis_right, (vis_right.shape[1]//4, vis_right.shape[0]//4))
        rr.log("left", rr.Image(vis_left_small))
        rr.log("right", rr.Image(vis_right_small))
        if rect_left_lines is not None and rect_right_lines is not None:
            rect_left_small = cv2.resize(rect_left_lines, (rect_left_lines.shape[1]//4, rect_left_lines.shape[0]//4))
            rect_right_small = cv2.resize(rect_right_lines, (rect_right_lines.shape[1]//4, rect_right_lines.shape[0]//4))
            rr.log("rect_left_live", rr.Image(rect_left_small))
            rr.log("rect_right_live", rr.Image(rect_right_small))
        last_log = now
    # --- Poll for keyboard input ---
    input_ready, _, _ = select.select([sys.stdin], [], [], 0)
    if input_ready:
        print("DEBUG: Input detected by select!") 
        key = sys.stdin.readline()
        print(f"DEBUG: Key read: '{key}'") 
        print(f"DEBUG: Enter pressed. ret_l={ret_l}, ret_r={ret_r}") 
        if not ret_l or not ret_r:
            print("[INFO] Checkerboard not detected in both images. Image not saved.")
        else:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            fname = os.path.join(IMG_DIR, f'{timestamp}_usb.jpg')
            cv2.imwrite(fname, frame)
            print(f"[INFO] Saved image for calibration: {fname}")
            captured_pairs.append((left.copy(), right.copy(), fname))
            print(f"[INFO] Captured and saved: {fname}")
            print(f"[INFO] Attempting calibration/rectification with {len(captured_pairs)} pairs...")
            calib_result = calibrate_and_rectify(captured_pairs)
            map1x, map1y, map2x, map2y = calib_result[:4]
            # Save calibration results if available
            if map1x is not None:
                print("[INFO] Calibration and rectification successful. Logging rectified images.")
                # Update latest maps for live preview
                latest_maps['map1x'] = map1x
                latest_maps['map1y'] = map1y
                latest_maps['map2x'] = map2x
                latest_maps['map2y'] = map2y
                mtx_l = dist_l = mtx_r = dist_r = R = T = E = F = R1 = R2 = P1 = P2 = Q = None
                if len(calib_result) > 4:
                    mtx_l, dist_l, mtx_r, dist_r, R, T, E, F, R1, R2, P1, P2, Q = calib_result[4:]
                calib_dict = {
                    'mtx_l': mtx_l,
                    'dist_l': dist_l,
                    'mtx_r': mtx_r,
                    'dist_r': dist_r,
                    'R': R,
                    'T': T,
                    'E': E,
                    'F': F,
                    'R1': R1,
                    'R2': R2,
                    'P1': P1,
                    'P2': P2,
                    'Q': Q,
                }
                write_calib_yaml(calib_dict, CALIB_YAML_PATH)
                rect_left = cv2.remap(left, map1x, map1y, cv2.INTER_LINEAR)
                rect_right = cv2.remap(right, map2x, map2y, cv2.INTER_LINEAR)
                rect_left_lines = draw_epipolar_lines(rect_left)
                rect_right_lines = draw_epipolar_lines(rect_right)
                rect_left_small = cv2.resize(rect_left_lines, (rect_left_lines.shape[1]//4, rect_left_lines.shape[0]//4))
                rect_right_small = cv2.resize(rect_right_lines, (rect_right_lines.shape[1]//4, rect_right_lines.shape[0]//4))
                rr.log("rect_left", rr.Image(rect_left_small))
                rr.log("rect_right", rr.Image(rect_right_small))
            else:
                print("[INFO] Not enough valid pairs for calibration.")

cap.release()
cv2.destroyAllWindows()
