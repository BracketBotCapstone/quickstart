#!/usr/bin/env python3

# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
import glob
import time

def capture_calibration_frames(num_frames=20):
    """Capture frames for calibration"""
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return None
        
    # Set resolution to capture full stereo image
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    # Wait for camera to initialize
    time.sleep(2)
    
    # Create a directory for calibration frames in the setup folder
    calibration_dir = os.path.join(os.path.dirname(__file__), "calibration_frames")
    if not os.path.exists(calibration_dir):
        os.makedirs(calibration_dir)
    
    # Print instructions for the user
    print("\n=== Camera Calibration Instructions ===")
    print("1. Open 'current_stereo.jpg' in your IDE or image viewer to see the camera feed")
    print("   (The image file is being constantly overwritten with the current camera view)")
    print("2. Hold the chessboard pattern in front of the cameras")
    print("3. Move the chessboard to different positions and angles")
    print("4. The system will automatically capture frames when the chessboard is detected")
    print("5. Try to cover different areas of the camera view for best calibration results")
    print("6. Press Ctrl+C when you're done or wait until enough frames are captured")
    print("===================================\n")
    # Chessboard dimensions
    pattern_size = (9, 6)  # Interior corners on the chessboard
    square_size = 0.025  # Physical size of squares in meters
    
    # Prepare object points
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    left_imgpoints = []  # 2D points in left image plane
    right_imgpoints = []  # 2D points in right image plane
    
    frames_captured = 0
    frame_count = 0
    
    print("Saving frames continuously. Press Ctrl+C when done.")
    print("Camera feed is being saved to 'current_stereo.jpg' - open this file to see the live feed.")
    print(f"Calibration frames will be saved to: {calibration_dir}")
    
    try:
        while frames_captured < num_frames:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
                
            # Split frame into left and right
            frame_width = frame.shape[1] // 2
            left_frame = frame[:, :frame_width]
            right_frame = frame[:, frame_width:]
            
            # Save current frame
            cv2.imwrite(os.path.join(calibration_dir, "current_stereo.jpg"), frame)
            
            # Convert to grayscale
            left_gray = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
            
            # Try to find chessboard every 30 frames
            frame_count += 1
            if frame_count % 30 == 0:
                # Find chessboard corners
                left_found, left_corners = cv2.findChessboardCorners(left_gray, pattern_size, None)
                right_found, right_corners = cv2.findChessboardCorners(right_gray, pattern_size, None)
                
                if left_found and right_found:
                    # Refine corner detection
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    left_corners = cv2.cornerSubPix(left_gray, left_corners, (11,11), (-1,-1), criteria)
                    right_corners = cv2.cornerSubPix(right_gray, right_corners, (11,11), (-1,-1), criteria)
                    
                    # Draw corners
                    left_frame_corners = left_frame.copy()
                    right_frame_corners = right_frame.copy()
                    cv2.drawChessboardCorners(left_frame_corners, pattern_size, left_corners, left_found)
                    cv2.drawChessboardCorners(right_frame_corners, pattern_size, right_corners, right_found)
                    
                    # Save frame with corners
                    combined = np.hstack((left_frame_corners, right_frame_corners))
                    cv2.imwrite(os.path.join(calibration_dir, f"calibration_frame_{frames_captured}.jpg"), combined)
                    
                    # Store points
                    objpoints.append(objp)
                    left_imgpoints.append(left_corners)
                    right_imgpoints.append(right_corners)
                    
                    frames_captured += 1
                    print(f"Captured calibration frame {frames_captured}/{num_frames}")
            
            time.sleep(0.1)  # Small delay to prevent overwhelming the system
    
    except KeyboardInterrupt:
        print("\nStopping capture")
    finally:
        cap.release()
    
    return objpoints, left_imgpoints, right_imgpoints, left_gray.shape[::-1]

def calibrate_stereo_cameras(objpoints, left_imgpoints, right_imgpoints, image_size):
    """Calibrate the stereo cameras and compute essential matrices"""
    
    # Calibrate left camera
    ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
        objpoints, left_imgpoints, image_size, None, None)
    
    # Calibrate right camera
    ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
        objpoints, right_imgpoints, image_size, None, None)
    
    # Stereo calibration
    flags = cv2.CALIB_FIX_INTRINSIC
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
    
    ret_stereo, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv2.stereoCalibrate(
        objpoints, left_imgpoints, right_imgpoints,
        mtx_left, dist_left,
        mtx_right, dist_right,
        image_size, criteria=criteria, flags=flags)
    
    # Compute rectification transforms
    rect_left, rect_right, proj_left, proj_right, Q, roi_left, roi_right = cv2.stereoRectify(
        mtx_left, dist_left,
        mtx_right, dist_right,
        image_size, R, T)
    
    # Compute mapping matrices
    left_maps = cv2.initUndistortRectifyMap(mtx_left, dist_left, rect_left, proj_left, image_size, cv2.CV_32FC1)
    right_maps = cv2.initUndistortRectifyMap(mtx_right, dist_right, rect_right, proj_right, image_size, cv2.CV_32FC1)
    
    # Save calibration data to lib directory
    lib_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'lib')
    calibration_file = os.path.join(lib_dir, 'stereo_calibration.npz')
    np.savez(calibration_file,
             left_maps_x=left_maps[0],
             left_maps_y=left_maps[1],
             right_maps_x=right_maps[0],
             right_maps_y=right_maps[1],
             Q=Q)
    
    return True

def main():
    print("Starting stereo camera calibration...")
    print("Please show the chessboard pattern at different angles and distances")
    
    # Capture calibration frames
    ret = capture_calibration_frames()
    if ret is None:
        return False
    
    objpoints, left_imgpoints, right_imgpoints, image_size = ret
    
    if len(objpoints) < 5:
        print("Not enough calibration frames captured")
        return False
    
    # Perform calibration
    print("Calibrating cameras...")
    success = calibrate_stereo_cameras(objpoints, left_imgpoints, right_imgpoints, image_size)
    
    if success:
        lib_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'lib', 'stereo_calibration.npz')
        print(f"Calibration successful! Calibration data saved to '{lib_path}'")
    else:
        print("Calibration failed!")
    
    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 