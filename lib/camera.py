import sys
import os
# import pyrealsense2 as rs
import cv2
import numpy as np

class RealsenseCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))
        
        # Enable streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming
        self.pipeline.start(self.config)
        
    def get_frames(self):
        # Wait for a coherent pair of frames
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return color_image, depth_image

    def release(self):
        # Stop streaming
        self.pipeline.stop()

class USBCamera:
    def __init__(self, index=0):
        # Initialize USB camera
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise Exception("Could not open video device")
        # Set properties if needed
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame
    
    def release(self):
        self.cap.release()

class StereoCamera:
    def __init__(self, device_id=0, scale=1.0):
        # Open the camera with V4L2 backend
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        
        # Store the scale factor
        self.scale = scale

        # Set MJPEG format
        success_fourcc = self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # Set resolution to 2560x720
        success_width = self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        success_height = self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        # Set frame rate to 30 FPS
        success_fps = self.cap.set(cv2.CAP_PROP_FPS, 30)
        # Set buffer size to 1
        success_buffer = self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Check if settings were successful
        print(f"Set FOURCC to MJPG: {success_fourcc}")
        print(f"Set width to 2560: {success_width}")
        print(f"Set height to 720: {success_height}")
        print(f"Set FPS to 30: {success_fps}")
        print(f"Set buffer size to 1: {success_buffer}")

        # Verify actual settings
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = ''.join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        buffer_size = self.cap.get(cv2.CAP_PROP_BUFFERSIZE)
        print(f"Actual FOURCC: {fourcc_str}")
        print(f"Actual Resolution: {width}x{height}")
        print(f"Actual FPS: {fps}")
        print(f"Actual buffer size: {buffer_size}")
        
        if scale != 1.0:
            print(f"Images will be scaled by factor: {scale}")

    def set_scale(self, scale):
        """Set the scaling factor for the stereo images"""
        self.scale = scale
        print(f"Scale factor set to: {scale}")
        
    def get_scale(self):
        """Get the current scaling factor"""
        return self.scale

    def get_stereo(self, scale=None):
        """
        Get stereo images from the camera.
        
        Args:
            scale: Optional scaling factor to override the default scale.
                  If None, uses the scale set during initialization.
        
        Returns:
            Tuple of (left_image, right_image), or (None, None) if capture failed.
        """
        ret, frame = self.cap.read()
        if not ret:
            return None, None
            
        # Split the frame horizontally into left and right images
        left = frame[:, :1280]
        right = frame[:, 1280:]
        
        # Apply scaling if needed
        use_scale = scale if scale is not None else self.scale
        if use_scale != 1.0:
            h, w = left.shape[:2]
            new_h, new_w = int(h * use_scale), int(w * use_scale)
            left = cv2.resize(left, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            right = cv2.resize(right, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            
        return left, right

    def release(self):
        self.cap.release()

if __name__ == "__main__":
    realsense = RealsenseCamera()
    usb = USBCamera()
    
    # Example usage of StereoCamera
    stereo_cam = StereoCamera(0, scale=0.5)  # Initialize with 50% scaling
    left, right = stereo_cam.get_stereo()
    if left is not None and right is not None:
        cv2.imwrite("left.jpg", left)
        cv2.imwrite("right.jpg", right)
        print("Saved stereo images to left.jpg and right.jpg")
        
        # Try with a different scale
        left_full, right_full = stereo_cam.get_stereo(scale=1.0)
        cv2.imwrite("left_full.jpg", left_full)
        cv2.imwrite("right_full.jpg", right_full)
        print("Saved full-size stereo images")
    else:
        print("Failed to capture stereo images")
    stereo_cam.release()
