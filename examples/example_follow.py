# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
import json
from time import time
from ultralytics import YOLO
from lib.camera import StereoCamera
from lib.odrive_uart import ODriveUART

# Silence YOLO
os.environ['ULTRALYTICS_HIDE_CONSOLE'] = '1'
os.environ['YOLO_VERBOSE'] = 'False'
os.environ['ULTRALYTICS_QUIET'] = 'True'
os.environ['DISABLE_ULTRALYTICS_VERSIONING_CHECK'] = 'True'

TURN_SPEED = 0.3
CENTER_THRESHOLD = 0.05
FORWARD_SPEED = 0.25  # Speed for moving forward/backward
TARGET_WIDTH_RATIO = 0.4  # Target width of person relative to image width
WIDTH_THRESHOLD = 0.05  # Acceptable range around target width

class PersonFollower:
    def __init__(self):
        self.camera = StereoCamera(scale=0.5)
        
        model_path = "yolov8n.pt"
        ncnn_path = "yolov8n_ncnn_model"
        
        if not os.path.exists(ncnn_path):
            self.model = YOLO(model_path if os.path.exists(model_path) else 'yolov8n')
            self.model.export(format="ncnn")
        
        self.model = YOLO(ncnn_path)
        
        with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'r') as f:
            motor_dirs = json.load(f)
        
        self.motor = ODriveUART(port='/dev/ttyAMA1', left_axis=0, right_axis=1, dir_left=motor_dirs['left'], dir_right=motor_dirs['right'])
        
        self.motor.start_left()
        self.motor.start_right()
        self.motor.enable_velocity_ramp_mode_left()
        self.motor.enable_velocity_ramp_mode_right()
        self.motor.disable_watchdog_left()
        self.motor.disable_watchdog_right()
        self.motor.clear_errors_left()
        self.motor.clear_errors_right()
        
        self.debug_dir = 'debug_images'
        os.makedirs(self.debug_dir, exist_ok=True)
        
        # Current motor commands
        self.current_left_speed = 0
        self.current_right_speed = 0
    
    def save_debug_images(self, left_image, annotated, best_person=None):
        """Save debug images with basic annotations"""
        # Save detection result
        cv2.imwrite(os.path.join(self.debug_dir, 'detection.jpg'), annotated)
        
        # Create combined visualization
        if best_person is not None:
            h, w = left_image.shape[:2]
            center_x = int((best_person[0] + best_person[2]) / 2)
            center_y = int((best_person[1] + best_person[3]) / 2)
            
            combined = left_image.copy()
            
            # Draw crosshair at person center
            cv2.line(combined, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
            cv2.line(combined, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)
            
            # Draw vertical center line
            cv2.line(combined, (w//2, 0), (w//2, h), (255, 0, 0), 1)
            
            cv2.imwrite(os.path.join(self.debug_dir, 'combined.jpg'), combined)
    
    def detect_and_follow(self):
        """Main loop to detect and follow a person"""
        # Get images
        left, right = self.camera.get_stereo()
        if left is None or right is None:
            return
        
        # Detect people
        results = self.model(left, classes=[0], verbose=False)
        
        # Find the largest person detection
        best_person = None
        max_area = 0
        
        for result in results[0].boxes:
            box = result.xyxy[0].cpu().numpy()
            area = (box[2] - box[0]) * (box[3] - box[1])
            if area > max_area:
                max_area = area
                best_person = box
        
        # Get annotated image
        annotated = results[0].plot()
        
        if best_person is None:
            self.motor.set_speed_mps_left(0)
            self.motor.set_speed_mps_right(0)
            self.current_left_speed = 0
            self.current_right_speed = 0
            self.save_debug_images(left, annotated)
            return
        
        # Get center point and width of the person
        center_x = (best_person[0] + best_person[2]) / 2
        image_center_x = left.shape[1] / 2
        x_error = (center_x - image_center_x) / image_center_x  # -1 to 1
        
        # Calculate width ratio of person relative to image
        person_width = best_person[2] - best_person[0]
        image_width = left.shape[1]
        width_ratio = person_width / image_width
        width_error = width_ratio - TARGET_WIDTH_RATIO
        
        # Determine forward/backward speed based on width
        forward_speed = 0
        if abs(width_error) > WIDTH_THRESHOLD:
            # If person is too far (small), move forward
            # If person is too close (large), move backward
            forward_speed = -FORWARD_SPEED * (width_error / abs(width_error))
        
        # Bang-bang control: fixed speed turn based on which side person is on
        if abs(x_error) < CENTER_THRESHOLD:
            # Person is centered - maintain forward/backward motion only
            self.current_left_speed = forward_speed
            self.current_right_speed = forward_speed
        elif x_error > 0:
            # Person is to the right - turn right
            self.current_left_speed = TURN_SPEED*abs(x_error) + forward_speed
            self.current_right_speed = -TURN_SPEED*abs(x_error) + forward_speed
        else:
            # Person is to the left - turn left
            self.current_left_speed = -TURN_SPEED*abs(x_error) + forward_speed
            self.current_right_speed = TURN_SPEED*abs(x_error) + forward_speed
        
        # Send commands to motors
        self.motor.set_speed_mps_left(self.current_left_speed)
        self.motor.set_speed_mps_right(self.current_right_speed)
        # print(self.current_left_speed, self.current_right_speed)
        
        # Save debug images
        self.save_debug_images(left, annotated, best_person)
    
    def cleanup(self):
        """Clean up resources"""
        self.motor.set_speed_mps_left(0)
        self.motor.set_speed_mps_right(0)
        self.motor.clear_errors_left()
        self.motor.clear_errors_right()

def main():
    follower = PersonFollower()
    try:
        while True:
            follower.detect_and_follow()
    except KeyboardInterrupt:
        follower.cleanup()

if __name__ == "__main__":
    main()
