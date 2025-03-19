import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from lib.package_utils import ensure_package

ensure_package("numpy")
ensure_package("opencv-python")
ensure_package("mediapipe")

import json
import numpy as np
import cv2
import mediapipe as mp
from lib.camera import StereoCamera
from lib.odrive_uart import ODriveUART

# Load motor directions
with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'r') as f:
    motor_dirs = json.load(f)

# Initialize motor controller
motor = ODriveUART(
    port='/dev/ttyAMA1',
    left_axis=0, right_axis=1,
    dir_left=motor_dirs['left'], 
    dir_right=motor_dirs['right']
)

# Initialize camera
camera = StereoCamera()

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)
mp_draw = mp.solutions.drawing_utils

# Start motors
motor.start_left()
motor.start_right()
motor.enable_velocity_mode_left()
motor.enable_velocity_mode_right()
motor.disable_watchdog_left()
motor.disable_watchdog_right()
motor.clear_errors_left()
motor.clear_errors_right()

# Constants
SPEED = 0.15  # Speed in meters per second

def check_hand_gesture(hand_landmarks):
    """Check if the hand is open or closed"""
    # Get y coordinates of finger tips and middle points
    tips = [hand_landmarks.landmark[tip].y for tip in [8, 12, 16, 20]]  # Index, Middle, Ring, Pinky
    mids = [hand_landmarks.landmark[mid].y for mid in [6, 10, 14, 18]]  # Middle points of fingers
    
    # Check how many fingers are extended
    extended_fingers = sum(tip < mid for tip, mid in zip(tips, mids))
    
    if extended_fingers >= 3:
        return "open"  # Open hand - forward
    elif extended_fingers <= 1:
        return "closed"  # Closed fist - backward
    else:
        return "other"  # Other gesture - stop

try:
    print("Hand gesture controls:")
    print("  - Show OPEN HAND (fingers extended) to move FORWARD")
    print("  - Show CLOSED FIST to move BACKWARD")
    print("  - Any other gesture or no hand to STOP")
    print("Press Ctrl+C to stop")
    
    while True:
        # Get image from left camera (we'll use only left for hand detection)
        left, _ = camera.get_stereo()
        
        if left is None:
            print("Failed to capture image")
            continue
        
        # Convert to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(left, cv2.COLOR_BGR2RGB)
        
        # Process the frame
        results = hands.process(rgb_frame)
        
        # Default to stopping
        movement = "stop"
        
        # Check for hand landmarks
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks for debugging
                mp_draw.draw_landmarks(left, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Check hand gesture
                gesture = check_hand_gesture(hand_landmarks)
                if gesture == "open":
                    movement = "forward"
                    print("Open hand detected - Moving forward!")
                elif gesture == "closed":
                    movement = "backward"
                    print("Closed fist detected - Moving backward!")
        
        # Set motor speeds based on hand gesture
        if movement == "forward":
            motor.set_speed_mps_left(SPEED)
            motor.set_speed_mps_right(SPEED)
        elif movement == "backward":
            motor.set_speed_mps_left(-SPEED)
            motor.set_speed_mps_right(-SPEED)
        else:
            motor.set_speed_mps_left(0)
            motor.set_speed_mps_right(0)
        
        # Save debug image
        cv2.imwrite("hand_debug.jpg", left)

except KeyboardInterrupt:
    print("\nStopping...")
except Exception as e:
    print(f"Error: {e}")
finally:
    # Cleanup
    motor.set_speed_mps_left(0)
    motor.set_speed_mps_right(0)
    motor.clear_errors_left()
    motor.clear_errors_right()
    camera.release()
    hands.close()
    print("Shutdown complete") 