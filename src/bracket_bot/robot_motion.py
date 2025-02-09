# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

import time
import math
from lib.odrive_uart import ODriveUART
from lib.imu import FilteredMPU6050
import json
import os

class RobotController:
    def __init__(self):
        # Initialize IMU
        self.imu = FilteredMPU6050()
        self.imu.calibrate()

        # Load motor directions from config
        try:
            with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'r') as f:
                motor_dirs = json.load(f)
                left_dir = motor_dirs['left']
                right_dir = motor_dirs['right']
        except Exception as e:
            raise Exception("Error reading motor_dir.json")

        # Initialize ODrive
        self.motor_controller = ODriveUART(
            port='/dev/ttyAMA1', 
            left_axis=0, 
            right_axis=1,
            dir_left=left_dir, 
            dir_right=right_dir
        )

        # Constants
        self.WHEEL_DIAMETER = 0.165  # meters
        self.WHEEL_CIRCUMFERENCE = self.WHEEL_DIAMETER * math.pi
        self.MAX_SPEED = 0.5  # m/s
        self.ACCEL_RATE = 0.15
        self.DECEL_DISTANCE = 0.15  # meters
        self.TURN_SPEED = 0.2
        self.YAW_THRESHOLD = 2.0  # degrees

        # Start motors
        self.start_motors()

    def start_motors(self):
        """Initialize and start the motors in velocity mode"""
        self.motor_controller.start_left()
        self.motor_controller.start_right()
        self.motor_controller.enable_velocity_mode_left()
        self.motor_controller.enable_velocity_mode_right()

    def drive_distance(self, distance_meters):
        """Drive forward/backward a specified distance in meters"""
        print(f"Driving {distance_meters} meters...")
        start_time = time.time()
        # Calculate target position
        target_rotations = distance_meters / self.WHEEL_CIRCUMFERENCE
        initial_pos = self.motor_controller.get_position_turns_left()
        target_pos = initial_pos + target_rotations
        
        # Accelerate
        current_speed = 0.0
        while current_speed < self.MAX_SPEED:
            elapsed = time.time() - start_time
            current_speed = min(self.ACCEL_RATE * elapsed, self.MAX_SPEED)
            self.motor_controller.set_speed_mps_left(current_speed)
            self.motor_controller.set_speed_mps_right(current_speed)
            time.sleep(0.01)

        # Monitor position until target
        while True:
            current_pos = self.motor_controller.get_position_turns_left()
            distance_remaining = (target_pos - current_pos) * self.WHEEL_CIRCUMFERENCE
            
            if distance_remaining < self.DECEL_DISTANCE:
                current_speed = max(0.05, (distance_remaining / self.DECEL_DISTANCE) * self.MAX_SPEED)
                self.motor_controller.set_speed_mps_left(current_speed)
                self.motor_controller.set_speed_mps_right(current_speed)
                
            if current_pos >= target_pos:
                break
            time.sleep(0.01)

        self.stop_motors()
        end_time = time.time()
        print(f"Drive distance of {distance_meters} meters completed in {end_time - start_time:.2f} seconds.")

    def turn_degrees(self, degrees):
        """Turn the robot by a specified number of degrees (positive = left, negative = right)"""
        print(f"Turning {degrees} degrees...")
        start_time = time.time()
        # Get initial yaw
        _, _, initial_yaw = self.imu.get_orientation()
        initial_yaw = initial_yaw % 360
        if initial_yaw < 0:
            initial_yaw += 360

        # Initialize unwrapped yaw tracking
        previous_yaw = initial_yaw
        unwrapped_yaw = 0.0
        target_unwrapped_yaw = -degrees  # Negative because of coordinate system

        stable_start_time = None
        while True:
            _, _, current_yaw = self.imu.get_orientation()
            current_yaw = current_yaw % 360
            if current_yaw < 0:
                current_yaw += 360

            # Calculate delta_yaw
            delta_yaw = current_yaw - previous_yaw
            if delta_yaw > 180:
                delta_yaw -= 360
            elif delta_yaw < -180:
                delta_yaw += 360

            unwrapped_yaw += delta_yaw
            previous_yaw = current_yaw
            yaw_diff = target_unwrapped_yaw - unwrapped_yaw

            # Set motor speeds based on turn direction
            if yaw_diff > 0:  # Need to turn clockwise
                self.motor_controller.set_speed_mps_left(-self.TURN_SPEED)
                self.motor_controller.set_speed_mps_right(self.TURN_SPEED)
            else:  # Need to turn counter-clockwise
                self.motor_controller.set_speed_mps_left(self.TURN_SPEED)
                self.motor_controller.set_speed_mps_right(-self.TURN_SPEED)

            # Check if we're done turning
            if abs(yaw_diff) < self.YAW_THRESHOLD:
                if stable_start_time is None:
                    stable_start_time = time.time()
                elif time.time() - stable_start_time >= 1.0:
                    break
            else:
                stable_start_time = None
                
            time.sleep(0.01)

        self.stop_motors()
        end_time = time.time()
        print(f"Turn of {degrees} degrees completed in {end_time - start_time:.2f} seconds.")

    def stop_motors(self):
        """Stop both motors"""
        print("Stopping motors...")
        start_time = time.time()
        self.motor_controller.stop_left()
        self.motor_controller.stop_right()
        time.sleep(0.5)
        end_time = time.time()
        print(f"Motors stopped in {end_time - start_time:.2f} seconds.")

    def cleanup(self):
        """Cleanup method to ensure motors are stopped"""
        self.stop_motors()