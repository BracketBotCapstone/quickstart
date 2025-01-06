import json
import os
import time

from nodes.control.odrive_uart import ODriveUART
from nodes.sensors.imu.imu_ahrs_filter import FilteredIMU


def test_motor_direction():
    """
    Test the direction of the motors using an IMU to determine the correct 
    direction for forward motion. The results are saved to a JSON file.
    """
    # Initialize IMU and motors
    imu = FilteredIMU()
    imu.calibrate()

    # Initialize motor controller with specified parameters
    motor_controller = ODriveUART('/dev/ttyAMA1', left_axis=0, right_axis=1, dir_left=1, dir_right=1)
    
    # Dictionary to store motor directions
    directions = {'left': 1, 'right': 1}

    # Test each motor
    for name in ['left', 'right']:
        print(f"\nTesting {name} motor...")
        
        # Start motor in closed loop control
        if name == 'left':
            motor_controller.start_left()
            motor_controller.enable_velocity_mode_left()
        else:
            motor_controller.start_right()
            motor_controller.enable_velocity_mode_right()

        # Clear any errors
        if name == 'left':
            if motor_controller.check_errors_left():
                print("Clearing left motor errors...")
                motor_controller.clear_errors_left()
        else:
            if motor_controller.check_errors_right():
                print("Clearing right motor errors...")
                motor_controller.clear_errors_right()

        # Get baseline gyro reading
        imu.update()
        pitch, roll, yaw = imu.get_orientation()
        print(f"Baseline gyro: {yaw}")
        
        # Spin motor at a set speed
        if name == 'left':
            motor_controller.set_speed_rpm_left(30)
        else:
            motor_controller.set_speed_rpm_right(30)

        # Allow motor to spin for a short duration
        curr_time = time.time()
        while time.time() - curr_time < 0.5:
            time.sleep(0.001)
            imu.update()

        # Get gyro reading during spin
        pitch_2, roll_2, yaw_2 = imu.get_orientation()
        print(f"Spin gyro: {yaw_2}")

        # Stop motor
        if name == 'left':
            motor_controller.stop_left()
        else:
            motor_controller.stop_right()

        # Determine direction based on gyro reading
        # Positive gyro means counterclockwise rotation when viewed from above
        yaw_diff = yaw_2 - yaw
        print(f"Gyro difference: {yaw_diff}")

        # Set direction based on gyro reading
        # We want positive direction to be forward motion
        if name == 'left':
            directions['left'] = -1 if yaw_diff > 0 else 1
        else:
            directions['right'] = 1 if yaw_diff > 0 else -1
        
        # Wait between tests
        time.sleep(0.5)
    
    # Save results to file in this script folder
    script_dir = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(script_dir, '../nodes/control/motor_dir.json'), 'w') as f:
        json.dump(directions, f)
    
    print("\nDirection test complete!")
    print(f"Left direction: {directions['left']}, Right direction: {directions['right']}")
    print(f"Results saved to motor_dir.json: {directions}")

if __name__ == '__main__':
    try:
        test_motor_direction()
    except Exception as e:
        print(f"Error occurred: {e}")
