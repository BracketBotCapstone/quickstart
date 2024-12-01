import json
import time
import numpy as np
from nodes.control.balance_control.odesc_uart import ODESC
from nodes.control.balance_control.imu import FilteredLSM6DS3

def test_motor_direction():
    # Initialize IMU and motors
    imu = FilteredLSM6DS3()
    imu.calibrate()

    motor_controller = ODESC('/dev/ttyAMA1', left_axis=1, right_axis=0, dir_left=1, dir_right=1)
    
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

        # Get baseline gyro reading
        imu.update()
        baseline_gyro = imu.gyro_RAW[2]  # Z-axis rotation
        print(f"Baseline gyro: {baseline_gyro}")
        
        # Spin motor
        if name == 'left':
            motor_controller.set_speed_rpm_left(30)
        else:
            motor_controller.set_speed_rpm_right(30)

        curr_time = time.time()
        while time.time() - curr_time < 0.5:
            time.sleep(0.01)
            imu.update()

        # Get gyro reading during spin
        spin_gyro = imu.gyro_RAW[2]
        print(f"Spin gyro: {spin_gyro}")

        # Stop motor
        if name == 'left':
            motor_controller.stop_left()
        else:
            motor_controller.stop_right()
    
        # Determine direction based on gyro reading
        # Positive gyro means counterclockwise rotation when viewed from above
        gyro_diff = spin_gyro - baseline_gyro
        print(f"Gyro difference: {gyro_diff}")
        
        # Set direction based on gyro reading
        # We want positive direction to be forward motion
        if name == 'left':
            directions['left'] = -1 if gyro_diff > 0 else 1
        else:
            directions['right'] = 1 if gyro_diff > 0 else -1
        
        time.sleep(0.5)  # Wait between tests
    
    # Save results to file
    with open('motor_dir.json', 'w') as f:
        json.dump(directions, f)
    
    print("\nDirection test complete!")
    print(f"Left direction: {directions['left']}, Right direction: {directions['right']}")
    print(f"Results saved to motor_dir.json: {directions}")

if __name__ == '__main__':
    try:
        test_motor_direction()
    except Exception as e:
        print(f"Error occurred: {e}")
