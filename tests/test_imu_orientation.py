import time

import numpy as np

from nodes.sensors.imu.imu_ahrs_filter import FilteredIMU


def determine_imu_orientation():
    print("This test will help determine IMU orientation by measuring accelerations and rotations")
    print("during manual movements of the robot")
    
    # Initialize IMU
    imu = FilteredIMU()
    
    # Calibrate IMU
    print("Calibrating IMU...")
    imu.calibrate()
    time.sleep(1)

    def collect_readings():
        readings = []
        print("Move the robot for 5 seconds...")
        start_time = time.time()
        while time.time() - start_time < 5:
            accel, gyro = imu.imu.read_sensor()
            readings.append((accel, gyro))
            time.sleep(0.02)
        return np.array(readings)

    def analyze_movement(readings, movement_type):
        accel_data = np.array([r[0] for r in readings])
        gyro_data = np.array([r[1] for r in readings])
        
        accel_range = np.ptp(accel_data, axis=0)
        gyro_range = np.ptp(gyro_data, axis=0)
        
        if movement_type == "linear":
            max_idx = np.argmax(accel_range)
            value = accel_range[max_idx]
            signal = "acceleration"
        else:  # pitch or yaw
            max_idx = np.argmax(gyro_range)
            value = gyro_range[max_idx]
            signal = "rotation"
            
        axis_names = ['X', 'Y', 'Z']
        return axis_names[max_idx], value, signal

    # Test linear forward motion
    input("\n=== Testing LINEAR FORWARD motion ===\nPress Enter, then move the robot forward and backward linearly...")
    readings = collect_readings()
    axis, value, signal = analyze_movement(readings, "linear")
    print(f"Linear motion primarily detected on {axis}-axis ({value:.3f} m/s² {signal})")
    
    # Test pitch motion
    input("\n=== Testing PITCH motion ===\nPress Enter, then tilt the robot forward and backward (pitch)...")
    readings = collect_readings()
    axis, value, signal = analyze_movement(readings, "pitch")
    print(f"Pitch motion primarily detected on {axis}-axis ({value:.3f} rad/s {signal})")
    
    # Test yaw motion
    input("\n=== Testing YAW motion ===\nPress Enter, then rotate the robot left and right (yaw)...")
    readings = collect_readings()
    axis, value, signal = analyze_movement(readings, "yaw")
    print(f"Yaw motion primarily detected on {axis}-axis ({value:.3f} rad/s {signal})")

if __name__ == '__main__':
    determine_imu_orientation()