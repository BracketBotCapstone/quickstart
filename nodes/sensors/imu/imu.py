import time

import board
import numpy as np
from adafruit_mpu6050 import MPU6050


class IMU():
    def __init__(self):
        """
        Initializes the IMU sensor and sets up initial parameters.
        """
        self.sensor = MPU6050(board.I2C())  # Initialize the MPU6050 sensor
        # self.sensor.gyro_range = adafruit_mpu6050.GyroRange.RANGE_1000_DPS # Set gyroscope range to ±1000 dps
        self.gyro_bias = np.array([0., 0., 0.])  # Initialize gyroscope bias

    def calibrate(self):
        """
        Calibrates the gyroscope by averaging 50 readings to determine the bias.
        
        Returns
        -------
        tuple
            A tuple containing the accelerometer data, gyroscope data with bias correction, 
            and the timestamp of the calibration.
        """
        self.gyro_bias = np.array([0., 0., 0.])  # Reset gyro bias
        for _ in range(50):
            _, gyro = self.read_sensor()  # Read sensor data
            self.gyro_bias += gyro / 50  # Accumulate gyro data for bias calculation
            time.sleep(0.01)  # Wait for 10ms between readings
        print('Calculated gyro bias:', self.gyro_bias)

        self.accel, gyro_raw = self.read_sensor()  # Get initial sensor readings
        self.gyro = gyro_raw - self.gyro_bias  # Apply bias correction
        self.t = time.monotonic()  # Record the current time

        return self.accel, self.gyro, self.t

    def read_sensor(self):
        """
        Reads the accelerometer and gyroscope data from the sensor.

        Returns
        -------
        tuple
            A tuple containing the accelerometer data and gyroscope data.
        """
        accel = np.array(self.sensor.acceleration)  # Get accelerometer data
        gyro = np.array(self.sensor.gyro)  # Get gyroscope data
        return accel, gyro

    def update(self):
        """
        Updates the IMU data by reading the sensor and applying bias correction.

        Returns
        -------
        tuple
            A tuple containing the accelerometer data, gyroscope data with bias correction,
            raw gyroscope data, and the timestamp of the sensor reading.
        """
        accel, gyro = self.read_sensor()  # Read sensor data
        self.accel = accel  # Update accelerometer data
        self.gyro = gyro - self.gyro_bias  # Apply bias correction to gyroscope data
        self.gyro_raw = gyro  # Store raw gyroscope data
        self.t = time.monotonic()  # Record the current time

        return self.accel, self.gyro, self.gyro_raw, self.t

    def get_accel(self):
        """
        Returns the accelerometer data.

        Returns
        -------
        numpy.ndarray
            The accelerometer data.
        """
        return self.accel

    def get_gyro(self):
        """
        Returns the gyroscope data with bias correction.

        Returns
        -------
        numpy.ndarray
            The gyroscope data with bias correction.
        """
        return self.gyro
    
    def get_gyro_raw(self):
        """
        Returns the raw gyroscope data without bias correction.

        Returns
        -------
        numpy.ndarray
            The raw gyroscope data.
        """
        return self.gyro_raw
    
    def get_time(self):
        """
        Returns the timestamp of the sensor reading since epoch.

        Returns
        -------
        float
            The timestamp of the sensor reading.
        """
        return self.t

if __name__ == "__main__":
    imu = IMU()  # Create an instance of the IMU class
    imu.calibrate()  # Calibrate the IMU
    while True:
        accel, gyro, gyro_raw, t = imu.update()  # Update and get sensor data
        print("Accelerometer: ", accel)
        print("Gyroscope: ", gyro)
        print("Gyroscope Raw: ", gyro_raw)
        print("Timestamp: ", t)
        time.sleep(0.1)  # Wait for 100ms before the next update
