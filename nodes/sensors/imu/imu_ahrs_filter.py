import time

import numpy as np

from nodes.sensors.imu.imu import IMU
from nodes.sensors.imu.madgwickahrs import MadgwickAHRS, Quaternion


class FilteredIMU():
    """
    Class for filtering IMU data using the MadgwickAHRS algorithm.
    """

    def __init__(self):
        """
        Initializes the FilteredIMU class with default values and instances.
        """
        self.imu = IMU()
        self.ahrs = MadgwickAHRS(beta=0.008, zeta=0.)
        self.accel = np.array([0, 0, 0])
        self.gyro = np.array([0, 0, 0])
        self.t = time.time()
        self.quat = Quaternion(1, 0, 0, 0)
        self.grav = np.array([0, 0, 0])
        self.ahrs.quaternion = self.quat
        self.alpha = 1  # LPF alpha: x[t] := a*x[t] + (1-a)*x[t-1]

    def calibrate(self):
        """
        Calibrates the IMU and sets the initial quaternion and gravity vector.
        """
        accel, gyro, t = self.imu.calibrate()
        self.accel, self.gyro = self.remap_accel_gyro(accel, gyro)
        self.t = t
        self.quat = self._calculate_initial_q(self.accel)
        self.grav = self.quat_rotate(self.quat.conj(), [0, 0, 1])
        self.ahrs.quaternion = self.quat

    def get_orientation(self):
        """
        Calculates the pitch, roll, and yaw angles of the robot.

        Returns
        -------
            tuple: The pitch, roll, and yaw angles in degrees.
        """
        gx, gy, gz = self.grav
        qw, qx, qy, qz = self.quat

        # Map gravity vector components to new axes
        gX_new = gy
        gY_new = -gx
        gZ_new = gz

        # Calculate pitch (technically roll about x-axis)
        pitch = np.degrees(np.arctan2(-gX_new, np.sqrt(gY_new**2 + gZ_new**2)))

        # Calculate roll (about y-axis)
        roll = np.degrees(np.arctan2(gY_new, gZ_new))

        # Calculate yaw (about z-axis)
        yaw = np.degrees(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2)))

        return pitch, roll, yaw

    def _calculate_initial_q(self, accel):
        """
        Calculates the initial quaternion based on accelerometer data.
        Args:
            accel (np.array): The accelerometer data.

        Returns
        -------
            Quaternion: The initial quaternion.
        """
        acc_norm = accel / np.linalg.norm(accel)

        # Estimate initial roll and pitch from accelerometer
        initial_roll = np.arctan2(acc_norm[1], acc_norm[2])
        initial_pitch = np.arctan2(-acc_norm[0], np.sqrt(acc_norm[1]**2 + acc_norm[2]**2))
        initial_yaw = 0

        # Initialize quaternion using the from_angle_axis function
        initial_q = Quaternion.from_angle_axis(initial_roll, 1, 0, 0)
        initial_q = initial_q * Quaternion.from_angle_axis(initial_pitch, 0, 1, 0)
        initial_q = initial_q * Quaternion.from_angle_axis(initial_yaw, 0, 0, 1)
        return initial_q

    def update(self):
        """
        Updates the IMU data and filters it using the MadgwickAHRS algorithm.
        """
        accel, gyro, gyro_raw, t = self.imu.update()
        self.accel, self.gyro = self.remap_accel_gyro(accel, gyro)

        # Store raw IMU data
        self.accel_RAW = self.accel
        self.gyro_RAW = self.gyro
        self.quat_RAW = self._calculate_initial_q(self.accel_RAW)
        self.grav_RAW = self.quat_rotate(self.quat_RAW.conj(), [0, 0, 1])

        # Filtering
        self.ahrs.samplePeriod = t - self.t
        self.ahrs.update_imu(self.gyro, self.accel)
        self.t = t

        # Setting vars
        quat = self.ahrs.quaternion
        self.quat = quat.q
        self.grav = self.quat_rotate(quat.conj(), [0, 0, 1])

    def remap_accel_gyro(self, accel, gyro):
        """
        Remaps the accelerometer and gyroscope data to a new coordinate system.
        Args:
            accel (np.array): The accelerometer data.
            gyro (np.array): The gyroscope data.

        Returns
        -------
            tuple: The remapped accelerometer and gyroscope data.
        """
        accel = np.array([-accel[1], accel[0], accel[2]])
        gyro = np.array([-gyro[1], gyro[0], gyro[2]])
        return accel, gyro

    def quat_rotate(self, q, v):
        """
        Rotates a vector by a quaternion.
        Args:
            q (Quaternion): The quaternion.
            v (list): The vector to rotate.

        Returns
        -------
            np.array: The rotated vector.
        """
        qv = np.concatenate(([0], v))
        return (q * Quaternion(qv) * q.conj()).q[1:]

if __name__ == "__main__":
    imu = FilteredIMU()
    imu.calibrate()

    while True:
        imu.update()
        pitch, roll, yaw = imu.get_orientation()
        print(f"roll: {roll}")
        time.sleep(1/200)
