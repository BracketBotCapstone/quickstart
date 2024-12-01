import numpy as np
import matplotlib.pyplot as plt
from nodes.control.balance_control.madgwickahrs import MadgwickAHRS, Quaternion

import time

class FilteredIMU():
    
    def __init__(self):
        self.ahrs = MadgwickAHRS(beta=0.008, zeta=0.)
        self.accel = np.array([0, 0, 0])
        self.gyro = np.array([0, 0, 0])
        self.t = time.time()
        self.quat = Quaternion(1, 0, 0, 0)
        self.grav = np.array([0, 0, 0])
        self.ahrs.quaternion = self.quat

    def calibrate(self, timestamp, ax, ay, az, gx, gy, gz):

        self.accel = np.array([ax, ay, az])
        self.gyro = np.array([gx, gy, gz])
        self.t = timestamp

        self.quat = self._calculate_initial_q(self.accel)
        self.grav = self.quat_rotate(self.quat.conj(), [0, 0, 1])
        self.ahrs.quaternion = self.quat

    def robot_angle(self):
        self.update()
        # pitch = angle of robot, it's actually about x axis so technically roll
        gx, gy, gz = self.grav
        return np.degrees(np.atan2(gz, np.sqrt(gx**2 + gy**2)))
    
    def robot_angle_RAW(self):
        gx, gy, gz = self.grav_RAW
        return np.degrees(np.arctan2(gz, np.sqrt(gx**2 + gy**2)))
    
    def robot_roll(self):
        self.update()
        gx, gy, gz = self.grav
        return np.degrees(np.arctan2(gy, gz))

    def robot_yaw(self):
        self.update()
        # Note: Yaw cannot be determined from gravity vector alone
        # We'll use the quaternion to calculate yaw
        qw, qx, qy, qz = self.quat
        return np.degrees(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2)))
    
    def get_orientation(self):
        self.update()
        gx, gy, gz = self.grav
        qw, qx, qy, qz = self.quat

        # Calculate pitch (technically roll about x-axis)
        pitch = np.degrees(np.arctan2(gz, np.sqrt(gx**2 + gy**2)))

        # Calculate roll (about y-axis)
        roll = np.degrees(np.arctan2(gy, gz))

        # Calculate yaw (about z-axis)
        yaw = np.degrees(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2)))

        return pitch, roll, yaw

    def _calculate_initial_q(self, accel):
        acc_norm = accel / np.linalg.norm(accel)

        # Estimate initial roll and pitch from accelerometer
        initial_roll = np.arctan2(acc_norm[1], acc_norm[2])
        initial_pitch = np.arctan2(-acc_norm[0], np.sqrt(acc_norm[1]**2 + acc_norm[2]**2))
        initial_yaw = 0
        # print('Initial roll, pitch, yaw:', np.degrees(initial_roll), np.degrees(initial_pitch), np.degrees(initial_yaw))

        # Initialize quaternion using the from_angle_axis function
        initial_q = Quaternion.from_angle_axis(initial_roll, 1, 0, 0)
        initial_q = initial_q * Quaternion.from_angle_axis(initial_pitch, 0, 1, 0)
        initial_q = initial_q * Quaternion.from_angle_axis(initial_yaw, 0, 0, 1)
        return initial_q

    def update(self, timestamp, ax, ay, az, gx, gy, gz):
        # sensor readings
        self.accel = np.array([ax, ay, az])
        self.gyro = np.array([gx, gy, gz])

        t = timestamp

        # store imu data
        self.accel_RAW = self.accel
        self.gyro_RAW = self.gyro
        self.quat_RAW = self._calculate_initial_q(self.accel_RAW)
        self.grav_RAW = self.quat_rotate(self.quat_RAW.conj(), [0, 0, 1])

        # filtering
        self.ahrs.samplePeriod = t - self.t
        self.ahrs.update_imu(self.gyro, self.accel)
        self.t = t

        # setting vars
        quat = self.ahrs.quaternion
        self.quat = quat.q
        self.grav = self.quat_rotate(quat.conj(), [0, 0, 1])

    def quat_rotate(self, q, v):
        """Rotate a vector v by a quaternion q"""
        qv = np.concatenate(([0], v))
        return (q * Quaternion(qv) * q.conj()).q[1:]
