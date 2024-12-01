import time
import logging
import numpy as np

import board
import numpy as np
import adafruit_lsm6ds
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3
from madgwick_py.madgwickahrs import MadgwickAHRS, Quaternion


from utils.messages.gyro_data_msg import GYRO_DATA_MSG
from utils.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from utils.topic_to_message_type import topic_to_message_type, TOPIC_GYRO, TOPIC_RAW_IMU, TOPIC_RAW_IMU_UNSCALED_BIASED
from utils.mqtt_utils import MQTTPublisher
from utils.logging_utils import Logger


class IMU():
    def __init__(self):
        self.sensor = LSM6DS3(board.I2C())
        self.sensor.gyro_range = adafruit_lsm6ds.GyroRange.RANGE_1000_DPS  # Set gyroscope range to ±1000 dps
        self.gyro_bias = np.array([0.,0.,0.])

    def calibrate(self):
        self.gyro_bias = np.array([0.,0.,0.])
        for _ in range(50):
            self.gyro_bias += np.array(self.sensor.gyro) / 50
            time.sleep(0.01)
        print('Calculated gyro bias:', self.gyro_bias)

        self.accel = np.array(self.sensor.acceleration)
        self.gyro = np.array(self.sensor.gyro) - self.gyro_bias
        self.t = time.time()

        return self.accel, self.gyro, self.t

    def update(self):
        # sensor readings
        self.accel = np.array(self.sensor.acceleration)
        self.gyro = np.array(self.sensor.gyro) - self.gyro_bias
        self.gyro_raw = np.array(self.sensor.gyro)
        self.t = time.time()

        return self.accel, self.gyro, self.gyro_raw, self.t

    def get_accel(self):
        """
        Returns the accelerometer data
        """
        return self.accel

    def get_gyro(self):
        """
        Returns the gyroscope data with bias correction
        """
        return self.gyro
    
    def get_gyro_raw(self):
        """
        Returns the raw gyroscope data without bias correction
        """
        return self.gyro_raw
    
    def get_time(self):
        """
        Returns the timestamp of the sensor reading since epoch
        """
        return self.t
    

class FilteredIMU():
    """
    Class for filtering IMU data using the MadgwickAHRS algorithm
    """
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
        return np.degrees(np.arctan2(gz, np.sqrt(gx**2 + gy**2)))
    
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
        # self.update()
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



class IMU_Node(object):
    def __init__(self, logging_level = logging.INFO):
        self.logger = Logger('imu_node', 'logs/imu_node.log', level=logging_level)
        self.imu = IMU()
        self.filtered_imu = FilteredIMU()
        self.mqtt_publisher = MQTTPublisher(broker_address="localhost",
                                            topic_to_message_map={TOPIC_GYRO: GYRO_DATA_MSG,
                                                                  TOPIC_RAW_IMU: RAW_IMU_DATA_MSG,
                                                                  TOPIC_RAW_IMU_UNSCALED_BIASED: RAW_IMU_DATA_MSG})
        self.loop_rate_hz = 200

    def run(self):
        self.mqtt_publisher.run()

        init_cycle = True

        try:
            while True:
                start_time = time.time()
                if init_cycle:
                    accel, gyro, t = self.imu.calibrate()
                    self.filtered_imu.calibrate(t, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2])
                    init_cycle = False
                else:
                    accel, gyro, gyro_raw, t = self.imu.update()
                    self.filtered_imu.update(t, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2])
                    pitch, roll, yaw = self.filtered_imu.get_orientation()

                    gyro_msg = GYRO_DATA_MSG(timestamp=t, pitch=pitch, roll=roll, yaw=yaw)
                    self.mqtt_publisher.publish_msg(TOPIC_GYRO, gyro_msg)

                    raw_imu_data = RAW_IMU_DATA_MSG()
                    raw_imu_data.timestamp = t
                    raw_imu_data.accel_x = accel[0]
                    raw_imu_data.accel_y = accel[1]
                    raw_imu_data.accel_z = accel[2]
                    raw_imu_data.gyro_x = gyro[0]
                    raw_imu_data.gyro_y = gyro[1]
                    raw_imu_data.gyro_z = gyro[2]
                    self.mqtt_publisher.publish_msg(TOPIC_RAW_IMU, raw_imu_data)

                    raw_imu_data_unscaled_biased = RAW_IMU_DATA_MSG()
                    raw_imu_data_unscaled_biased.timestamp = t
                    raw_imu_data_unscaled_biased.accel_x = accel[0]
                    raw_imu_data_unscaled_biased.accel_y = accel[1]
                    raw_imu_data_unscaled_biased.accel_z = accel[2]
                    raw_imu_data_unscaled_biased.gyro_x = gyro_raw[0]
                    raw_imu_data_unscaled_biased.gyro_y = gyro_raw[1]
                    raw_imu_data_unscaled_biased.gyro_z = gyro_raw[2]
                    self.mqtt_publisher.publish_msg(TOPIC_RAW_IMU_UNSCALED_BIASED, raw_imu_data_unscaled_biased)

                end_time = time.time()
                sleep_time = 1/self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            self.logger.info("IMU stopped by user.")
        finally:
            self.mqtt_publisher.stop()

if __name__ == "__main__":
    imu_node = IMU_Node()
    imu_node.run()
