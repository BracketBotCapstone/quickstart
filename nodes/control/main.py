import logging
import os
import time

import utils.constants as CFG
from nodes.control.lqr_balance import BalanceController
from utils.logging_utils import Logger
from utils.messages.gyro_data_msg import GYRO_DATA_MSG
from utils.messages.localization_initialized_msg import LOCALIZATION_INITIALIZED_MSG
from utils.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from utils.messages.robot_extended_pose_msg import ROBOT_EXTENDED_POSE_MSG
from utils.messages.target_velocity_msg import TARGET_VELOCITY_MSG
from utils.messages.trajectory_msg import TRAJECTORY_MSG
from utils.messages.wheel_velocities_data_msg import WHEEL_VELOCITIES_DATA_MSG
from utils.mqtt_utils import MQTTPublisher, MQTTSubscriber
from utils.topic_to_message_type import (
    TOPIC_GYRO,
    TOPIC_LOCALIZATION_INITIALIZED,
    TOPIC_PROCESSED_IMU,
    TOPIC_RAW_IMU,
    TOPIC_ROBOT_EXTENDED_POSE,
    TOPIC_TARGET_VELOCITY,
    TOPIC_TRAJECTORY,
    TOPIC_WHEEL_VELOCITIES,
)


class BalanceControl(object):
    def __init__(self, logging_level = logging.INFO):
        """
        Initialize the BalanceControl class.

        :param logging_level: The logging level to use for the logger.
        """
        self.logger = Logger('control', 'logs/control.log', level=logging_level)

        self.mqtt_subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={TOPIC_GYRO: GYRO_DATA_MSG,
                                                                                                TOPIC_RAW_IMU: RAW_IMU_DATA_MSG,
                                                                                                TOPIC_TRAJECTORY: TRAJECTORY_MSG,
                                                                                                TOPIC_LOCALIZATION_INITIALIZED: LOCALIZATION_INITIALIZED_MSG,
                                                                                                TOPIC_ROBOT_EXTENDED_POSE: ROBOT_EXTENDED_POSE_MSG,
                                                                                                TOPIC_PROCESSED_IMU: RAW_IMU_DATA_MSG,
                                                                                                TOPIC_TARGET_VELOCITY: TARGET_VELOCITY_MSG})
        self.mqtt_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_WHEEL_VELOCITIES: WHEEL_VELOCITIES_DATA_MSG})

        self.loop_rate_hz = 300

    def run(self):
        self.mqtt_subscriber.start()
        self.mqtt_publisher.run()

        enable_motor_control = os.getenv("ENABLE_MOTOR_CONTROL", CFG.MOTOR_CONTROL_ENABLE_MOTOR_CONTROL)
        controller = BalanceController(standalone=False,
                                        enable_motor_control=True if enable_motor_control == "1" else False,
                                        logger=self.logger)
        # Initialize variables to store sensor data
        imu_data = None
        raw_imu_data_dict = None

        self.velocity_target = 0.0
        self.yaw_rate_target = 0.0
        self.trajectory_msg = None
        self.localization_initialized = False
        self.robot_extended_pose_msg = None
        self.processed_imu_msg = None
        self.target_velocity_msg = None

        try:
            while True:
                start_time = time.time()
                current_time = time.time()

                gyro_data = self.mqtt_subscriber.get_latest_message(TOPIC_GYRO)
                raw_imu_data = self.mqtt_subscriber.get_latest_message(TOPIC_RAW_IMU)
                trajectory_msg: TRAJECTORY_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_TRAJECTORY)
                localization_initialized_msg: LOCALIZATION_INITIALIZED_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_LOCALIZATION_INITIALIZED)
                robot_extended_pose_msg: ROBOT_EXTENDED_POSE_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_ROBOT_EXTENDED_POSE)
                processed_imu_msg: RAW_IMU_DATA_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_PROCESSED_IMU)
                target_velocity_msg: TARGET_VELOCITY_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_TARGET_VELOCITY)
        
                if localization_initialized_msg is not None:
                    self.localization_initialized = localization_initialized_msg.initialized

                if trajectory_msg is not None:
                    # Only update the trajectory message if it has changed/new one is available
                    self.trajectory_msg = trajectory_msg

                if robot_extended_pose_msg is not None:
                    self.robot_extended_pose_msg = robot_extended_pose_msg

                if processed_imu_msg is not None:
                    self.processed_imu_msg = processed_imu_msg

                if gyro_data is not None:
                    timestamp = gyro_data.timestamp
                    imu_data = {}
                    imu_data['orientation'] = [0, 0, 0]
                    imu_data['orientation'][0] = gyro_data.pitch
                    imu_data['orientation'][1] = gyro_data.roll
                    imu_data['orientation'][2] = gyro_data.yaw
                else:
                    imu_data = None

                if raw_imu_data is not None:
                    raw_imu_data_dict = {}
                    raw_imu_data_dict['accel'] = [0, 0, 0]
                    raw_imu_data_dict['accel'][0] = raw_imu_data.accel_x
                    raw_imu_data_dict['accel'][1] = raw_imu_data.accel_y
                    raw_imu_data_dict['accel'][2] = raw_imu_data.accel_z
                    raw_imu_data_dict['gyro'] = [0, 0, 0]
                    raw_imu_data_dict['gyro'][0] = raw_imu_data.gyro_x
                    raw_imu_data_dict['gyro'][1] = raw_imu_data.gyro_y
                    raw_imu_data_dict['gyro'][2] = raw_imu_data.gyro_z
                else:
                    raw_imu_data_dict = None

                # if self.robot_extended_pose_msg is not None: 
                #     imu_data = {}
                #     imu_data['orientation'] = [0, 0, 0]
                #     imu_data['orientation'][0] = self.robot_extended_pose_msg.phi_a_b_x_rad
                #     imu_data['orientation'][1] = -self.robot_extended_pose_msg.phi_a_b_y_rad
                #     imu_data['orientation'][2] = -self.robot_extended_pose_msg.phi_a_b_z_rad
                # else:
                #     imu_data = None

                # if self.processed_imu_msg is not None:
                #     raw_imu_data_dict = {}
                #     raw_imu_data_dict['accel'] = [0, 0, 0]
                #     raw_imu_data_dict['accel'][0] = self.processed_imu_msg.accel_x
                #     raw_imu_data_dict['accel'][1] = self.processed_imu_msg.accel_y
                #     raw_imu_data_dict['accel'][2] = self.processed_imu_msg.accel_z
                #     raw_imu_data_dict['gyro'] = [0, 0, 0]
                #     raw_imu_data_dict['gyro'][0] = self.processed_imu_msg.gyro_y
                #     raw_imu_data_dict['gyro'][1] = self.processed_imu_msg.gyro_x
                #     raw_imu_data_dict['gyro'][2] = self.processed_imu_msg.gyro_z
                # else:
                #     raw_imu_data_dict = None

                if target_velocity_msg is not None:
                    self.velocity_target = target_velocity_msg.linear_velocity_mps
                    self.yaw_rate_target = target_velocity_msg.angular_velocity_radps
                else:
                    if self.trajectory_msg is not None:
                        trajectory = self.trajectory_msg.trajectory
                        # find the first time in the trajectory that is greater than the current time
                        found_time = False
                        for _time, vel_yaw_rate in trajectory.items():
                            if float(_time) > current_time:
                                self.velocity_target = vel_yaw_rate[0]
                                self.yaw_rate_target = vel_yaw_rate[1]
                                found_time = True
                                break
                        if not found_time:
                            self.velocity_target = 0.0
                            self.yaw_rate_target = 0.0
                    else:
                        self.velocity_target = 0.0
                        self.yaw_rate_target = 0.0

                # Call the balance_step function with the updated imu_data
                current_time = time.time()
                if imu_data is not None and raw_imu_data_dict is not None and self.localization_initialized:
                    l_vel_mps, r_vel_mps = controller.balance_step(gyro_filtered=imu_data['orientation'],
                                                                    imu_gyro_data=raw_imu_data_dict['gyro'],
                                                                    velocity_target_mps=self.velocity_target,
                                                                    yaw_rate_target_rad_s=self.yaw_rate_target,
                                                                    robot_pose=self.robot_extended_pose_msg)

                    wheel_velocities_msg = WHEEL_VELOCITIES_DATA_MSG()
                    wheel_velocities_msg.timestamp = time.time()
                    wheel_velocities_msg.left_vel_mps = l_vel_mps
                    wheel_velocities_msg.right_vel_mps = r_vel_mps
                    self.mqtt_publisher.publish_msg(TOPIC_WHEEL_VELOCITIES, wheel_velocities_msg)

                end_time = time.time()
                sleep_time = 1/self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            self.logger.info("Balance stopped by user.")
        finally:
            controller.stop()
            self.mqtt_subscriber.stop()
            self.mqtt_publisher.stop()

if __name__ == "__main__":
    balance_control = BalanceControl()
    balance_control.run()
