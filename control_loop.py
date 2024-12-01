import os
import argparse
import time
import logging

from lqr_balance import BalanceController
import utils.constants as CFG

from utils.messages.wheel_velocities_data_msg import WHEEL_VELOCITIES_DATA_MSG
from utils.messages.gyro_data_msg import GYRO_DATA_MSG
from utils.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from utils.messages.trajectory_msg import TRAJECTORY_MSG

from utils.topic_to_message_type import TOPIC_GYRO, TOPIC_RAW_IMU, TOPIC_WHEEL_VELOCITIES, TOPIC_TRAJECTORY
from utils.mqtt_utils import MQTTPublisher, MQTTSubscriber

from utils.logging_utils import Logger

class BalanceControl(object):
    def __init__(self, logging_level = logging.INFO):
        self.logger = Logger('balance_control', 'logs/balance_control.log', level=logging_level)
        self.last_balance_step_time = time.time()
        self.last_wheel_vel_time = time.time()
        self.start_time = time.time()
        self.init = False

        self.mqtt_subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={TOPIC_GYRO: GYRO_DATA_MSG,
                                                                                                TOPIC_RAW_IMU: RAW_IMU_DATA_MSG,
                                                                                                TOPIC_TRAJECTORY: TRAJECTORY_MSG})
        self.mqtt_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_WHEEL_VELOCITIES: WHEEL_VELOCITIES_DATA_MSG})

        self.loop_rate_hz = 200

    def run(self):
        self.mqtt_subscriber.start()
        self.mqtt_publisher.run()

        enable_motor_control = os.getenv("ENABLE_MOTOR_CONTROL", CFG.MOTOR_CONTROL_ENABLE_MOTOR_CONTROL)
        controller = BalanceController(enable_motor_control=True if enable_motor_control == "1" else False,
                                       logger=self.logger)
        # Initialize variables to store sensor data
        imu_data = None
        raw_imu_data_dict = None
        imu_timestamp = 0

        self.velocity_target = None
        self.yaw_rate_target = None
        self.trajectory_msg = None

        try:
            while True:
                start_time = time.time()
                current_time = time.time()

                gyro_data = self.mqtt_subscriber.get_latest_message(TOPIC_GYRO)
                raw_imu_data = self.mqtt_subscriber.get_latest_message(TOPIC_RAW_IMU)
                trajectory_msg: TRAJECTORY_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_TRAJECTORY)

                if trajectory_msg is not None:
                    self.trajectory_msg = trajectory_msg

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
                    imu_timestamp = raw_imu_data.timestamp
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

                if self.trajectory_msg is not None:
                    trajectory = self.trajectory_msg.trajectory
                    # find the first time in the trajectory that is greater than the current time
                    for _time, vel_yaw_rate in trajectory.items():
                        if float(_time) > current_time:
                            print(f"Setting velocity target to {vel_yaw_rate[0]} and yaw rate target to {vel_yaw_rate[1]}")
                            self.velocity_target = vel_yaw_rate[0]
                            self.yaw_rate_target = vel_yaw_rate[1]
                            break
                    self.velocity_target = 0.0
                    self.yaw_rate_target = 0.0
                else:
                    self.velocity_target = 0.0
                    self.yaw_rate_target = 0.0

                if not self.init and (time.time() - self.start_time) >= 2:
                    self.init = True

                # Call the balance_step function with the updated imu_data
                current_time = time.time()
                if imu_data is not None and raw_imu_data_dict is not None:
                    l_vel_mps, r_vel_mps = controller.balance_step(gyro_filtered=imu_data['orientation'],
                                                                    imu_gyro_data=raw_imu_data_dict['gyro'],
                                                                    imu_accel_data=raw_imu_data_dict['accel'],
                                                                    imu_timestamp=imu_timestamp,
                                                                    enable=self.init,
                                                                    velocity_target=self.velocity_target,
                                                                    yaw_rate_target=self.yaw_rate_target)
                    # Calculate and print the lag
                    lag = current_time - self.last_balance_step_time
                    # print(f"Lag between balance_step calls: {lag:.4f} seconds")
                    self.last_balance_step_time = current_time

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
    parser = argparse.ArgumentParser(description="Run the balancing robot control system.")
    args = parser.parse_args()

    balance_control = BalanceControl()
    balance_control.run()
