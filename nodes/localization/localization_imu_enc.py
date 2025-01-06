import logging
import os
import time

import numpy as np
import scipy.constants
import yaml
from navlie.lib import IMU

import utils.constants as CFG
from nodes.localization.filtering.ekf import WheelEncoderEKF
from nodes.localization.preprocessor.imu_preprocessing import IMUPreprocessor
from nodes.localization.utils.input_processing import (
    form_imu_message,
    form_localization_message,
    form_robot_extended_pose_message,
    form_robot_pose_message,
)
from nodes.localization.utils.static_initializer import StaticInitializer
from utils.logging_utils import Logger
from utils.messages.extended_pose_w_bias_msg import EXTENDED_POSE_W_BIAS_MSG
from utils.messages.localization_initialized_msg import LOCALIZATION_INITIALIZED_MSG
from utils.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from utils.messages.robot_extended_pose_msg import ROBOT_EXTENDED_POSE_MSG
from utils.messages.robot_pose_msg import ROBOT_POSE_MSG
from utils.messages.wheel_velocities_data_msg import WHEEL_VELOCITIES_DATA_MSG
from utils.mqtt_utils import MQTTPublisher, MQTTSubscriber
from utils.topic_to_message_type import (
    TOPIC_EXTENDED_POSE_W_BIAS,
    TOPIC_LOCALIZATION_INITIALIZED,
    TOPIC_PROCESSED_IMU,
    TOPIC_RAW_IMU_UNSCALED_BIASED,
    TOPIC_ROBOT_EXTENDED_POSE,
    TOPIC_ROBOT_POSE,
    TOPIC_WHEEL_VELOCITIES,
)


class LocalizationIMUEncoder:
    def __init__(self, logging_level=logging.INFO):
        """
        Initializes the LocalizationIMUEncoder class.

        Args:
            logging_level (int): The logging level for the logger.
        """
        self.logger = Logger('localization_imu_encoder', 'logs/localization_imu_encoder.log', level=logging_level)

        # Initialize MQTT subscriber and publisher
        self.mqtt_subscriber = MQTTSubscriber(
            broker_address="localhost",
            topic_to_message_map={
                TOPIC_RAW_IMU_UNSCALED_BIASED: RAW_IMU_DATA_MSG,
                TOPIC_WHEEL_VELOCITIES: WHEEL_VELOCITIES_DATA_MSG
            }
        )

        self.mqtt_publisher = MQTTPublisher(
            broker_address="localhost",
            topic_to_message_map={
                TOPIC_EXTENDED_POSE_W_BIAS: EXTENDED_POSE_W_BIAS_MSG,
                TOPIC_PROCESSED_IMU: RAW_IMU_DATA_MSG,
                TOPIC_ROBOT_POSE: ROBOT_POSE_MSG,
                TOPIC_ROBOT_EXTENDED_POSE: ROBOT_EXTENDED_POSE_MSG,
                TOPIC_LOCALIZATION_INITIALIZED: LOCALIZATION_INITIALIZED_MSG
            }
        )

        # Retrieve YAML file location
        base_config_loc = os.getenv("BASE_CONFIG_LOC", CFG.LOCALIZATION_BASE_CONFIG_LOC)
        base_config_loc = f"/home/{os.getenv('USER')}/{base_config_loc}"

        # Load configuration from YAML file
        with open(base_config_loc, 'r') as stream:
            self.loc_config = yaml.safe_load(stream)

        # Declare IMUPreprocessor and EKF
        self.imu_preprocessor = IMUPreprocessor(self.loc_config)
        self.ekf = WheelEncoderEKF(yaml_filepath=base_config_loc)

        self.static_initializer = StaticInitializer(self.loc_config)

        self.initialized = False
        self.loop_rate_hz = 200

    def run(self):
        """
        Starts the main loop for processing input data and publishing results.
        """
        self.mqtt_subscriber.start()
        self.mqtt_publisher.run()
        try:
            while True:
                start_time = time.time()
                self.process_input()
                end_time = time.time()
                sleep_time = 1/self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            self.logger.info("Stopping localization node!")
            self.mqtt_subscriber.stop()
            self.mqtt_publisher.stop()

    def process_input(self):
        """
        Processes incoming IMU and encoder data, updates the EKF, and publishes results.
        """
        imu_data = self.mqtt_subscriber.get_latest_message(TOPIC_RAW_IMU_UNSCALED_BIASED)
        encoder_data = self.mqtt_subscriber.get_latest_message(TOPIC_WHEEL_VELOCITIES)

        # Process IMU data
        if imu_data is not None:
            if not self.initialized:
                t_k, omega_b_k, a_b_k = self.imu_preprocessor.preprocess_sample(imu_data)
                
                # Add sample to static initializer
                self.static_initializer.add_sample(omega_b_k=omega_b_k, a_b_k=a_b_k, t_k=t_k)

                localization_initialized_msg = LOCALIZATION_INITIALIZED_MSG()
                localization_initialized_msg.timestamp = time.time()
                localization_initialized_msg.initialized = False
                self.mqtt_publisher.publish_msg(TOPIC_LOCALIZATION_INITIALIZED, localization_initialized_msg)

                # If static initializer is initialized, retrieve biases
                if self.static_initializer.is_initialized():
                    # Fit splines once initialization window is set
                    self.static_initializer.imu_smoother.fit_splines()

                    # Retrieve gyro bias
                    b_g = self.static_initializer.get_gyro_bias()

                    C_ab_0 = self.static_initializer.get_initial_rotation_estimate()

                    if self.loc_config['physical_params']['gravity_up']:
                        g_a = np.array([0, 0, scipy.constants.g]).reshape(3, 1)
                        b_a = self.static_initializer.get_accel_bias(C_ab_0, g_a)
                    else:
                        b_a = self.static_initializer.get_accel_bias(C_ab_0)

                    # Zero-initialize EKF with biases
                    self.ekf.static_initialize(b_g, b_a, C_ab_0)

                    print("Initialized!")
                    self.initialized = True
                    self.static_initializer.reset()
            else:
                try:
                    # TODO: figure this out, timing of imu_data is not consistent
                    t_k, omega_b_k, a_b_k = self.imu_preprocessor.input_preprocess(imu_data)
                except Exception as e:
                    print(e)
                    return

                # Aggregate IMU object
                imu_k = IMU(gyro=omega_b_k, accel=a_b_k, stamp=t_k)

                # Propagate EKF state
                self.ekf.predict(imu_k, t_k)

                # Generate unbiased IMU message
                omega_b_k_ub, a_b_k_ub = self.ekf.get_unbiased_imu(imu_k)

                # Output processed IMU message
                imu_msg = form_imu_message(t_k, omega_b_k_ub, a_b_k_ub)
                self.mqtt_publisher.publish_msg(TOPIC_PROCESSED_IMU, imu_msg)

                localization_initialized_msg = LOCALIZATION_INITIALIZED_MSG()
                localization_initialized_msg.timestamp = time.time()
                localization_initialized_msg.initialized = True
                self.mqtt_publisher.publish_msg(TOPIC_LOCALIZATION_INITIALIZED, localization_initialized_msg)

        # Process encoder data
        if encoder_data is not None:
            if (self.ekf.delta_t is None) or (not self.initialized):
                return

            t_k = encoder_data.timestamp
            v_l = encoder_data.left_vel_mps
            v_r = encoder_data.right_vel_mps

            # Correct EKF state
            self.ekf.correct(v_l, v_r)

        # Publish messages if initialized
        if self.initialized and self.ekf.delta_t is not None:
            loc_msg = form_localization_message(self.ekf.t_k, self.ekf.x_k)
            self.mqtt_publisher.publish_msg(TOPIC_EXTENDED_POSE_W_BIAS, loc_msg)

            robot_pose_msg = form_robot_pose_message(loc_msg)
            self.mqtt_publisher.publish_msg(TOPIC_ROBOT_POSE, robot_pose_msg)

            robot_extended_pose_msg = form_robot_extended_pose_message(loc_msg)
            self.mqtt_publisher.publish_msg(TOPIC_ROBOT_EXTENDED_POSE, robot_extended_pose_msg)


if __name__ == '__main__':
    loc = LocalizationIMUEncoder()
    loc.run()
