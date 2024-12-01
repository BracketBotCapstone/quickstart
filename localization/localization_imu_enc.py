import os
import numpy as np
import time
import scipy.constants
import logging

import sys
sys.path.append('..')
from utils.topic_to_message_type import TOPIC_EXTENDED_POSE_W_BIAS, TOPIC_RAW_IMU_UNSCALED_BIASED, TOPIC_WHEEL_VELOCITIES, TOPIC_PROCESSED_IMU
from utils.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from utils.messages.wheel_velocities_data_msg import WHEEL_VELOCITIES_DATA_MSG
from utils.messages.extended_pose_w_bias_msg import EXTENDED_POSE_W_BIAS_MSG
from utils.mqtt_utils import MQTTSubscriber, MQTTPublisher
from utils.logging_utils import Logger
import utils.constants as CFG

from localization.preprocessor.imu_preprocessing import IMUPreprocessor
from localization.filtering.ekf import WheelEncoderEKF
from localization.utils.input_processing import form_localization_message, form_imu_message
from localization.utils.static_initializer import StaticInitializer

from navlie.lib import IMU

import yaml


import time
class LocalizationIMUEncoder:
    def __init__(self, logging_level = logging.INFO):
        self.logger = Logger('localization_imu_encoder', 'logs/localization_imu_encoder.log', level=logging_level)

        self.mqtt_subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={TOPIC_RAW_IMU_UNSCALED_BIASED: RAW_IMU_DATA_MSG, TOPIC_WHEEL_VELOCITIES: WHEEL_VELOCITIES_DATA_MSG})

        self.mqtt_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_EXTENDED_POSE_W_BIAS: EXTENDED_POSE_W_BIAS_MSG, TOPIC_PROCESSED_IMU: RAW_IMU_DATA_MSG})

        # retrieve yaml file location
        base_config_loc = os.getenv("BASE_CONFIG_LOC", CFG.LOCALIZATION_BASE_CONFIG_LOC)
        ## add /home/$USER to the beginning of the path
        base_config_loc = f"/home/{os.getenv('USER')}/{base_config_loc}"

        # retrieve yaml file
        with open(base_config_loc, 'r') as stream:
            self.loc_config = yaml.safe_load(stream)

        # declare IMUPreprocessor and EKF
        self.imu_preprocessor = IMUPreprocessor(self.loc_config)
        self.ekf = WheelEncoderEKF(yaml_filepath=base_config_loc)

        self.static_initializer = StaticInitializer(self.loc_config)

        self.loop_rate_hz = 200

    def run(self):
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

    def process_input(self, event = None):

        imu_data = self.mqtt_subscriber.get_latest_message(TOPIC_RAW_IMU_UNSCALED_BIASED)
        encoder_data = self.mqtt_subscriber.get_latest_message(TOPIC_WHEEL_VELOCITIES)

        # start_time = time.time()

        # if event is IMU input, preprocess and either static initialize or propogate EKF state
        if imu_data is not None:

            recv_time = time.time()

            if self.static_initializer.is_initialized() == False:
                t_k, omega_b_k, a_b_k = self.imu_preprocessor.preprocess_sample(imu_data)
                
                # add sample to static initializer
                self.static_initializer.add_sample(omega_b_k = omega_b_k, a_b_k = a_b_k, t_k = t_k)

                # if static initializer is initialized, retrieve biases
                if self.static_initializer.is_initialized():

                    # fit splines once initialization window is set
                    self.static_initializer.imu_smoother.fit_splines()

                    # retrieve gyro bias
                    b_g = self.static_initializer.get_gyro_bias()

                    C_ab_0 = self.static_initializer.get_initial_rotation_estimate()

                    if (self.loc_config['physical_params']['gravity_up']):
                        g_a = np.array([0, 0, scipy.constants.g]).reshape(3, 1)
                        b_a = self.static_initializer.get_accel_bias(C_ab_0, g_a)
                    else:
                        b_a = self.static_initializer.get_accel_bias(C_ab_0)

                    # zero-initialize EKF with biases
                    self.ekf.static_initialize(b_g, b_a, C_ab_0)
            else:
                try:
                    # TODO: figure this out
                    t_k, omega_b_k, a_b_k = self.imu_preprocessor.input_preprocess(imu_data)
                except Exception as e:
                    print(e)
                    return

                # aggregate IMU object
                imu_k = IMU(gyro = omega_b_k, accel = a_b_k, stamp=t_k)

                # propogate EKF state
                self.ekf.predict(imu_k, t_k)    

                # output processed IMU message
                imu_msg = form_imu_message(t_k, omega_b_k, a_b_k)
                self.mqtt_publisher.publish_msg(TOPIC_PROCESSED_IMU, imu_msg)

        if encoder_data is not None:

            recv_time_enc = time.time()

            if (self.ekf.delta_t is None) or (not self.static_initializer.is_initialized()):
                return

            t_k = encoder_data.timestamp

            v_l = encoder_data.left_vel_mps
            v_r = encoder_data.right_vel_mps

            # correct EKF state
            self.ekf.correct(v_l, v_r)

            # form localization message and output
            # print(f"(enc_t - ekf_t) = {t_k - self.ekf.t_k}")
            # print(f"(now - ekf_t) = {time.time() - self.ekf.t_k}")
            loc_msg = form_localization_message(self.ekf.t_k, self.ekf.x_k)
            self.mqtt_publisher.publish_msg(TOPIC_EXTENDED_POSE_W_BIAS, loc_msg)

        # end_time = time.time()
        # print(f"Localization processing time: {end_time - start_time}")
            # print(f"(now - recv_time_enc) = {time.time() - recv_time_enc}")

if __name__ == '__main__':
    loc = LocalizationIMUEncoder()
    loc.run()
