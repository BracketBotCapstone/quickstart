import os

import numpy as np
import scipy.constants
import yaml
from pymlg.numpy import SO3

from nodes.localization.modelling.spline_smoothing import AngularAccelerationSmoother
from utils.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG


class IMUPreprocessor:
    def __init__(self, loc_config : dict):
        """
        Initialize the IMUPreprocessor with the localization configuration.

        Parameters
        ----------
        loc_config (dict): The localization configuration dictionary.
        """
        # open localization config file to retrieve imu config
        imu_config_loc = loc_config['config_params']['imu_config_loc']

        ## add /home/$USER to the beginning of the path
        imu_config_loc = f"/home/{os.getenv('USER')}/{imu_config_loc}"

        # retrieve configuration from yaml
        with open(imu_config_loc, 'r') as stream:
            self.imu_config = yaml.safe_load(stream)

        # initialize units
        self.acc_units = self.imu_config['imu_params']['accel_units']
        self.gyro_units = self.imu_config['imu_params']['gyro_units']

        # generate transformation matrix
        self.r_bi = np.array(self.imu_config['imu_params']['r_bi']).reshape(3, 1)
        self.phi_bi = np.array(self.imu_config['imu_params']['phi_bi']).reshape(3, 1)

        self.C_bi = SO3.Exp(np.array(self.phi_bi))

        # initialize spline smoother for angular acceleration
        self.angular_acceleration_smoother = AngularAccelerationSmoother(self.imu_config['imu_params']['spline_samples'])

    def preprocess_sample(self, imu_data : RAW_IMU_DATA_MSG):
        """
        Preprocesses the angular velocity data by transforming it to the body frame

        Parameters
        ----------
            imu_data : dict
                Dictionary containing the raw IMU data

        Returns
        -------
            omega_b_k : np.ndarray with shape (3, 1)
                Angular velocity in the body frame
            a_b_k : np.ndarray with shape (3, 1)
                Proper acceleration in the body frame
        """
        # retrieve angular velocity
        omega_i_k = np.array([imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z]).reshape(3, 1)

        a_i_k = np.array([imu_data.accel_x, imu_data.accel_y, imu_data.accel_z]).reshape(3, 1)

        # retrieve time of the measurement
        t_k = imu_data.timestamp

        # convert units if necessary
        if self.gyro_units == 'deg/s':
            omega_i_k *= np.pi / 180.0

        if self.acc_units == 'g':
            a_i_k *= scipy.constants.g

        # preprocess data
        omega_b_k = self.C_bi @ omega_i_k

        a_i_k = self.C_bi @ a_i_k

        return t_k, omega_b_k, a_i_k

    def input_preprocess(self, imu_data : RAW_IMU_DATA_MSG):
        """
        
        Parameters
        ----------
            imu_data : dict
                Dictionary containing the raw IMU data

        Returns
        -------
            omega_b_k : np.ndarray with shape (3, 1)
                Angular velocity in the body frame
            a_b_k : np.ndarray with shape (3, 1)
                Proper acceleration in the body frame
        """
        # retrieve angular velocity and proper acceleration
        omega_i_k = np.array([imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z]).reshape(3, 1)
        a_i_k = np.array([imu_data.accel_x, imu_data.accel_y, imu_data.accel_z]).reshape(3, 1)

        # retrieve time of the measurement
        t_k = imu_data.timestamp

        # convert units if necessary
        if self.acc_units == 'g':
            # convert from g to m/s^2
            a_i_k *= scipy.constants.g

        if self.gyro_units == 'deg/s':
            # convert from deg/s to rad/s
            omega_i_k *= np.pi / 180.0

        # preprocess data
        omega_b_k, a_b_k = self.preprocess(omega_i_k, a_i_k, t_k)

        return t_k, omega_b_k, a_b_k

    def preprocess(self, omega_i_k : np.ndarray, a_i_k : np.ndarray, t_k : float):
        """
        Preprocesses the IMU data by transforming the angular velocity and proper acceleration to the body frame

        Parameters
        ----------
            omega_i_k : np.ndarray with shape (3, 1)
                Angular velocity in the IMU frame
            a_i_k : np.ndarray with shape (3, 1)
                Proper acceleration in the IMU frame
            t_k : float
                Time of the measurement

        Returns
        -------
            omega_b_k : np.ndarray with shape (3, 1)
                Angular velocity in the body frame
            alpha_b_k : np.ndarray with shape (3, 1)
                Proper acceleration in the body frame
        """
        # transform angular velocity to body frame
        omega_b_k = self.C_bi @ omega_i_k

        # transform proper acceleration to body frame
        a_b_k_offset = self.C_bi @ a_i_k

        # add angular velocity instance to the smoother
        self.angular_acceleration_smoother.add_omega_sample(omega_b_k.reshape(1, 3), t_k)

        # if the smoother has enough samples, proceed with rigid body kinematic correction, if not, return rotated values
        if self.angular_acceleration_smoother.has_enough_samples():

            # compute spline
            self.angular_acceleration_smoother.fit_angular_velocity_spline()

            # compute angular acceleration
            alpha_b_k_smoothed = self.angular_acceleration_smoother.get_angular_acceleration()

            # print("Offset acceleration is", a_b_k_offset)

            # calculate corrected proper acceleration
            a_b_k = (a_b_k_offset.T + np.cross(alpha_b_k_smoothed.T, -self.r_bi.T) + np.cross(omega_b_k.T, np.cross(omega_b_k.T, -self.r_bi.T))).T

            # print("Corrected acceleration is", a_b_k)

        else:
            a_b_k = a_b_k_offset

        return omega_b_k, a_b_k
