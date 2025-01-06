import numpy as np
import scipy.constants

from nodes.localization.modelling.spline_smoothing import IMUSmoother


class StaticInitializer:
    def __init__(self, loc_config : dict):
        """
        Initialize the StaticInitializer with the localization configuration.

        Parameters
        ----------
        loc_config (dict): The localization configuration dictionary.
        """
        # retrieve static init period
        self.static_init_period = loc_config['initialization_params']['static_init_period']

        # initialize spline smoother for angular acceleration
        self.imu_smoother = IMUSmoother()

    def reset(self):
        """
        Resets the static initializer.
        """
        self.imu_smoother = IMUSmoother()

    def add_sample(self, omega_b_k : np.array, a_b_k : np.array, t_k : float):
        """
        Adds angular velocity data to the static initializer

        Parameters
        ----------
            omega_i_k : np.ndarray with shape (3, 1)
                Angular velocity in the inertial frame
            t_k : float
                Time of the measurement
        """
        self.imu_smoother.add_sample(omega_b_k = omega_b_k, a_b_k = a_b_k, t_k = t_k)

    def get_gyro_bias(self):
        """
        Returns the bias estimates

        Returns
        -------
            b_g : np.ndarray with shape (3, 1)
                Bias estimates
        """
        return self.imu_smoother.get_gyro_bias()
    
    def get_accel_bias(self, C_ab : np.ndarray, g_a = np.array([0, 0, -scipy.constants.g]).reshape(3, 1)):
        """
        Returns the accel bias estimate

        Parameters
        ----------
            C_ab: np.ndarray with shape (3, 3)
                Rotation matrix from the body to the inertial frame
            g_a: np.ndarray with shape (3, 1)
                Acceleration due to gravity in the body frame

        Returns
        -------
            b_a: np.ndarray with shape (3, 1)
        """
        return self.imu_smoother.get_accel_bias(C_ab, g_a)
    
    def get_initial_rotation_estimate(self):
        """
        Returns the initial rotation estimate

        Returns
        -------
            C_ab: np.ndarray with shape (3, 3)
                Initial rotation estimate to a gravity-aligned frame
        """
        return self.imu_smoother.get_initial_C_ab()
    
    def is_initialized(self):
        """
        Returns whether the static initializer has been initialized

        Returns
        -------
            bool
                Whether the static initializer has been initialized
        """
        if len(self.imu_smoother.t_k_buff) < 2:
            return False
        
        return (self.imu_smoother.t_k_buff[-1] - self.imu_smoother.t_k_buff[0]) > self.static_init_period