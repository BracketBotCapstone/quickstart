import numpy as np
from navlie import MeasurementModel
from navlie.lib import IMUState
from pymlg.numpy import SO3


class WheelEncoderGravityAligned(MeasurementModel):
    """
    Wheel Encoder Measurement model for some kind of high-frequency, high-resolution rotary sensor on a wheel with a known radius.
    This applies the updates in the normal frame direction (that is, along the ground) and is more correct for a flat surface when modelling a moving pendulum.
    Currently, assumes a left perturbation.
    """

    def __init__(self, R: np.array):
        """
        Initialize the WheelEncoderGravityAligned model with a given covariance matrix.

        Parameters
        ----------
        R (np.array): Covariance matrix for the measurement model.
        """
        self.R = R

    def evaluate(self, x: IMUState) -> np.ndarray:
        """
        Evaluate the velocity in the ground-aligned frame.

        Parameters
        ----------
        x (IMUState): The state containing attitude and velocity.

        Returns
        -------
        np.ndarray: The velocity in the ground-aligned frame.
        """
        # Retrieve state parameters
        C_ab = x.attitude
        v_a = x.velocity

        # Precompute trigonometric values to avoid recomputation
        c21, c11 = C_ab[1, 0], C_ab[0, 0]
        gamma = np.arctan2(c21, c11)
        phi_gamma = np.array([0, 0, gamma])

        # Use precomputed rotation matrix if possible
        C_ga = SO3.Exp(phi_gamma).T
        v_zw_g = C_ga @ v_a

        return v_zw_g
    
    def jacobian(self, x: IMUState) -> np.ndarray:
        """
        Compute the Jacobian of the measurement model.

        Parameters
        ----------
        x (IMUState): The state containing attitude and velocity.

        Returns
        -------
        np.ndarray: The Jacobian matrix.
        """
        C_ab = x.attitude
        v_a = x.velocity

        G_vu = np.zeros((3, 15))

        # Precompute trigonometric values
        c21, c11 = C_ab[1, 0], C_ab[0, 0]
        gamma = np.arctan2(c21, c11)
        phi_gamma = np.array([0, 0, gamma])

        C_ag = SO3.Exp(phi_gamma)

        # Precompute coefficients
        denom = c11**2 + c21**2
        coeff_y = c11 / denom
        coeff_x = -c21 / denom

        B = np.array([1, 0, 0])
        A = np.array([0, 1, 0])
        J = np.array([1, 0, 0])

        # Optimize wedge product calculations
        wedge_CB = SO3.wedge(C_ab @ B)
        d_gamma_phi = -coeff_y * (A @ wedge_CB) - coeff_x * (J @ wedge_CB)

        d_C_gamma = (C_ag @ SO3.wedge(np.array([0, 0, 1]))).T

        G_vu[:, 0:3] = -C_ag.T @ SO3.wedge(v_a) + d_C_gamma @ v_a @ d_gamma_phi
        G_vu[:, 3:6] = C_ag.T

        return G_vu
    
    def covariance(self, x: IMUState) -> np.ndarray:
        """
        Return the covariance matrix for the measurement model.

        Parameters
        ----------
        x (IMUState): The state (not used in this function).

        Returns
        -------
        np.ndarray: The covariance matrix.
        """
        return self.R

class WheelEncoder(MeasurementModel):
    """
    Wheel Encoder Measurement model for some kind of high-frequency, high-resolution rotary sensor on a wheel with a known radius.
    """

    def __init__(self, R: np.array):
        """
        Initialize the WheelEncoder model with a given covariance matrix.

        Parameters
        ----------
        R (np.array): Covariance matrix for the measurement model.
        """
        self.R = R

    def evaluate(self, x: IMUState) -> np.ndarray:
        """
        Evaluate the velocity in the body frame.

        Parameters
        ----------
        x (IMUState): The state containing attitude and velocity.

        Returns
        -------
        np.ndarray: The velocity in the body frame.
        """
        C_ab = x.attitude
        v_a = x.velocity

        # Direct computation without unnecessary transposition
        v_b = C_ab.T @ v_a

        return v_b
    
    def jacobian(self, x: IMUState) -> np.ndarray:
        """
        Compute the Jacobian of the measurement model.

        Parameters
        ----------
        x (IMUState): The state containing attitude and velocity.

        Returns
        -------
        np.ndarray: The Jacobian matrix.
        """
        C_ab = x.attitude
        v_a = x.velocity

        jac = np.zeros((3, 15))

        # Optimize matrix operations
        wedge_va = SO3.wedge(v_a)
        jac[:, 3:6] = np.eye(3)
        jac[:, :3] = C_ab.T @ wedge_va @ C_ab

        return jac
    
    def covariance(self, x: IMUState) -> np.ndarray:
        """
        Return the covariance matrix for the measurement model.

        Parameters
        ----------
        x (IMUState): The state (not used in this function).

        Returns
        -------
        np.ndarray: The covariance matrix.
        """
        return self.R