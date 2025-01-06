from typing import Any

import numpy as np
import scipy.constants
import torch
from csaps import csaps
from pymlg.numpy import SO3
from pymlg.torch import SO3 as SO3t
from scipy.interpolate import interp1d


class IMUSmoother:
    """
    A close derivative of the SplineSmoother class, this class smooths a rolling window of angular velocity inputs and computes an angular acceleration from them.
    """

    def __init__(self):
        """
        Parameters
        ----------
        spline_samples : int
            Target number of angular velocity samples
        """
        self.angular_velocity_spline = None

        self.acceleration_spline = None

        self.omega_b_k_buff = []

        self.a_b_k_buff = []

        self.t_k_buff = []

    def fit_splines(self):

        # aggregate buffers into np arrays for spline computation
        stamps = np.array(self.t_k_buff)
        omega_b = np.array(self.omega_b_k_buff).reshape(-1, 3)
        a_b = np.array(self.a_b_k_buff).reshape(-1, 3)
        
        self.angular_velocity_spline = csaps(stamps, omega_b.T, smooth=0.9999)

        self.acceleration_spline = csaps(stamps, a_b.T, smooth=0.9999)

    def add_sample(self, omega_b_k : np.ndarray, a_b_k : np.ndarray, t_k : float):
        """
        Parameters
        ----------
        omega_b : np.array with shape (1, 3)
            Transpose vector representing a single angular velocity sample in the body frame

        t_k : float
            Time of sample in seconds
        """
        # append samples
        self.omega_b_k_buff.append(omega_b_k)

        self.a_b_k_buff.append(a_b_k)

        self.t_k_buff.append(t_k)

    def gram_schmidt_helper(self, g_i_3 : np.ndarray) -> np.ndarray:
        """
        Performs the Gram-Schmidt process to retrieve an orthonormal basis from as set of linearly independent vectors. In this case, we retrieve C_gi, the rotation matrix from the IMU frame to the global gravity-aligned frame (which we initialize as having the same yaw and position as the IMU frame). Largely ported from OpenVINS' StaticInitializer: https://github.com/rpng/open_vins/blob/master/ov_init/src/utils/helper.h#L138, https://github.com/rpng/open_vins/blob/17b73cfe4b870ade0a65f9eb217d8aab58deae19/ov_init/src/static/StaticInitializer.cpp#L122

        Parameters
        ----------
        g_i_3 : np.ndarray with shape (3, 1)
            The gravity vector in the IMU frame

        Returns
        -------
        C_gi : np.ndarray with shape (3, 3)
            Rotation matrix from the IMU frame to the global gravity-aligned frame
        """
        # get normalized basis vector
        g_i_3 = g_i_3 / np.linalg.norm(g_i_3)

        # we want to initialize y-axis "forward" in the gravity-aligned frame, so set it to greater inner product (as that is where the robot is leaning regardless)
        e_1 = np.array([1, 0, 0]).reshape(3, 1)
        e_2 = np.array([0, 1, 0]).reshape(3, 1)

        # i1 = np.dot(e_1.T, g_i_3) / np.linalg.norm(g_i_3)
        # i2 = np.dot(e_2.T, g_i_3) / np.linalg.norm(g_i_3)

        g_i_1 = -np.cross(g_i_3.ravel(), e_2.ravel()).reshape(3, 1)

        g_i_1 = g_i_1 / np.linalg.norm(g_i_1)

        g_i_2 = np.cross(g_i_3.ravel(), g_i_1.ravel()).reshape(3, 1)

        g_i_2 = g_i_2 / np.linalg.norm(g_i_2)

        C_gi = np.vstack((g_i_1.T, g_i_2.T, g_i_3.T)).T

        return C_gi


    def get_gyro_bias(self) -> np.ndarray:
        """
        Returns the mean angular velocity at the current time step from spline

        Returns
        -------
        b_g : np.ndarray with shape (3, 1)
            Mean angular velocity in the body frame
        """
        # get mean angular velocity and acceleration at current time step
        
        b_g = self.angular_velocity_spline(self.t_k_buff[-1], 0).T

        return b_g
    
    def get_accel_bias(self, C_ab : np.ndarray, g_a : np.ndarray) -> np.ndarray:
        """
        Return an estimated initial accelerometer bias, from an estimated gravity-aligned global frame

        Parameters
        ----------
        C_ab : np.ndarray with shape (3, 3)
            Rotation matrix from the body frame to a global gravity-aligned frame
        
        g_a : np.ndarray with shape (3, 1)
            Gravity vector in the global frame

        Returns
        -------
        b_a : np.ndarray with shape (3, 1)
            Estimated accelerometer bias in the body frame
        """
        # rotate "inertial" (gravity-aligned) gravity vector into body-frame, subtract from smoothened accelorometer sampling, and return as bias estimate

        g_i = self.acceleration_spline(self.t_k_buff[-1], 0).reshape(3, 1)

        return g_i + C_ab @ g_a
    
    def get_initial_C_ab(self) -> np.ndarray:
        """
        Using the gram-schmidt process, returns the initial static estimate for a C_ab, where a is some gravity and yaw-aligned inertial frame and b is the body frame.

        Returns
        -------
        C_ab : np.ndarray with shape (3, 3)
            Initial estimated rotation matrix from the body frame to a global gravity-aligned frame
        """
        # get gravity vector at current time step
        g_i = self.acceleration_spline(self.t_k_buff[-1], 0).reshape(3, 1)

        # get rotation matrix from the IMU frame to the global gravity-aligned frame
        C_gi = self.gram_schmidt_helper(g_i)

        return C_gi
    
class AngularAccelerationSmoother:
    """
    A close derivative of the SplineSmoother class, this class smooths a rolling window of angular velocity inputs and computes an angular acceleration from them.
    """

    def __init__(self, spline_samples: int):
        """
        Parameters
        ----------
        spline_samples : int
            Target number of angular velocity samples
        """
        self.spline_samples = spline_samples

        self.angular_velocity_spline = None

        self.omega_b_k_buff = []

        self.t_k_buff = []

    def has_enough_samples(self) -> bool:
        """
        Returns True if the buffer has enough samples to fit a spline
        """
        return len(self.omega_b_k_buff) >= self.spline_samples

    def fit_angular_velocity_spline(self):
        # Check if there are enough samples to fit splines
        if len(self.t_k_buff) < self.spline_samples:
            return

        # aggregate buffers into np arrays for spline computation
        stamps = np.array(self.t_k_buff)
        omega_b = np.array(self.omega_b_k_buff).reshape(-1, 3)
        
        self.ang_vel_spline = csaps(stamps, omega_b.T, smooth=0.9999)

    def add_omega_sample(self, omega_b_k : np.ndarray, t_k : float):
        """
        Parameters
        ----------
        omega_b : np.array with shape (1, 3)
            Transpose vector representing a single angular velocity sample in the body frame

        t_k : float
            Time of sample in seconds
        """
        # append samples
        self.omega_b_k_buff.append(omega_b_k)
        self.t_k_buff.append(t_k)

        # pop oldest samples if buffer is too large
        if len(self.omega_b_k_buff) > self.spline_samples:
            self.omega_b_k_buff.pop(0)
            self.t_k_buff.pop(0)

    def get_angular_acceleration(self) -> np.ndarray:
        """
        Returns the angular acceleration at the current time step

        Returns
        -------
        alpha_b : np.ndarray with shape (3, 1)
            Angular acceleration in the body frame
        """
        # get angular acceleration at current time step
        alpha_b = self.ang_vel_spline(self.t_k_buff[-1], 1).T

        return alpha_b

class SplineSmoother:
    """
    This class holds a groundtruth dataset and provides several convenient getters.
    """

    def __init__(
        self,
        stamps: np.ndarray,
        position_data: np.ndarray,
        velocity_data: np.ndarray,
        quaternion_data: np.ndarray,
        quat_order: str,
        frame_id: Any,
    ):
        """
        Parameters
        ----------
        stamps : np.ndarray with shape (N,)
            Timestamps of the data
        position_data : np.ndarray with shape (N, 3)
            Position data where each row is a 3D position
        velocity_data : np.ndarray with shape (N, 3)
            Velocity data where each row is a 3D velocity
        quaternion_data : np.ndarray with shape (N, 4)
            Attitude data where each row is a quaternion
        quat_order : str
            Order of the quaternion, either 'wxyz' or 'xyzw'. Used for conversions to other orientation reps.
        frame_id : Any
            Optional frame ID to assign to this data. Will be used as the state
            ID when converting to ``pynav`` states.
        """
        self.stamps = stamps
        self.raw_position = position_data
        self.raw_velocity = velocity_data
        self.raw_quaternion = quaternion_data
        self.frame_id = frame_id
        self.quat_order = quat_order

        self._fit_position_spline(self.stamps, self.raw_position)
        self._fit_velocity_spline(self.stamps, self.raw_velocity)
        self._fit_quaternion_spline(self.stamps, self.raw_quaternion)

        #:np.ndarray: Boolean array containing a static flag for each data point
        self.static_mask = self.get_static_mask(1, 0.0008)

    def _fit_position_spline(self, stamps, pos):
        # Fit splines
        self._pos_spline = csaps(stamps, pos.T, smooth=0.9999)

    def _fit_velocity_spline(self, stamps, vel):
        # Fit splines
        self._vel_spline = csaps(stamps, vel.T, smooth=0.9999)

    def _fit_quaternion_spline(self, stamps, quat):
        # Normalize quaternion
        quat /= np.linalg.norm(quat, axis=1)[:, None]

        # Resolve quaternion ambiguities so that quaternion trajectories look
        # smooth. This is recursive so cannot be vectorized.
        for idx, q in enumerate(quat[1:]):
            q_old = quat[idx]
            if np.linalg.norm((-q - q_old)) < np.linalg.norm((q - q_old)):
                q *= -1

        self._quat_spline = csaps(stamps, quat.T, smooth=0.99999)

    def position(self, stamps: np.ndarray) -> np.ndarray:
        """
        Get the position at one or more query times.

        Parameters
        ----------
        stamps : np.ndarray
            Query times

        Returns
        -------
        np.ndarray with shape (N,3)
            Position data
        """
        return self._pos_spline(stamps, 0).T

    def velocity(self, stamps: np.ndarray) -> np.ndarray:
        """
        Get the velocity at one or more query times.

        Parameters
        ----------
        stamps : np.ndarray
            Query times

        Returns
        -------
        np.ndarray with shape (N,3)
            velocity data
        """
        return self._pos_spline(stamps, 1).T

    def acceleration(self, stamps: np.ndarray) -> np.ndarray:
        """
        Get the acceleration at one or more query times.

        Parameters
        ----------
        stamps : np.ndarray
            Query times

        Returns
        -------
        np.ndarray with shape (N,3)
            acceleration data
        """
        return self._pos_spline(stamps, 2).T

    def accelerometer(self, stamps: np.ndarray, g_a=None) -> np.ndarray:
        """
        Get simuluated accelerometer readings

        Parameters
        ----------
        stamps : float or np.ndarray
            query times
        g_a : List[float], optional
            gravity vector, by default [0, 0, -9.80665]

        Returns
        -------
        ndarray with shape `(len(stamps),3)`
            Accelerometer readings
        """
        if g_a is None:
            g_a = [0, 0, -scipy.constants.g]

        a_zwa_a = self._vel_spline(stamps, 1).T
        C_ab = self.rot_matrix(stamps)
        C_ba = np.transpose(C_ab, axes=[0, 2, 1])
        g_a = np.array(g_a).reshape((-1, 1))
        return (C_ba @ (np.expand_dims(a_zwa_a, 2) - g_a)).squeeze()

    def quaternion(self, stamps):
        """
        Get the quaternion at one or more query times.

        Parameters
        ----------
        stamps : np.ndarray
            Query times

        Returns
        -------
        np.ndarray with shape (N,4)
            quaternion data
        """
        q = self._quat_spline(stamps, 0).T
        return q / np.linalg.norm(q, axis=1)[:, None]
    
    def rot_matrix(self, stamps):
        """
        Get the DCM/rotation matrix at one or more query times.

        Parameters
        ----------
        stamps : np.ndarray
            Query times

        Returns
        -------
        np.ndarray with shape (N,3,3)
            DCM/rotation matrix data
        """
        quat = self.quaternion(stamps)
        quat = torch.Tensor(quat).reshape(-1, 4)

        C_ab = SO3t.from_quat(quat, self.quat_order).numpy().reshape(-1, 3, 3)

        return C_ab

    def pose_matrix(self, stamps):
        """
        Get the SE(3) pose matrix at one or more query times.

        Parameters
        ----------
        stamps : np.ndarray
            Query times

        Returns
        -------
        np.ndarray with shape (N,4,4)
            pose data
        """
        r = self.position(stamps)
        C = self.rot_matrix(stamps)
        T = np.zeros((C.shape[0], 4, 4))
        T[:, :3, :3] = C
        T[:, :3, 3] = r
        T[:, 3, 3] = 1
        return T

    def extended_pose_matrix(self, stamps):
        """
        Get the SE_2(3) extended pose matrix at one or more query times.

        Parameters
        ----------
        stamps : np.ndarray
            Query times

        Returns
        -------
        np.ndarray with shape (N,5,5)
            extended pose data
        """
        r = self.position(stamps)
        C = self.rot_matrix(stamps)
        v = self.velocity(stamps)
        T = np.zeros((C.shape[0], 5, 5))
        T[:, :3, :3] = C
        T[:, :3, 3] = v
        T[:, :3, 4] = r
        T[:, 3, 3] = 1
        T[:, 4, 4] = 1
        return T

    def angular_velocity(self, stamps: np.ndarray) -> np.ndarray:
        """
        Get the angular velocity at one or more query times.    

        Parameters
        ----------
        stamps : np.ndarray
            Query times

        Returns
        -------
        np.ndarray with shape (N,3)
            angular velocity data
        """
        q = self._quat_spline(stamps, 0)
        q = q / np.linalg.norm(q, axis=0)
        q_dot = np.atleast_2d(self._quat_spline(stamps, 1)).T
        eta = q[0]
        eps = q[1:]

        # Vectorized computation
        e = eps.reshape(3, -1, 1)
        S = np.hstack((-2 * e, 2 * (eta * np.eye(3) - SO3.wedge(e))))
        omega = (S @ np.expand_dims(q_dot, 2)).squeeze()
        return omega

    def body_velocity(self, stamps: np.ndarray) -> np.ndarray:
        """
        Get the body-frame-resolved translational velocity
        at one or more query times.

        Parameters
        ----------
        stamps : np.ndarray
            Query times

        Returns
        -------
        np.ndarray with shape (N,3)
            body-frame-resolved velocity data
        """
        v_zw_a = self.velocity(stamps)
        v_zw_a = np.expand_dims(v_zw_a, 2)
        C_ab = self.rot_matrix(stamps)
        C_ba = np.transpose(C_ab, axes=[0, 2, 1])
        return (C_ba @ v_zw_a).squeeze().T

    def get_static_mask(
        self, window_size: float, std_dev_threshold: float
    ) -> np.ndarray:
        """
        Detects static moments in the mocap data.

        Parameters
        ----------
        window_size : float
            window size for variance threshold
        std_dev_threshold : float
            threshold value

        Returns
        -------
        np.ndarray
            boolean mask of static moments. True if static at that time.
        """
        # Average mocap frequency
        freq = 1 / ((self.stamps[-1] - self.stamps[0]) / self.stamps.size)

        window_half_width = round(window_size / 2 * freq)
        cov_threshold = std_dev_threshold**2
        is_static = np.zeros((self.stamps.size,), bool)

        for i in range(window_half_width, self.stamps.size - window_half_width):

            pos = self.raw_position[i - window_half_width : i + window_half_width]
            pos_cov = np.cov(pos.T)

            if np.trace(pos_cov) < cov_threshold:
                # Then it is a static
                is_static[i] = True
                if i == window_half_width:
                    is_static[:window_half_width] = True
                elif i == self.stamps.size - window_half_width - 1:
                    is_static[i:] = True

        return is_static

    def is_static(self, stamps: np.ndarray, static_mask=None) -> np.ndarray:
        """
        Returns true or false if the body is detected to be static at time t.
        If ``t`` is a list or numpy array, then it will return a boolean array
        for each point.

        Parameters
        ----------
        t : float or List[float] or numpy.ndarray
            Query stamps
        static_mask : np.ndarray with shape (N,1), optional
            To use a custom static mask to refer to for the static state. This
            must be a boolean array with as many elements as there are stamps
            in this object. If None, then the static mask is computed automatically
            with default parameters. By default None.

        Returns
        -------
        bool or numpy.ndarray
            True if the body is static at time t, False otherwise.
        """
        if static_mask is None:
            static_mask = self.static_mask

        indexes = np.array(range(len(self.stamps)))
        nearest_time_idx = interp1d(
            self.stamps,
            indexes,
            "nearest",
            bounds_error=False,
            fill_value="extrapolate",
        )
        return static_mask[nearest_time_idx(stamps).astype(int)]

