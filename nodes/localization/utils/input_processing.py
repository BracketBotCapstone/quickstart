import numpy as np
from navlie.types import StateWithCovariance
from pymlg.numpy import SE3, SO3

from utils.messages.extended_pose_w_bias_msg import EXTENDED_POSE_W_BIAS_MSG
from utils.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from utils.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from utils.messages.robot_extended_pose_msg import ROBOT_EXTENDED_POSE_MSG
from utils.messages.robot_pose_grid_coords_msg import ROBOT_POSE_GRID_COORDS_MSG
from utils.messages.robot_pose_msg import ROBOT_POSE_MSG


def form_se3_from_localization_message(msg: EXTENDED_POSE_W_BIAS_MSG) -> np.ndarray:
    """
    Forms an SE3 transformation matrix from a localization message.

    Args:
        msg (EXTENDED_POSE_W_BIAS_MSG): The localization message containing position and orientation data.

    Returns
    -------
        np.ndarray: The SE3 transformation matrix.
    """
    # Extract position vector
    r_a_b = np.array([msg.r_a_b_x, msg.r_a_b_y, msg.r_a_b_z]).reshape(1, 3)

    # Extract orientation vector
    phi_a_b = np.array([msg.phi_a_b_x, msg.phi_a_b_y, msg.phi_a_b_z]).reshape(1, 3)

    # Compute rotation matrix using SO3 exponential map
    C_ab = SO3.Exp(phi_a_b)

    # Form SE3 transformation matrix
    T_ab = SE3.from_components(C_ab, r_a_b)

    return T_ab

def form_imu_message(t_k: float, omega_b_k: np.array, a_b_k: np.array) -> RAW_IMU_DATA_MSG:
    """
    Forms a RAW_IMU_DATA_MSG from given IMU data.

    Args:
        t_k (float): Timestamp of the IMU data.
        omega_b_k (np.array): Angular velocity data.
        a_b_k (np.array): Acceleration data.

    Returns
    -------
        RAW_IMU_DATA_MSG: The populated IMU message.
    """
    # init dora IMU message
    # imu_msg = INIT_MSG(MSG.RAW_IMU_DATA_MSG())
    imu_msg = RAW_IMU_DATA_MSG()

    # Populate the message with timestamp and sensor data
    imu_msg.timestamp = t_k
    imu_msg.gyro_x = omega_b_k[0][0]
    imu_msg.gyro_y = omega_b_k[1][0]
    imu_msg.gyro_z = omega_b_k[2][0]
    imu_msg.accel_x = a_b_k[0][0]
    imu_msg.accel_y = a_b_k[1][0]
    imu_msg.accel_z = a_b_k[2][0]

    return imu_msg

def form_localization_message(t_k: float, x_k: StateWithCovariance) -> EXTENDED_POSE_W_BIAS_MSG:
    """
    Forms an EXTENDED_POSE_W_BIAS_MSG from a state with covariance.

    Args:
        t_k (float): Timestamp of the state.
        x_k (StateWithCovariance): The state with covariance data.

    Returns
    -------
        EXTENDED_POSE_W_BIAS_MSG: The populated localization message.
    """
    # init dora localization message
    # loc_msg = INIT_MSG(MSG.EXTENDED_POSE_W_BIAS_MSG())
    loc_msg = EXTENDED_POSE_W_BIAS_MSG()

    # Aggregate state members
    phi_ab_k = SO3.Log(x_k.state.attitude).reshape(3)
    r_a = x_k.state.position
    v_a = x_k.state.velocity
    b_g = x_k.state.bias[:3]
    b_a = x_k.state.bias[3:]

    # Populate the message with state data
    loc_msg.timestamp = t_k
    loc_msg.r_a_b_x = r_a[0]
    loc_msg.r_a_b_y = r_a[1]
    loc_msg.r_a_b_z = r_a[2]
    loc_msg.phi_a_b_x = phi_ab_k[0]
    loc_msg.phi_a_b_y = phi_ab_k[1]
    loc_msg.phi_a_b_z = phi_ab_k[2]
    loc_msg.v_a_b_x = v_a[0]
    loc_msg.v_a_b_y = v_a[1]
    loc_msg.v_a_b_z = v_a[2]
    loc_msg.bias_gyro_x = b_g[0]
    loc_msg.bias_gyro_y = b_g[1]
    loc_msg.bias_gyro_z = b_g[2]
    loc_msg.bias_accel_x = b_a[0]
    loc_msg.bias_accel_y = b_a[1]
    loc_msg.bias_accel_z = b_a[2]

    return loc_msg

def form_robot_pose_message(msg: EXTENDED_POSE_W_BIAS_MSG) -> ROBOT_POSE_MSG:
    """
    Forms a ROBOT_POSE_MSG from an extended pose message.

    Args:
        msg (EXTENDED_POSE_W_BIAS_MSG): The extended pose message.

    Returns
    -------
        ROBOT_POSE_MSG: The populated robot pose message.
    """
    pose_msg = ROBOT_POSE_MSG()
    pose_msg.timestamp = msg.timestamp
    pose_msg.x_m = msg.r_a_b_x
    pose_msg.y_m = msg.r_a_b_y
    pose_msg.theta_rad = -1 * msg.phi_a_b_z  # Convert orientation to robot pose format
    return pose_msg

def form_robot_extended_pose_message(msg: EXTENDED_POSE_W_BIAS_MSG) -> ROBOT_EXTENDED_POSE_MSG:
    """
    Forms a ROBOT_EXTENDED_POSE_MSG from an extended pose message.

    Args:
        msg (EXTENDED_POSE_W_BIAS_MSG): The extended pose message.

    Returns
    -------
        ROBOT_EXTENDED_POSE_MSG: The populated robot extended pose message.
    """
    pose_msg = ROBOT_EXTENDED_POSE_MSG()
    pose_msg.timestamp = msg.timestamp
    pose_msg.r_a_b_x_m = msg.r_a_b_x
    pose_msg.r_a_b_y_m = msg.r_a_b_y
    pose_msg.r_a_b_z_m = msg.r_a_b_z
    pose_msg.v_a_b_x_mps = msg.v_a_b_x
    pose_msg.v_a_b_y_mps = msg.v_a_b_y
    pose_msg.v_a_b_z_mps = msg.v_a_b_z
    pose_msg.phi_a_b_x_rad = msg.phi_a_b_x
    pose_msg.phi_a_b_y_rad = msg.phi_a_b_y
    pose_msg.phi_a_b_z_rad = msg.phi_a_b_z
    return pose_msg

def form_flattened_occupancy_grid_message(t_k: float, flat_occupancy_grid: np.array, width: int) -> OCCUPANCY_GRID_MSG:
    """
    Forms an OCCUPANCY_GRID_MSG from a flattened occupancy grid.

    Args:
        t_k (float): Timestamp of the grid data.
        flat_occupancy_grid (np.array): The flattened occupancy grid data.
        width (int): The width of the grid.

    Returns
    -------
        OCCUPANCY_GRID_MSG: The populated occupancy grid message.
    """
    # init message
    grid_msg = OCCUPANCY_GRID_MSG()

    # Populate the message with grid data
    grid_msg.timestamp = t_k
    grid_msg.width = width
    grid_msg.flattened_grid_list = flat_occupancy_grid.tolist()
    return grid_msg    

def form_robot_pose_grid_coords_message(extended_pose_msg: EXTENDED_POSE_W_BIAS_MSG, grid_cell_size_m: float, grid_width_m: float) -> ROBOT_POSE_GRID_COORDS_MSG:
    """
    Forms a ROBOT_POSE_GRID_COORDS_MSG from an extended pose message and grid parameters.

    Args:
        extended_pose_msg (EXTENDED_POSE_W_BIAS_MSG): The extended pose message.
        grid_cell_size_m (float): The size of each grid cell in meters.
        grid_width_m (float): The width of the grid in meters.

    Returns
    -------
        ROBOT_POSE_GRID_COORDS_MSG: The populated robot pose grid coordinates message.
    """
    # Calculate grid offset
    offset = grid_width_m / 2

    # Compute grid coordinates
    grid_coords = np.array([
        int((extended_pose_msg.r_a_b_x + offset) / grid_cell_size_m),
        int((extended_pose_msg.r_a_b_y + offset) / grid_cell_size_m)
    ])

    # Initialize and populate the message
    msg = ROBOT_POSE_GRID_COORDS_MSG()
    msg.timestamp = extended_pose_msg.timestamp
    msg.x_grid = int(grid_coords[0])
    msg.y_grid = int(grid_coords[1])
    msg.theta_rad = -1 * extended_pose_msg.phi_a_b_z  # Convert orientation to grid format
    return msg
