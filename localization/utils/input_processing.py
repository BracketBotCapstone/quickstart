import os
import numpy as np
import time

from utils.messages.extended_pose_w_bias_msg import EXTENDED_POSE_W_BIAS_MSG
from utils.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from utils.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG

from navlie.types import StateWithCovariance
from navlie.lib import IMU, IMUState

from pymlg.numpy import SO3, SE3

def form_se3_from_localization_message(msg : EXTENDED_POSE_W_BIAS_MSG) -> np.ndarray:
    
    r_a_b = np.array([msg.r_a_b_x, msg.r_a_b_y, msg.r_a_b_z]).reshape(1, 3)

    phi_a_b = np.array([msg.phi_a_b_x, msg.phi_a_b_y, msg.phi_a_b_z]).reshape(1, 3)

    C_ab = SO3.Exp(phi_a_b)

    T_ab = SE3.from_components(C_ab, r_a_b)

    return T_ab

def form_imu_message(t_k : float, omega_b_k : np.array, a_b_k : np.array) -> RAW_IMU_DATA_MSG:
    
    # init dora IMU message
    # imu_msg = INIT_MSG(MSG.RAW_IMU_DATA_MSG())
    imu_msg = RAW_IMU_DATA_MSG()
    # populate the message
    imu_msg.timestamp = t_k
    # omega_b_k[0] is an array, so we need to access the first element
    imu_msg.gyro_x = omega_b_k[0][0]
    imu_msg.gyro_y = omega_b_k[1][0]
    imu_msg.gyro_z = omega_b_k[2][0]
    imu_msg.accel_x = a_b_k[0][0]
    imu_msg.accel_y = a_b_k[1][0]
    imu_msg.accel_z = a_b_k[2][0]

    return imu_msg

def form_localization_message(t_k : float, x_k : StateWithCovariance) -> EXTENDED_POSE_W_BIAS_MSG:
    
    # init dora localization message
    # loc_msg = INIT_MSG(MSG.EXTENDED_POSE_W_BIAS_MSG())
    loc_msg = EXTENDED_POSE_W_BIAS_MSG()

    # aggregate state members
    phi_ab_k = SO3.Log(x_k.state.attitude).reshape(3)
    r_a = x_k.state.position
    v_a = x_k.state.velocity
    b_g = x_k.state.bias[:3]
    b_a = x_k.state.bias[3:]

    # populate the message
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

def form_flattened_occupancy_grid_message(t_k : float, flat_occupancy_grid : np.array, width : int) -> OCCUPANCY_GRID_MSG:
    
    # init message
    grid_msg = OCCUPANCY_GRID_MSG()
    # populate the message
    grid_msg.timestamp = t_k
    grid_msg.width = width
    grid_msg.flattened_grid_list = flat_occupancy_grid.tolist()
    return grid_msg
