from .messages.gyro_data_msg import GYRO_DATA_MSG
from .messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from .messages.wheel_velocities_data_msg import WHEEL_VELOCITIES_DATA_MSG
from .messages.extended_pose_w_bias_msg import EXTENDED_POSE_W_BIAS_MSG
from .messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from .messages.trajectory_msg import TRAJECTORY_MSG

# Topic variables
TOPIC_GYRO = '/imu/gyro'
TOPIC_RAW_IMU = '/imu/raw_imu'
TOPIC_RAW_IMU_UNSCALED_BIASED = '/imu/raw_imu_unscaled_biased'
TOPIC_PROCESSED_IMU = '/localization/processed_imu'
TOPIC_WHEEL_VELOCITIES = '/wheel_velocities_mps'
TOPIC_EXTENDED_POSE_W_BIAS = '/localization/extended_pose_state'
TOPIC_OCCUPANCY_GRID = '/mapping/occupancy_grid'
TOPIC_TRAVERSABILITY_GRID = '/mapping/traversability_grid'
TOPIC_TRAJECTORY = '/planning/trajectory'

# Topic to message type mapping
topic_to_message_type = {
    '/imu/gyro': GYRO_DATA_MSG,
    '/imu/raw_imu': RAW_IMU_DATA_MSG,
    '/imu/raw_imu_unscaled_biased': RAW_IMU_DATA_MSG,
    '/localization/processed_imu': RAW_IMU_DATA_MSG,
    '/wheel_velocities_mps': WHEEL_VELOCITIES_DATA_MSG,
    '/localization/extended_pose_state': EXTENDED_POSE_W_BIAS_MSG,
    '/mapping/occupancy_grid': OCCUPANCY_GRID_MSG,
    '/mapping/traversability_grid': OCCUPANCY_GRID_MSG,
    '/planning/trajectory': TRAJECTORY_MSG,
}
