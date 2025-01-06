import logging
import os
import time

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

import utils.constants as CFG
from utils.logging_utils import Logger
from utils.messages.extended_pose_w_bias_msg import EXTENDED_POSE_W_BIAS_MSG
from utils.messages.gyro_data_msg import GYRO_DATA_MSG
from utils.messages.mqtt_message_base import MqttMessageBase
from utils.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from utils.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from utils.messages.wheel_velocities_data_msg import WHEEL_VELOCITIES_DATA_MSG
from utils.mqtt_utils import MQTTSubscriber
from utils.topic_to_message_type import (
    TOPIC_EXTENDED_POSE_W_BIAS,
    TOPIC_GYRO,
    TOPIC_OCCUPANCY_GRID,
    TOPIC_PROCESSED_IMU,
    TOPIC_RAW_IMU_UNSCALED_BIASED,
    TOPIC_WHEEL_VELOCITIES,
)


class RerunViewer:
    def __init__(self, logging_level=logging.INFO):
        """
        Initialize the RerunViewer with logging and MQTT subscriber setup.
        
        :param logging_level: The logging level to use for the logger.
        """
        self.logger = Logger('rerun_viewer', 'logs/rerun_viewer.log', level=logging_level)
        self.ip_address = os.getenv("HOST_IP_ADDRESS", CFG.RERUN_HOST_IP_ADDRESS)
        self.imu_data = None
        self.imu_us_ub_data = None
        self.encoder_distance = None
        self.wheel_ang_velocities = None
        self.extended_pose_data = None
        self.processed_imu_data = None
        self.occupancy_grid_data = None
        self.time_series = []
        self.last_periodic_log_time = time.time()
        self.last_delayed_log_time = time.time()
        self.frame = None

        # Initialize MQTT subscriber with topic to message type mapping
        self.mqtt_subscriber = MQTTSubscriber(
            broker_address="localhost",
            topic_to_message_map={
                TOPIC_GYRO: GYRO_DATA_MSG,
                TOPIC_OCCUPANCY_GRID: OCCUPANCY_GRID_MSG,
                TOPIC_RAW_IMU_UNSCALED_BIASED: RAW_IMU_DATA_MSG,
                TOPIC_WHEEL_VELOCITIES: WHEEL_VELOCITIES_DATA_MSG,
                TOPIC_EXTENDED_POSE_W_BIAS: EXTENDED_POSE_W_BIAS_MSG,
                TOPIC_PROCESSED_IMU: RAW_IMU_DATA_MSG,
            }
        )

        self.loop_rate_hz = 50

    def setup_rerun(self):
        """
        Setup the Rerun dashboard with the specified blueprint and connect to the server.
        """
        rr.init(
            "robot_dashboard",
            spawn=False,
            default_blueprint=rrb.Vertical(
                rrb.Horizontal(
                    rrb.TimeSeriesView(origin="/encoder", name="Encoders"),
                    rrb.TimeSeriesView(origin="/gyro", name="Gyro"),
                ),
                rrb.Horizontal(
                    rrb.TimeSeriesView(origin="/timeseries", name="Timeseries"),
                    rrb.TextLogView(origin="/", name="Logged data"),
                ),
                rrb.Horizontal(
                    rrb.Spatial2DView(origin="/camera", name="Camera 2D"),
                ),
                row_shares=[1, 1, 1]
            )
        )
        rr.connect_tcp(f"{self.ip_address}:9876")

    def run(self):
        """
        Start the Rerun viewer, process input data, and handle graceful shutdown.
        """
        self.setup_rerun()
        self.mqtt_subscriber.start()
        try:
            while True:
                start_time = time.time()
                self.process_input()
                end_time = time.time()
                sleep_time = 1 / self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            self.logger.info("Rerun viewer stopped by user.")
            self.mqtt_subscriber.stop()

    def process_input(self, event=None):
        """
        Process incoming MQTT messages and log data periodically.
        
        :param event: Optional event parameter for future use.
        """
        # Retrieve latest messages from MQTT topics
        self.imu_data = self.mqtt_subscriber.get_latest_message(TOPIC_GYRO)
        self.imu_us_ub_data = self.mqtt_subscriber.get_latest_message(TOPIC_RAW_IMU_UNSCALED_BIASED)
        self.wheel_ang_velocities = self.mqtt_subscriber.get_latest_message(TOPIC_WHEEL_VELOCITIES)
        self.extended_pose_data = self.mqtt_subscriber.get_latest_message(TOPIC_EXTENDED_POSE_W_BIAS)
        self.processed_imu_data = self.mqtt_subscriber.get_latest_message(TOPIC_PROCESSED_IMU)
        self.occupancy_grid_data = self.mqtt_subscriber.get_latest_message(TOPIC_OCCUPANCY_GRID)

        current_time = time.time()
        # Log data every 0.1 seconds
        if current_time - self.last_periodic_log_time >= 0.1:
            if self.imu_data is not None:
                self.log_msg_values('gyro', self.imu_data)
            if self.imu_us_ub_data is not None:
                self.log_msg_values('raw_imu_unscaled_biased', self.imu_us_ub_data)
            if self.wheel_ang_velocities is not None:
                self.log_msg_values('wheel_velocities_mps', self.wheel_ang_velocities)
            if self.extended_pose_data is not None:
                self.log_msg_values('extended_pose_state', self.extended_pose_data)
            if self.processed_imu_data is not None:
                self.log_msg_values('processed_imu', self.processed_imu_data)

            self.last_periodic_log_time = current_time

        self.log_data()

    def log_msg_values(self, id, MqttMessage: MqttMessageBase):
        """
        Log message values to Rerun, filtering out private and None attributes.
        
        :param id: Identifier for the message type.
        :param MqttMessage: The message object containing data to log.
        """
        for name, val in MqttMessage.__dict__.items():
            if not name.startswith('_') and val is not None:
                rr.log("{}/{}".format(id, name), rr.Scalar(val))

    def log_data(self):
        """
        Custom handling of data logging to Rerun, including occupancy grid and processed IMU data.
        """
        current_time = time.time()
        # Log data every 0.5 seconds
        if current_time - self.last_delayed_log_time >= 0.5:
            if self.occupancy_grid_data is not None:
                # Assuming you know the width and height of the grid
                width = self.occupancy_grid_data.width

                # Convert the flattened list back to a 2D numpy array
                grid_array = np.array(self.occupancy_grid_data.flattened_grid_list).reshape((width, width))

                # Convert to a black and white image
                bw_image = (grid_array * 255).astype(np.uint8)

                # Log the image to Rerun
                rr.log("occupancy_grid", rr.Image(bw_image))

            # Log current processed IMU values
            if self.processed_imu_data is not None:
                rr.log("processed_imu", rr.TextLog(
                    f"Timestamp: {self.processed_imu_data.timestamp:.6f}\n"
                    f"Gyro X: {self.processed_imu_data.gyro_x:.2f}°/s\n"
                    f"Gyro Y: {self.processed_imu_data.gyro_y:.2f}°/s\n"
                    f"Gyro Z: {self.processed_imu_data.gyro_z:.2f}°/s\n"
                    f"Accel X: {self.processed_imu_data.accel_x:.2f}g\n"
                    f"Accel Y: {self.processed_imu_data.accel_y:.2f}g\n"
                    f"Accel Z: {self.processed_imu_data.accel_z:.2f}g"
                ))

            if self.extended_pose_data is not None:
                rr.log("pose", rr.TextLog(
                    f"X: {self.extended_pose_data.r_a_b_x:.2f}m\n"
                    f"Y: {self.extended_pose_data.r_a_b_y:.2f}m\n"
                ))
                rr.log("world_pose", rr.Points2D([self.extended_pose_data.r_a_b_x, self.extended_pose_data.r_a_b_y]))

            self.last_delayed_log_time = current_time

def main():
    """
    Main function to create and run the RerunViewer instance.
    """
    viewer = RerunViewer()
    viewer.run()

if __name__ == "__main__":
    main()
