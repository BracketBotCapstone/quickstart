import logging
import os
import time

import utils.constants as CFG
from nodes.localization.utils.input_processing import (
    form_flattened_occupancy_grid_message,
    form_robot_pose_grid_coords_message,
    form_se3_from_localization_message,
)
from nodes.mapping.wv_manager import DepthWavemapManager
from utils.logging_utils import Logger
from utils.messages.extended_pose_w_bias_msg import EXTENDED_POSE_W_BIAS_MSG
from utils.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from utils.messages.robot_pose_grid_coords_msg import ROBOT_POSE_GRID_COORDS_MSG
from utils.mqtt_utils import MQTTPublisher, MQTTSubscriber
from utils.topic_to_message_type import (
    TOPIC_EXTENDED_POSE_W_BIAS,
    TOPIC_OCCUPANCY_GRID,
    TOPIC_ROBOT_POSE_GRID_COORDS,
    TOPIC_TRAVERSABILITY_GRID,
)


class DepthWavemapNode:
    def __init__(self, logging_level=logging.INFO):
        """
        Initializes the DepthWavemapNode with logging, configuration, and MQTT setup.
        
        :param logging_level: The logging level to use for the logger.
        """
        self.logger = Logger('depth_wavemap_node', 'logs/depth_wavemap_node.log', level=logging_level)
        self.logger.info("Initializing DepthWavemapNode")

        # Retrieve yaml file location
        map_config_loc = os.getenv("BASE_CONFIG_LOC", CFG.MAPPING_BASE_CONFIG_LOC)

        # Add /home/$USER to the beginning of the path
        map_config_loc = f"/home/{os.getenv('USER')}/{map_config_loc}"

        # Initialize depth wavemap manager
        self.depth_wavemap_manager = DepthWavemapManager(map_config_loc)

        # Initialize the publisher
        self.publisher = MQTTPublisher(
            broker_address="localhost",
            topic_to_message_map={
                TOPIC_OCCUPANCY_GRID: OCCUPANCY_GRID_MSG,
                TOPIC_TRAVERSABILITY_GRID: OCCUPANCY_GRID_MSG,
                TOPIC_ROBOT_POSE_GRID_COORDS: ROBOT_POSE_GRID_COORDS_MSG
            }
        )

        # Initialize pose subscriber
        self.mqtt_subscriber = MQTTSubscriber(
            broker_address="localhost",
            topic_to_message_map={TOPIC_EXTENDED_POSE_W_BIAS: EXTENDED_POSE_W_BIAS_MSG}
        )

        # Set loop rate in Hz
        self.loop_rate_hz = 10

    def run(self):
        """
        Starts the DepthWavemapNode, processing input and publishing messages in a loop.
        """
        self.mqtt_subscriber.start()
        self.publisher.run()

        # Start pipeline
        self.depth_wavemap_manager.start_pipeline()
        
        try:
            while True:
                start_time = time.time()
                self.process_input()
                end_time = time.time()
                sleep_time = 1/self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            self.logger.info("DepthWavemapNode stopped by user. Stopping and saving map!")
            self.depth_wavemap_manager.save_map()
            self.logger.info("Saved map!")
            self.mqtt_subscriber.stop()
        
    def process_input(self):
        """
        Processes incoming messages, updates the depth wavemap, and publishes grid messages.
        """
        # Get the latest extended pose data
        self.extended_pose_data = self.mqtt_subscriber.get_latest_message(TOPIC_EXTENDED_POSE_W_BIAS)
        if self.extended_pose_data is not None:
            # Retrieve timestamp and T_ab_k from message
            t_k = self.extended_pose_data.timestamp
            T_ab_k = form_se3_from_localization_message(self.extended_pose_data)

            # Update depth wavemap manager with new pose
            self.depth_wavemap_manager.integrate_depth_image(t_k, T_ab_k)

            # Generate grid message and output
            grid_msg = form_flattened_occupancy_grid_message(
                t_k,
                self.depth_wavemap_manager.get_flattened_upper_body_occupancy_grid(),
                self.depth_wavemap_manager.occupancy_grid.grid_length
            )

            # Publish the occupancy grid
            self.publisher.publish_msg(TOPIC_OCCUPANCY_GRID, grid_msg)

            # Generate traversability grid message and output
            traversability_grid_msg = form_flattened_occupancy_grid_message(
                t_k,
                self.depth_wavemap_manager.get_flattened_traversability_grid(),
                self.depth_wavemap_manager.occupancy_grid.grid_length
            )
            # Publish the traversability grid
            self.publisher.publish_msg(TOPIC_TRAVERSABILITY_GRID, traversability_grid_msg)

            # Generate robot pose grid coords message and output
            robot_pose_grid_coords_msg = form_robot_pose_grid_coords_message(
                self.extended_pose_data,
                self.depth_wavemap_manager.occupancy_grid.query_cell_width,
                self.depth_wavemap_manager.occupancy_grid.planar_spread
            )
            self.publisher.publish_msg(TOPIC_ROBOT_POSE_GRID_COORDS, robot_pose_grid_coords_msg)

if __name__ == "__main__":
    node = DepthWavemapNode()
    node.run()
