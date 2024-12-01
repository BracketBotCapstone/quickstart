import os
import time
import logging

import sys
sys.path.append('..')

import utils.constants as CFG

from mapping.wv_manager import DepthWavemapManager

from utils.mqtt_utils import MQTTSubscriber, MQTTPublisher
from utils.logging_utils import Logger

from utils.messages.extended_pose_w_bias_msg import EXTENDED_POSE_W_BIAS_MSG
from utils.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from utils.topic_to_message_type import TOPIC_EXTENDED_POSE_W_BIAS, TOPIC_OCCUPANCY_GRID, TOPIC_TRAVERSABILITY_GRID

from localization.utils.input_processing import form_se3_from_localization_message, form_flattened_occupancy_grid_message

class DepthWavemapNode:
    def __init__(self, logging_level = logging.INFO):
        self.logger = Logger('depth_wavemap_node', 'logs/depth_wavemap_node.log', level=logging_level)

        self.logger.info("Initializing DepthWavemapNode")

        # retrieve yaml file location
        map_config_loc = os.getenv("BASE_CONFIG_LOC", CFG.MAPPING_BASE_CONFIG_LOC)

        # add /home/$USER to the beginning of the path
        map_config_loc = f"/home/{os.getenv('USER')}/{map_config_loc}"

        # initialize depth wavemap manager
        self.depth_wavemap_manager = DepthWavemapManager(map_config_loc)

        # initialize grid publisher
        self.grid_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_OCCUPANCY_GRID: OCCUPANCY_GRID_MSG})

        # initialize traversability publisher
        self.traversability_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_TRAVERSABILITY_GRID: OCCUPANCY_GRID_MSG})

        # initialize pose subscriber
        self.mqtt_subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={TOPIC_EXTENDED_POSE_W_BIAS: EXTENDED_POSE_W_BIAS_MSG})

        self.loop_rate_hz = 20

    def run(self):

        self.mqtt_subscriber.start()
        self.grid_publisher.run()
        self.traversability_publisher.run()

        # start pipeline
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
        finally:
            self.depth_wavemap_manager.rs_manager.stop_pipeline()
        
    def process_input(self):
        self.extended_pose_data = self.mqtt_subscriber.get_latest_message(TOPIC_EXTENDED_POSE_W_BIAS)
        if self.extended_pose_data is not None:
            # print("Received extended pose state message!")

            # retrieve timestamp and T_ab_k from message
            t_k = self.extended_pose_data.timestamp
            T_ab_k = form_se3_from_localization_message(self.extended_pose_data)

            # print(f"Integrating depth image at time {t_k}")

            # update depth wavemap manager with new pose
            self.depth_wavemap_manager.integrate_depth_image(t_k, T_ab_k)

            # generate grid message and output
            grid_msg = form_flattened_occupancy_grid_message(t_k, self.depth_wavemap_manager.get_flattened_upper_body_occupancy_grid(), self.depth_wavemap_manager.occupancy_grid.grid_length)

            # publish the occupancy grid
            self.grid_publisher.publish_msg(TOPIC_OCCUPANCY_GRID, grid_msg)

            # generate traversability grid message and output
            traversability_grid_msg = form_flattened_occupancy_grid_message(t_k, self.depth_wavemap_manager.get_flattened_traversability_grid(), self.depth_wavemap_manager.occupancy_grid.grid_length)

            # publish the traversability grid
            self.traversability_publisher.publish_msg(TOPIC_TRAVERSABILITY_GRID, traversability_grid_msg)

if __name__ == "__main__":
    node = DepthWavemapNode()
    node.run()
