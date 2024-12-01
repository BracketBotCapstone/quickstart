import logging
import time
import os
import yaml
import numpy as np
import random

import sys
sys.path.append('.')
from utils.messages.extended_pose_w_bias_msg import EXTENDED_POSE_W_BIAS_MSG
from utils.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from utils.topic_to_message_type import TOPIC_TRAVERSABILITY_GRID, TOPIC_EXTENDED_POSE_W_BIAS
from utils.mqtt_utils import MQTTSubscriber, MQTTPublisher
from utils.logging_utils import Logger
from utils import constants as CFG

class RoombaNode:
    def __init__(self):
        self.logger = Logger('roomba_node', 'logs/roomba_node.log', level=logging.INFO)
        
        # Initialize subscribers
        self.mqtt_subscriber = MQTTSubscriber(broker_address="localhost", 
                                            topic_to_message_map={
                                                TOPIC_TRAVERSABILITY_GRID: OCCUPANCY_GRID_MSG,
                                                TOPIC_EXTENDED_POSE_W_BIAS: EXTENDED_POSE_W_BIAS_MSG
                                            })
        
        # Initialize velocity publisher
        self.velocity_publisher = MQTTPublisher(broker_address="localhost")
        
        # Load map config
        map_config_loc = os.getenv("BASE_CONFIG_LOC", CFG.MAPPING_BASE_CONFIG_LOC)
        map_config_loc = f"/home/{os.getenv('USER')}/{map_config_loc}"
        with open(map_config_loc, 'r') as stream:
            self.map_config = yaml.safe_load(stream)
            
        self.grid_cell_size_m = self.map_config['grid_params']['grid_cell_size']
        self.grid_width_m = self.map_config['grid_params']['default_planar_spread']
        
        self.traversability_grid = None
        self.robot_pose = None
        
        self.loop_rate_hz = 10
        self.obstacle_threshold = 0.0  # 20cm detection radius
        
    def convert_pose_to_grid_coords(self, pose):
        offset = self.grid_width_m / 2
        return (int((-pose[0] + offset) / self.grid_cell_size_m), 
                int((pose[1] + offset) / self.grid_cell_size_m))

    def check_obstacles_nearby(self, grid_x, grid_y, grid):
        search_radius = int(self.obstacle_threshold / self.grid_cell_size_m)
        height, width = grid.shape
        
        for i in range(max(0, grid_x - search_radius), min(width, grid_x + search_radius + 1)):
            for j in range(max(0, grid_y - search_radius), min(height, grid_y + search_radius + 1)):
                if grid[j,i] > 0.5:  # If cell is occupied
                    return True
        return False
    
    def process_traversability_grid(self,traversability_grid: OCCUPANCY_GRID_MSG):
        width = traversability_grid.width

        # Convert the flattened list back to a 2D numpy array
        grid_array = np.array(traversability_grid.flattened_grid_list).reshape((width, width))

        return grid_array

    def process_extended_pose_w_bias(self,extended_pose_w_bias: EXTENDED_POSE_W_BIAS_MSG):
        Tr_a_b = np.array([extended_pose_w_bias.r_a_b_x, extended_pose_w_bias.r_a_b_y, extended_pose_w_bias.r_a_b_z]).reshape(1, 3)
        phi_a_b = np.array([extended_pose_w_bias.phi_a_b_x, extended_pose_w_bias.phi_a_b_y, extended_pose_w_bias.phi_a_b_z]).reshape(1, 3)
        return Tr_a_b[0, 0], Tr_a_b[0, 1], phi_a_b[0, 2]

    # def publish_velocity(self, linear_vel, angular_vel):
    #     msg = {"linear": linear_vel, "angular": angular_vel}
    #     self.velocity_publisher.publish_dict("robot/velocity", msg)

    def publish_velocity(self, linear_vel, angular_vel):
        msg = {
            'timestamp': time.time(),
            'trajectory': {time.time()+100: [linear_vel, angular_vel]},
        }
        self.velocity_publisher.publish_dict("/planning/trajectory", msg)

    def run(self):
        self.mqtt_subscriber.start()
        self.velocity_publisher.run()
        
        try:
            while True:
                start_time = time.time()

                # Wait for initial traversability grid
                while self.traversability_grid is None:
                    time.sleep(0.005)
                    self.traversability_grid = self.mqtt_subscriber.get_latest_message(TOPIC_TRAVERSABILITY_GRID)

                # Get latest traversability grid if available
                traversability_grid = self.mqtt_subscriber.get_latest_message(TOPIC_TRAVERSABILITY_GRID)
                if traversability_grid is not None:
                    self.traversability_grid = traversability_grid

                # Wait for initial pose
                while self.robot_pose is None:
                    time.sleep(0.001)
                    self.robot_pose = self.mqtt_subscriber.get_latest_message(TOPIC_EXTENDED_POSE_W_BIAS)

                # Process grid and pose
                grid = self.process_traversability_grid(self.traversability_grid)
                robot_x, robot_y, robot_yaw = self.process_extended_pose_w_bias(self.robot_pose)
                
                # Convert to grid coordinates
                grid_x, grid_y = self.convert_pose_to_grid_coords((robot_x, robot_y))
                
                # if grid[200, 200] == 0:
                #     self.logger.info(f"200,200 is traversable")
                # else:
                #     self.logger.info(f"200,200 is not traversable")


                # traversable = self.check_obstacles_nearby(grid_x, grid_y, grid)
                traversable = False
                if grid[grid_y, grid_x] == 0:
                    traversable = True
                    self.logger.info(f"Current position {(grid_x,grid_y)} is traversable")
                else:
                    self.logger.info(f"Current position {(grid_x,grid_y)} is not traversable")

                # Check for nearby obstacles
                if traversable:
                    # Obstacle detected - turn random direction
                    self.logger.info("Obstacle detected! Turning...")
                    turn_angle = random.uniform(-np.pi/2, np.pi/2)  # Random angle between -90 and 90 degrees
                    self.publish_velocity(0.2, 0.0)
                    time.sleep(1.0)  # Wait for turn
                    self.publish_velocity(0.0, turn_angle)
                    time.sleep(1.0)  # Wait for turn
                else:
                    print('gucci')
                
                # Move forward
                self.publish_velocity(0.2, 0.0)
            
                # Sleep to maintain loop rate
                end_time = time.time()
                sleep_time = 1/self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

                self.robot_pose = None
                self.traversability_grid = None
                    
        except KeyboardInterrupt:
            self.logger.info("Roomba node stopped by user")
            self.mqtt_subscriber.stop()
            self.velocity_publisher.stop()

if __name__ == "__main__":
    node = RoombaNode()
    node.run()
