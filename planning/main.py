import logging
logging.getLogger('matplotlib').setLevel(logging.WARNING)

import time
import os
import yaml

import numpy as np

import sys
sys.path.append('..')
from d_star import DStar
from lookahead_controller import LookaheadController
from utils.messages.extended_pose_w_bias_msg import EXTENDED_POSE_W_BIAS_MSG
from utils.messages.gyro_data_msg import GYRO_DATA_MSG
from utils.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from utils.messages.trajectory_msg import TRAJECTORY_MSG
from utils.topic_to_message_type import TOPIC_TRAVERSABILITY_GRID, TOPIC_EXTENDED_POSE_W_BIAS, TOPIC_GYRO, TOPIC_TRAJECTORY
from utils.mqtt_utils import MQTTSubscriber, MQTTPublisher
from utils.logging_utils import Logger

from utils import constants as CFG

class TrajectoryPlanner(object):
    def __init__(self):
        self.logger = Logger('trajectory_planner', 'logs/trajectory_planner.log', level=logging.INFO)

        self.mqtt_subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={TOPIC_TRAVERSABILITY_GRID: OCCUPANCY_GRID_MSG,
                                                                                                TOPIC_EXTENDED_POSE_W_BIAS: EXTENDED_POSE_W_BIAS_MSG,
                                                                                                "/target_point": None})
        self.mqtt_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_TRAJECTORY: TRAJECTORY_MSG})

        self.loop_rate_hz = 5

        self.traversability_grid = None
        self.extended_pose_w_bias = None
        self.gyro_data = None
        self.target_point_msg = None

        self.dstar = DStar()

        # Grid parameters
        # retrieve yaml file location
        map_config_loc = os.getenv("BASE_CONFIG_LOC", CFG.MAPPING_BASE_CONFIG_LOC)

        # add /home/$USER to the beginning of the path
        map_config_loc = f"/home/{os.getenv('USER')}/{map_config_loc}"

        # retrieve configuration yaml
        with open(map_config_loc, 'r') as stream:
            self.map_config = yaml.safe_load(stream)

        self.grid_cell_size_m = self.map_config['grid_params']['grid_cell_size']
        self.grid_width_m = self.map_config['grid_params']['default_planar_spread']

    def run(self):
        self.mqtt_subscriber.start()
        self.mqtt_publisher.run()

        replan_time = time.time()

        try:
            while True:
                start_time = time.time()

                while self.traversability_grid is None:
                    time.sleep(0.005)
                    self.traversability_grid = self.mqtt_subscriber.get_latest_message(TOPIC_TRAVERSABILITY_GRID)

                traversability_grid = self.mqtt_subscriber.get_latest_message(TOPIC_TRAVERSABILITY_GRID)
                if traversability_grid is not None:
                    self.traversability_grid = traversability_grid

                while self.extended_pose_w_bias is None:
                    time.sleep(0.001)
                    self.extended_pose_w_bias = self.mqtt_subscriber.get_latest_message(TOPIC_EXTENDED_POSE_W_BIAS)

                target_point_msg = self.mqtt_subscriber.get_latest_message("/target_point")
                if target_point_msg is not None:
                    self.target_point_msg = target_point_msg

                traversability_grid = process_traversability_grid(self.traversability_grid)

                world_x_pos, world_y_pos, world_yaw = process_extended_pose_w_bias(self.extended_pose_w_bias)

                # print(f"Pose: ({world_x_pos}, {world_y_pos}, {world_yaw})")

                if (time.time() - replan_time) > 2.0 and self.target_point_msg is not None:

                    grid_start_pose_x, grid_start_pose_y = convert_pose_to_grid_coords((world_x_pos, world_y_pos), self.grid_cell_size_m, self.grid_width_m)
                    # grid_goal_pose_x, grid_goal_pose_y = convert_pose_to_grid_coords((goal_pose_x, goal_pose_y), self.grid_cell_size_m, self.grid_width_m)
    
                    print(f"Target point: {self.target_point_msg}")
                    try:
                        target_point = self.target_point_msg['data']
                        grid_goal_pose_x = target_point[0]
                        grid_goal_pose_y = target_point[1]
                    except:
                        try:
                            grid_goal_pose_x = target_point[0]
                            grid_goal_pose_y = target_point[1]
                        except:
                            print("Target point is not in the correct format", self.target_point_msg)
                            continue

                    print(f"Attempting to plan from ({grid_start_pose_x}, {grid_start_pose_y}) to ({grid_goal_pose_x}, {grid_goal_pose_y})")

                    self.dstar.initialize(traversability_grid, (grid_start_pose_x, grid_start_pose_y), (grid_goal_pose_x, grid_goal_pose_y))
                    # self.dstar.plot_path()

                    path = self.dstar.run()

                    if path:
                        path_world_pose_list = [convert_grid_coords_to_pose(path[i], self.grid_cell_size_m, self.grid_width_m) for i in range(len(path))]
                        print("Path found:", path_world_pose_list)
                        # self.dstar.plot_path()
                    else:
                        # self.dstar.plot_path()
                        print("No path found")
                        continue

                    # trajectory planning
                    lookahead_controller = LookaheadController(lookahead_distance=0.05, max_linear_velocity=0.5, max_angular_velocity=0.1, acceleration=0.1)
                    goal_pose = lookahead_controller.find_goal_pose(path_world_pose_list, (world_x_pos, world_y_pos))
                    trajectory = lookahead_controller.compute_trajectory((world_x_pos, world_y_pos, world_yaw), goal_pose)

                    # Publish trajectory
                    trajectory_msg = TRAJECTORY_MSG()
                    trajectory_msg.timestamp = time.time()
                    traj_dict = {}
                    for i in range(len(trajectory)):
                        traj_dict[trajectory[i][0]] = [trajectory[i][1], trajectory[i][2]]
                    trajectory_msg.trajectory = traj_dict
                    self.mqtt_publisher.publish_msg(TOPIC_TRAJECTORY, trajectory_msg)

                    # Reset variables
                    self.traversability_grid = None
                    self.extended_pose_w_bias = None
                    self.gyro_data = None
                    replan_time = time.time()

                end_time = time.time()
                sleep_time = 1/self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            self.logger.info("Trajectory planner stopped by user.")
        finally:
            self.mqtt_subscriber.stop()
            self.mqtt_publisher.stop()

def convert_grid_coords_to_pose(grid_coords, grid_cell_size_m: float, grid_width_m: float):
    offset = grid_width_m / 2
    return (grid_coords[0] * grid_cell_size_m - offset, grid_coords[1] * grid_cell_size_m - offset)

def convert_pose_to_grid_coords(pose, grid_cell_size_m: float, grid_width_m: float):
    offset = grid_width_m / 2
    return int((-pose[0] + offset) / grid_cell_size_m), int((pose[1] + offset) / grid_cell_size_m)

def process_traversability_grid(traversability_grid: OCCUPANCY_GRID_MSG):
    width = traversability_grid.width

    # Convert the flattened list back to a 2D numpy array
    grid_array = np.array(traversability_grid.flattened_grid_list).reshape((width, width))

    return grid_array

def process_extended_pose_w_bias(extended_pose_w_bias: EXTENDED_POSE_W_BIAS_MSG):
    Tr_a_b = np.array([extended_pose_w_bias.r_a_b_x, extended_pose_w_bias.r_a_b_y, extended_pose_w_bias.r_a_b_z]).reshape(1, 3)
    phi_a_b = np.array([extended_pose_w_bias.phi_a_b_x, extended_pose_w_bias.phi_a_b_y, extended_pose_w_bias.phi_a_b_z]).reshape(1, 3)
    return Tr_a_b[0, 0], Tr_a_b[0, 1], phi_a_b[0, 2]

if __name__ == "__main__":
    trajectory_planner = TrajectoryPlanner()
    trajectory_planner.run()
