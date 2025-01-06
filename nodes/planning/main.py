import logging
import os
import time

import numpy as np
import yaml

import utils.constants as CFG
from nodes.planning.d_star import DStar
from nodes.planning.lookahead_controller import LookaheadController
from utils.logging_utils import Logger
from utils.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from utils.messages.robot_pose_grid_coords_msg import ROBOT_POSE_GRID_COORDS_MSG
from utils.messages.robot_pose_msg import ROBOT_POSE_MSG
from utils.messages.target_point_msg import TARGET_POINT_MSG
from utils.messages.trajectory_msg import TRAJECTORY_MSG
from utils.mqtt_utils import MQTTPublisher, MQTTSubscriber
from utils.topic_to_message_type import (
    TOPIC_ROBOT_POSE,
    TOPIC_ROBOT_POSE_GRID_COORDS,
    TOPIC_TARGET_POINT,
    TOPIC_TRAJECTORY,
    TOPIC_TRAVERSABILITY_GRID,
)

# Set logging level for matplotlib to WARNING
logging.getLogger('matplotlib').setLevel(logging.WARNING)

class TrajectoryPlanner(object):
    def __init__(self):
        """
        Initialize the TrajectoryPlanner with necessary configurations and MQTT setup.
        """
        self.logger = Logger('trajectory_planner', 'logs/trajectory_planner.log', level=logging.INFO)

        # Initialize MQTT subscriber and publisher
        self.mqtt_subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={
            TOPIC_TRAVERSABILITY_GRID: OCCUPANCY_GRID_MSG,
            TOPIC_ROBOT_POSE: ROBOT_POSE_MSG,
            TOPIC_TARGET_POINT: TARGET_POINT_MSG,
            TOPIC_ROBOT_POSE_GRID_COORDS: ROBOT_POSE_GRID_COORDS_MSG
        })
        self.mqtt_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={
            TOPIC_TRAJECTORY: TRAJECTORY_MSG
        })

        self.loop_rate_hz = 10  # Loop rate in Hz

        # Initialize message variables
        self.traversability_grid = None
        self.robot_pose_msg = None
        self.robot_pose_grid_coords_msg = None
        self.target_point_msg = None

        self.dstar = DStar()  # Initialize D* pathfinding algorithm

        # Grid parameters
        # Retrieve yaml file location
        map_config_loc = os.getenv("BASE_CONFIG_LOC", CFG.MAPPING_BASE_CONFIG_LOC)
        # Add /home/$USER to the beginning of the path
        map_config_loc = f"/home/{os.getenv('USER')}/{map_config_loc}"

        # Retrieve configuration yaml
        with open(map_config_loc, 'r') as stream:
            self.map_config = yaml.safe_load(stream)

        # Extract grid parameters from configuration
        self.grid_cell_size_m = self.map_config['grid_params']['grid_cell_size']
        self.grid_width_m = self.map_config['grid_params']['default_planar_spread']

    def run(self):
        """
        Main loop for the trajectory planner. Subscribes to necessary topics, processes messages,
        and publishes planned trajectories.
        """
        self.mqtt_subscriber.start()
        self.mqtt_publisher.run()

        replan_time = time.time()  # Initialize replan time

        # Initialize lookahead controller
        lookahead_controller = LookaheadController(
            lookahead_distance=0.45,
            max_linear_velocity=0.5,
            max_angular_velocity=0.5,
            acceleration=0.1
        )
        self.prev_goal_pose = None
        self.prev_trajectory = None

        try:
            while True:
                # Get start time for setting loop rate
                start_time = time.time()

                # Get traversability grid
                while self.traversability_grid is None:
                    time.sleep(0.005)
                    self.traversability_grid = self.mqtt_subscriber.get_latest_message(TOPIC_TRAVERSABILITY_GRID)
                traversability_grid = self.mqtt_subscriber.get_latest_message(TOPIC_TRAVERSABILITY_GRID)
                if traversability_grid is not None:
                    self.traversability_grid = traversability_grid

                # Get robot pose
                while self.robot_pose_msg is None:
                    time.sleep(0.001)
                    self.robot_pose_msg = self.mqtt_subscriber.get_latest_message(TOPIC_ROBOT_POSE)
                robot_pose_msg = self.mqtt_subscriber.get_latest_message(TOPIC_ROBOT_POSE)
                if robot_pose_msg is not None:
                    self.robot_pose_msg = robot_pose_msg

                # Get robot pose in grid coordinates
                while self.robot_pose_grid_coords_msg is None:
                    time.sleep(0.001)
                    self.robot_pose_grid_coords_msg = self.mqtt_subscriber.get_latest_message(TOPIC_ROBOT_POSE_GRID_COORDS)
                robot_pose_grid_coords_msg = self.mqtt_subscriber.get_latest_message(TOPIC_ROBOT_POSE_GRID_COORDS)
                if robot_pose_grid_coords_msg is not None:
                    self.robot_pose_grid_coords_msg = robot_pose_grid_coords_msg

                # Get target point message
                target_point_msg = self.mqtt_subscriber.get_latest_message(TOPIC_TARGET_POINT)
                if target_point_msg is not None:
                    self.target_point_msg = target_point_msg

                # Process traversability grid
                traversability_grid = process_traversability_grid(self.traversability_grid)
                world_x_pos = self.robot_pose_msg.x_m
                world_y_pos = self.robot_pose_msg.y_m
                world_yaw = self.robot_pose_msg.theta_rad

                # Check if it's time to replan
                if time.time() - replan_time > 4.0 and self.target_point_msg is not None:
                    grid_start_pose_x = self.robot_pose_grid_coords_msg.x_grid
                    grid_start_pose_y = self.robot_pose_grid_coords_msg.y_grid

                    try:
                        grid_goal_pose_x = self.target_point_msg.x_grid
                        grid_goal_pose_y = self.target_point_msg.y_grid
                    except:
                        print("Target point is not in the correct format", self.target_point_msg)
                        continue

                    print(f"Attempting to plan from ({grid_start_pose_x}, {grid_start_pose_y}) to ({grid_goal_pose_x}, {grid_goal_pose_y})")

                    # If within tolerance of the target point, don't replan
                    if np.linalg.norm([grid_start_pose_x - grid_goal_pose_x, grid_start_pose_y - grid_goal_pose_y]) < 0.1:
                        self.prev_trajectory = None
                        self.target_point_msg = None
                        continue

                    # Initialize D* algorithm and run pathfinding
                    self.dstar.initialize(traversability_grid, (grid_start_pose_y, grid_start_pose_x), (grid_goal_pose_y, grid_goal_pose_x))
                    path = self.dstar.run()

                    if path:
                        # Convert path to world coordinates
                        path_pose_list = [convert_grid_coords_to_pose(path[i], self.grid_cell_size_m, self.grid_width_m) for i in range(len(path))]
                        print("Path found:", path_pose_list[0], path_pose_list[-1])
                        self.dstar.plot_path()
                    else:
                        self.dstar.plot_path()
                        print("No path found")
                        continue

                    # Trajectory planning
                    goal_pose = lookahead_controller.find_goal_pose(path_pose_list, (world_x_pos, world_y_pos))
                    self.prev_trajectory = lookahead_controller.compute_trajectory((world_x_pos, world_y_pos, world_yaw), goal_pose)

                    if self.prev_trajectory is not None:
                        # Publish trajectory
                        trajectory_msg = TRAJECTORY_MSG()
                        trajectory_msg.timestamp = time.time()
                        traj_dict = {}
                        for i in range(len(self.prev_trajectory)):
                            traj_dict[self.prev_trajectory[i][0]] = [self.prev_trajectory[i][1], self.prev_trajectory[i][2]]
                        trajectory_msg.trajectory = traj_dict
                        self.mqtt_publisher.publish_msg(TOPIC_TRAJECTORY, trajectory_msg)

                    # Reset variables
                    self.traversability_grid = None
                    self.robot_pose_msg = None
                    self.robot_pose_grid_coords_msg = None
                    replan_time = time.time()

                # Calculate sleep time to maintain loop rate
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
    """
    Convert grid coordinates to world coordinates.

    Parameters
    ----------
    grid_coords : tuple
        The grid coordinates (x, y).
    grid_cell_size_m : float
        The size of each grid cell in meters.
    grid_width_m : float
        The width of the grid in meters.

    Returns
    -------
    tuple
        (x, y) in world coordinates.
    """
    offset = grid_width_m / 2
    return (-(grid_coords[1] * grid_cell_size_m - offset), grid_coords[0] * grid_cell_size_m - offset)

def process_traversability_grid(traversability_grid: OCCUPANCY_GRID_MSG):
    """
    Process the traversability grid message.

    Parameters
    ----------
    traversability_grid : OCCUPANCY_GRID_MSG
        The traversability grid message.

    Returns
    -------
    np.array
        2D numpy array of the traversability grid.
    """
    width = traversability_grid.width

    # Convert the flattened list back to a 2D numpy array
    grid_array = np.array(traversability_grid.flattened_grid_list).reshape((width, width))

    return grid_array

if __name__ == "__main__":
    trajectory_planner = TrajectoryPlanner()
    trajectory_planner.run()
