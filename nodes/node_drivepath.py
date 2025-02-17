"""Node for path planning."""

import logging
import time

import numpy as np

import lib.constants as CFG
from lib.logging_utils import Logger
from lib.planning.d_star import DStar
from lib.messages.mqtt_utils import MQTTPublisher, MQTTSubscriber
from lib.messages.path_plan_msg import PATH_PLAN_MSG
from lib.messages.robot_pose_grid_coords_msg import ROBOT_POSE_GRID_COORDS_MSG
from lib.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from lib.messages.target_point_msg import TARGET_POINT_MSG
from lib.messages.topic_to_message_type import (TOPIC_PATH_PLAN,
                                                TOPIC_TRAVERSABILITY_GRID,
                                                TOPIC_ROBOT_POSE_GRID_COORDS,
                                                TOPIC_TARGET_POINT)

# Set logging level for matplotlib to WARNING
logging.getLogger('matplotlib').setLevel(logging.WARNING)


class DrivePathNode(object):
    """DrivePathNode class."""

    def __init__(self, logging_level=logging.INFO):
        """Initialize the DrivePathNode class."""
        self.logger = Logger('drivepath', 'logs/drivepath.log', level=logging_level)

        self.mqtt_subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={TOPIC_TARGET_POINT: TARGET_POINT_MSG,
                                                                                                TOPIC_ROBOT_POSE_GRID_COORDS: ROBOT_POSE_GRID_COORDS_MSG,
                                                                                                TOPIC_TRAVERSABILITY_GRID: OCCUPANCY_GRID_MSG})
        self.mqtt_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_PATH_PLAN: PATH_PLAN_MSG})

        # Initialize message variables
        self.path_plan_msg = None
        self.traversability_grid_msg = None
        self.robot_pose_grid_coords_msg = None
        self.current_target_point_msg = None
        self.target_point_msg = None

        self.dstar = DStar()  # Initialize D* pathfinding algorithm

        # Extract grid parameters from configuration
        self.grid_cell_size_m = CFG.MAPPING_GRID_GRID_CELL_SIZE
        self.grid_width_m = CFG.MAPPING_GRID_DEFAULT_PLANAR_SPREAD

        self.loop_rate_hz = 30

        self.path_pose_list = None
        self.path_grid_coords_list = None

    def run(self):
        self.mqtt_subscriber.start()
        self.mqtt_publisher.run()

        replan_time = time.time()  # Initialize replan time

        try:
            while True:
                start_time = time.time()

                # Get traversability grid
                traversability_grid_msg = self.mqtt_subscriber.get_latest_message(TOPIC_TRAVERSABILITY_GRID)
                if traversability_grid_msg is not None:
                    self.traversability_grid_msg = traversability_grid_msg

                # Get robot pose in grid coordinates
                self.robot_pose_grid_coords_msg = self.mqtt_subscriber.get_latest_message(TOPIC_ROBOT_POSE_GRID_COORDS)
                if self.robot_pose_grid_coords_msg is not None:
                    grid_start_pose_x = self.robot_pose_grid_coords_msg.x_grid
                    grid_start_pose_y = self.robot_pose_grid_coords_msg.y_grid
                else:
                    grid_start_pose_x = None
                    grid_start_pose_y = None

                # Get target point message
                target_point_changed = False
                target_point_msg = self.mqtt_subscriber.get_latest_message(TOPIC_TARGET_POINT)
                if target_point_msg is not None:
                    if self.target_point_msg is None or self.target_point_msg.x_grid != target_point_msg.x_grid or self.target_point_msg.y_grid != target_point_msg.y_grid:
                        target_point_changed = True
                    self.target_point_msg = target_point_msg

                if self.target_point_msg is not None:
                    try:
                        grid_goal_pose_x = self.target_point_msg.x_grid
                        grid_goal_pose_y = self.target_point_msg.y_grid
                    except:
                        print("Target point is not in the correct format", self.target_point_msg)
                        continue

                    # If within tolerance of the target point, don't replan
                    if grid_start_pose_x is not None and grid_start_pose_y is not None:
                        if np.linalg.norm([grid_start_pose_x - grid_goal_pose_x, grid_start_pose_y - grid_goal_pose_y]) < 1.5:
                            self.target_point_msg = None
                            self.path_pose_list = None
                            self.traversability_grid_msg = None
                            self.path_grid_coords_list = None

                            print("Target point has been reached, not replanning.")
                            continue

                # Check if it's time to replan
                if (time.time() - replan_time > 10.0) and \
                    self.target_point_msg is not None and \
                    self.traversability_grid_msg is not None and \
                    self.robot_pose_grid_coords_msg is not None:

                    # Process traversability grid
                    traversability_grid = process_traversability_grid_msg(self.traversability_grid_msg)

                    new_obstacle_in_path = False
                    if self.path_grid_coords_list is not None:
                        for grid_coords in self.path_grid_coords_list:
                            if traversability_grid[grid_coords[0], grid_coords[1]] == 1:
                                new_obstacle_in_path = True
                                break

                    if target_point_changed or new_obstacle_in_path:

                        print(f"Attempting to plan from ({grid_start_pose_x}, {grid_start_pose_y}) to ({grid_goal_pose_x}, {grid_goal_pose_y})")

                        # Initialize D* algorithm and run pathfinding
                        self.dstar.initialize(traversability_grid, (grid_start_pose_y, grid_start_pose_x), (grid_goal_pose_y, grid_goal_pose_x))
                        path = self.dstar.run()
                        self.path_grid_coords_list = path

                        if path:
                            # Convert path to world coordinates
                            self.path_pose_list = [convert_grid_coords_to_pose(path[i], self.grid_cell_size_m, self.grid_width_m) for i in range(len(path))]

                            print("Path found:")
                            for i in range(0, len(self.path_pose_list)):
                                print(f"Pose {i}: {self.path_pose_list[i]}")
                            # self.dstar.plot_path()
                        else:
                            self.path_pose_list = None
                            # self.dstar.plot_path()
                            print("No path found")
                            continue

                        replan_time = time.time()

                # Publish path plan message
                if self.path_pose_list is not None:
                    self.path_plan_msg = PATH_PLAN_MSG()
                    self.path_plan_msg.timestamp = time.time()
                    self.path_plan_msg.path_pose_list = self.path_pose_list
                    self.mqtt_publisher.publish_msg(TOPIC_PATH_PLAN, self.path_plan_msg)

                end_time = time.time()
                sleep_time = 1/self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            self.logger.info("Planning node stopped by user.")
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
    return ((grid_coords[1] * grid_cell_size_m - offset), grid_coords[0] * grid_cell_size_m - offset)

def process_traversability_grid_msg(traversability_grid_msg: OCCUPANCY_GRID_MSG):
    """
    Process the traversability grid message.

    Parameters
    ----------
    traversability_grid_msg : OCCUPANCY_GRID_MSG
        The traversability grid message.

    Returns
    -------
    np.array
        2D numpy array of the traversability grid.
    """
    width = traversability_grid_msg.width

    # Convert the flattened list back to a 2D numpy array
    grid_array = np.array(traversability_grid_msg.flattened_grid_list).reshape((width, width))

    return grid_array


if __name__ == "__main__":
    drivepath_node = DrivePathNode()
    drivepath_node.run()
