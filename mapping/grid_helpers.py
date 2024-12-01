import pywavemap as wave
import os
import numpy as np
import cv2

import yaml

import copy

def generate_node_indicies(map_min_cell_width : float, planar_spread : float, z_lower_lim : float, z_upper_lim : float) -> np.ndarray:

    # generate range of z values to query
    z_range = np.arange(np.round(z_lower_lim / map_min_cell_width), np.round(z_upper_lim / map_min_cell_width) + 1)

    # retrieve positions corresponding to a certain planar spread 
    steps = np.round(planar_spread / map_min_cell_width)

    x = np.arange(-steps / 2, steps / 2)
    y = np.arange(-steps / 2, steps / 2)

    # generate 3D meshgrid  
    node_idx_mgrid = np.array(np.meshgrid(x, y, z_range))

    return node_idx_mgrid.T.reshape(-1, 3), x.shape[0], z_range.shape[0]

class DynamicOccupancyGrid:
    """
    Helper class for creating and updating an occupancy grid from a Wavemap environmental map.
    """

    def __init__(self, map_config_loc):
        # load map config
        with open(map_config_loc, 'r') as stream:
            try:
                self.map_config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        # load realsense physical config
        realsense_config_loc = self.map_config['config_params']['realsense_config_loc']
        # add /home/$USER to the beginning of the path
        realsense_config_loc = f"/home/{os.getenv('USER')}/{realsense_config_loc}"

        with open(realsense_config_loc, 'r') as stream:
            try:
                self.realsense_config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        # retrieve height bounds from RealSense config
        self.r_bc = np.array(self.realsense_config['extrinsics']['r_bc']).reshape(3, )

        # retrieve lower height limit
        self.z_lower_lim = self.map_config['grid_params']['z_lower_lim']

        # retrieve upper height limit
        self.z_upper_lim = self.r_bc[2] + self.map_config['grid_params']['z_upper_lim_epsilon']

        # retrieve remaining grid parameters for construction
        self.query_cell_width = self.map_config['grid_params']['grid_cell_size']
        self.planar_spread = self.map_config['grid_params']['default_planar_spread']

        # retrieve physical body radius
        self.physical_body_rad = self.map_config['grid_params']['body_radius'] + self.map_config['grid_params']['body_radius_epsilon']

        # convert physical body radius to cell width
        self.physical_body_rad_cell_width = int(np.round(self.physical_body_rad / self.query_cell_width))

        # construct node indicies for lower body
        self.z_lower_body_lim = self.map_config['grid_params']['z_lower_body_limit']

        self.lower_body_indicies, self.grid_length, self.lower_body_grid_height = generate_node_indicies(self.query_cell_width, self.planar_spread, self.z_lower_lim, self.z_lower_body_lim)

        # construct node indicies for upper body
        self.upper_body_indicies, self.grid_length, self.upper_body_grid_height = generate_node_indicies(self.query_cell_width, self.planar_spread, self.z_lower_body_lim, self.z_upper_lim)

        # generate query height
        self.query_height = wave.convert.cell_width_to_height(self.query_cell_width, self.map_config['map_params']['min_cell_width'])

        # initialize query point for lower body
        self.lower_body_query_points = np.concatenate((np.tile(self.query_height, (len(self.lower_body_indicies), 1)), self.lower_body_indicies), axis=1)

        # initialize query point for upper body
        self.upper_body_query_points = np.concatenate((np.tile(self.query_height, (len(self.upper_body_indicies), 1)), self.upper_body_indicies), axis=1)

        # initialize lower body occupancy grid
        self.lower_body_occupancy_grid = np.zeros((self.grid_length, self.grid_length))

        # initialize upper body occupancy grid
        self.upper_body_occupancy_grid = np.zeros((self.grid_length, self.grid_length))

        # initialize traversability grid
        self.traversability_grid = np.zeros((self.grid_length, self.grid_length))

        # Initialize video writer
        self.video_writer = cv2.VideoWriter(
            'occupancy_grid.mp4', 
            cv2.VideoWriter_fourcc(*'mp4v'), 
            10,  # frames per second
            (self.grid_length, self.grid_length),  # frame size
            isColor=False  # grayscale
        )

    def reset_occupancy_grid(self):
        """
        Reset occupancy grid to all zeros.
        """

        self.lower_body_occupancy_grid = np.zeros((self.grid_length, self.grid_length))
        self.upper_body_occupancy_grid = np.zeros((self.grid_length, self.grid_length))

    def reset_query_size(self, new_query_cell_width : float):
        """
        Reset query size to new query cell width.
        """

        # update query cell width
        self.query_cell_width = new_query_cell_width

        # update query height
        self.query_height = wave.convert.cell_width_to_height(self.query_cell_width, self.map_config['map_params']['min_cell_width'])

        # generate upper and lower body node indicies
        self.lower_body_indicies, self.grid_length, self.lower_body_grid_height = generate_node_indicies(self.query_cell_width, self.planar_spread, self.z_lower_lim, self.z_lower_body_lim)

        self.upper_body_indicies, self.grid_length, self.upper_body_grid_height = generate_node_indicies(self.query_cell_width, self.planar_spread, self.z_lower_body_lim, self.z_upper_lim)

        # update query points
        self.lower_body_query_points = np.concatenate((np.tile(self.query_height, (len(self.lower_body_indicies), 1)), self.lower_body_indicies), axis=1)

        self.upper_body_query_points = np.concatenate((np.tile(self.query_height, (len(self.upper_body_indicies), 1)), self.upper_body_indicies), axis=1)

    def update_occupancy_grid(self, map : wave.Map):
        """
        Update occupancy grid with new wavemap.
        """

        # query wavemap for upper and lower body
        # query_log_odds = map.get_cell_values(self.query_points)

        upper_body_log_odds = map.get_cell_values(self.upper_body_query_points)

        lower_body_log_odds = map.get_cell_values(self.lower_body_query_points)

        # convert to probabilities for thresholding
        # query_prob = np.exp(query_log_odds) / (1 + np.exp(query_log_odds))

        upper_body_prob = np.exp(upper_body_log_odds) / (1 + np.exp(upper_body_log_odds))

        lower_body_prob = np.exp(lower_body_log_odds) / (1 + np.exp(lower_body_log_odds))

        # reshape to occupancy values to collapse along z-axis
        # occupancy_values_raw = query_prob.reshape((self.grid_height, self.grid_length, self.grid_length)).T

        upper_body_occupancy_values_raw = upper_body_prob.reshape((self.upper_body_grid_height, self.grid_length, self.grid_length)).T

        lower_body_occupancy_values_raw = lower_body_prob.reshape((self.lower_body_grid_height, self.grid_length, self.grid_length)).T

        # update occupancy grids
        self.upper_body_occupancy_grid = np.any(upper_body_occupancy_values_raw > self.map_config['grid_params']['probability_thresh'], axis=2)

        self.lower_body_occupancy_grid = np.any(lower_body_occupancy_values_raw > self.map_config['grid_params']['probability_thresh'], axis=2)

        # update traversability grid
        self.traversability_grid = generate_traversability_grid(self.upper_body_occupancy_grid, self.lower_body_occupancy_grid, self.physical_body_rad_cell_width)

        # flip the grids to match the coordinate frame
        self.traversability_grid = self.traversability_grid[:, ::-1]
        self.upper_body_occupancy_grid = self.upper_body_occupancy_grid[:, ::-1]

        # Convert the grid to a format suitable for OpenCV
        grid_image = (self.upper_body_occupancy_grid * 255).astype(np.uint8)

        # Write the frame to the video
        self.video_writer.write(grid_image)

    def __del__(self):
        # Release the video writer when the class is destroyed
        self.video_writer.release()

def generate_traversability_grid(upper_body_occupancy : np.array, lower_body_occupancy : np.array, physical_body_rad : float) -> np.array:
    """
    Given physical occupancy representations for both the upper and lower body, find the traversability grid byfinding the nearest distance from every unoccupied cell in the upper body to an occupied cell in the lower body, and thresholding to physical traversability

    Parameters:
    -----------
        upper_body_occupancy : np.array
            The physical occupancy grid for the upper body
        lower_body_occupancy : np.array
            The physical occupancy grid for the lower body

    Returns:
    --------
        traversability_grid : np.array
            The traversability grid corresponding to the same environment
    """

    traversability_grid = copy.deepcopy(upper_body_occupancy)

    # find the indices of the occupied cells in the lower body
    lower_body_occupied_indices = np.argwhere(lower_body_occupancy)
    
    # for every occupied cell in the lower body, set the surrounding region to be occupied within the physical body radius
    for lower_body_occupied_index in lower_body_occupied_indices:
        lower_body_occupied_row, lower_body_occupied_col = lower_body_occupied_index

        # find index limits within the physical body radius
        lower_body_occupied_row_lower_lim = max(0, lower_body_occupied_row - physical_body_rad)
        lower_body_occupied_row_upper_lim = min(lower_body_occupied_row + physical_body_rad, upper_body_occupancy.shape[0])

        lower_body_occupied_col_lower_lim = max(0, lower_body_occupied_col - physical_body_rad)
        lower_body_occupied_col_upper_lim = min(lower_body_occupied_col + physical_body_rad, upper_body_occupancy.shape[1])

        # set the region to occupied in the upper body occupancy grid
        traversability_grid[lower_body_occupied_row_lower_lim:lower_body_occupied_row_upper_lim, lower_body_occupied_col_lower_lim:lower_body_occupied_col_upper_lim] = 1

    return traversability_grid
