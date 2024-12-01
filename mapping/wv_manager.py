import pywavemap as wave
import os
import pyrealsense2 as rs
import numpy as np
import cv2

from realsense_python.realsense_manager import RealSenseManager
from mapping.grid_helpers import DynamicOccupancyGrid

import yaml

from pymlg.numpy import SO3, SE3

import time

class DepthWavemapManager:
    def __init__(self, map_config_loc : str):

        print("Initializing DepthWavemapManager!")

        # retrieve configuration yaml
        with open(map_config_loc, 'r') as stream:
            self.map_config = yaml.safe_load(stream)

        # retrieve realsense configuration loc
        realsense_config_loc = self.map_config['config_params']['realsense_config_loc']

        ## add /home/$USER to the beginning of the path
        realsense_config_loc = f"/home/{os.getenv('USER')}/{realsense_config_loc}"

        # retrieve realsense configuration
        with open(realsense_config_loc, 'r') as stream:
            self.realsense_config = yaml.safe_load(stream)

        # instantiate occupancy grid
        self.occupancy_grid = DynamicOccupancyGrid(map_config_loc)

        json_file_loc = self.realsense_config['json_file_loc']
        # add /home/$USER to the beginning of the path
        json_file_loc = f"/home/{os.getenv('USER')}/{json_file_loc}"

        # start RealSense retrieval pipeline
        self.rs_manager = RealSenseManager(json_file_loc)

        # retrieve extrinsics
        r_bc = np.array(self.realsense_config['extrinsics']['r_bc']).reshape(1, 3)
        phi_bc = np.array(self.realsense_config['extrinsics']['phi_bc']).reshape(1, 3)

        C_bc = SO3.Exp(phi_bc)

        # form transformation matrix to apply to pose transformation
        self.T_bc = SE3.from_components(C_bc, r_bc)

        # update rate
        self.update_rate = self.map_config['config_params']['update_rate']

        # track current time
        self.t_k = None

        # Create a map
        self.local_submap = wave.Map.create({
            "type": "hashed_wavelet_octree",
            "min_cell_width": {
                "meters": self.map_config['map_params']['min_cell_width']
            },
            "remove_blocks_beyond_distance": {
                "meters": self.map_config['map_params']['block_removal_distance']
            }
        })

        # Create a measurement integration pipeline
        self.pipeline = wave.Pipeline(self.local_submap)
        # Add map operations
        self.pipeline.add_operation({
            "type": "threshold_map",
            "once_every": {
                "seconds": self.map_config['map_params']['threshold_delta']
            }
        })
        self.pipeline.add_operation({
            "type": "prune_map",
            "once_every": {
                "seconds": self.map_config['map_params']['pruning_delta']
            }
        })

    def start_pipeline(self):

        # start RealSense pipeline
        self.rs_manager.start_pipeline()

        # retrieve intrinsics
        self.realsense_intrinsics = self.rs_manager.get_intrinsics()

        # Add a measurement integrator
        self.pipeline.add_integrator(
            "my_integrator", {
                "projection_model": {
                    "type": "pinhole_camera_projector",
                    "width": self.realsense_intrinsics.width,
                    "height": self.realsense_intrinsics.height,
                    "fx": self.realsense_intrinsics.fx,
                    "fy": self.realsense_intrinsics.fy,
                    "cx": self.realsense_intrinsics.ppx,
                    "cy": self.realsense_intrinsics.ppy
                },
                "measurement_model": {
                    "type": "continuous_ray",
                    "range_sigma": {
                        "meters": self.map_config['map_params']['range_sigma']
                    },
                    "scaling_free": self.map_config['map_params']['scaling_free'],
                    "scaling_occupied": self.map_config['map_params']['scaling_occupied']
                },
                "integration_method": {
                    "type": "hashed_wavelet_integrator",
                    "min_range": {
                        "meters": self.map_config['map_params']['min_range']
                    },
                    "max_range": {
                        "meters": self.map_config['map_params']['max_range']
                    },
                    "max_update_resolution": {
                        "meters": self.map_config['map_params']['max_update_resolution']
                    },
                },
            })
        
    def integrate_depth_image(self, t_k : float, T_ab_k : np.ndarray):
        """
        Given a pose estimate from the localization node, integrate the current depth image into the local submap

        Parameters
        ----------
        T_ab : np.ndarray
            SE(3) pose estimate of the body in the inertial frame at timestep k
        """
        
        # first, check if we need to update the map
        # if self.t_k is not None:
        #     if t_k - self.t_k < 1 / self.update_rate:
        #         return

        # update to current time and update map
        self.t_k = t_k

        # retrieve depth image
        depth_np = self.rs_manager.get_metric_depth_frame()

        image = wave.Image(depth_np.T)

        # given T_ab, compute T_ac from extrinsics, and integrate image
        T_ac_k = T_ab_k @ self.T_bc

        T_ac_k = wave.Pose(T_ac_k)

        self.pipeline.run_pipeline(["my_integrator"], wave.PosedImage(T_ac_k, image))
        
        # after integrating, update occupancy grid
        self.occupancy_grid.update_occupancy_grid(self.local_submap)

    def get_upper_body_occupancy_grid(self):
        return self.occupancy_grid.upper_body_occupancy_grid
    
    def get_lower_body_occupancy_grid(self):
        return self.occupancy_grid.lower_body_occupancy_grid
    
    def get_traversability_grid(self):
        return self.occupancy_grid.traversability_grid

    def get_flattened_upper_body_occupancy_grid(self):
        return self.occupancy_grid.upper_body_occupancy_grid.flatten()
    
    def get_flattened_lower_body_occupancy_grid(self):
        return self.occupancy_grid.lower_body_occupancy_grid.flatten()
    
    def get_flattened_traversability_grid(self):
        return self.occupancy_grid.traversability_grid.flatten()

    def threshold_map(self):

        # Threshold map to restrict log-odds to specific-range, if specified
        self.local_submap.threshold()

    def save_map(self):

        # Prune and save map
        self.local_submap.prune()

        # get current map name, in time
        map_name = "map_" + str(int(time.time())) + ".wvmp"

        # Save the map
        output_map_path = self.map_config['map_params']['output_map_path'] + map_name
        # add /home/$USER to the beginning of the path
        output_map_path = f"/home/{os.getenv('USER')}/{output_map_path}"

        self.local_submap.store(output_map_path)

        # clean up pipeline and map
        del self.pipeline, self.local_submap
