import json
import time

import numpy as np
import pyrealsense2 as rs


class RealSenseManager:
    """
    Management class for a 435i RealSense Camera, using pyrealsense2.
    """

    def __init__(self, json_path: str):
        """
        Initialize the RealSenseManager with a JSON configuration file.

        :param json_path: Path to the JSON configuration file.
        """
        # Load JSON parameters for configuration
        self.jsonObj = json.load(open(json_path))
        self.json_string = str(self.jsonObj).replace("'", '\"')

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()

        # Placeholder for RealSense configuration
        self.cfg = None

    def start_pipeline(self):
        """
        Start the RealSense pipeline with the configuration specified in the JSON file.
        Handles enabling advanced mode and configuring streams.
        """
        # Create a configuration object
        config = rs.config()
        cfg = self.pipeline.start(config)

        # Get the device and enable advanced mode
        dev = cfg.get_device()
        advnc_mode = rs.rs400_advanced_mode(dev)
        while not advnc_mode.is_enabled():
            print("Trying to enable advanced mode...")
            advnc_mode.toggle_advanced_mode(True)
            # Device will disconnect and re-connect
            print("Sleeping for 5 seconds...")
            time.sleep(5)
            # Re-initialize the device
            dev = cfg.get_device()
            advnc_mode = rs.rs400_advanced_mode(dev)
            print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

        # Load the JSON configuration into advanced mode
        advnc_mode.load_json(self.json_string)

        # Restart the pipeline after loading into advanced mode
        self.pipeline.stop()
        print("W: ", int(self.jsonObj['stream-width']))
        print("H: ", int(self.jsonObj['stream-height']))
        print("FPS: ", int(self.jsonObj['stream-fps']))
        config.enable_stream(rs.stream.depth, int(self.jsonObj['stream-width']), int(self.jsonObj['stream-height']), rs.format.z16, int(self.jsonObj['stream-fps']))
        config.enable_stream(rs.stream.color, int(self.jsonObj['stream-width']), int(self.jsonObj['stream-height']), rs.format.bgr8, int(self.jsonObj['stream-fps']))
        self.cfg = self.pipeline.start(config)

        # Define colorizer and hole-filling filter
        self.colorizer = rs.colorizer()
        self.hole_filling = rs.hole_filling_filter(1)

        # Retrieve scaling units
        self.depth_scaling = int(self.jsonObj['param-depthunits'])

    def stop_pipeline(self):
        """
        Stop the RealSense pipeline.
        """
        self.pipeline.stop()

    def get_intrinsics(self):
        """
        Get the intrinsics of the depth stream.

        :return: Intrinsics of the depth stream.
        """
        return self.cfg.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    
    def get_depth_frame(self):
        """
        Get the current depth frame.

        :return: Depth frame.
        """
        return self.pipeline.wait_for_frames().get_depth_frame()
    
    def get_filtered_depth_frame(self):
        """
        Get the filtered depth frame with hole-filling applied.

        :return: Filtered depth frame.
        """
        d_k = self.pipeline.wait_for_frames().get_depth_frame()

        # Fill holes in the depth frame
        d_k_filled = self.hole_filling.process(d_k)

        return d_k_filled
    
    def get_metric_depth_frame(self):
        """
        Get the metric depth frame by applying scaling to the filtered depth frame.

        :return: Scaled depth frame in metric units.
        """
        d_k_filled = self.get_filtered_depth_frame()

        # Convert to numpy array and apply scaling factor
        depth_np = np.asanyarray(d_k_filled.get_data())
        depth_np_scaled = depth_np / self.depth_scaling

        return depth_np_scaled
    
    def get_color_frame(self):
        """
        Get the current color frame.

        :return: Color frame.
        """
        return self.pipeline.wait_for_frames().get_color_frame()
    
    def get_depth_and_color_frame(self):
        """
        Get both depth and color frames.

        :return: Tuple containing depth and color frames.
        """
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        return depth_frame, color_frame
    
    def get_colorized_depth_frame(self):
        """
        Get the colorized depth frame.

        :return: Colorized depth frame.
        """
        return self.colorizer.colorize(self.get_filtered_depth_frame())
    
if __name__ == "__main__":
    # Start the RealSense pipeline
    json_path = "nodes/mapping/config/default_435i.json"
    rs_manager = RealSenseManager(json_path)

    rs_manager.start_pipeline()

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            start = time.time()
            frames = rs_manager.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            end = time.time()
            print("Time: ", end - start)
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Initialize colorizer class
            colorizer = rs.colorizer()
            # Convert images to numpy arrays, using colorizer to generate appropriate colors
            depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Stack both images horizontally
            print("Depth shape: ", depth_image.shape)
            print("Color shape: ", color_image.shape)
            # images = np.hstack((color_image, depth_image))

            # # Show images
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense', images)
            # key = cv2.waitKey(1)
            # # Press esc or 'q' to close the image window
            # if key & 0xFF == ord('q') or key == 27:
            #     cv2.destroyAllWindows()
            #     break
    except Exception as e:
        print("Error: ", e)
        rs_manager.stop_pipeline()
    finally:
        # Stop streaming
        rs_manager.stop_pipeline()
