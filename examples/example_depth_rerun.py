# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Import the package utility and ensure required packages are installed
from lib.package_utils import ensure_package

# Check and install required packages
ensure_package("numpy")
# For depth calculation we need the contrib modules (ximgproc)
ensure_package("cv2", "opencv-contrib-python")
ensure_package("matplotlib")
ensure_package("rerun")

import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
from lib.camera import StereoCamera
import rerun as rr

#############################################
# ADJUSTABLE PARAMETERS - Modify as needed
#############################################

# Image scaling (smaller = faster but less detailed)
SCALE = 0.375  # 1.0 = full resolution, 0.5 = half, etc.

# Rectification strength (1.0 = full rectification, 0.0 = no rectification)
RECTIFICATION_STRENGTH = 0.125  # Reduce this value to make rectification less aggressive

# Camera parameters
FOCAL_LENGTH_MM = 2.53  # mm (from datasheet)
PIXEL_SIZE_UM = 3.0     # μm (from datasheet)
BASELINE_MM = 63.0      # mm (from BracketBot hardware) (actual measured camera-to-camera distance)

# Stereo matching parameters
MIN_DISPARITY = -32      # Minimum disparity (negative allows for objects crossing from left to right)
NUM_DISPARITIES = 160    # Range of disparity values (must be divisible by 16)
BLOCK_SIZE = 9          # Size of matching block (odd number: 5, 7, 9, 11...)
UNIQUENESS_RATIO = 5    # Lower values produce more matches but more noise
P1 = 8*3*BLOCK_SIZE**2  # Controls disparity smoothness (first parameter)
P2 = 32*3*BLOCK_SIZE**2 # Controls disparity smoothness (second parameter)
SPECKLE_WINDOW = 200    # Maximum area of speckles to remove
SPECKLE_RANGE = 2       # Maximum disparity variation in speckle region

# Post-processing options
ENABLE_WLS_FILTER = True  # Whether to use weighted least squares filter
WLS_LAMBDA = 8000         # Controls smoothing strength of WLS filter
WLS_SIGMA = 1.2           # Controls edge preservation of WLS filter

# Visualization options
USE_COLORMAP = cv2.COLORMAP_INFERNO  # COLORMAP_JET, COLORMAP_INFERNO, COLORMAP_VIRIDIS, etc.
SAVE_DEBUG_IMAGES = True             # Save additional debug visualizations

# Preprocessing options
USE_HISTOGRAM_EQ = True     # Apply histogram equalization
USE_CLAHE = True            # Apply Contrast Limited Adaptive Histogram Equalization
USE_GAUSSIAN_BLUR = True    # Apply Gaussian blur
BLUR_KERNEL_SIZE = 3        # Size of Gaussian blur kernel (odd number: 3, 5, 7...)

# Initialize rerun for visualization
import rerun.blueprint as rrb

# Create a blueprint for the visualization layout
DEFAULT_BLUEPRINT = rrb.Horizontal(
    rrb.Vertical(
        rrb.Spatial3DView(name="3D", origin="world"),
        rrb.TextDocumentView(name="Description", origin="/description"),
        row_shares=[7, 3],
    ),
    rrb.Vertical(
        rrb.Spatial2DView(
            name="RGB & Depth",
            origin="world/camera/image",
            contents=["world/camera/image/left", "world/camera/image/depth"],
            overrides={"world/camera/image/depth": [rr.components.Opacity(0.5)]},
        ),
        rrb.Tabs(
            rrb.Spatial2DView(name="Left", origin="world/camera/image", contents="world/camera/image/left"),
            rrb.Spatial2DView(name="Right", origin="world/camera/image", contents="world/camera/image/right"),
            rrb.Spatial2DView(name="Depth", origin="world/camera/image", contents="world/camera/image/depth"),
            rrb.Spatial2DView(name="Disparity", origin="world/camera/image", contents="world/camera/image/disparity"),
        ),
        name="2D",
        row_shares=[3, 3, 2],
    ),
    column_shares=[2, 1],
)

DESCRIPTION = """
# Stereo Depth Processing
Visualizes stereo depth processing from a stereo camera with RGB and Depth channels.

Parameters:
- Scale: {}
- Rectification strength: {}
- Disparity range: {} to {}
- Block size: {}
- WLS filter: {}
""".format(
    SCALE, 
    RECTIFICATION_STRENGTH, 
    MIN_DISPARITY, 
    MIN_DISPARITY + NUM_DISPARITIES,
    BLOCK_SIZE,
    'Enabled' if ENABLE_WLS_FILTER else 'Disabled'
)

rr.init("stereo_camera_stream", default_blueprint=DEFAULT_BLUEPRINT)

# Connect to the Rerun server at your specific IP address
rr.connect_tcp("192.168.2.24:9876")

print(f"Starting stereo depth processing with:")
print(f"- Scale: {SCALE}")
print(f"- Rectification strength: {RECTIFICATION_STRENGTH}")
print(f"- Disparity range: {MIN_DISPARITY} to {MIN_DISPARITY + NUM_DISPARITIES}")
print(f"- Block size: {BLOCK_SIZE}")
print(f"- WLS filter: {'Enabled' if ENABLE_WLS_FILTER else 'Disabled'}")

# Load calibration data
print("Loading calibration data...")
calib_data = np.load('../lib/stereo_calibration.npz')
left_map1 = calib_data['left_maps_x']
left_map2 = calib_data['left_maps_y']
right_map1 = calib_data['right_maps_x']
right_map2 = calib_data['right_maps_y']

# Scale calibration maps if needed
if SCALE != 1.0:
    h, w = left_map1.shape
    new_h, new_w = int(h * SCALE), int(w * SCALE)
    # Scale maps using proper interpolation
    left_map1 = cv2.resize(left_map1, (new_w, new_h), interpolation=cv2.INTER_LINEAR) * SCALE
    left_map2 = cv2.resize(left_map2, (new_w, new_h), interpolation=cv2.INTER_LINEAR) * SCALE
    right_map1 = cv2.resize(right_map1, (new_w, new_h), interpolation=cv2.INTER_LINEAR) * SCALE
    right_map2 = cv2.resize(right_map2, (new_w, new_h), interpolation=cv2.INTER_LINEAR) * SCALE

# Initialize camera
camera = StereoCamera()

def preprocess_image(img):
    """Apply pre-processing to enhance image for stereo matching."""
    if SCALE != 1.0:
        h, w = img.shape[:2]
        new_h, new_w = int(h * SCALE), int(w * SCALE)
        img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    
    # Convert to grayscale
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    
    # Apply Gaussian blur to reduce noise
    if USE_GAUSSIAN_BLUR:
        gray = cv2.GaussianBlur(gray, (BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0)
    
    # Apply histogram equalization
    if USE_HISTOGRAM_EQ and not USE_CLAHE:
        gray = cv2.equalizeHist(gray)
    
    # Apply CLAHE (better than simple histogram equalization)
    if USE_CLAHE:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
    
    return gray

def log_to_rerun(left, right, left_rect, right_rect, disparity_raw, disparity_processed, frame_count):
    """Log images and disparity maps to Rerun visualization."""
    # Log the description
    if frame_count == 0:
        rr.log("description", rr.TextDocument(DESCRIPTION, media_type=rr.MediaType.MARKDOWN), static=True)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)
    
    # Convert disparity to depth
    # Using a simple conversion formula: depth = (baseline * focal_length) / disparity
    if disparity_processed is not None:
        # Debug info about disparity
        print(f"Disparity stats - Min: {disparity_processed.min():.2f}, Max: {disparity_processed.max():.2f}, Mean: {np.mean(disparity_processed):.2f}")
        
        # Initialize depth map
        depth_map = np.zeros_like(disparity_processed, dtype=np.float32)
        
        # Use the calibrated baseline from the BracketBot configuration (63mm)
        CALIBRATED_BASELINE_MM = 63.0
        
        # Convert disparity to depth using the stereo formula
        # Focal length in pixels = focal_length_mm / pixel_size_mm * image_width
        focal_length_pixels = FOCAL_LENGTH_MM / (PIXEL_SIZE_UM / 1000) * left_rect.shape[1]
        
        # Since our disparity values are negative, we'll use absolute values
        # The more negative the disparity, the closer the object
        abs_disparity = np.abs(disparity_processed)
        
        # Only consider pixels with disparity greater than a threshold
        valid_mask = abs_disparity > 1.0
        
        # Calculate depth using the formula: depth = (baseline * focal_length) / disparity
        # Add a small offset to disparity to avoid division by zero and very large depths
        offset_disparity = abs_disparity + 0.1
        depth_map[valid_mask] = (CALIBRATED_BASELINE_MM * focal_length_pixels) / offset_disparity[valid_mask]
        
        # Calculate depth using the 63mm baseline from BracketBot
        # Use a simple formula: depth = (baseline * focal_length) / disparity
        # Add a small offset to avoid division by zero
        offset_disparity = abs_disparity[valid_mask] + 0.1
        depth_map[valid_mask] = (BASELINE_MM * focal_length_pixels) / offset_disparity
        
        # Don't clip the depth range, let it be the actual calculated values
        # Just ensure no negative values
        depth_map = np.maximum(depth_map, 0)
        
        # Debug info about depth
        valid_depths = depth_map[depth_map > 0]
        if len(valid_depths) > 0:
            print(f"Depth stats - Min: {valid_depths.min():.2f}, Max: {valid_depths.max():.2f}, Mean: {np.mean(valid_depths):.2f}, Valid pixels: {len(valid_depths)}")
        else:
            print("No valid depth values calculated!")
    else:
        depth_map = None
    
    # Log camera parameters
    if left_rect is not None:
        h, w = left_rect.shape[:2]
        focal_length_pixels = FOCAL_LENGTH_MM / (PIXEL_SIZE_UM / 1000) * w
        rr.log(
            "world/camera/image",
            rr.Pinhole(
                resolution=[w, h],
                focal_length=focal_length_pixels,
                principal_point=[w/2, h/2],
            ),
        )
    
    # Comment out original image logs for better performance
    if left is not None:
        rr.log("world/camera/image/left", rr.Image(left, color_model="BGR").compress(jpeg_quality=95))
    # if right is not None:
    #     rr.log("world/camera/image/right", rr.Image(right, color_model="BGR").compress(jpeg_quality=95))
    
    # Comment out rectified image logs for better performance
    # if left_rect is not None:
    #     # Convert to color if grayscale
    #     if len(left_rect.shape) == 2:
    #         left_rect_color = cv2.cvtColor(left_rect, cv2.COLOR_GRAY2BGR)
    #     else:
    #         left_rect_color = left_rect.copy()
    #         
    #     # Draw horizontal lines for epipolar visualization
    #     height = left_rect.shape[0]
    #     for y in range(0, height, 50):
    #         cv2.line(left_rect_color, (0, y), (left_rect.shape[1], y), (0, 255, 0), 1)
    #         
    #     rr.log("world/camera/image/left_rect", rr.Image(left_rect_color, color_model="BGR").compress(jpeg_quality=95))
    
    # Comment out disparity map logs for better performance
    # if disparity_processed is not None:
    #     # Normalize for visualization
    #     disp_processed_viz = cv2.normalize(disparity_processed, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    #     # Apply colormap
    #     disp_color = cv2.applyColorMap(disp_processed_viz, USE_COLORMAP)
    #     rr.log("world/camera/image/disparity", rr.Image(disp_color, color_model="BGR").compress(jpeg_quality=95))
    
    # Comment out depth map logs for better performance
    # if depth_map is not None:
    #     # Scale depth for visualization (similar to the RGBD example)
    #     DEPTH_IMAGE_SCALING = 1e3  # Scale to millimeters
    #     rr.log("world/camera/image/depth", rr.DepthImage(depth_map, meter=DEPTH_IMAGE_SCALING))
        
        # Create point cloud from depth
        if frame_count % 5 == 0:  # Only log point cloud every 5 frames to reduce overhead
            try:
                # Create a point cloud from the depth map
                points = []
                colors = []
                
                # Use a smaller step size for more detailed point cloud
                step = 5  # Skip fewer pixels for better visualization
                point_count = 0
                
                # Create a synthetic grid of points if we don't have enough real depth points
                # This ensures we always have something to visualize
                synthetic_mode = False
                
                # DEPTH_SCALE is a tunable constant that you can adjust
                # Smaller values will make the depth effect less pronounced
                # Larger values will make the depth effect more pronounced
                DEPTH_SCALE = 0.000002  # Adjust this value to tune the depth effect
                
                # Z_OFFSET centers the point cloud at Z=0
                # This will be calculated automatically based on the depth values
                
                # Create a grid of points spread out in X and Y at Z=0, then move in Z based on depth
                for y in range(0, depth_map.shape[0], step):
                    for x in range(0, depth_map.shape[1], step):
                        depth = depth_map[y, x]
                        if depth > 0:  # Any valid depth point
                            # Spread points out in X and Y with a large scale for visibility
                            # This creates a "billboard" of points at Z=0
                            X = (x / depth_map.shape[1] - 0.5) * 4.0  # Spread out more in X
                            Y = (y / depth_map.shape[0] - 0.5) * 4.0  # Spread out more in Y
                            
                            # Z starts at 0, then moves proportional to depth
                            # Use the tunable DEPTH_SCALE constant
                            Z = depth * DEPTH_SCALE
                            
                            points.append([X, Y, Z])
                            
                            # Get color from the left image - using original left image for better color
                            if len(left.shape) == 3:  # Color image
                                # Make sure we're within bounds of the original image
                                orig_y = min(int(y * left.shape[0] / depth_map.shape[0]), left.shape[0]-1)
                                orig_x = min(int(x * left.shape[1] / depth_map.shape[1]), left.shape[1]-1)
                                
                                # OpenCV uses BGR format, but Rerun expects RGB
                                b, g, r = left[orig_y, orig_x]
                                
                                # Make colors more vibrant
                                r = min(int(r * 1.2), 255)
                                g = min(int(g * 1.2), 255)
                                b = min(int(b * 1.2), 255)
                                
                                colors.append([r, g, b])  # RGB order for Rerun
                            else:  # Grayscale
                                orig_y = min(int(y * left.shape[0] / depth_map.shape[0]), left.shape[0]-1)
                                orig_x = min(int(x * left.shape[1] / depth_map.shape[1]), left.shape[1]-1)
                                gray = left[orig_y, orig_x]
                                colors.append([gray, gray, gray])
                            
                            point_count += 1
                
                # If we don't have enough real points, create synthetic points
                if point_count < 100:
                    print(f"Not enough valid depth points ({point_count}), using synthetic data")
                    synthetic_mode = True
                    
                    # Create a synthetic grid of points
                    grid_size = 50
                    for i in range(grid_size):
                        for j in range(grid_size):
                            # Create a more pronounced wavy surface
                            x = (i - grid_size/2) / 10.0
                            y = (j - grid_size/2) / 10.0
                            z = 5.0 + 3.0 * np.sin(x) * np.cos(y)  # Even more pronounced wavy surface
                            
                            points.append([x, y, z])
                            
                            # Use color from image if possible, otherwise use synthetic color
                            if len(left.shape) == 3:
                                # Map grid position to image coordinates
                                img_x = int((i / grid_size) * left.shape[1])
                                img_y = int((j / grid_size) * left.shape[0])
                                if 0 <= img_x < left.shape[1] and 0 <= img_y < left.shape[0]:
                                    b, g, r = left[img_y, img_x]
                                    colors.append([r, g, b])
                                else:
                                    # Synthetic color based on position
                                    r = int(255 * i / grid_size)
                                    g = int(255 * j / grid_size)
                                    b = 128
                                    colors.append([r, g, b])
                            else:
                                # Synthetic grayscale
                                gray = int(255 * (i + j) / (2 * grid_size))
                                colors.append([gray, gray, gray])
                else:
                    print(f"Generated {point_count} points from depth data")
                
                if points:  # Only log if we have points
                    points_array = np.array(points)
                    colors_array = np.array(colors)
                    
                    print(f"Logging point cloud with {len(points_array)} points")
                    print(f"Point cloud bounds: X[{points_array[:,0].min():.2f}, {points_array[:,0].max():.2f}], "
                          f"Y[{points_array[:,1].min():.2f}, {points_array[:,1].max():.2f}], "
                          f"Z[{points_array[:,2].min():.2f}, {points_array[:,2].max():.2f}]")
                    
                    # Calculate Z_OFFSET to center the point cloud at Z=0
                    Z_OFFSET = np.mean(points_array[:, 2])
                    
                    # Apply Z_OFFSET to center the point cloud at Z=0
                    centered_points = points_array.copy()
                    centered_points[:, 2] -= Z_OFFSET
                    
                    # Print the tuning values for reference
                    print(f"DEPTH_SCALE = {DEPTH_SCALE} - Adjust this value in the code to tune the depth effect")
                    print(f"Z_OFFSET = {Z_OFFSET} - Points are centered at Z=0")
                    
                    # Log the point cloud to the 3D view with larger points for better visibility
                    rr.log("world/point_cloud", rr.Points3D(
                        positions=centered_points, 
                        colors=colors_array,
                        radii=0.05  # Larger points for better visibility
                    ))
                    
                    # Log a larger coordinate frame for better visibility
                    rr.log("world/origin", rr.Transform3D(translation=[0, 0, 0]))
                    rr.log("world/axes", rr.Arrows3D(
                        vectors=[[10.0, 0, 0], [0, 10.0, 0], [0, 0, 10.0]],  # Very large arrows
                        origins=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]
                    ))
                    
                    # Just add a simple coordinate frame for reference
                    rr.log("world/origin", rr.Transform3D(translation=[0, 0, 0]))
                    rr.log("world/axes", rr.Arrows3D(
                        vectors=[[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]],
                        origins=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]
                    ))
                    
                    # Also create a mesh for better visualization
                    # This creates a surface from the point cloud
                    try:
                        # Create a simplified mesh from the point cloud for better visualization
                        # We'll create a grid of points and connect them
                        grid_size = 50
                        grid_points = np.zeros((grid_size, grid_size, 3))
                        grid_colors = np.zeros((grid_size, grid_size, 3))
                        grid_valid = np.zeros((grid_size, grid_size), dtype=bool)
                        
                        # Map points to grid
                        for i, (point, color) in enumerate(zip(points_array, colors_array)):
                            # Map 3D point to grid cell
                            grid_x = int((point[0] + 2) / 4 * (grid_size-1))  # Map X range [-2, 2] to [0, grid_size-1]
                            grid_y = int((point[1] + 2) / 4 * (grid_size-1))  # Map Y range [-2, 2] to [0, grid_size-1]
                            
                            if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
                                if not grid_valid[grid_y, grid_x] or point[2] < grid_points[grid_y, grid_x, 2]:
                                    grid_points[grid_y, grid_x] = point
                                    grid_colors[grid_y, grid_x] = color
                                    grid_valid[grid_y, grid_x] = True
                        
                        # Create triangles from valid grid points
                        vertices = []
                        colors_list = []
                        triangles = []
                        
                        # Add valid points as vertices
                        vertex_map = {}
                        for y in range(grid_size):
                            for x in range(grid_size):
                                if grid_valid[y, x]:
                                    vertex_idx = len(vertices)
                                    vertex_map[(y, x)] = vertex_idx
                                    vertices.append(grid_points[y, x])
                                    colors_list.append(grid_colors[y, x])
                        
                        # Create triangles
                        for y in range(grid_size-1):
                            for x in range(grid_size-1):
                                if (grid_valid[y, x] and grid_valid[y+1, x] and 
                                    grid_valid[y, x+1] and grid_valid[y+1, x+1]):
                                    # Create two triangles for this quad
                                    triangles.append([vertex_map[(y, x)], 
                                                    vertex_map[(y+1, x)], 
                                                    vertex_map[(y, x+1)]])
                                    triangles.append([vertex_map[(y+1, x)], 
                                                    vertex_map[(y+1, x+1)], 
                                                    vertex_map[(y, x+1)]])
                        
                        if vertices and triangles:
                            vertices_array = np.array(vertices)
                            triangles_array = np.array(triangles)
                            colors_array = np.array(colors_list)
                            
                            # Log the mesh
                            rr.log("world/depth_mesh", 
                                  rr.Mesh3D(
                                      vertices=vertices_array,
                                      colors=colors_array / 255.0,  # Normalize colors to [0,1]
                                      triangles=triangles_array
                                  ))
                    except Exception as e:
                        # Silently fail for mesh creation - it's just a visualization enhancement
                        pass
            except Exception as e:
                print(f"Error creating point cloud: {e}")

# Create stereo matcher
left_matcher = cv2.StereoSGBM_create(
    minDisparity=MIN_DISPARITY,
    numDisparities=NUM_DISPARITIES,
    blockSize=BLOCK_SIZE,
    P1=P1,
    P2=P2,
    disp12MaxDiff=1,
    uniquenessRatio=UNIQUENESS_RATIO,
    speckleWindowSize=SPECKLE_WINDOW,
    speckleRange=SPECKLE_RANGE,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_HH4  # HH4 mode for better quality
)

# Create right matcher for WLS filter
if ENABLE_WLS_FILTER:
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    
    # Create and configure WLS filter
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(WLS_LAMBDA)
    wls_filter.setSigmaColor(WLS_SIGMA)

def compute_disparity(left_rect, right_rect):
    """Compute disparity map with configured parameters."""
    # Compute left disparity
    left_disp = left_matcher.compute(left_rect, right_rect)
    
    if ENABLE_WLS_FILTER:
        # Compute right disparity for WLS filtering
        right_disp = right_matcher.compute(right_rect, left_rect)
        
        # Apply WLS filtering
        filtered_disp = wls_filter.filter(
            left_disp, left_rect, disparity_map_right=right_disp
        ).astype(np.float32) / 16.0
        
        return left_disp.astype(np.float32) / 16.0, filtered_disp
    else:
        return left_disp.astype(np.float32) / 16.0, left_disp.astype(np.float32) / 16.0

# Function to compute disparity and visualize results
def process_stereo_frame(left, right, frame_count=0):
    """Process stereo frames to produce disparity map."""
    # Apply preprocessing
    left_processed = preprocess_image(left)
    right_processed = preprocess_image(right)
    
    # Rectify images
    left_rect_full = cv2.remap(left_processed, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rect_full = cv2.remap(right_processed, right_map1, right_map2, cv2.INTER_LINEAR)
    
    # Blend rectified images with original based on rectification strength
    if RECTIFICATION_STRENGTH < 1.0:
        left_rect = cv2.addWeighted(left_rect_full, RECTIFICATION_STRENGTH, 
                                   left_processed, 1.0 - RECTIFICATION_STRENGTH, 0)
        right_rect = cv2.addWeighted(right_rect_full, RECTIFICATION_STRENGTH, 
                                    right_processed, 1.0 - RECTIFICATION_STRENGTH, 0)
    else:
        left_rect = left_rect_full
        right_rect = right_rect_full
    
    # Calculate disparity
    raw_disp, filtered_disp = compute_disparity(left_rect, right_rect)
    
    # Log to Rerun
    rr.set_time_sequence("frame", frame_count)
    log_to_rerun(left, right, left_rect, right_rect, raw_disp, filtered_disp, frame_count)
    
    # If we're using partial rectification, also log the full rectification for comparison
    if RECTIFICATION_STRENGTH < 1.0:
        rr.log("stereo/left/full_rectified", rr.Image(left_rect_full))
        rr.log("stereo/right/full_rectified", rr.Image(right_rect_full))
    
    # Create colored visualization
    normalized_disp = cv2.normalize(filtered_disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    colored_disp = cv2.applyColorMap(normalized_disp, USE_COLORMAP)
    
    # Create depth overlay
    overlay = cv2.addWeighted(
        cv2.resize(left, (colored_disp.shape[1], colored_disp.shape[0])),
        0.3, colored_disp, 0.7, 0
    )
    
    return colored_disp, normalized_disp, overlay, filtered_disp

# Main loop
print("Press Ctrl+C to stop")
try:
    frame_count = 0
    start_time = time.time()
    fps_counter = 0
    fps = 0.0  # Initialize fps variable
    
    while True:
        # Capture stereo frames
        left, right = camera.get_stereo()
        
        if left is None or right is None:
            print("Failed to capture images")
            continue
        
        # Set time for this frame
        rr.set_time_sequence("frame", frame_count)
        
        # Process frames
        colored_disp, raw_disp, overlay, disparity = process_stereo_frame(left, right, frame_count)
        
        # Display information every second
        fps_counter += 1
        if time.time() - start_time >= 1.0:
            fps = fps_counter / (time.time() - start_time)
            print(f"FPS: {fps:.1f}, Frame: {frame_count}, "
                 f"Disparity range: {disparity.min():.1f} to {disparity.max():.1f}")
            fps_counter = 0
            start_time = time.time()
        
        # Log metrics to Rerun
        rr.log("metrics/fps", rr.Scalar(fps))
        rr.log("metrics/disparity_min", rr.Scalar(disparity.min()))
        rr.log("metrics/disparity_max", rr.Scalar(disparity.max()))
        
        frame_count += 1
        
except KeyboardInterrupt:
    print("\nStopping...")
except Exception as e:
    print(f"Error: {e}")
finally:
    camera.release()
    print("Shutdown complete")
