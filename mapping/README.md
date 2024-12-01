# BracketBot Mapping Version 1.0

This is the mapping node for the BracketBot project. This node is responsible for creating a 3D map of the robot environment using a Depth Camera (currently a RealSense D435i) and proprioceptive data (IMU, wheel encoders).

The mapping node uses the [Wavemap](https://ethz-asl.github.io/wavemap/index.html#) library from the [ASL](https://asl.ethz.ch/) at ETH Zürich. Wavemap performs volumetric mapping by using Haar wavelets to represent occupancy probabilities in a hierarchical manner ([paper](https://www.roboticsproceedings.org/rss19/p065.pdf)).

## Current Capabilities
At every instance of an ``EXTENDED_POSE_W_BIAS`` message from the localization node, the mapping node calls ``DepthWavemapManager.integrate_depth_frame()`` to retrieve the corresponding depth frame and update the map, and then calls ``DynamicOccupancyGrid.updateOccupancyGrid()`` to update the occupancy grid representation. The relevant objects here are ``DepthWavemapManager.local_submap`` and ``DynamicOccupancyGrid.occupany_grid``. The current occupany grid implementation is currently a 20mx20m static grid initialized to the origin at startup.

In short, turn your robot on, make sure that the mapping node is included in the ``dataflow.yml`` file that you're running, give the ``StaticInitializer`` a few seconds to initialize the "world" pose, and then start moving the robot around. The map will build incrementally.

## Visualization of the Map

We currently don't have a way to visualize the map or the occupancy grid in real-time, but when you stop your dora instance (whether with a ``ctrl+c`` or a ``dora stop``), the node will save it's timestamped map to the ``mapping/maps`` directory.

Then, you can visualize the complete map in ROS by following the instructions [here](https://ethz-asl.github.io/wavemap/pages/installation/ros1.html#installation-ros1-docker).
Specifically, you can just run the following command on your local x86 machine, otherwise follow the instructions on the link for a native installation.
1. Run:
```
docker build --tag=wavemap_ros1 --pull - <<< $(curl -s https://raw.githubusercontent.com/ethz-asl/wavemap/main/tooling/docker/ros1/incremental.Dockerfile)
```
2. Download the map you want to visualize from the RPi into your local ``/home/$USER/data`` directory.
3. After installation, download/copy and run the [``run_in_docker.sh``](https://github.com/ethz-asl/wavemap/blob/d0d64d8cc5b3b6193a5af7d3496cfbe8d44a5303/tooling/scripts/run_in_docker.sh)
4. Once the container is running, navigate to the ``/home/$USER/data`` directory and run ``ls`` to see the map you downloaded.
5. Run ``roscore & rviz`` to start the ROS core and Rviz.
6. Load wavemap’s rviz plugin by clicking: ``Add`` → ``By display type`` → ``wavemap_rviz_plugin`` → ``WavemapMap``
7. In the plugin settings, under Source select File
8. Load the map you just downloaded by clicking: Loaded map → Choose file

## Current Notes (11/15/24):
- Currently, the extrinsics in the ``nodes/mapping/config/realsense_phys_config.yaml`` file are for the old, upright RealSense mount. Additionally, since the cables we're currently using are USB2, the resolution specification in ``nodes/mapping/config/default_435i.json`` is set to 640x480.