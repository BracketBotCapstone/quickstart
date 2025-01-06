# BracketBot Localization Version 1.0

This is the localization node for the BracketBot project. This node is responsible for localizing the robot using purely proprioceptive data (IMU, wheel encoders). It uses the [navlie](https://github.com/decargroup/navlie) library for estimation, and the [pymlg](https://github.com/decargroup/pymlg) for state implementation (both from the [DECAR Group](https://github.com/decargroup) at McGill University).

## Current Capabilities
Turn the robot on, leave it untouched for 2 seconds (the static initialization period is 1 second, but it takes a moment for the data to start streaming), and then start moving. The localization node will ingest encoder readings and IMU data to estimate the robot's pose in the world frame (initialized to the gravity-aligned frame of the turning center at startup).

## Current Notes (11/15/24):
Currently, there's no associated timestamp with the encoder measurements (they're just streamed as they come in), so the localization node technically uses the latest IMU message as the corresponding timestamp. Both sources are streamed fast enough that this doesn't currently cause much aliasing. All relevant parameters are in ``/nodes/localization/config``.