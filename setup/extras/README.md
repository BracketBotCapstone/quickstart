# Optional Setup Scripts

This directory contains setup scripts for optional hardware or software components that are not part of the core system setup performed by the main `setup_os.sh` script.

These scripts should typically be run manually after the main setup is complete, if the corresponding hardware/software is needed.

## Scripts:

*   `setup_realsense.sh`: Sets up Intel RealSense cameras.
*   `setup_hardware_pwm.sh`: Configures hardware PWM using the provided `.dts` file.
*   `pwm-pi5.dts`: Device Tree Source file needed for hardware PWM setup.
*   `create_custom_boot_service.sh`: A template script to help create and enable a systemd service for running a custom script (Python, Bash, etc.) on boot. Copy and edit this template, then run your edited version. 