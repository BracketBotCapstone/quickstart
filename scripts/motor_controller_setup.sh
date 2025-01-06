#!/bin/bash

# Exit on error
set -e

echo -e "\n\033[1;33mMotor Controller Setup\033[0m"
echo -e "\nThis script will run two calibration steps:"
echo "1. ODrive motor calibration - requires robot to be on a stand with wheels free to spin"
echo "2. Motor direction calibration - requires robot to be on the ground with some open space"

# First calibration - ODrive
echo -e "\n\033[1;36mStep 1: ODrive Motor Calibration\033[0m"
echo "IMPORTANT: Place the robot on a stand or on its side so the wheels can move freely!"
while true; do
    read -p "Are the wheels lifted off the ground? (yes/no) " yn
    case $yn in
        [Yy]* ) 
            echo "Running ODrive calibration..."
            python3 calibrate_odrive.py
            break;;
        [Nn]* ) 
            echo "Please place the robot on a stand and try again.";;
        * ) 
            echo "Please answer yes or no.";;
    esac
done

# Second calibration - Motor direction
echo -e "\n\033[1;36mStep 2: Motor Direction Calibration\033[0m"
echo "IMPORTANT: Place the robot on the ground with some open space around it!"
while true; do
    read -p "Is the robot on the ground with space to move? (yes/no) " yn
    case $yn in
        [Yy]* ) 
            echo "Running motor direction calibration..."
            python3 calibrate_motor_direction.py
            break;;
        [Nn]* ) 
            echo "Please place the robot on the ground with space to move and try again.";;
        * ) 
            echo "Please answer yes or no.";;
    esac
done

echo -e "\n\033[1;32mMotor controller setup complete!\033[0m"