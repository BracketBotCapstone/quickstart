#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Update package lists
echo "Updating package lists..."
sudo apt update

# Install and enable SSH service
echo "Installing and enabling SSH service..."
sudo apt install -y ssh
sudo systemctl enable ssh
sudo systemctl start ssh

# Check SSH service status
echo "Checking SSH service status..."
sudo SYSTEMD_PAGER='' systemctl status ssh

# Install Atuin
echo "Installing Atuin..."
curl --proto '=https' --tlsv1.2 -LsSf https://setup.atuin.sh | sh

# Configure Atuin to disable up arrow key binding
echo "Configuring Atuin to disable up arrow key binding..."

# Remove existing Atuin init lines if any
sed -i '/eval "$(atuin init/d' "$HOME/.bashrc"

# Add Atuin init with --disable-up-arrow option
echo 'eval "$(atuin init bash --disable-up-arrow)"' >> "$HOME/.bashrc"






# Install Python development packages
echo "Installing Python development packages..."
sudo apt install -y python3-dev python3-pip python3-venv tmux

# Create a Python virtual environment in the home directory
echo "Creating a Python virtual environment..."
cd ~
python3 -m venv .venv

# Activate the virtual environment
echo "Activating the virtual environment..."
source ~/.venv/bin/activate

# Make the virtual environment activate automatically in new terminals
echo "Configuring automatic virtual environment activation..."
if ! grep -Fxq 'source ~/.venv/bin/activate' ~/.bashrc; then
    echo 'source ~/.venv/bin/activate' >> ~/.bashrc
fi

# Install RPi.GPIO and lgpio libraries inside the virtual environment
echo "Installing RPi.GPIO and lgpio libraries..."
pip install RPi.GPIO rpi-lgpio

# Install Adafruit MPU6050 library
echo "Installing Adafruit MPU6050 library..."
pip install adafruit-circuitpython-mpu6050

# Install mosquitto broker and clients
echo "Installing mosquitto broker and clients..."
sudo apt install -y mosquitto mosquitto-clients ufw
sudo systemctl enable mosquitto
sudo systemctl start mosquitto

# Configure mosquitto ports
echo "Configuring mosquitto ports..."
sudo bash -c 'cat > /etc/mosquitto/conf.d/default.conf << EOL
listener 1883 127.0.0.1
protocol mqtt

listener 9001
protocol websockets
allow_anonymous true
EOL'

# Restart mosquitto to apply changes
sudo ufw allow 9001
sudo systemctl restart mosquitto

# Install paho-mqtt Python package
echo "Installing paho-mqtt..."
pip install paho-mqtt

# Install ODrive package
echo "Installing ODrive package..."
pip install odrive==0.5.1.post0

# Patch the ODrive package baud rate
echo "Patching the ODrive package baud rate..."
SED_PATH=$(python -c "import fibre; import os; print(os.path.join(os.path.dirname(fibre.__file__), 'serial_transport.py'))")
sed -i 's/DEFAULT_BAUDRATE = 115200/DEFAULT_BAUDRATE = 460800/' "$SED_PATH"

# Run ODrive udev setup
echo "Running ODrive udev setup..."
ODRIVE_TOOL_PATH=$(which odrivetool)
sudo "$ODRIVE_TOOL_PATH" udev-setup

# Add current user to dialout and audio groups
echo "Adding current user to 'dialout' and 'audio' groups..."
sudo usermod -a -G dialout,audio $USER

# Configure boot settings
echo "Configuring boot settings..."
sudo sh -c 'printf "\ndisable_poe_fan=1\nenable_uart=1\ndtoverlay=uart1-pi5\ndtparam=i2c_arm=on\ndtoverlay=i2c1\n" >> /boot/firmware/config.txt'

# Install git
# echo "Installing git..."
# sudo apt install -y git

# # Clone the BracketBot quickstart repository
# echo "Cloning the BracketBot quickstart repository..."
# cd ~
# git clone https://github.com/BracketBotCapstone/quickstart
# cd quickstart

# Install required Python packages
echo "Installing required Python packages..."
pip install numpy sympy control matplotlib pyserial libtmux

# Setup WiFi Access Point
echo -e "\nWould you like to set up a WiFi access point? (y/n)"
read -r setup_ap
if [[ "$setup_ap" =~ ^[Yy]$ ]]; then
    echo "Setting up WiFi access point..."
    bash "$HOME/quickstart/setup/setup_accesspoint.sh"
else
    echo "Skipping WiFi access point setup..."
fi


echo -e "\e[31mSetup complete! You NEED TO REBOOT BEFORE RUNNING THE ODRIVE CALIBRATION\e[0m"
echo -e "\e[31mRebooting in:\e[0m"
for i in {20..1}; do
    echo -e "\e[31m$i...\e[0m"
    sleep 1
done
sudo reboot
