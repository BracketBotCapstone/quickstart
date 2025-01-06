#!/bin/bash

# Shell script to setup each RPi for the BracketBot project

set -e # Exit on non-zero status

export python_version=3.11
export gcc_version=13
export clang_version=17
export ssh_port=8888

# Ask the user if they want to reboot at the end of the script
read -p "Do you want to reboot at the end of the script for all changes to take effect? (yes/no): " user_input
user_input=${user_input,,}

echo "Updating package lists..."
sudo apt-get install -y software-properties-common && sudo apt-get update # Update package lists

sudo apt-get install -y \
    apt-transport-https \
    curl sudo unzip zip libacl1-dev jq \
    bash-completion htop \
    i2c-tools cmake \
    libi2c-dev \
    libdbus-1-dev libfreetype-dev libtbb-dev libglfw3-dev \
    gcc-$gcc_version g++-$gcc_version gdb \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 \
    libxcb-render-util0 libxcb-shape0 libxcb-util1 libxcb-xinerama0 \
    libxcb-xinput0 libxcb-xkb1 libxkbcommon-x11-0 libxcb-cursor0 \
    libxrender1 libxtst6 libxi6 fontconfig \
    v4l-utils \
    vim \
    usbutils \
    portaudio19-dev \
    x11-apps \
    git \
    git-lfs

# Python setup
echo "Installing Python development packages..."
sudo apt-get install -y python3-venv python3-dev python3-pip python3-rpi.gpio python3-demjson

# Install and enable SSH service
echo "Installing and enabling SSH service..."
sudo apt install -y ssh
sudo systemctl enable ssh
sudo systemctl start ssh

# Check SSH service status
echo "Checking SSH service status..."
sudo SYSTEMD_PAGER='' systemctl status ssh

echo "Installing Atuin..."                                          # Install shell history tool
curl --proto '=https' --tlsv1.2 -LsSf https://setup.atuin.sh | sh

echo "Configuring Atuin to disable up arrow key binding..."
sed -i '/eval "$(atuin init/d' "$HOME/.bashrc"                      # Remove existing Atuin init
echo 'eval "$(atuin init bash --disable-up-arrow)"' >> "$HOME/.bashrc"  # Add new Atuin config

# MQTT
echo "Installing MQTT..."
sudo apt-get install -y mosquitto mosquitto-clients ufw
# enable mosquitto service to start on boot
sudo systemctl enable mosquitto
sudo systemctl start mosquitto

echo "Configuring mosquitto ports..."                              # Configure MQTT ports
sudo bash -c 'cat > /etc/mosquitto/conf.d/default.conf << EOL
listener 1883 127.0.0.1
protocol mqtt

listener 9001
protocol websockets
allow_anonymous true
EOL'

sudo ufw allow 9001                                                # Allow websocket port
sudo systemctl restart mosquitto                                   # Restart MQTT service

# User setup
echo "Setting up user..."
export username=$USER
sudo useradd -m $username -l -u "$(echo $uid | cut -d: -f1)" \
    && sudo usermod -u "$(echo $uid | cut -d: -f1)" $username \
    && sudo groupmod -g "$(echo $gid | cut -d: -f2)" $username \
    && sudo usermod -aG sudo,video,i2c,dialout,plugdev,audio $username \
    && echo $username ' ALL=(ALL:ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && mkdir -p /home/$username/.cache /home/$username/.ssh \
    && ssh-keyscan github.com >> /home/$username/.ssh/known_hosts \
    && sudo chown -R $username:$username /home/$username

# Install tmux
echo "Installing tmux..."
sudo apt-get install -y tmux

# Create a Python virtual environment in the home directory
echo "Creating a Python virtual environment..."
cd ~
python3.11 -m venv .venv

# Activate the virtual environment
echo "Activating the virtual environment..."
source ~/.venv/bin/activate

# Make the virtual environment activate automatically in new terminals
echo "Configuring automatic virtual environment activation..."
if ! grep -Fxq 'source ~/.venv/bin/activate' ~/.bashrc; then
    echo 'source ~/.venv/bin/activate' >> ~/.bashrc
fi

echo "Installing python packages..."
pip install -e "$HOME/quickstart"

echo "Building the config and messages..."
cd "$HOME/quickstart"
CONFIG=bot_quickstart CONFIG_MSGS=bot_quickstart_msgs make build

# Patch the ODrive package baud rate
echo "Patching the ODrive package baud rate..."
SED_PATH=$(python -c "import fibre; import os; print(os.path.join(os.path.dirname(fibre.__file__), 'serial_transport.py'))")
sed -i 's/DEFAULT_BAUDRATE = 115200/DEFAULT_BAUDRATE = 460800/' "$SED_PATH"

# Run ODrive udev setup
echo "Running ODrive udev setup..."
ODRIVE_TOOL_PATH=$(which odrivetool)
sudo "$ODRIVE_TOOL_PATH" udev-setup

# Configure boot settings
echo "Configuring boot settings..."
# sudo sed -i '7i dtparam=i2c_vc=on' /boot/firmware/config.txt
# only add the overlay if it's not already in the file
if ! grep -q "disable_poe_fan=1" /boot/firmware/config.txt; then
    sudo echo "disable_poe_fan=1" | sudo tee -a /boot/firmware/config.txt
fi
if ! grep -q "enable_uart=1" /boot/firmware/config.txt; then
    sudo echo "enable_uart=1" | sudo tee -a /boot/firmware/config.txt
fi
if ! grep -q "dtoverlay=uart1" /boot/firmware/config.txt; then
    sudo echo "dtoverlay=uart1" | sudo tee -a /boot/firmware/config.txt
fi
if ! grep -q "dtoverlay=uart1-pi5" /boot/firmware/config.txt; then
    sudo echo "dtoverlay=uart1-pi5" | sudo tee -a /boot/firmware/config.txt
fi
if ! grep -q "dtparam=i2c_arm=on" /boot/firmware/config.txt; then
    sudo echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt
fi
if ! grep -q "dtoverlay=i2c1" /boot/firmware/config.txt; then
    sudo echo "dtoverlay=i2c1" | sudo tee -a /boot/firmware/config.txt
fi

echo "Configuring hardware PWM..."
bash "$HOME/quickstart/setup/setup_hardware_pwm.sh"

echo -e "\n\e[94mWould you like to set up a WiFi access point? (y/n)\e[0m"
read -r setup_ap
if [[ "$setup_ap" =~ ^[Yy]$ ]]; then                            # Optional WiFi AP setup
    echo "Setting up WiFi access point..."
    bash "$HOME/quickstart/scripts/setup_accesspoint.sh"
else
    echo "Skipping WiFi access point setup..."
fi

# New section for RealSense installation
read -p "Do you want to perform the librealsense installation locally (needed for using realsense cameras)? (Warning: This will take a long time) (yes/no): " realsense_input
realsense_input=${realsense_input,,}

if [[ "$realsense_input" == "yes" ]]; then
    bash "$HOME/quickstart/scripts/setup_realsense.sh"
else
    echo "RealSense installation skipped."
fi

# Cleanup
echo "Cleaning up..."
sudo apt-get clean

if [[ "$user_input" == "yes" ]]; then
    echo "Rebooting now..."
    sudo reboot
else
    echo "No reboot will be performed, perform manually for all changes to take effect"
fi
