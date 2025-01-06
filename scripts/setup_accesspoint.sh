#!/bin/bash

# Script Name: setup_persistent_hotspot.sh
# Description: Automates the setup of a persistent Wi-Fi hotspot on Raspberry Pi using NetworkManager.
# Usage: sudo bash accesspoint_setup.sh

# Exit immediately if a command exits with a non-zero status
set -e

# ============================
# Configuration Variables
# ============================

# Define the SSID and password for the hotspot
SSID="$USER-bracketbot"
PASSWORD="12345678"  # Replace with a strong password

# Define the hotspot connection name
CONNECTION_NAME="MyHotspot"

# Define the virtual interface name
VIRTUAL_IFACE="wlan0_ap"

# Define the path for the setup script
SETUP_SCRIPT="/usr/local/bin/setup_hotspot.sh"

# Define the path for the systemd service
SYSTEMD_SERVICE="/etc/systemd/system/hotspot.service"

# ============================
# Check if Script Has Been Run Before
# ============================

if [ -f "$SETUP_SCRIPT" ] || [ -f "$SYSTEMD_SERVICE" ]; then
    echo "It appears that this script has been run before."
    echo "Do you want to:"
    echo "  1) Stop running the script"
    echo "  2) Delete the previous setup and run it again"
    read -p "Please enter your choice [1/2]: " choice
    if [ "$choice" == "1" ]; then
        echo "Exiting script."
        exit 0
    elif [ "$choice" == "2" ]; then
        echo "Cleaning up previous setup..."

        # Stop and disable the systemd service
        if systemctl is-active --quiet hotspot.service; then
            sudo systemctl stop hotspot.service
        fi
        sudo systemctl disable hotspot.service 2>/dev/null || true

        # Remove the systemd service file
        sudo rm -f "$SYSTEMD_SERVICE"

        # Reload systemd daemon
        sudo systemctl daemon-reload

        # Remove the setup script
        sudo rm -f "$SETUP_SCRIPT"

        # Delete the hotspot connection if it exists
        if nmcli connection show | grep -q "$CONNECTION_NAME"; then
            sudo nmcli connection delete "$CONNECTION_NAME"
        fi

        echo "Previous setup has been cleaned up."
    else
        echo "Invalid choice. Exiting."
        exit 1
    fi
fi

# ============================
# Step 1: Create the Hotspot Setup Script
# ============================

echo "Creating the hotspot setup script at $SETUP_SCRIPT..."

sudo tee "$SETUP_SCRIPT" > /dev/null <<EOF
#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Configuration Variables
SSID="$SSID"
PASSWORD="$PASSWORD"
CONNECTION_NAME="$CONNECTION_NAME"
VIRTUAL_IFACE="$VIRTUAL_IFACE"

# Function to create virtual interface
create_virtual_interface() {
    if ! iw dev | grep -q "\$VIRTUAL_IFACE"; then
        echo "Creating virtual interface \$VIRTUAL_IFACE..."
        sudo iw dev wlan0 interface add \$VIRTUAL_IFACE type __ap
    else
        echo "Virtual interface \$VIRTUAL_IFACE already exists."
    fi
}

# Function to bring up the virtual interface
bring_up_interface() {
    echo "Bringing up interface \$VIRTUAL_IFACE..."
    sudo ip link set \$VIRTUAL_IFACE up
}

# Function to configure NetworkManager hotspot
configure_hotspot() {
    echo "Configuring NetworkManager hotspot..."

    # Delete existing hotspot connection if it exists
    if nmcli connection show | grep -q "\$CONNECTION_NAME"; then
        echo "Deleting existing connection '\$CONNECTION_NAME'..."
        sudo nmcli connection delete "\$CONNECTION_NAME"
        sleep 1
    fi

    # Create a new hotspot connection
    echo "Creating new hotspot connection '\$CONNECTION_NAME'..."
    sudo nmcli connection add type wifi ifname \$VIRTUAL_IFACE con-name "\$CONNECTION_NAME" autoconnect yes ssid "\$SSID"
    sleep 1

    # Modify hotspot settings
    sudo nmcli connection modify "\$CONNECTION_NAME" 802-11-wireless.mode ap
    sudo nmcli connection modify "\$CONNECTION_NAME" 802-11-wireless.band bg
    sudo nmcli connection modify "\$CONNECTION_NAME" ipv4.method shared
    sudo nmcli connection modify "\$CONNECTION_NAME" wifi-sec.key-mgmt wpa-psk
    sudo nmcli connection modify "\$CONNECTION_NAME" wifi-sec.psk "\$PASSWORD"
    sleep 1

    # Bring up the hotspot connection
    echo "Activating hotspot connection '\$CONNECTION_NAME'..."
    sudo nmcli connection up "\$CONNECTION_NAME"
    sleep 1
}

# Function to ensure SSH is enabled and running
configure_ssh() {
    echo "Ensuring SSH service is enabled and running..."
    if ! systemctl is-active ssh >/dev/null 2>&1; then
        sudo systemctl enable ssh
        sudo systemctl start ssh
    else
        sudo systemctl restart ssh
    fi
}

# Execute the functions
create_virtual_interface
bring_up_interface
configure_hotspot
configure_ssh

echo "Setup complete."
echo "The access point '\$SSID' is now active with password '\$PASSWORD'."
echo "You can SSH into the Raspberry Pi using its hostname over the access point network."
EOF

# Make the setup script executable
sudo chmod +x "$SETUP_SCRIPT"

# ============================
# Step 2: Create the systemd Service
# ============================

echo "Creating the systemd service at $SYSTEMD_SERVICE..."

sudo tee "$SYSTEMD_SERVICE" > /dev/null <<EOF
[Unit]
Description=Persistent Wi-Fi Hotspot Setup
After=network.target

[Service]
Type=oneshot
ExecStart=$SETUP_SCRIPT
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# ============================
# Step 3: Reload systemd and Enable the Service
# ============================

echo "Reloading systemd daemon and enabling the hotspot service..."

sudo systemctl daemon-reload
sudo systemctl enable hotspot.service
sudo systemctl start hotspot.service

# ============================
# Step 4: Verify the Service Status
# ============================

echo "Verifying the hotspot service status..."

sudo systemctl status hotspot.service --no-pager

# ============================
# Step 5: Final Instructions
# ============================

echo ""
echo "Hotspot setup script and systemd service have been created and started successfully."
echo "The hotspot will now persist across reboots."
echo "To verify, you can reboot your Raspberry Pi and check the hotspot status using:"
echo "  sudo nmcli connection show --active"
echo "You should see '$CONNECTION_NAME' listed as active."