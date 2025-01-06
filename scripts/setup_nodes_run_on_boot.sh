#!/bin/bash

# Variables
SERVICE_NAME="start_all_nodes.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"
USERNAME=$(whoami)
VENV_PATH="/home/$USERNAME/.venv"
WORKING_DIR="/home/$USERNAME/quickstart/launcher"
SCRIPT_NAME="start_all_nodes.py"
LOG_FILE="$WORKING_DIR/start_all_nodes.log"
ERR_FILE="$WORKING_DIR/start_all_nodes.err"

# Ensure the script is executable
chmod +x "$WORKING_DIR/$SCRIPT_NAME"

# Create the systemd service file
sudo bash -c "cat > $SERVICE_PATH" <<EOL
[Unit]
Description=Start All Nodes Script
After=network.target

[Service]
Type=simple
KillMode=process
User=$USERNAME
WorkingDirectory=$WORKING_DIR
ExecStart=/bin/bash -c 'source $VENV_PATH/bin/activate && exec python3 $SCRIPT_NAME'
Restart=on-failure
Environment=PYTHONUNBUFFERED=1
StandardOutput=append:$LOG_FILE
StandardError=append:$ERR_FILE

[Install]
WantedBy=multi-user.target
EOL

# Reload systemd daemon
sudo systemctl daemon-reload

# Enable and start the service
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl start "$SERVICE_NAME"

echo "Setup complete. The service $SERVICE_NAME has been created and started."