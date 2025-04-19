#!/bin/bash

# ------------------------------------------------------------------------------
# TEMPLATE SCRIPT for creating a Systemd service to run a script on boot.
#
# HOW TO USE:
# 1. Copy this script to a new file (e.g., setup_my_cool_app_service.sh).
# 2. Edit the 'USER CONFIGURATION' section below with your specific details.
# 3. Make sure the script you want to run (`SCRIPT_TO_RUN_PATH`) exists and
#    is executable (`chmod +x /path/to/your/script`).
# 4. Run the *new* script (e.g., `bash setup_my_cool_app_service.sh`).
#
# NOTE: This script needs to be run with sudo privileges indirectly because it
#       generates commands that use sudo (copying files to /etc, systemctl).
#       However, run the script itself as the user who will own the process
#       if RUN_AS_USER is set to $USER, or manually ensure permissions.
# ------------------------------------------------------------------------------

set -e # Exit immediately if a command exits with a non-zero status.

# --- USER CONFIGURATION - EDIT THESE VARIABLES --- #

# 1. Service Name: Choose a unique name for your service (e.g., my-app, data-logger).
#    The script will create a file named <SERVICE_NAME>.service.
SERVICE_NAME="my_custom_service"

# 2. Description: A short description of what your service does.
SERVICE_DESCRIPTION="My Custom Application Service"

# 3. Script to Run: The *absolute* path to the script you want to run on boot.
#    Make sure this script is executable (`chmod +x`).
SCRIPT_TO_RUN_PATH="/home/$USER/my_project/my_script.py" # Example: /home/ollama/my_app/start.sh

# 4. Interpreter: The *absolute* path to the interpreter for your script.
#    - For Python scripts IN THE VENV: Use the venv's python executable.
#    - For standard Bash scripts: Use /bin/bash.
#    - For other interpreters, provide the full path.
INTERPRETER_PATH="/home/$USER/quickstart-venv/bin/python3" # Example: /bin/bash

# 5. User to Run As: The system user the service should run under.
#    Using $USER (the user running this setup script) is common.
#    Use 'root' with caution if necessary.
RUN_AS_USER="$USER"

# 6. Working Directory: The directory the script should be in when it runs.
#    This is important for scripts that use relative paths.
WORKING_DIR="$(dirname "$SCRIPT_TO_RUN_PATH")" # Defaults to the script's directory

# 7. Restart Policy: What to do if the script exits.
#    Options: no, on-success, on-failure, on-abnormal, on-watchdog, on-abort, or always.
RESTART_POLICY="on-failure"

# 8. Startup Dependencies: Services that should be running before this one starts.
#    Commonly 'network-online.target' if your script needs network access.
#    Separate multiple targets with spaces.
AFTER_DEPENDENCIES="network-online.target"

# --- END OF USER CONFIGURATION --- #

# --- Service File Generation --- #

SERVICE_FILE_PATH="/etc/systemd/system/${SERVICE_NAME}.service"
TEMP_SERVICE_FILE="/tmp/${SERVICE_NAME}.service.$$"

# Check if the script path exists
if [ ! -f "$SCRIPT_TO_RUN_PATH" ]; then
    echo "Error: Script to run does not exist at: $SCRIPT_TO_RUN_PATH" >&2
    echo "Please correct SCRIPT_TO_RUN_PATH in this script." >&2
    exit 1
fi

# Check if the interpreter path exists
if [ ! -x "$INTERPRETER_PATH" ]; then
    echo "Error: Interpreter does not exist or is not executable at: $INTERPRETER_PATH" >&2
    echo "Please correct INTERPRETER_PATH in this script." >&2
    exit 1
fi

echo "--- Generating Systemd Service File --- "
echo "Service Name: ${SERVICE_NAME}.service"
echo "Description:  ${SERVICE_DESCRIPTION}"
echo "ExecStart:    ${INTERPRETER_PATH} ${SCRIPT_TO_RUN_PATH}"
echo "User:         ${RUN_AS_USER}"
echo "Working Dir:  ${WORKING_DIR}"
echo "Restart:      ${RESTART_POLICY}"
echo "After:        ${AFTER_DEPENDENCIES}"
echo ""

# Create the service file content
cat > "$TEMP_SERVICE_FILE" << EOL
[Unit]
Description=${SERVICE_DESCRIPTION}
After=${AFTER_DEPENDENCIES}

[Service]
User=${RUN_AS_USER}
Group=$(id -gn "$RUN_AS_USER")
WorkingDirectory=${WORKING_DIR}
ExecStart=${INTERPRETER_PATH} ${SCRIPT_TO_RUN_PATH}
Restart=${RESTART_POLICY}
# Optional: Set environment variables if needed
# Environment="PYTHONUNBUFFERED=1"
# Environment="OTHER_VAR=value"

[Install]
WantedBy=multi-user.target
EOL

echo "Service file content generated successfully in $TEMP_SERVICE_FILE"

# --- Installation and Activation --- #

echo "
--- Installing and Enabling Service --- "

echo "The following commands will be run using sudo:"

# 1. Copy the service file to the systemd directory
CMD_COPY="sudo cp '$TEMP_SERVICE_FILE' '$SERVICE_FILE_PATH'"
echo "1. $CMD_COPY"

# 2. Set permissions for the service file
CMD_CHMOD="sudo chmod 644 '$SERVICE_FILE_PATH'"
echo "2. $CMD_CHMOD"

# 3. Reload systemd manager configuration
CMD_RELOAD="sudo systemctl daemon-reload"
echo "3. $CMD_RELOAD"

# 4. Enable the service to start on boot
CMD_ENABLE="sudo systemctl enable '${SERVICE_NAME}.service'"
echo "4. $CMD_ENABLE"

# 5. Start the service immediately (optional, comment out if you only want it to start on next boot)
CMD_START="sudo systemctl start '${SERVICE_NAME}.service'"
echo "5. $CMD_START"

# 6. Check the status of the service
CMD_STATUS="sudo systemctl status '${SERVICE_NAME}.service' --no-pager -l"
echo "6. $CMD_STATUS"

echo ""
read -p "Press Enter to execute these commands, or Ctrl+C to cancel..."

# Execute the commands
echo "Executing commands..."
$CMD_COPY
$CMD_CHMOD
$CMD_RELOAD
$CMD_ENABLE
$CMD_START

echo "
--- Service Status --- "
$CMD_STATUS

# Clean up temporary file
rm "$TEMP_SERVICE_FILE"

echo "
--- Done --- "
echo "Service '${SERVICE_NAME}' created and enabled."
echo "To check logs: sudo journalctl -u ${SERVICE_NAME}.service -f"
echo "To stop:      sudo systemctl stop ${SERVICE_NAME}.service"
echo "To disable:   sudo systemctl disable ${SERVICE_NAME}.service" 