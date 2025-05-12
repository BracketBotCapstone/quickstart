#!/bin/bash

set -e # Exit immediately if a command exits with a non-zero status

# --- Helper Functions ---

# Function to activate virtual environment if not already active
activate_venv() {
    if [ -z "$VIRTUAL_ENV" ]; then
        echo "Activating virtual environment: ~/quickstart-venv/bin/activate"
        if [ -f "$HOME/quickstart-venv/bin/activate" ]; then
            source ~/quickstart-venv/bin/activate
        else
            echo "Error: Virtual environment activation script not found at ~/quickstart-venv/bin/activate" >&2
            exit 1
        fi
    else
        # Venv already active, which might happen if sourced in .bashrc
        echo "Virtual environment already active."
    fi
}

# Function for timed confirmation, takes message and function name to run
timed_confirm() {
    local message=$1
    local function_to_run=$2
    local timeout=5 # seconds
    local key_pressed=false
    local confirm="n" # Default to no if key is pressed but input is not y/Y

    # Print the message without a newline, add a space
    echo -n -e "\\n\\e[94m${message}\\e[0m "

    # Prompt user and show countdown on the same line
    for (( i=timeout; i>0; i-- )); do
        # Add blue color code here for the prompt line
        printf "\r\e[94m${message} Press any key to manually confirm/skip (automatically proceeding in %d seconds...)\e[0m " "$i"
        # Read with timeout of 1 second, non-blocking (-n 1), silent (-s)
        if read -t 1 -n 1 -s; then
            key_pressed=true
            break # Exit loop if key is pressed
        fi
    done

    # Clear the countdown line
    printf "\r%*s\r" "$(tput cols)" "" # Print spaces across the screen width, then return cursor
    # Re-print the original message cleanly after clearing the line
    echo -e "\\e[94m${message}\\e[0m"

    if $key_pressed; then
        # Clear potential buffered input from during countdown
        while read -t 0.1 -n 1 discard_char; do :; done

        # Add blue color code and change to (Y/n)
        read -rp $'
\e[94mManually proceed with this setup step? (Y/n): \e[0m' confirm < /dev/tty
        # Default to Yes if Enter is pressed (confirm is empty)
        if [[ -z "$confirm" || "$confirm" =~ ^[Yy]$ ]]; then
            echo " Proceeding with manual confirmation..."
            "$function_to_run"
        else
            echo " Skipping this step..."
        fi
    else
        echo -e "\\e[94mTimeout reached, proceeding automatically...\\e[0m"
        "$function_to_run"
    fi
}

# ===========================
#      MAIN SETUP SCRIPT
# ===========================

echo "===== Starting Full OS Setup ====="

# --- Initial Update Block ---
_setup_initial_update() {
    echo -e "\\n--- Updating Package Lists ---"
    sudo apt update
}
timed_confirm "Updating package lists..." _setup_initial_update

# --- User Groups Block ---
_setup_user_groups() {
    echo -e "\\n--- Configuring User Groups (dialout, audio) ---"
    echo "Adding current user ($USER) to 'dialout' and 'audio' groups..."
    sudo usermod -a -G dialout,audio $USER
}
timed_confirm "Configuring user groups..." _setup_user_groups

# --- Pinout Tool Block ---
_setup_pinout_tool() {
    echo -e "\\n--- Installing Pinout Tool ---"
    echo "Creating pinout script at /usr/local/bin/pinout..."
    sudo tee /usr/local/bin/pinout > /dev/null << 'EOF_PINOUT'
#!/usr/bin/env bash
# Raspberry Pi 5 GPIO header ASCII pin‑out
# Place this file on your PATH and `chmod +x` it.

cat <<'EOF'
Raspberry Pi 5 GPIO Pinout Reference

   _______      _______      _________
+-| USB   |----| USB   |----| Ethernet |--+
| |       |    |       |    |          |  |
| |       |    |       |    |          |  |
| |_______|    |_______|    |          |  |
|                           |_________ |  |                ┌──────────────────────────────────────────────────┐
|  __________                             |                ├─────────────────────┼─────┼──────────────────────┤
| | 40 | 39 |                             |                │   GPIO21 (PCM_DOUT) │40│39│ GND                  │
| | 38 | 37 |                             |                │    GPIO20 (PCM_DIN) │38│37│ GPIO26               │
| | 36 | 35 |                             |                │              GPIO16 │36│35│ GPIO19 (PCM_FS)      │
| | 34 | 33 |                             |                │                 GND │34│33│ GPIO13 (PWM1)        │
| | 32 | 31 |      Raspberry Pi 5         |                │       GPIO12 (PWM0) │32│31│ GPIO6                │
| | 30 | 29 |                             |                │                 GND │30│29│ GPIO5                │
| | 28 | 27 |                             |                │       GPIO1 (ID_SC) │28│27│ GPIO0 (ID_SD)        │
| | 26 | 25 |                             |                │         GPIO7 (CE1) │26│25│ GND                  │
| | 24 | 23 |==============================================│         GPIO8 (CE0) │24│23│ GPIO11 (SCLK)        │
| | 22 | 21 |                             |                |              GPIO25 │22│21│ GPIO9 (MISO)         │
| | 20 | 19 |                             |                │                 GND │20│19│ GPIO10 (MOSI)        │
| | 18 | 17 |                             |                │              GPIO24 │18│17│ 3V3                  │
| | 16 | 15 |                             |                │              GPIO23 │16│15│ GPIO22               │
| | 14 | 13 |                             |                │                 GND │14│13│ GPIO27               │
| | 12 | 11 |                             |                │    GPIO18 (PCM_CLK) │12│11│ GPIO17               │
| | 10 | 9  |                             |                │        GPIO15 (RXD) │10│9 │ GND                  │
| |  8 | 7  |                             |                │        GPIO14 (TXD) │8 │7 │ GPIO4 (GPCLK0)       │
| |  6 | 5  |                             |                │                 GND │6 │5 │ GPIO3 (SCL)          │
| |  4 | 3  |                             |                │                  5V │4 │3 │ GPIO2 (SDA)          │
| |  2 | 1  |                             |                │                  5V │2 │1 │ 3V3                  │
| |_________|                             |                └──────────────────────────────────────────────────┘
|                                         |
+-----------------------------------------+

EOF
EOF_PINOUT

    echo "Setting execute permissions for /usr/local/bin/pinout..."
    sudo chmod 755 /usr/local/bin/pinout
}
timed_confirm "Installing pinout display tool..." _setup_pinout_tool

# --- ODrive Voltage Prompt Block ---
_setup_odrive_voltage_prompt() {
    echo -e "\n--- Setting up ODrive Voltage Prompt for Bash (\\~/.bashrc) ---"
    local BASHRC="$HOME/.bashrc"

    # Check if the marker exists
    if ! grep -q "# ─── Add ODrive voltage to prompt ───" "$BASHRC"; then
        echo "Adding ODrive voltage prompt snippet to $BASHRC..."
        cat >> "$BASHRC" << 'EOF_ODRIVE_BASH'

# ─── Add ODrive voltage to prompt ───
update_odrive_prompt() {
    # 1) Read the raw voltage (plain number) or ?? on error
    local raw_v
    raw_v=$(python3 -m quickstart.lib.odrive_uart 2>/dev/null | tr -d '\r\n') || raw_v='??'

    # 2) Decide colour and build the guarded [voltage] segment
    local v
    if [[ $raw_v =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        # Pick colour: red <17.5, orange <18.5, yellow <19.5, else green
        local color
        case 1 in
            $(( $(awk "BEGIN{print ($raw_v < 17.5)}") )) ) color='31' ;; # red
            $(( $(awk "BEGIN{print ($raw_v < 18.5)}") )) ) color='33' ;; # orange
            $(( $(awk "BEGIN{print ($raw_v < 19.5)}") )) ) color='93' ;; # yellow
            * )                     color='32' ;;                       # green
        esac
        v='\[\e['"$color"'m\]'"${raw_v}V"'\[\e[0m\]'
    else
        v='??V'
    fi

    # 3) Active virtual-env, if any (printable text, so no guards)
    local venv_prefix=""
    [[ -n $VIRTUAL_ENV ]] && venv_prefix="($(basename "$VIRTUAL_ENV")) "

    # 4) Window title (OSC sequence) – fully wrapped
    local title='\[\e]0;\u@\h: \w\a\]'

    # 5) Assemble PS1:  title  venv  user@host  [voltage] : cwd $
    PS1="${title}${venv_prefix}${debian_chroot:+($debian_chroot)}\[\e[1;32m\]\u@\h\[\e[0m\] [${v}]:\[\e[1;34m\]\w \$\[\e[0m\] "
}

# Make sure we rebuild the prompt before every command
case "$PROMPT_COMMAND" in
  *update_odrive_prompt*) ;;  # already present
  *) PROMPT_COMMAND="update_odrive_prompt${PROMPT_COMMAND:+; $PROMPT_COMMAND}" ;;
esac

EOF_ODRIVE_BASH
    else
        echo "ODrive voltage prompt already configured in $BASHRC. Skipping..."
    fi
}
timed_confirm "Setting up ODrive voltage prompt in Bash..." _setup_odrive_voltage_prompt

# --- SSH Setup Block ---
_setup_ssh() {
    echo -e "\\n--- Setting up SSH ---"
    echo "Installing and enabling SSH service..."
    sudo apt install -y ssh
    sudo systemctl enable ssh
    sudo systemctl start ssh
    echo "Checking SSH service status..."
    sudo SYSTEMD_PAGER='' systemctl status ssh
}
timed_confirm "Installing and configuring SSH..." _setup_ssh

# --- Shell Tools Block ---
_setup_shell_tools() {
    echo -e "\\n--- Setting up Shell Tools (Atuin, Zoxide) ---"
    echo "Installing Atuin (shell history)..."
    curl --proto '=https' --tlsv1.2 -LsSf https://setup.atuin.sh | sh
    echo "Installing zoxide (directory navigation)..."
    curl -sSfL https://raw.githubusercontent.com/ajeetdsouza/zoxide/main/install.sh | sh
    echo "Configuring zoxide in ~/.bashrc..."
    if ! grep -Fxq 'eval "$(zoxide init bash)"' "$HOME/.bashrc"; then
        echo '' >> "$HOME/.bashrc"
        echo '# Initialize Zoxide' >> "$HOME/.bashrc"
        echo 'eval "$(zoxide init bash)"' >> "$HOME/.bashrc"
    fi
    echo "Configuring Atuin in ~/.bashrc (disabling up-arrow)..."
    sed -i '/eval "$(atuin init/d' "$HOME/.bashrc"
    echo '' >> "$HOME/.bashrc"
    echo '# Initialize Atuin (history sync, disable up-arrow override)' >> "$HOME/.bashrc"
    echo 'eval "$(atuin init bash --disable-up-arrow)"' >> "$HOME/.bashrc"
}
timed_confirm "Installing shell tools (Atuin, Zoxide)..." _setup_shell_tools

# --- Python Base Environment Block ---
_setup_python_base() {
    echo -e "\\n--- Setting up Python Base Environment (venv) ---"
    echo "Installing Python development packages and tmux..."
    sudo apt install -y python3-dev python3-pip python3-venv tmux
    echo "Creating Python virtual environment 'quickstart-venv' in ~ ..."
    original_dir=$(pwd)
    cd ~
    python3 -m venv quickstart-venv
    cd "$original_dir"
    echo "Configuring automatic venv activation in ~/.bashrc..."
    if ! grep -Fxq 'source ~/quickstart-venv/bin/activate' "$HOME/.bashrc"; then
        echo '' >> "$HOME/.bashrc"
        echo '# Activate quickstart Python environment' >> "$HOME/.bashrc"
        echo 'if [ -f "$HOME/quickstart-venv/bin/activate" ]; then' >> "$HOME/.bashrc"
        echo '  source "$HOME/quickstart-venv/bin/activate"' >> "$HOME/.bashrc"
        echo 'fi' >> "$HOME/.bashrc"
    fi
    activate_venv
}
timed_confirm "Setting up Python base environment (venv, .bashrc activation)..." _setup_python_base

# --- MQTT Broker Block ---
_setup_mqtt_broker() {
    echo -e "\\n--- Setting up MQTT Broker (Mosquitto) ---"
    echo "Installing mosquitto broker, clients, and ufw..."
    sudo apt install -y mosquitto mosquitto-clients ufw
    echo "Enabling and starting Mosquitto service..."
    sudo systemctl enable mosquitto
    sudo systemctl start mosquitto
    echo "Configuring Mosquitto listeners (MQTT on 1883 localhost, WebSockets on 9001)..."
    sudo bash -c 'cat > /etc/mosquitto/conf.d/default.conf << EOL_MOSQUITTO
listener 1883 localhost
protocol mqtt

listener 9001
protocol websockets
allow_anonymous true
EOL_MOSQUITTO'
    echo "Allowing Mosquitto WebSocket port 9001 through firewall (ufw)..."
    sudo ufw allow 9001
    echo "Restarting Mosquitto service..."
    sudo systemctl restart mosquitto
}
timed_confirm "Setting up MQTT Broker (Mosquitto)..." _setup_mqtt_broker

# --- Core Python Libraries Block ---
_setup_core_python_libs() {
    echo -e "\\n--- Installing Core Python Libraries (inside venv) ---"
    activate_venv
    echo "Installing RPi.GPIO and lgpio libraries..."
    python3 -m pip install RPi.GPIO rpi-lgpio
    echo "Installing Adafruit MPU6050 library..."
    python3 -m pip install adafruit-circuitpython-mpu6050
    echo "Installing paho-mqtt library..."
    python3 -m pip install paho-mqtt
    echo "Installing OpenCV library..."
    python3 -m pip install opencv-python
    echo "Installing other required Python packages (numpy, control, etc.)..."
    python3 -m pip install numpy sympy control matplotlib pyserial libtmux sshkeyboard fastask
}
timed_confirm "Installing core Python libraries (GPIO, IMU, MQTT, numpy, etc.)..." _setup_core_python_libs

# --- PYTHONPATH Configuration Block ---
_setup_pythonpath() {
    echo -e "\\n--- Adding HOME to PYTHONPATH in ~/.bashrc ---"
    if ! grep -Fxq 'export PYTHONPATH="$HOME:$PYTHONPATH"' "$HOME/.bashrc"; then
        echo '' >> "$HOME/.bashrc"
        echo 'export PYTHONPATH="$HOME:$PYTHONPATH"' >> "$HOME/.bashrc"
    else
        echo "PYTHONPATH already configured in ~/.bashrc. Skipping..."
    fi
}

timed_confirm "Adding HOME to PYTHONPATH for quickstart imports..." _setup_pythonpath

# --- ODrive Python Library Block ---
_setup_odrive_python_lib() {
    echo -e "\\n--- Installing ODrive Python Library (inside venv) ---"
    activate_venv
    echo "Installing ODrive library (v0.5.1.post0)..."
    python3 -m pip install odrive==0.5.1.post0
}
timed_confirm "Installing ODrive Python library..." _setup_odrive_python_lib

# --- ODrive udev Setup Block ---
_setup_odrive_udev() {
    echo -e "\\n--- Setting up ODrive udev Rules (requires ODrive lib) ---"
    activate_venv
    echo "Finding odrivetool path..."
    ODRIVE_TOOL_PATH=$(which odrivetool)
    if [ -z "$ODRIVE_TOOL_PATH" ]; then
        echo "Error: odrivetool command not found in PATH after installing odrive package." >&2
        echo "Something might have gone wrong with the Python library installation." >&2
        exit 1
    fi
    echo "Running ODrive udev setup using: $ODRIVE_TOOL_PATH ..."
    sudo "$ODRIVE_TOOL_PATH" udev-setup
}
timed_confirm "Setting up ODrive udev rules..." _setup_odrive_udev

# --- Boot Configuration Block ---
_setup_boot_config() {
    echo -e "\\n--- Configuring Boot Settings (/boot/firmware/config.txt) ---"
    echo "Ensuring specific settings are present: disable_poe_fan=1, enable_uart=1, dtoverlay=uart1-pi5, dtparam=i2c_arm=on, dtoverlay=i2c1, dtparam=spi=on"
    CONFIG_FILE="/boot/firmware/config.txt"
    {
        grep -q "^disable_poe_fan=1" $CONFIG_FILE || printf "\ndisable_poe_fan=1\n"
        grep -q "^enable_uart=1" $CONFIG_FILE || printf "enable_uart=1\n"
        grep -q "^dtoverlay=uart1-pi5" $CONFIG_FILE || printf "dtoverlay=uart1-pi5\n"
        grep -q "^dtparam=i2c_arm=on" $CONFIG_FILE || printf "dtparam=i2c_arm=on\n"
        grep -q "^dtoverlay=i2c1" $CONFIG_FILE || printf "dtoverlay=i2c1\n"
        grep -q "^dtparam=spi=on" $CONFIG_FILE || printf "dtparam=spi=on\n"
    } | sudo tee -a $CONFIG_FILE > /dev/null
}
timed_confirm "Configuring boot settings (config.txt)..." _setup_boot_config

# --- Access Point Setup Block ---
_setup_accesspoint() {
    echo "--- Running Access Point Setup ---"
    local SSID="$USER-bracketbot"
    local PASSWORD="12345678"
    local CONNECTION_NAME="MyHotspot"
    local VIRTUAL_IFACE="wlan0_ap"
    local SETUP_SCRIPT="/usr/local/bin/setup_hotspot.sh"
    local SYSTEMD_SERVICE="/etc/systemd/system/hotspot.service"

    if [ -f "$SETUP_SCRIPT" ] || [ -f "$SYSTEMD_SERVICE" ]; then
        echo "Access point setup files detected ($SETUP_SCRIPT or $SYSTEMD_SERVICE exists)."
        echo "Assuming setup was already done. To force re-run, manually delete these files and disable/stop the hotspot.service."
        if ! systemctl is-active --quiet hotspot.service; then
          echo "Attempting to start existing hotspot service..."
          sudo systemctl start hotspot.service || echo "Failed to start existing service."
        fi
        echo "--- Skipping Access Point Setup Re-creation ---"
        return 0
    fi

    echo "Creating the hotspot setup script at $SETUP_SCRIPT..."
    sudo tee "$SETUP_SCRIPT" > /dev/null << EOF_AP_SCRIPT
#!/bin/bash
set -e
SSID="$SSID"
PASSWORD="$PASSWORD"
CONNECTION_NAME="$CONNECTION_NAME"
VIRTUAL_IFACE="$VIRTUAL_IFACE"

create_virtual_interface() {
    if ! iw dev | grep -q "$VIRTUAL_IFACE"; then
        echo "Creating virtual interface $VIRTUAL_IFACE..."
        sudo iw dev wlan0 interface add "$VIRTUAL_IFACE" type __ap || echo "Failed to add virtual interface. Is iw installed?"
    else
        echo "Virtual interface $VIRTUAL_IFACE already exists."
    fi
}
bring_up_interface() {
    echo "Bringing up interface $VIRTUAL_IFACE..."
    sudo ip link set "$VIRTUAL_IFACE" up || echo "Failed to bring up interface $VIRTUAL_IFACE"
}
configure_hotspot() {
    echo "Configuring NetworkManager hotspot..."
    if nmcli c show "$CONNECTION_NAME" > /dev/null 2>&1; then
        echo "Deleting existing connection '$CONNECTION_NAME'..."
        sudo nmcli connection delete "$CONNECTION_NAME" || echo "Failed to delete existing connection $CONNECTION_NAME"
        sleep 1
    fi
    echo "Creating new hotspot connection '$CONNECTION_NAME'..."
    sudo nmcli connection add type wifi ifname "$VIRTUAL_IFACE" con-name "$CONNECTION_NAME" autoconnect yes ssid "$SSID" || echo "Failed to add connection $CONNECTION_NAME"
    sleep 1
    sudo nmcli connection modify "$CONNECTION_NAME" 802-11-wireless.mode ap || echo "Failed: set mode ap"
    sudo nmcli connection modify "$CONNECTION_NAME" 802-11-wireless.band bg || echo "Failed: set band bg"
    sudo nmcli connection modify "$CONNECTION_NAME" ipv4.method shared || echo "Failed: set ipv4 shared"
    sudo nmcli connection modify "$CONNECTION_NAME" wifi-sec.key-mgmt wpa-psk || echo "Failed: set key-mgmt wpa-psk"
    sudo nmcli connection modify "$CONNECTION_NAME" wifi-sec.psk "$PASSWORD" || echo "Failed: set psk"
    sleep 1
    echo "Activating hotspot connection '$CONNECTION_NAME'..."
    sudo nmcli connection up "$CONNECTION_NAME" || echo "Failed to bring up connection $CONNECTION_NAME"
    sleep 1
}
configure_ssh() {
    echo "Ensuring SSH service is enabled and running..."
    if ! systemctl is-active ssh >/dev/null 2>&1; then
        sudo systemctl enable ssh || echo "Failed: enable ssh"
        sudo systemctl start ssh || echo "Failed: start ssh"
    else
        sudo systemctl restart ssh || echo "Failed: restart ssh"
    fi
}
create_virtual_interface
bring_up_interface
configure_hotspot
configure_ssh
echo "Access Point Setup script finished."
EOF_AP_SCRIPT
    sudo chmod +x "$SETUP_SCRIPT"

    echo "Creating the systemd service at $SYSTEMD_SERVICE..."
    sudo tee "$SYSTEMD_SERVICE" > /dev/null << EOF_AP_SERVICE
[Unit]
Description=Persistent Wi-Fi Hotspot Setup (via setup_os.sh)
After=network.target NetworkManager.service
BindsTo=sys-subsystem-net-devices-wlan0.device

[Service]
Type=oneshot
ExecStart=$SETUP_SCRIPT
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF_AP_SERVICE

    echo "Reloading systemd daemon and enabling the hotspot service..."
    sudo systemctl daemon-reload
    sudo systemctl enable hotspot.service
    sudo systemctl start hotspot.service
    echo "Verifying the hotspot service status..."
    sudo systemctl status hotspot.service --no-pager -l || echo "Service status check failed."
    echo "--- Access Point Setup Complete ---"
}
timed_confirm "Setting up WiFi access point..." _setup_accesspoint

# --- Speaker Setup Block ---
_setup_speaker() {
    echo "--- Running Speaker Setup ---"
    echo "Installing speaker system dependencies..."
    sudo apt-get update
    sudo apt-get install -y python3-pyaudio libportaudio2 libportaudiocpp0 portaudio19-dev libsndfile1

    activate_venv
    echo "Installing speaker Python packages..."
    python3 -m pip install --upgrade pip
    python3 -m pip install pyaudio pyalsaaudio elevenlabs sounddevice soundfile python-dotenv

    echo "Testing speaker package imports..."
    python3 -c "import sounddevice; import soundfile; import elevenlabs; print('Speaker packages installation successful!')" || echo "Speaker package test import failed."

    echo "--- Speaker Setup Complete ---"
}
timed_confirm "Setting up Speaker dependencies..." _setup_speaker

# --- Mic Setup Block ---
_setup_mic() {
    echo "--- Running Mic Setup ---"
    echo "Installing microphone system dependencies..."
    sudo apt-get update
    sudo apt-get install -y portaudio19-dev

    activate_venv
    echo "Installing microphone Python packages..."
    python3 -m pip install --upgrade pip
    python3 -m pip install pyaudio

    echo "Testing microphone package imports..."
    python3 -c "import pyaudio; print('PyAudio installation successful!')" || echo "Mic package test import failed."

    echo "--- Mic Setup Complete ---"
}
timed_confirm "Setting up Mic dependencies..." _setup_mic

# --- LED Setup Block ---
_setup_led() {
    echo "--- Running LED Setup ---"
    echo "Installing LED system dependencies..."
    sudo apt-get update
    sudo apt-get install -y python3-spidev

    echo "Enabling SPI interface via raspi-config..."
    export DEBIAN_FRONTEND=noninteractive
    sudo raspi-config nonint do_spi 0 || echo "raspi-config command failed. SPI might not be enabled."

    activate_venv
    echo "Installing LED Python packages..."
    python3 -m pip install Pi5Neo spidev

    echo "Applying patch to Pi5Neo library (changing sleep time)..."
    PI5NEO_PATH=$(python3 -c "import importlib.util; spec = importlib.util.find_spec('pi5neo'); print(spec.origin if spec else '')")
    if [ -f "$PI5NEO_PATH" ]; then
        sudo sed -i 's/time\.sleep(0\.1)/time.sleep(0.01)/g' "$PI5NEO_PATH"
        echo "Patched $PI5NEO_PATH successfully."
    else
        echo "Warning: pi5neo.py not found at expected location: '$PI5NEO_PATH'. Skipping patch."
    fi

    echo "--- LED Setup Complete ---"
}
timed_confirm "Setting up LED dependencies..." _setup_led

# --- CAD Model Download Block ---
_setup_cad_model() {
    echo -e "\n--- Downloading Bracketbot CAD model ---"
    mkdir -p "$HOME/quickstart/lib"
    wget -q -O "$HOME/quickstart/lib/Bracketbot.stl" "https://github.com/BracketBotCapstone/quickstart/releases/download/bracketbot-cad-model/Bracketbot.stl"
    echo "--- Downloaded Bracketbot.stl to $HOME/quickstart/lib ---"
}
timed_confirm "Downloading Bracketbot CAD model into lib folder..." _setup_cad_model

# --- Final Steps ---
echo -e "\\n\\e[31m===== OS SETUP COMPLETE! =====\\e[0m"
echo -e "\\e[31mYou NEED TO REBOOT before running hardware-related tasks like ODrive calibration.\\e[0m"
echo -e "\\e[31mThe motor controller setup script (\`quickstart/setup/motor_controller_setup.sh\`) should be run AFTER rebooting.\\e[0m"
echo -e "\\e[31mRebooting in:\\e[0m"
for i in {20..1}; do
    echo -e "\\e[31m$i...\\e[0m"
    sleep 1
done
sudo reboot
