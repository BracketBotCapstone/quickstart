#!/bin/bash

set -e # Exit on non-zero status

# Note: This script expects pwm-pi5.dts to be in the same directory

echo "--- Running Hardware PWM Setup ---"

echo "Compiling Device Tree Source..."
dtc -I dts -O dtb -o pwm-pi5.dtbo pwm-pi5.dts
echo "Copying Device Tree Overlay to /boot/firmware/overlays/..."
sudo cp pwm-pi5.dtbo /boot/firmware/overlays/

echo "Checking if dtoverlay=pwm-pi5 already exists in config.txt..."
if ! grep -q "^dtoverlay=pwm-pi5" /boot/firmware/config.txt; then
  echo "Adding dtoverlay=pwm-pi5 to /boot/firmware/config.txt..."
  echo "dtoverlay=pwm-pi5" | sudo tee -a /boot/firmware/config.txt
else
  echo "dtoverlay=pwm-pi5 already present in /boot/firmware/config.txt."
fi

# Add permissions to user so you don't need sudo to run python scripts
echo "Configuring PWM permissions..."
if ! groups $USER | grep &>/dev/null '\bdialout\b'; then
    sudo usermod -aG dialout $USER
    echo "Added user $USER to dialout group (required for PWM permissions)"
else
    echo "User $USER already in dialout group."
fi

PWM_RULES_FILE="/etc/udev/rules.d/99-pwm-permissions.rules"
echo "Creating/Updating udev rule file: $PWM_RULES_FILE"
sudo tee $PWM_RULES_FILE > /dev/null << EOT
SUBSYSTEM=="pwm*", PROGRAM="/bin/sh -c '\\
        chown -R root:dialout /sys/class/pwm && chmod -R 770 /sys/class/pwm;\\
        chown -R root:dialout /sys/devices/platform/soc/*.pwm/pwm/pwmchip* && chmod -R 770 /sys/devices/platform/soc/*.pwm/pwm/pwmchip*;\\
        chown -R root:dialout /sys/devices/platform/axi/*.pcie/*.pwm/pwm/pwmchip* && chmod -R 770 /sys/devices/platform/axi/*.pcie/*.pwm/pwm/pwmchip*\\
'"
EOT

echo "Reloading udev rules..."
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "--- Hardware PWM Setup Complete ---"
echo -e "\n\e[31mREBOOT REQUIRED for changes to take effect.\e[0m"
echo "Reboot your Pi with: sudo reboot" 