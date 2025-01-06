# See: https://gist.github.com/Gadgetoid/b92ad3db06ff8c264eef2abf0e09d569

dtc -I dts -O dtb -o pwm-pi5.dtbo pwm-pi5.dts
sudo cp pwm-pi5.dtbo /boot/firmware/overlays/
# only add the overlay if it's not already in the file
if ! grep -q "dtoverlay=pwm-pi5" /boot/firmware/config.txt; then
    sudo echo "dtoverlay=pwm-pi5" | sudo tee -a /boot/firmware/config.txt
    echo "Overlay added to /boot/firmware/config.txt"
fi

# Add permissions to user so you don't need sudo to run python scripts
if ! groups $USER | grep &>/dev/null '\bdialout\b'; then
    sudo usermod -aG dialout $USER
    echo "Added user to dialout group"
else
    echo "User already in dialout group"
fi

sudo tee -a /etc/udev/rules.d/99-com.rules > /dev/null << EOT
SUBSYSTEM=="pwm*", PROGRAM="/bin/sh -c '\\
        chown -R root:dialout /sys/class/pwm && chmod -R 770 /sys/class/pwm;\\
        chown -R root:dialout /sys/devices/platform/soc/*.pwm/pwm/pwmchip* && chmod -R 770 /sys/devices/platform/soc/*.pwm/pwm/pwmchip*;\\
        chown -R root:dialout /sys/devices/platform/axi/*.pcie/*.pwm/pwm/pwmchip* && chmod -R 770 /sys/devices/platform/axi/*.pcie/*.pwm/pwm/pwmchip*\\
'"
EOT
echo "Added udev rule for PWM permissions"

echo -e "For changes to take effect reboot your Pi with:\n\nsudo reboot\n"