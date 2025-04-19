#! /bin/bash

sudo apt-get -qy update && sudo apt-get -qy upgrade && sudo apt-get -qy dist-upgrade
sudo apt-get install -qy guvcview libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get install -qy git wget cmake build-essential

# Prompt the user to disconnect the RealSense device
echo "Please disconnect the RealSense device if it's currently connected."
read -p "Press Enter once you've disconnected the device..."

# Confirm with the user
while true; do
    read -p "Have you disconnected the RealSense device? (y/n): " answer
    case $answer in
        [Yy]* ) echo "Proceeding with the setup..."; break;;
        [Nn]* ) echo "Please disconnect the device before continuing.";;
        * ) echo "Please answer yes (y) or no (n).";;
    esac
done

sudo apt-get install -qy libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts-hwe.sh
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release -DFORCE_LIBUVC=true
make -j$(($(nproc)-1)) && sudo make -j$(($(nproc)-1)) install
cd ..
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && sudo udevadm trigger
cd build
cmake .. -DBUILD_PYTHON_BINDINGS=bool:true -DPYTHON_EXECUTABLE=$(which python3)
make -j$(($(nproc)-1)) && sudo make -j$(($(nproc)-1)) install

# Use the correct virtual environment
source ~/quickstart-venv/bin/activate
pip install numpy
pip install opencv-python

# Copy the built Python bindings to the virtual environment's site-packages
SITE_PACKAGES=$(python3 -c "import site; print(site.getsitepackages()[0])")
# Note: The exact .so filenames might vary based on Python version/architecture.
# Find the correct files and copy them.
cp -T ~/librealsense/build/Release/pyrealsense2*.so "$SITE_PACKAGES/pyrealsense2.so"
cp -T ~/librealsense/build/Release/pyrealsense2*.so.* "$SITE_PACKAGES/"
cp -T ~/librealsense/build/Release/pyrsutils*.so "$SITE_PACKAGES/pyrsutils.so"
cp -T ~/librealsense/build/Release/pyrsutils*.so.* "$SITE_PACKAGES/"


echo "RealSense Python bindings have been installed in the virtual environment."
echo -e "\nYou can test the installation with the following steps:"
echo "1. Activate the virtual environment: source ~/quickstart-venv/bin/activate"
echo "2. Run the following command to test the pyrealsense2 package:"
echo "   python3 -c \"import pyrealsense2 as rs; print(rs.pipeline());\""
# echo "3. Run the test script:"
# echo "   python3 test_realsense.py"
echo -e "\nIf you encounter any issues, please refer to the README for troubleshooting steps." 