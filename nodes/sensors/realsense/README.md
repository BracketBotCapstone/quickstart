# Realsense camera

## Setup Steps

The following are the steps to get the c/cpp libraries and the pyrealsense2 package working on the raspberry pi running ubuntu. Steps are a bit involved.
This part is only if you are doing the setup outside what the setup_os.sh script does.

```bash
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo apt-get install guvcview libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev v4l-utils
sudo apt-get install git wget cmake build-essential
# Unplug realsense and run the following:
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts-hwe.sh
mkdir build && cd build
sudo make uninstall && make clean
cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release -DFORCE_LIBUVC=true
make -j$(($(nproc)-1)) && sudo make -j$(($(nproc)-1)) install
cd ..
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && sudo udevadm trigger
cd build
cmake .. -DBUILD_PYTHON_BINDINGS=bool:true -DPYTHON_EXECUTABLE=$(which python3)
make -j$(($(nproc)-1)) && sudo make -j$(($(nproc)-1)) install
```

```bash
# Now to use the pyrealsense2 package, the so's need to be copied into the python site-packages or dist-packages directory of the python environment you are using.
# The below is a full example if using a virtual env.
# Note: Replace PYTHON_VERSION with the python version you are using
cd ~/quickstart/nodes/sensors/realsense
python3 -m venv venv
source venv/bin/activate
pip install numpy
pip install opencv-python
deactivate
export PYTHON_VERSION=11
cd venv/lib/python3.${PYTHON_VERSION}/site-packages
cp -T ~/librealsense/build/Release/pyrealsense2.cpython-3${PYTHON_VERSION}-aarch64-linux-gnu.so pyrealsense2.so
cp -T ~/librealsense/build/Release/pyrealsense2.cpython-3${PYTHON_VERSION}-aarch64-linux-gnu.so.2.55 pyrealsense2.so.2.55
cp -T ~/librealsense/build/Release/pyrsutils.cpython-3${PYTHON_VERSION}-aarch64-linux-gnu.so pyrsutils.so
cp -T ~/librealsense/build/Release/pyrsutils.cpython-3${PYTHON_VERSION}-aarch64-linux-gnu.so.2.55 pyrsutils.so.2.55
cd ~/quickstart/nodes/sensors/realsense  
source venv/bin/activate
# Then you can test with the following:
python3 -c "import pyrealsense2 as rs; print(rs.pipeline());"
python3 ~/quickstart/nodes/sensors/realsense/realsense_manager.py
```

The shared object will be installed in /usr/local/lib, header files in /usr/local/include.
The binary demos, tutorials and test files will be copied into /usr/local/bin

librealsense tools are also installed in the docker environment and after following the above steps and can be run with the project names from the CMakeLists.txt files in https://github.com/IntelRealSense/librealsense/tree/master/tools
* realsense-viewer
* rs-depth-quality
* rs-hello-realsense
* rs-imu-calibration.py directly from /librealsense/tools/rs-imu-calibration
* more...

# Python Example

```bash
cd ~/quickstart/nodes/sensors/realsense
python3 realsense_manager.py
```

# Random docs for reference
Follow this:
https://github.com/IntelRealSense/librealsense/issues/11506#issuecomment-1599168850
* Which references this ROS installation guide: https://answers.ros.org/question/363889/intel-realsens-on-ubuntu-2004-ros-noetic-installation-desription/

If using raspbian and not ubuntu, then the following should be done (untested):
https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435

If nothing worked so far, then try this:
https://github.com/IntelRealSense/librealsense/blob/master/doc/libuvc_installation.md

These are the instructions for x86 or amd:
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

# Rerun example
https://dev.to/rerunio/visualize-live-streaming-frames-from-intel-realsense-depth-sensor-10lk

