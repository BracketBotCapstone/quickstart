import platform
import subprocess

from setuptools import find_packages, setup
from setuptools.command.install import install

is_arm = platform.machine() in ('aarch64', 'armv7l')

# Define dependencies
dependencies = [
    "ahrs>=0.3.1",
    "numpy<2.0.0",
    "opencv-python>=4.1.1",
    "toml>=0.10.2",
    "click>=8.1.7",
    "rerun-sdk>=0.18.2",
    "tomlkit>=0.11.1",
    "odrive==0.5.1.post0",
    "adafruit-circuitpython-mpu6050>=1.2.4",
    "tqdm>=4.66.5",
    "torch>=2.4.1",
    "scipy>=1.14.0",
    "csaps>=1.2.0",
    "cmake>=3.25.0",
    "scikit-build>=0.17.6",
    "pywavemap @ git+https://github.com/ethz-asl/wavemap.git#subdirectory=library/python",
    "matplotlib>=3.9.2",
    "rpi-hardware-pwm>=0.2.2",
    "pyserial>=3.5",
    "smbus2>=0.5.0",
    "navlie @ git+https://github.com/decargroup/navlie@main",
    "pymlg @ git+https://github.com/decargroup/pymlg@main",
    "control>=0.10.1",
    "sympy>=1.12.1",
    "rpi-lgpio>=0.5",
    "PyYAML>=6.0.1",
    "paho-mqtt>=1.6.1",
    "libtmux>=0.37.0",
    "RPi.GPIO>=0.7.1",
    "rpi-lgpio>=0.6",
    "ruff>=0.7.1"
]

# Add pyrealsense2 dependency based on platform
if is_arm:
    pass
    # NOTE: currently, we don't need this since we're building the package in the container manually. Otherwise, For ARM, we'll use the local package
    # package_data["bracket_bot"].append("third_party/pyrealsense2/**/*")
else:
    # For x86, we'll use the PyPI package
    dependencies.append("pyrealsense2")

class CustomInstallCommand(install):
    """Customized setuptools install command to patch ODrive package."""

    def run(self):
        # Run the standard install process
        install.run(self)
        
        # Patch the ODrive package baud rate
        print("Patching the ODrive package baud rate...")
        try:
            sed_path = subprocess.check_output(
                ["python", "-c", "import fibre; import os; print(os.path.join(os.path.dirname(fibre.__file__), 'serial_transport.py'))"],
                text=True
            ).strip()
            subprocess.run(['sed', '-i', 's/DEFAULT_BAUDRATE = 115200/DEFAULT_BAUDRATE = 460800/', sed_path], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Failed to patch ODrive package: {e}")

setup(
    name="bracket_bot",
    version="0.3.6",
    description="Bracket Bot Python Package",
    author="Jai P",
    author_email="j2prajap@uwaterloo.ca",
    license="MIT License",
    packages=find_packages(include=[
        "utils",
        "control",
        "realsense",
        "rerun_viewer",
        "imu"
    ]),
    package_dir={
        "utils": "utils",
        "control": "nodes/control",
        "realsense": "nodes/sensors/realsense",
        "rerun_viewer": "nodes/rerun_viewer",
        "imu": "nodes/sensors/imu"
    },
    # package_data=package_data,
    install_requires=dependencies,
    python_requires=">=3.8",
    cmdclass={
        'install': CustomInstallCommand,
    },
)
