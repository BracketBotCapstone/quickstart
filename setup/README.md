# Quickstart Robot Setup Guide

Welcome! So you've just received your robot. Follow these steps to get it up and running.

For more detailed documentation, visit our comprehensive guide at https://docs.bracket.bot/

## 1. Clone the Repository

First, you need to clone this repo onto your Raspberry Pi.

```bash
# Navigate to your home directory
cd ~

# Clone the repository to user home (/home/<USER>/quickstart OR ~/quickstart OR $HOME/quickstart, these are all equivalent)
git clone https://github.com/BracketBotCapstone/quickstart.git ~/quickstart
```

This will download the `quickstart` code into your home directory.

## 2. Run the OS Setup Script

Next, navigate into the setup directory and run the main setup script. This will configure the operating system, install dependencies, and set up necessary services.

```bash
cd ~/quickstart/setup
bash setup_os.sh
```

## 3. Calibrate the Drive System

Once the OS setup is complete, you need to calibrate the motor controller and determine the motor directions.

**IMPORTANT:** Before running the next command:
*   Place the robot on the floor in a **flat, open area**.
*   Ensure there is at least **1 meter of clear space** around the robot in all directions.
*   The robot **will move** during this process. **Do not touch it** while it is calibrating.

Run the calibration script:

```bash
cd ~/quickstart/setup
python3 calibrate_drive.py
```

Follow the prompts in the script. It will first calibrate the ODrive motor controller and then test the motor directions.

## 4. Test Drive with WASD Example

If the `calibrate_drive.py` script completes successfully, congratulations! Your robot should be ready for basic control.

Navigate to the examples directory and run the WASD control script:

```bash
cd ~/quickstart/examples
python3 example_wasd.py
```

You should now be able to drive the robot using the W, A, S, and D keys on your keyboard (ensure the terminal window running the script is focused).

---

If you encounter any issues join our [Discord](https://discord.gg/RXRgJyAq93)!