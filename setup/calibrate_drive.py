# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import time
import fibre
import fibre.serial_transport
import odrive
from odrive.enums import *
import json
import numpy as np
import math

from lib.odrive_uart import ODriveUART, reset_odrive
from lib.imu import FilteredMPU6050

# ANSI escape codes for colors
BLUE = '\033[94m'
YELLOW = '\033[93m'
GREEN = '\033[92m'
RED = '\033[91m'
BOLD = '\033[1m'
RESET = '\033[0m'

# ODrive calibration constants and helper functions
ENCODER_ERRORS = {name: value for name, value in vars(odrive.enums).items() if name.startswith('ENCODER_ERROR')}
CONTROLLER_ERRORS = {name: value for name, value in vars(odrive.enums).items() if name.startswith('CONTROLLER_ERROR')}
MOTOR_ERRORS = {name: value for name, value in vars(odrive.enums).items() if name.startswith('MOTOR_ERROR')}
AXIS_ERRORS = {name: value for name, value in vars(odrive.enums).items() if name.startswith('AXIS_ERROR')}


# Helper function to wait until the axis reaches idle state
def wait_for_idle(axis):
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

# Helper function to reconnect to the ODrive after reboot
def connect_odrive():
    print("Connecting to ODrive...")
    import odrive
    odrv = odrive.find_any(path='serial:/dev/ttyAMA1', timeout=15)
    if odrv is None:
        raise Exception('ODrive timed out')
    return odrv

def save_and_reboot(odrv):
    print("Saving configuration...")
    try:
        odrv.save_configuration()
        print("Configuration saved successfully.")
        
        print("Rebooting ODrive...")
        try:
            odrv.reboot()
        except:
            # Exception is expected as connection is lost during reboot
            # Close the hanging connection
            odrv.__channel__.serial_device.close()
            
    except Exception as e:
        print(f"Error saving configuration: {str(e)}")
        return None
    
    time.sleep(1)
    return connect_odrive()


def print_errors(error_type, error_value):
    """Print errors for a given component type and error value."""
    if error_value == 0:
        return
    error_dict = {name: value for name, value in vars(odrive.enums).items() 
                 if name.startswith(f'{error_type.upper()}_ERROR')}
    
    error_string = ""
    for error_name, error_code in error_dict.items():
        if error_value & error_code:
            error_string += f"{error_name.replace(f'{error_type.upper()}_ERROR_', '').lower().replace('_', ' ')}, "
    error_string = error_string.rstrip(", ")
    print(f"\033[91m{error_type.capitalize()} error {hex(error_value)}: {error_string}\033[0m")

# Function to calibrate a single axis
def calibrate_axis(odrv0, axis):
    print(f"Calibrating axis{axis}...")
    
    # Clear errors
    print("Checking errors...")
    getattr(odrv0, f'axis{axis}').clear_errors()
    
    # Wait for a moment to ensure errors are cleared
    time.sleep(1)
    
    # Print current errors to verify they're cleared
    axis_error = getattr(odrv0, f'axis{axis}').error
    motor_error = getattr(odrv0, f'axis{axis}').motor.error 
    encoder_error = getattr(odrv0, f'axis{axis}').encoder.error

    if axis_error or motor_error or encoder_error:
        print(f"Axis {axis} errors:")
        if axis_error:
            print_errors('axis', axis_error)
        if motor_error:
            print_errors('motor', motor_error)
        if encoder_error:
            print_errors('encoder', encoder_error)
        return odrv0, False
    
    # -------- ODrive Configuration --------
    print("Configuring ODrive...")
    getattr(odrv0, f'axis{axis}').config.watchdog_timeout=0.5
    getattr(odrv0, f'axis{axis}').config.enable_watchdog=False
    getattr(odrv0, f'axis{axis}').motor.config.calibration_current = 5
    getattr(odrv0, f'axis{axis}').motor.config.pole_pairs = 15
    getattr(odrv0, f'axis{axis}').motor.config.resistance_calib_max_voltage = 4
    getattr(odrv0, f'axis{axis}').motor.config.requested_current_range = 25 #Requires config save and reboot
    getattr(odrv0, f'axis{axis}').motor.config.current_control_bandwidth = 100
    getattr(odrv0, f'axis{axis}').motor.config.torque_constant = 8.27 / 16.0
    getattr(odrv0, f'axis{axis}').encoder.config.mode = ENCODER_MODE_HALL
    getattr(odrv0, f'axis{axis}').encoder.config.cpr = 90
    getattr(odrv0, f'axis{axis}').encoder.config.calib_scan_distance = 150
    getattr(odrv0, f'axis{axis}').encoder.config.bandwidth = 100
    getattr(odrv0, f'axis{axis}').controller.config.pos_gain = 1
    getattr(odrv0, f'axis{axis}').controller.config.vel_gain = 0.02 * getattr(odrv0, f'axis{axis}').motor.config.torque_constant * getattr(odrv0, f'axis{axis}').encoder.config.cpr
    getattr(odrv0, f'axis{axis}').controller.config.vel_integrator_gain = 0.1 * getattr(odrv0, f'axis{axis}').motor.config.torque_constant * getattr(odrv0, f'axis{axis}').encoder.config.cpr
    getattr(odrv0, f'axis{axis}').controller.config.vel_limit = 10
    getattr(odrv0, f'axis{axis}').controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    
    odrv0 = save_and_reboot(odrv0)

    # -------- Motor Calibration --------
    print("Starting motor calibration...")

    getattr(odrv0, f'axis{axis}').requested_state = AXIS_STATE_MOTOR_CALIBRATION
    wait_for_idle(getattr(odrv0, f'axis{axis}'))
    
    # Check for errors
    error = getattr(odrv0, f'axis{axis}').motor.error
    if error != 0:
        print_errors('motor', error)
        return odrv0, False
    else:
        print("Motor calibration successful.")
        # Validate phase resistance and inductance
        resistance = getattr(odrv0, f'axis{axis}').motor.config.phase_resistance
        inductance = getattr(odrv0, f'axis{axis}').motor.config.phase_inductance
        print(f"Measured phase resistance: {resistance} Ohms")
        print(f"Measured phase inductance: {inductance} H")
    
        if not (0.1 <= resistance <= 1.0):
            print("Warning: Phase resistance out of expected range!")
        if not (0.0001 <= inductance <= 0.005):
            print("Warning: Phase inductance out of expected range!")
    
        # Mark motor as pre-calibrated
        getattr(odrv0, f'axis{axis}').motor.config.pre_calibrated = True
    
    # -------- Skipping Hall Polarity Calibration --------
    print("Skipping Hall polarity calibration as per your request.")
    
    # -------- Encoder Offset Calibration --------
    print("Starting encoder offset calibration...")
    getattr(odrv0, f'axis{axis}').requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    wait_for_idle(getattr(odrv0, f'axis{axis}'))
    
    # Check for errors
    error = getattr(odrv0, f'axis{axis}').encoder.error
    if error != 0:
        print_errors('encoder', error)
        return odrv0, False
    else:
        print("Encoder calibration successful.")
        # Validate phase offset float
        phase_offset_float = getattr(odrv0, f'axis{axis}').encoder.config.offset_float
        print(f"Phase offset float: {phase_offset_float}")
    
        if abs((phase_offset_float % 1) - 0.5) > 0.1:
            print("Warning: Phase offset float is out of expected range!")
    
        # Mark encoder as pre-calibrated
        getattr(odrv0, f'axis{axis}').encoder.config.pre_calibrated = True
    
    # -------- Test Motor Control --------
    print("Testing motor control...")
    
    # Enter closed-loop control
    getattr(odrv0, f'axis{axis}').requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(1)  # Wait for state to settle
    
    # Command a velocity
    print("Spinning motor at 0.5 turns/sec...")
    getattr(odrv0, f'axis{axis}').controller.input_vel = 0.5
    time.sleep(2)
    
    # Stop the motor
    print("Stopping motor...")
    getattr(odrv0, f'axis{axis}').controller.input_vel = 0
    time.sleep(1)
    
    # Switch back to idle
    getattr(odrv0, f'axis{axis}').requested_state = AXIS_STATE_IDLE
    
    # -------- Automatic Startup Configuration --------
    print("Configuring automatic startup...")
    
    # Set axis to start in closed-loop control on startup
    getattr(odrv0, f'axis{axis}').config.startup_closed_loop_control = True

    return odrv0, True

def test_motor_direction():
    # Initialize IMU
    imu = FilteredMPU6050()
    imu.calibrate()
    
    # Reset ODrive before initializing motors
    reset_odrive()
    time.sleep(3)  # Wait for ODrive to reset
    
    motor_controller = ODriveUART(port='/dev/ttyAMA1', left_axis=0, right_axis=1, dir_left=1, dir_right=1)
    directions = {'left': 1, 'right': 1}

    angle_threshold = 5.0  # degrees
    max_spin_duration = 5.0  # seconds

    for name in ['left', 'right']:
        time.sleep(1)
        print(f"\nTesting {name} motor...")

        # Start motor in velocity mode and clear errors
        if name == 'left':
            motor_controller.start_left()
            motor_controller.enable_velocity_mode_left()
            if motor_controller.check_errors_left():
                print("Clearing left motor errors...")
                motor_controller.clear_errors_left()
        else:
            motor_controller.start_right()
            motor_controller.enable_velocity_mode_right()
            if motor_controller.check_errors_right():
                print("Clearing right motor errors...")
                motor_controller.clear_errors_right()

        # Spin motor (e.g., at 30 RPM)
        if name == 'left':
            motor_controller.set_speed_rpm_left(30)
        else:
            motor_controller.set_speed_rpm_right(30)

        # Get initial yaw
        _, _, initial_yaw = imu.get_orientation()
        start_time = time.time()

        # Wait until we exceed angle_threshold or hit max_spin_duration
        while True:
            time.sleep(0.01)
            _, _, current_yaw = imu.get_orientation()
            angle_diff = current_yaw - initial_yaw

            if abs(angle_diff) >= angle_threshold:
                break
            if (time.time() - start_time) > max_spin_duration:
                print("Reached max spin duration without hitting angle threshold.")
                break

        # Stop the motor
        if name == 'left':
            motor_controller.stop_left()
        else:
            motor_controller.stop_right()

        print(f"Yaw difference before stopping: {angle_diff:.2f} deg")

        # Determine final direction based on sign of yaw difference
        if abs(angle_diff) < angle_threshold:
            print("Angle change too small; defaulting to forward (+1).")
            directions[name] = 1
        else:
            if name == 'left':
                directions['left'] = -1 if angle_diff > 0 else 1
            else:
                directions['right'] = 1 if angle_diff > 0 else -1

        time.sleep(0.5)

    # Save direction results
    with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'w') as f:
        json.dump(directions, f)

    print("\nDirection test complete!")
    print(f"Left direction: {directions['left']}, Right direction: {directions['right']}")
    print(f"Results saved to ~/quickstart/lib/motor_dir.json: {directions}")

def calibrate_odrive():
    print("Finding an ODrive...")
    odrv0 = connect_odrive()
    print("Found ODrive.")

    # ASCII art for clear space warning
    print(r"""
{YELLOW}
          1m radius
        _________________
      /         ^         \
     /          |          \
    |          1m           |
    |           |           |
    | <--1m-->{{BOT}}<--1m--> |
    |           |           |
    |          1m           |
     \          |          /
      \_________v_________/
      
{RESET}
    """.format(YELLOW=YELLOW, RESET=RESET))

    print(f"{BOLD}WARNING:{RESET} The robot needs clear space to move during calibration.")
    confirmation = input(f"{BLUE}Ensure the robot has at least 1 meter of clear space around it.\nIs the area clear? [yes/no]: {RESET}").lower()
    if confirmation.lower() != 'yes':
        print(f'{YELLOW}Please ensure the area is clear and rerun the script.{RESET}')
        return False
    print()

    for axis in [0,1]:
        odrv0, success = calibrate_axis(odrv0, axis)
        if success:
            print('\033[92m' + f"Axis {axis} calibration completed successfully." + '\033[0m')
            print()
        else:
            print('\033[91m' + f"Axis {axis} calibration failed." + '\033[0m')
            print('\nPlease fix the issue with this axis before rerunning this script.')
            return False

    odrv0 = save_and_reboot(odrv0)

    print('\033[94m' + "\nODrive setup complete." + '\033[0m')
    return True

if __name__ == '__main__':
    try:
        print("\n\033[1;33mDrive Calibration\033[0m")
        print("\nThis script will run two calibration steps:")
        print("1. ODrive motor calibration - requires robot to be on a stand with wheels free to spin")
        print("2. Motor direction calibration - requires robot to be on the ground with some open space")
        
        # First calibration - ODrive
        print("\n\033[1;36mStep 1: ODrive Motor Calibration\033[0m")
        if not calibrate_odrive():
            sys.exit(1)
            
        # Second calibration - Motor direction
        print("\n\033[1;36mStep 2: Motor Direction Calibration\033[0m")
        # Removed confirmation prompt - assuming robot is ready
        # confirmation = input("Place the robot on the ground with space to move.\nIs the robot on the ground with space to move? [yes/no] ").lower()
        # if confirmation.lower() != 'yes':
        #     print('Rerun this script once the robot is on the ground with space to move.')
        #     sys.exit(1)
            
        test_motor_direction()
        
        print("\n\033[1;32mDrive calibration complete!\033[0m")
        
    except Exception as e:
        print(f"\n\033[91mError occurred: {e}\033[0m")
        sys.exit(1)
