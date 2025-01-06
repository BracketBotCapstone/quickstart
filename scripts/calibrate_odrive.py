import time

import fibre
import fibre.serial_transport
import odrive
from odrive.enums import *

# Error dictionaries for different components
ENCODER_ERRORS = {name: value for name, value in vars(odrive.enums).items() if name.startswith('ENCODER_ERROR')}
CONTROLLER_ERRORS = {name: value for name, value in vars(odrive.enums).items() if name.startswith('CONTROLLER_ERROR')}
MOTOR_ERRORS = {name: value for name, value in vars(odrive.enums).items() if name.startswith('MOTOR_ERROR')}
AXIS_ERRORS = {name: value for name, value in vars(odrive.enums).items() if name.startswith('AXIS_ERROR')}

def wait_for_idle(axis):
    """
    Wait until the axis reaches the idle state.
    
    :param axis: The axis object to monitor.
    """
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

def connect_odrive():
    """
    Connect to the ODrive device.
    
    :return: The connected ODrive object.
    :raises Exception: If the ODrive connection times out.
    """
    print("Connecting to ODrive...")
    import odrive
    odrv = odrive.find_any(path='serial:/dev/ttyAMA1', timeout=5)
    if odrv is None:
        raise Exception('ODrive timed out')
    return odrv

def save_and_reboot(odrv):
    """
    Save the current configuration and reboot the ODrive.
    
    :param odrv: The ODrive object to save and reboot.
    :return: The reconnected ODrive object.
    """
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
        print(f"Error saving configuration: {e!s}")
        return None
    
    time.sleep(1)
    return connect_odrive()

def print_errors(error_type, error_value):
    """
    Print errors for a given component type and error value.
    
    :param error_type: The type of component (e.g., 'axis', 'motor', 'encoder').
    :param error_value: The error value to interpret.
    """
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

def calibrate_axis(odrv0, axis):
    """
    Calibrate a single axis of the ODrive.
    
    :param odrv0: The ODrive object.
    :param axis: The axis number to calibrate.
    :return: Tuple of the ODrive object and a boolean indicating success.
    """
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
    getattr(odrv0, f'axis{axis}').config.watchdog_timeout = 0.5
    getattr(odrv0, f'axis{axis}').config.enable_watchdog = False
    getattr(odrv0, f'axis{axis}').motor.config.calibration_current = 5
    getattr(odrv0, f'axis{axis}').motor.config.pole_pairs = 15
    getattr(odrv0, f'axis{axis}').motor.config.resistance_calib_max_voltage = 4
    getattr(odrv0, f'axis{axis}').motor.config.requested_current_range = 25 # Requires config save and reboot
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

# Main script
print("Finding an ODrive...")
try:
    odrv0 = connect_odrive()
except:
    fibre.serial_transport.DEFAULT_BAUDRATE = 115200
    odrv0 = connect_odrive()
print("Found ODrive.")

# User confirmation for safety
confirmation = input("Make sure the wheels are lifted on the ground before proceeding.\n\nAre the wheels off the ground? [yes/no] ").lower()
if confirmation.lower() != 'yes':
    print('Rerun this script once the wheels have been lifted off the ground.')
    exit(0)
print()

# Calibrate each axis
for axis in [0, 1]:
    odrv0, success = calibrate_axis(odrv0, axis)
    if success:
        print('\033[92m' + f"Axis {axis} calibration completed successfully." + '\033[0m')
        print()
    else:
        print('\033[91m' + f"Axis {axis} calibration failed." + '\033[0m')
        print('\nPlease fix the issue with this axis before rerunning this script.')
        exit(0)

# Set the UART baudrate to a higher level
try:
    odrv0.config.uart_baudrate = 4 * 115200
except Exception:
    # Close the hanging connection
    odrv0.__channel__.serial_device.close()

fibre.serial_transport.DEFAULT_BAUDRATE = 4 * 115200

# Final save and reboot
odrv0 = save_and_reboot(odrv0)

print('\033[94m' + "\nODrive setup complete." + '\033[0m')