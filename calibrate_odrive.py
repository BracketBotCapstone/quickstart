import odrive
from odrive.enums import *
import time

# Helper function to wait until the axis reaches idle state
def wait_for_idle(axis):
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

# Helper function to reconnect to the ODrive after reboot
def reconnect_odrive():
    print("Waiting for ODrive to reboot...")
    time.sleep(5)
    print("Reconnecting to ODrive...")
    return odrive.find_any()

# Function to calibrate a single axis
def calibrate_axis(odrv0, axis):
    print(f"Calibrating axis{axis}...")
    
    # Clear errors
    print("Clearing errors...")
    getattr(odrv0, f'axis{axis}').clear_errors()
    
    # Wait for a moment to ensure errors are cleared
    time.sleep(1)
    
    # Print current errors to verify they're cleared
    print("Current errors:")
    print(getattr(odrv0, f'axis{axis}').error)
    print(getattr(odrv0, f'axis{axis}').motor.error)
    print(getattr(odrv0, f'axis{axis}').encoder.error)
    
    # -------- ODrive Configuration --------
    print("Configuring ODrive...")
    getattr(odrv0, f'axis{axis}').config.watchdog_timeout=0.5
    getattr(odrv0, f'axis{axis}').config.enable_watchdog=True
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
    
    # -------- Motor Calibration --------
    print("Starting motor calibration...")

    getattr(odrv0, f'axis{axis}').requested_state = AXIS_STATE_MOTOR_CALIBRATION
    wait_for_idle(getattr(odrv0, f'axis{axis}'))
    
    # Check for errors
    if getattr(odrv0, f'axis{axis}').motor.error != 0:
        print(f"Motor calibration failed with error: {hex(getattr(odrv0, f'axis{axis}').motor.error)}")
        return False
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
    if getattr(odrv0, f'axis{axis}').encoder.error != 0:
        print(f"Encoder calibration failed with error: {hex(getattr(odrv0, f'axis{axis}').encoder.error)}")
        return False
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

    return True

# Main script
print("Finding an ODrive...")
odrv0 = odrive.find_any()
print("Found ODrive.")

# Ask user which axis to calibrate
while True:
    axis_choice = input("Which axis do you want to calibrate? (0, 1, or both): ").lower()
    if axis_choice in ['0', '1', 'both']:
        break
    else:
        print("Invalid input. Please enter 0, 1, or both.")

axes_to_calibrate = [0, 1] if axis_choice == 'both' else [int(axis_choice)]

for axis in axes_to_calibrate:
    if calibrate_axis(odrv0, axis):
        print(f"Axis {axis} calibration completed successfully.")
    else:
        print(f"Axis {axis} calibration failed.")

# Set the UART baudrate to a higher level
odrv0.config.uart_baudrate = 4*115200

# Final save and reboot
print("Saving configuration and rebooting ODrive...")
odrv0.save_configuration()

# Handle the expected exception during reboot
try:
    odrv0.reboot()
except:
    pass  # The exception is expected due to the USB connection loss

# Reconnect to ODrive after reboot
odrv0 = reconnect_odrive()

print("ODrive setup complete.")
