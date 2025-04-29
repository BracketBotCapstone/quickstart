import time
import serial
import odrive.enums
import os
import json

# This is l-gpio
# from RPi import GPIO  # Import GPIO module

# # GPIO setup for resetting ODrive
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(5, GPIO.OUT)

class ODriveUART:
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    ERROR_DICT = {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("AXIS_ERROR_")}

    SERIAL_PORT = '/dev/ttyAMA1'
    _bus = serial.Serial(
        port=SERIAL_PORT,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )

    def __init__(self, port='/dev/ttyAMA1', left_axis=0, right_axis=1, dir_left=None, dir_right=None):
        # If motor directions not supplied, attempt to load them from motor_dir.json.
        if dir_left is None or dir_right is None:
            cfg_path = os.path.join(os.path.dirname(__file__), "motor_dir.json")
            try:
                with open(cfg_path, "r") as f:
                    cfg = json.load(f)
                    dir_left = cfg.get("left", 1)
                    dir_right = cfg.get("right", 1)
            except FileNotFoundError:
                print(f"[ODriveUART] motor_dir.json not found at {cfg_path}. Using default directions (1,1).")
                dir_left = dir_left or 1
                dir_right = dir_right or 1
            except Exception as exc:
                print(f"[ODriveUART] Error reading motor_dir.json ({exc}). Using default directions (1,1).")
                dir_left = dir_left or 1
                dir_right = dir_right or 1
        self.bus = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        self.left_axis = left_axis
        self.right_axis = right_axis
        self.dir_left = dir_left
        self.dir_right = dir_right

        # Clear the ASCII UART buffer
        self.bus.reset_input_buffer()
        self.bus.reset_output_buffer()

    def send_command(self, command: str):
        self.bus.reset_input_buffer()
        self.bus.write(f"{command}\n".encode())
        # Wait for the response if it's a read command
        if command.startswith('r') or command.startswith('f'):
            # Read until a newline character is encountered
            response = self.bus.readline().decode('ascii').strip()
            # If the response is empty, print a debug message
            if response == '':
                print(f"No response received for command: {command}")
            return response

    def get_errors_left(self):
        return self.get_errors(self.left_axis)

    def get_errors_right(self):
        return self.get_errors(self.right_axis)

    def has_errors(self):
        for axis in [0,1]:
            error_response = self.send_command(f'r axis{axis}.error')
            try:
                cleaned_response = ''.join(c for c in error_response if c.isdigit())
                error_code = int(cleaned_response)
            except ValueError:
                print(f"Unexpected error response format: {error_response}")
                return True
            if error_code != 0:
                return True
        return False    
    
    def dump_errors(self):
        error_sources = [
            "axis0","axis0.encoder", "axis0.controller", "axis0.motor",
            "axis1","axis1.encoder", "axis1.controller", "axis1.motor"
        ]
        print('======= ODrive Errors =======')
        for src in error_sources:
            error_response = self.send_command(f'r {src}.error')
            try:
                cleaned_response = ''.join(c for c in error_response if c.isdigit())
                error_code = int(cleaned_response)
            except ValueError:
                print(f"Unexpected error response format: {error_response}")
                continue

            if error_code == 0:
                print(src+'.error=0x0: \033[92mNone\033[0m')
                continue

            error_prefix = f"{src.split('.')[-1].strip('01').upper()}_ERROR"
            error_dict = {name: value for name, value in vars(odrive.enums).items() if name.startswith(error_prefix)}
            error_string = ""
            for error_name, code in error_dict.items():
                if error_code & code:
                    error_string += f"{error_name.replace(error_prefix + '_', '').lower().replace('_', ' ')}, "
            error_string = error_string.rstrip(", ")
            print(f"{src}.error={hex(error_code)}: \033[91m{error_string}\033[0m")
        print('=============================')

    def enable_torque_mode_left(self):
        self.enable_torque_mode(self.left_axis)

    def enable_torque_mode_right(self):
        self.enable_torque_mode(self.right_axis)

    def enable_torque_mode(self, axis):
        self.send_command(f'w axis{axis}.controller.config.control_mode 1')
        self.send_command(f'w axis{axis}.controller.config.input_mode 1')
        print(f"Axis {axis} set to torque control mode")

    def enable_velocity_mode_left(self):
        self.enable_velocity_mode(self.left_axis)

    def enable_velocity_mode_right(self):
        self.enable_velocity_mode(self.right_axis)

    def enable_velocity_mode(self, axis):
        self.send_command(f'w axis{axis}.controller.config.control_mode 2')
        self.send_command(f'w axis{axis}.controller.config.input_mode 1')
        print(f"Axis {axis} set to velocity control mode")

    def enable_velocity_ramp_mode_left(self):
        self.enable_velocity_ramp_mode(self.left_axis)

    def enable_velocity_ramp_mode_right(self):
        self.enable_velocity_ramp_mode(self.right_axis)

    def enable_velocity_ramp_mode(self, axis):
        self.send_command(f'w axis{axis}.controller.config.control_mode 2')
        self.send_command(f'w axis{axis}.controller.config.input_mode 2')
        print(f"Axis {axis} set to ramped velocity control mode")

    def set_velocity_ramp_rate_left(self, ramp_rate):
        self.set_velocity_ramp_rate(self.left_axis, ramp_rate)

    def set_velocity_ramp_rate_right(self, ramp_rate):
        self.set_velocity_ramp_rate(self.right_axis, ramp_rate)

    def set_velocity_ramp_rate(self, axis, ramp_rate):
        self.send_command(f'w axis{axis}.controller.config.vel_ramp_rate {ramp_rate:.4f}')
        print(f"Axis {axis} velocity ramp rate set to {ramp_rate:.4f} turns/s^2")

    def start_left(self):
        self.start(self.left_axis)

    def start_right(self):
        self.start(self.right_axis)

    def start(self, axis):
        self.send_command(f'w axis{axis}.requested_state 8')

    def set_speed_rpm_left(self, rpm):
        self.set_speed_rpm(self.left_axis, rpm, self.dir_left)

    def set_speed_rpm_right(self, rpm):
        self.set_speed_rpm(self.right_axis, rpm, self.dir_right)

    def set_speed_rpm(self, axis, rpm, direction):
        rps = rpm / 60
        self.send_command(f'w axis{axis}.controller.input_vel {rps * direction:.4f}')
        
    def set_speed_mps_left(self, mps):
        WHEEL_DIAMETER_MM = 165
        rps = mps / (WHEEL_DIAMETER_MM * 0.001 * 3.14159)
        self.send_command(f'w axis{self.left_axis}.controller.input_vel {rps * self.dir_left:.4f}')
        
    def set_speed_mps_right(self, mps):
        WHEEL_DIAMETER_MM = 165
        rps = mps / (WHEEL_DIAMETER_MM * 0.001 * 3.14159)
        self.send_command(f'w axis{self.right_axis}.controller.input_vel {rps * self.dir_right:.4f}')

    def set_torque_nm_left(self, nm):
        self.set_torque_nm(self.left_axis, nm, self.dir_left)

    def set_torque_nm_right(self, nm):
        self.set_torque_nm(self.right_axis, nm, self.dir_right)

    def set_torque_nm(self, axis, nm, direction):
        torque_bias = 0.05 # Small torque bias in Nm
        adjusted_torque = nm * direction + (torque_bias * direction * (1 if nm >= 0 else -1))
        # self.send_command(f'w axis{axis}.controller.input_torque {adjusted_torque:.4f}')
        self.send_command(f'c {axis} {adjusted_torque:.4f}')
        self.send_command(f'u {axis}')

    def get_speed_rpm_left(self):
        return self.get_speed_rpm(self.left_axis, self.dir_left)

    def get_speed_rpm_right(self):
        return self.get_speed_rpm(self.right_axis, self.dir_right)

    def get_speed_rpm(self, axis, direction):
        response = self.send_command(f'r axis{axis}.encoder.vel_estimate')
        return float(response) * direction * 60

    def get_position_turns_left(self):
        return self.get_position_turns(self.left_axis, self.dir_left)

    def get_position_turns_right(self):
        return self.get_position_turns(self.right_axis, self.dir_right)

    def get_position_turns(self, axis, direction):
        response = self.send_command(f'r axis{axis}.encoder.pos_estimate')
        return float(response) * direction

    def get_pos_vel_left(self):
        return self.get_pos_vel(self.left_axis, self.dir_left)

    def get_pos_vel_right(self):
        return self.get_pos_vel(self.right_axis, self.dir_right)

    def get_pos_vel(self, axis, direction):
        pos, vel = self.send_command(f'f {axis}').split(' ')
        return float(pos) * direction, float(vel) * direction * 60

    def stop_left(self):
        self.stop(self.left_axis)

    def stop_right(self):
        self.stop(self.right_axis)

    def stop(self, axis):
        self.send_command(f'w axis{axis}.controller.input_vel 0')
        self.send_command(f'w axis{axis}.controller.input_torque 0')
        # Going at high torque and changing to idle causes overcurrent
        # self.send_command(f'w axis{axis}.requested_state 1')

    def check_errors_left(self):
        return self.check_errors(self.left_axis)

    def check_errors_right(self):
        return self.check_errors(self.right_axis)

    def check_errors(self, axis):
        response = self.send_command(f'r axis{axis}.error')
        try:
            # Remove any non-numeric characters (like 'd' for decimal)
            cleaned_response = ''.join(c for c in response if c.isdigit())
            return int(cleaned_response) != 0
        except ValueError:
            print(f"Unexpected response format: {response}")
            return True # Assume there's an error if we can't parse the response

    def clear_errors_left(self):
        self.clear_errors(self.left_axis)

    def clear_errors_right(self):
        self.clear_errors(self.right_axis)

    def clear_errors(self, axis):
        self.send_command(f'w axis{axis}.error 0')
        self.send_command(f'w axis{axis}.requested_state {self.AXIS_STATE_CLOSED_LOOP_CONTROL}')

    def enable_watchdog_left(self):
        self.enable_watchdog(self.left_axis)

    def enable_watchdog_right(self):
        self.enable_watchdog(self.right_axis)

    def enable_watchdog(self, axis):
        self.send_command(f'w axis{axis}.config.enable_watchdog 1')

    def disable_watchdog_left(self):
        self.disable_watchdog(self.left_axis)

    def disable_watchdog_right(self):
        self.disable_watchdog(self.right_axis)

    def disable_watchdog(self, axis):
        self.send_command(f'w axis{axis}.config.enable_watchdog 0')
        
    def set_watchdog_timeout(self, timeout):
        self.send_command(f'w axis0.config.watchdog_timeout {timeout}')
        self.send_command(f'w axis1.config.watchdog_timeout {timeout}')

    def get_bus_voltage(self):
        response = self.send_command('r vbus_voltage')
        try:
            voltage = round(float(response), 1)
            if voltage < 17.5:
                # Red
                return "\033[31m" + str(voltage) + "V\033[0m"
            elif voltage < 18.5:
                # Orange
                return "\033[33m" + str(voltage) + "V\033[0m"
            elif voltage < 19.5:
                # Yellow
                return "\033[93m" + str(voltage) + "V\033[0m"
            else:
                # Green
                return "\033[32m" + str(voltage) + "V\033[0m"
        except ValueError:
            return None

def reset_odrive():
    # GPIO.output(5, GPIO.LOW)
    # time.sleep(0.1)
    # GPIO.output(5, GPIO.HIGH)
    print("ODrive reset attempted")

# (The previous built-in demo that spun the motors has been removed to avoid
# unexpected side-effects when this module is imported.  For a standalone test
# see `quickstart/examples/example_wasd.py` or write your own script.)

# ---------------------------------------------------------------------------
# Minimal CLI entry-point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    """Lightweight command-line helper.

    Usage examples:
        python -m quickstart.lib.odrive_uart                # prints Vbus
    """
    import argparse
    import sys

    parser = argparse.ArgumentParser(description='ODrive UART helper')
    parser.add_argument('--port', default='/dev/ttyAMA1', help='Serial port of ODrive UART-A')
    parser.add_argument('--raw', action='store_true', help='Print raw bus voltage without formatting')
    args = parser.parse_args()

    try:
        voltage = ODriveUART().get_bus_voltage()
    except Exception:
        # Swallow any low-level error and print placeholder so prompt doesn't break
        print('??')
        sys.exit(0)

    if voltage is None:
        print('??')
    elif args.raw:
        print(voltage)
    else:
        print(f'{voltage}')
