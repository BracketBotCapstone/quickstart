import time

import odrive.enums
import serial
from RPi import GPIO  # Import GPIO module

# GPIO setup for resetting ODrive
GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.OUT)

class ODriveUART:
    """
    A class to interface with ODrive motor controllers over UART.
    """

    AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    ERROR_DICT = {k: v for k, v in odrive.enums.__dict__.items() if k.startswith("AXIS_ERROR_")}

    def __init__(self, port='/dev/ttyAMA1', left_axis=1, right_axis=0, dir_left=1, dir_right=1):
        """
        Initialize the ODriveUART class with the specified parameters.
        """
        self.bus = serial.Serial(
            port=port,
            baudrate=460800,
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
        """
        Send a command to the ODrive and return the response if applicable.
        """
        self.bus.reset_input_buffer()
        self.bus.write(f"{command}\n".encode())
        if command.startswith('r') or command.startswith('f'):
            response = self.bus.readline().decode('ascii').strip()
            if response == '':
                print(f"No response received for command: {command}")
            return response
    
    def get_errors_left(self):
        """
        Get errors for the left axis.
        """
        return self.get_errors(self.left_axis)

    def get_errors_right(self):
        """
        Get errors for the right axis.
        """
        return self.get_errors(self.right_axis)

    def get_errors(self, axis):
        """
        Get errors for the specified axis.
        """
        error_code = -1
        error_name = 'Unknown error'
        error_response = self.send_command(f'r axis{axis}.error')
        try:
            cleaned_response = ''.join(c for c in error_response if c.isdigit())
            error_code = int(cleaned_response)
            error_name = self.ERROR_DICT.get(error_code, error_name)
        except ValueError:
            print(f"Unexpected error response format: {error_response}")
        return error_code, error_name

    def has_errors(self):
        """
        Check if there are any errors on either axis.
        """
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
        """
        Print all errors for both axes and their components.
        """
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
        """
        Enable torque control mode for the left axis.
        """
        self.enable_torque_mode(self.left_axis)

    def enable_torque_mode_right(self):
        """
        Enable torque control mode for the right axis.
        """
        self.enable_torque_mode(self.right_axis)

    def enable_torque_mode(self, axis):
        """
        Enable torque control mode for the specified axis.
        """
        self.send_command(f'w axis{axis}.controller.config.control_mode 1')
        self.send_command(f'w axis{axis}.controller.config.input_mode 1')
        print(f"Axis {axis} set to torque control mode")

    def enable_velocity_mode_left(self):
        """
        Enable velocity control mode for the left axis.
        """
        self.enable_velocity_mode(self.left_axis)

    def enable_velocity_mode_right(self):
        """
        Enable velocity control mode for the right axis.
        """
        self.enable_velocity_mode(self.right_axis)

    def enable_velocity_mode(self, axis):
        """
        Enable velocity control mode for the specified axis.
        """
        self.send_command(f'w axis{axis}.controller.config.control_mode 2')
        self.send_command(f'w axis{axis}.controller.config.input_mode 1')
        print(f"Axis {axis} set to velocity control mode")

    def start_left(self):
        """
        Start the left axis.
        """
        self.start(self.left_axis)

    def start_right(self):
        """
        Start the right axis.
        """
        self.start(self.right_axis)

    def start(self, axis):
        """
        Start the specified axis.
        """
        self.send_command(f'w axis{axis}.requested_state 8')

    def set_speed_rpm_left(self, rpm):
        """
        Set the speed in RPM for the left axis.
        """
        self.set_speed_rpm(self.left_axis, rpm, self.dir_left)

    def set_speed_rpm_right(self, rpm):
        """
        Set the speed in RPM for the right axis.
        """
        self.set_speed_rpm(self.right_axis, rpm, self.dir_right)

    def set_speed_rpm(self, axis, rpm, direction):
        """
        Set the speed in RPM for the specified axis.
        """
        rps = rpm / 60
        self.send_command(f'w axis{axis}.controller.input_vel {rps * direction:.4f}')

    def set_torque_nm_left(self, nm):
        """
        Set the torque in Nm for the left axis.
        """
        self.set_torque_nm(self.left_axis, nm, self.dir_left)

    def set_torque_nm_right(self, nm):
        """
        Set the torque in Nm for the right axis.
        """
        self.set_torque_nm(self.right_axis, nm, self.dir_right)

    def set_torque_nm(self, axis, nm, direction):
        """
        Set the torque in Nm for the specified axis.
        """
        torque_bias = 0.05 # Small torque bias in Nm
        adjusted_torque = nm * direction + (torque_bias * direction * (1 if nm >= 0 else -1))
        self.send_command(f'c {axis} {adjusted_torque:.4f}')
        self.send_command(f'u {axis}')

    def get_speed_rpm_left(self):
        """
        Get the speed in RPM for the left axis.
        """
        return self.get_speed_rpm(self.left_axis, self.dir_left)

    def get_speed_rpm_right(self):
        """
        Get the speed in RPM for the right axis.
        """
        return self.get_speed_rpm(self.right_axis, self.dir_right)

    def get_speed_rpm(self, axis, direction):
        """
        Get the speed in RPM for the specified axis.
        """
        response = self.send_command(f'r axis{axis}.encoder.vel_estimate')
        return float(response) * direction * 60

    def get_position_turns_left(self):
        """
        Get the position in turns for the left axis.
        """
        return self.get_position_turns(self.left_axis, self.dir_left)

    def get_position_turns_right(self):
        """
        Get the position in turns for the right axis.
        """
        return self.get_position_turns(self.right_axis, self.dir_right)

    def get_position_turns(self, axis, direction):
        """
        Get the position in turns for the specified axis.
        """
        response = self.send_command(f'r axis{axis}.encoder.pos_estimate')
        return float(response) * direction
    
    def get_pos_vel_left(self):
        """
        Get the position and velocity for the left axis.
        """
        return self.get_pos_vel(self.left_axis, self.dir_left)

    def get_pos_vel_right(self):
        """
        Get the position and velocity for the right axis.
        """
        return self.get_pos_vel(self.right_axis, self.dir_right)

    def get_pos_vel(self, axis, direction):
        """
        Get the position and velocity for the specified axis.
        """
        pos, vel = self.send_command(f'f {axis}').split(' ')
        return float(pos) * direction, float(vel) * direction * 60

    def stop_left(self):
        """
        Stop the left axis.
        """
        self.stop(self.left_axis)

    def stop_right(self):
        """
        Stop the right axis.
        """
        self.stop(self.right_axis)

    def stop(self, axis):
        """
        Stop the specified axis.
        """
        self.send_command(f'w axis{axis}.controller.input_vel 0')
        self.send_command(f'w axis{axis}.controller.input_torque 0')

    def check_errors_left(self):
        """
        Check for errors on the left axis.
        """
        return self.check_errors(self.left_axis)

    def check_errors_right(self):
        """
        Check for errors on the right axis.
        """
        return self.check_errors(self.right_axis)

    def check_errors(self, axis):
        """
        Check for errors on the specified axis.
        """
        response = self.send_command(f'r axis{axis}.error')
        try:
            cleaned_response = ''.join(c for c in response if c.isdigit())
            return int(cleaned_response) != 0
        except ValueError:
            print(f"Unexpected response format: {response}")
            return True

    def clear_errors_left(self):
        """
        Clear errors on the left axis.
        """
        self.clear_errors(self.left_axis)

    def clear_errors_right(self):
        """
        Clear errors on the right axis.
        """
        self.clear_errors(self.right_axis)

    def clear_errors(self, axis):
        """
        Clear errors on the specified axis.
        """
        self.send_command(f'w axis{axis}.error 0')
        self.send_command(f'w axis{axis}.requested_state {self.AXIS_STATE_CLOSED_LOOP_CONTROL}')

    def enable_watchdog_left(self):
        """
        Enable the watchdog for the left axis.
        """
        self.enable_watchdog(self.left_axis)

    def enable_watchdog_right(self):
        """
        Enable the watchdog for the right axis.
        """
        self.enable_watchdog(self.right_axis)

    def enable_watchdog(self, axis):
        """
        Enable the watchdog for the specified axis.
        """
        self.send_command(f'w axis{axis}.config.enable_watchdog 1')

    def disable_watchdog_left(self):
        """
        Disable the watchdog for the left axis.
        """
        self.disable_watchdog(self.left_axis)

    def disable_watchdog_right(self):
        """
        Disable the watchdog for the right axis.
        """
        self.disable_watchdog(self.right_axis)

    def disable_watchdog(self, axis):
        """
        Disable the watchdog for the specified axis.
        """
        self.send_command(f'w axis{axis}.config.enable_watchdog 0')

def reset_odrive():
    """
    Reset the ODrive by toggling the GPIO pin.
    """
    GPIO.output(5, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(5, GPIO.HIGH)
    print("ODrive reset attempted")

if __name__ == '__main__':
    # Initialize with directions for left and right motors
    motor_controller = ODriveUART('/dev/ttyAMA1', left_axis=1, right_axis=0, dir_left=1, dir_right=-1)

    motor_controller.start_left()
    motor_controller.start_right()

    try:
        motor_controller.enable_torque_mode_left()
        motor_controller.enable_torque_mode_right()

        motor_controller.set_torque_nm_left(nm=1.0)
        motor_controller.set_torque_nm_right(nm=1.0)

        while True:
            if motor_controller.check_errors_left():
                print("\nError detected on left motor. Clearing...")
                motor_controller.clear_errors_left()
                time.sleep(1)
            if motor_controller.check_errors_right():
                print("\nError detected on right motor. Clearing...")
                motor_controller.clear_errors_right()
                time.sleep(1)

    except Exception as e:
        print(e)
    finally:
        motor_controller.stop_left()
        motor_controller.stop_right()
