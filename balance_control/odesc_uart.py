import time
import serial
import odrive.enums

from RPi import GPIO  # Import GPIO module

# GPIO setup for resetting ODrive
GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.OUT)

class ODESC:
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    ERROR_DICT = {k: v for k, v in odrive.enums.__dict__.items() if k.startswith("AXIS_ERROR_")}

    def __init__(self, port='/dev/ttyAMA1', left_axis=1, right_axis=0, dir_left=1, dir_right=1):
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

    def get_errors(self, axis):
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
        self.send_command(f'w axis{axis}.requested_state 1')

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

def reset_odrive():
    GPIO.output(5, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(5, GPIO.HIGH)
    print("ODrive reset attempted")

if __name__ == '__main__':
    # Initialize with directions for left and right motors
    motor_controller = ODESC('/dev/ttyAMA1', left_axis=1, right_axis=0, dir_left=1, dir_right=-1)

    motor_controller.start_left()
    motor_controller.start_right()

    # motor_controller.enable_watchdog_left()
    # motor_controller.enable_watchdog_right()

    try:
        # reset_odrive()
        motor_controller.enable_torque_mode_left()
        motor_controller.enable_torque_mode_right()

        # motor_controller.set_speed_rpm_left(rpm=50)
        motor_controller.set_torque_nm_left(nm=1.0)
        # time.sleep(3)
        # motor_controller.set_speed_rpm_right(rpm=100)
        motor_controller.set_torque_nm_right(nm=1.0)
        # time.sleep(3)

        while True:
            # time.sleep(1.0)
            # print(f"Motor 1 speed: {motor_controller.get_speed_rpm_left()}")
            # print(f"Motor 2 speed: {motor_controller.get_speed_rpm_right()}")
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
