import time
import serial
import odrive.enums


class ODriveUART:
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    ERROR_DICT = {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("AXIS_ERROR_")}

    def __init__(self, port, axis_num=0, dir=1):
        self.bus = serial.Serial(
            port=port,
            # baudrate=115200,
            baudrate=460800,
            # baudrate=921600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        self.axis_num = axis_num
        self.dir = dir

        # Clear the ASCII UART buffer
        self.bus.reset_input_buffer()
        self.bus.reset_output_buffer()

    def send_command(self, command: str):
        self.bus.reset_input_buffer()
        self.bus.write(f"{command}\n".encode())
        # Wait for the response if it's a read command
        if command.startswith('r'):
            # Read until a newline character is encountered
            response = self.bus.readline().decode('ascii').strip()
            # If the response is empty, print a debug message
            if response == '':
                print(f"No response received for command: {command}")
            return response

    def get_errors(self):
        error_code = -1
        error_name = 'Unknown error'
        # Get error code
        error_response = self.send_command(f'r axis{self.axis_num}.error')
        try:
            cleaned_response = ''.join(c for c in error_response if c.isdigit())
            error_code = int(cleaned_response)
            error_name = self.ERROR_DICT.get(error_code,error_name)
        except ValueError:
            print(f"Unexpected error response format: {error_response}")
            
        return error_code, error_name
        
    def enable_torque_mode(self):
        self.send_command(f'w axis{self.axis_num}.controller.config.control_mode 1')
        self.send_command(f'w axis{self.axis_num}.controller.config.input_mode 1')
        print(f"Axis {self.axis_num} set to torque control mode")

    def enable_velocity_mode(self):
        self.send_command(f'w axis{self.axis_num}.controller.config.control_mode 2')
        self.send_command(f'w axis{self.axis_num}.controller.config.input_mode 1')
        print(f"Axis {self.axis_num} set to velocity control mode")

    def start(self):
        self.send_command(f'w axis{self.axis_num}.requested_state 8')

    def set_speed_rpm(self, rpm):
        rps = rpm / 60
        self.send_command(f'w axis{self.axis_num}.controller.input_vel {rps * self.dir:.4f}')
        self.send_command(f'u {self.axis_num}')

    def set_torque_nm(self, nm):
        torque_bias = 0.05  # Small torque bias in Nm
        adjusted_torque = nm * self.dir + (torque_bias * self.dir * (1 if nm >= 0 else -1))
        self.send_command(f'w axis{self.axis_num}.controller.input_torque {adjusted_torque:.4f}')
        self.send_command(f'u {self.axis_num}')

    def get_speed_rpm(self):
        response = self.send_command(f'r axis{self.axis_num}.encoder.vel_estimate')
        return float(response) * self.dir * 60

    def get_position_turns(self):
        response = self.send_command(f'r axis{self.axis_num}.encoder.pos_estimate')
        return float(response) * self.dir

    def stop(self):
        self.send_command(f'w axis{self.axis_num}.controller.input_vel 0')
        self.send_command(f'w axis{self.axis_num}.controller.input_torque 0')
        self.send_command(f'w axis{self.axis_num}.requested_state 1')

    def check_errors(self):
        response = self.send_command(f'r axis{self.axis_num}.error')
        try:
            # Remove any non-numeric characters (like 'd' for decimal)
            cleaned_response = ''.join(c for c in response if c.isdigit())
            return int(cleaned_response) != 0
        except ValueError:
            print(f"Unexpected response format: {response}")
            return True  # Assume there's an error if we can't parse the response

    def clear_errors(self):
        self.send_command(f'w axis{self.axis_num}.error 0')
        self.send_command(f'w axis{self.axis_num}.requested_state {self.AXIS_STATE_CLOSED_LOOP_CONTROL}')

if __name__ == '__main__':
    motor1 = ODriveUART('/dev/ttyAMA1', axis_num=0, dir=-1)
    motor2 = ODriveUART('/dev/ttyAMA1', axis_num=1, dir=1)
    motor1.start()
    motor2.start()

    try:
        motor1.set_speed_rpm(20)
        time.sleep(3)
        motor2.set_speed_rpm(20)
        time.sleep(3)

        while True:
            # Check for errors and clear if necessary
            for motor in [motor1, motor2]:
                if motor.check_errors():
                    print(f"\nError detected on {'left' if motor == motor1 else 'right'} motor. Clearing...")
                    motor.clear_errors()
                    time.sleep(1)

    except Exception as e:
        print(e)
    finally:
        motor1.stop()
        motor2.stop()
