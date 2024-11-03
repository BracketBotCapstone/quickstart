import time
import traceback
import numpy as np
import sys
import struct
import fcntl
import os
import select
import matplotlib.pyplot as plt
from imu import FilteredLSM6DS3
from odrive_uart import ODriveUART
from lqr import LQR_gains
import json
import signal
import subprocess

# This is l-gpio
from RPi import GPIO  # Import GPIO module


# GPIO setup for resetting ODrive
GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.OUT)

def reset_odrive():
    GPIO.output(5, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(5, GPIO.HIGH)
    print("ODrive reset attempted")


WHEEL_RADIUS = 6.5 * 0.0254 / 2  # 6.5 inches diameter converted to meters
WHEEL_DIST = 0.235  # 23.5 cm from wheel to center
YAW_RATE_TO_MOTOR_TORQUE = (WHEEL_DIST / WHEEL_RADIUS) * 0.1  # Assuming a rough torque constant
MOTOR_TURNS_TO_LINEAR_POS = WHEEL_RADIUS * 2 * np.pi
RPM_TO_METERS_PER_SECOND = WHEEL_RADIUS * 2 * np.pi / 60
MAX_TORQUE = 2.5  # Nm, adjust based on your motor's specifications
MAX_SPEED = 0.4  # m/s, set maximum linear speed

def balance():
    imu = FilteredLSM6DS3()
    
    K_balance = LQR_gains(
        #     [x, v, θ, ω, δ, δ']
        Q_diag=[100,100,1,10,10,1],
        #     [pitch, yaw]
        R_diag=[1, 1]
    )
    K_drive = LQR_gains(
        #     [x, v, θ, ω, δ, δ']
        Q_diag=[1,100,0.1,10,1,10],
        #     [pitch, yaw]
        R_diag=[0.1, 1]
    )
    print(K_balance.round(2))
    Dt = 1./400.

    prev_time = time.time()
    
    zero_angle = -0.75
    desired_yaw_rate = 0
    desired_vel = 0

    # For plotting
    times = []
    positions = []
    desired_positions = []
    velocities = []
    desired_velocities = []
    pitches = []
    desired_pitches = []
    pitch_rates = []
    desired_pitch_rates = []
    yaws = []
    desired_yaws = []
    yaw_rates = []
    desired_yaw_rates = []
    start_plot_time = time.time()

    reset_odrive()  # Reset ODrive before initializing motors
    time.sleep(1)   # Wait for ODrive to reset
    # Initialize motors
    # Read motor directions from saved file
    try:
        with open('motor_dir.json', 'r') as f:
            motor_dirs = json.load(f)
            left_dir = motor_dirs['left']
            right_dir = motor_dirs['right']
    except Exception as e:
        raise Exception("Error reading motor_dir.json")
    
    right_motor = ODriveUART('/dev/ttyAMA1', axis_num=0, dir=left_dir)
    left_motor = ODriveUART('/dev/ttyAMA1', axis_num=1, dir=right_dir)
    left_motor.enable_torque_mode()
    right_motor.enable_torque_mode()
    left_motor.start()
    right_motor.start()
    left_motor.set_speed_rpm(0)
    right_motor.set_speed_rpm(0)

    # Function to reset ODrive and re-initialize motors
    def reset_and_initialize_motors():
        nonlocal left_motor, right_motor
        reset_odrive()
        time.sleep(1)  # Give ODrive time to reset
        try:
            right_motor = ODriveUART('/dev/ttyAMA1', axis_num=0, dir=left_dir)
            left_motor = ODriveUART('/dev/ttyAMA1', axis_num=1, dir=right_dir)
            right_motor.clear_errors()
            left_motor.clear_errors()
            left_motor.enable_torque_mode()
            right_motor.enable_torque_mode()
            left_motor.start()
            right_motor.start()
            print("Motors re-initialized successfully.")
        except Exception as e:
            print(f"Error re-initializing motors: {e}")

    # Initialize joystick
    try:
        jsdev = open('/dev/input/js0', 'rb')
    except FileNotFoundError:
        print("Joystick not found at /dev/input/js0")
        sys.exit(1)

    # Get the controller name
    buf = bytearray(64)
    JSIOCGNAME = 0x80006a13 + (len(buf) << 16)
    fcntl.ioctl(jsdev, JSIOCGNAME, buf)
    controller_name = buf.rstrip(b'\x00').decode('utf-8')
    print(f"Joystick found: {controller_name}")

    # Set the joystick device to non-blocking mode
    fd = jsdev.fileno()
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

    # Initialize joystick variables
    left_stick_x = 0.0
    right_trigger = 0.0
    left_trigger = 0.0
    dpad_up = False
    dpad_down = False
    a_button_pressed = False

    # Get the absolute path of the script
    script_path = os.path.abspath(__file__)

    # Record starting position
    try:
        l_pos = left_motor.get_position_turns()
        r_pos = right_motor.get_position_turns()
        start_pos = (l_pos + r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS
        start_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*WHEEL_DIST)
    except Exception as e:
        print(f"Error reading initial motor positions: {e}")
        reset_and_initialize_motors()
        # Try again after reset
        try:
            l_pos = left_motor.get_position_turns()
            r_pos = right_motor.get_position_turns()
            start_pos = (l_pos + r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS
            start_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*WHEEL_DIST)
        except Exception as e:
            print(f"Error reading motor positions after reset: {e}")
            return
    imu.calibrate()
    start_time = time.time()
    cycle_count = 0
    prev_vel = 0  # Initialize previous velocity for moving average
    try:
        while True:
            loop_start_time = time.time()

            # Check for ODriveUART errors every 10 cycles
            if cycle_count % 20 == 0:
                try:
                    left_error_code, left_error = left_motor.get_errors()
                    right_error_code, right_error = right_motor.get_errors()
                    if left_error_code != 0 or right_error_code != 0:
                        print(f"Detected ODriveUART errors - Left: {left_error_code} ({left_error}), Right: {right_error_code} ({right_error})")
                        reset_and_initialize_motors()
                        continue
                except Exception as e:
                    print('Error checking motor errors:', e)
                    reset_and_initialize_motors()
                    continue

            cycle_count += 1

            # Read joystick input
            rlist, _, _ = select.select([jsdev], [], [], 0.0)
            if jsdev in rlist:
                evbuf = jsdev.read(8)
                while evbuf:
                    time_, value, type_, number = struct.unpack('IhBB', evbuf)
                    if type_ & 0x01:  # JS_EVENT_BUTTON
                        if number == 0:  # A button
                            a_button_pressed = value == 1
                    elif type_ & 0x02:  # JS_EVENT_AXIS
                        if number == 0:  # Left stick X-axis
                            left_stick_x = value / 32767.0  # Normalize to -1.0 to 1.0
                        elif number == 4:  # Right trigger
                            right_trigger = (value + 32768) / 65535.0  # Normalize to 0.0 to 1.0
                        elif number == 5:  # Left trigger
                            left_trigger = (value + 32768) / 65535.0  # Normalize to 0.0 to 1.0
                        elif number == 7:  # D-pad up/down
                            if value < 0:
                                dpad_up = True
                            elif value > 0:
                                dpad_down = True
                            else:
                                dpad_up = dpad_down = False
                    evbuf = jsdev.read(8)

            # Check if A button is pressed to stop the program
            if a_button_pressed:
                print("A button pressed. Resetting ODrive.")
                reset_odrive()
                reset_and_initialize_motors()
                a_button_pressed = False

            # Map joystick inputs to desired velocities and yaw rates
            LINEAR_SPEED = 0.3  # Adjusted to match MAX_SPEED
            ANGULAR_SPEED = 2.0
            ZERO_ANGLE_STEP = 0.025

            # Handle zero angle adjustment
            if dpad_up:
                zero_angle += ZERO_ANGLE_STEP
                print(f"Zero angle adjusted to: {zero_angle:.2f}")
                dpad_up = False
            elif dpad_down:
                zero_angle -= ZERO_ANGLE_STEP
                print(f"Zero angle adjusted to: {zero_angle:.2f}")
                dpad_down = False

            if right_trigger > 0.1:
                if right_trigger > 0.8:
                    if desired_vel != MAX_SPEED:
                        print('FAST FORWARD')
                    desired_vel = MAX_SPEED
                else:
                    if desired_vel != MAX_SPEED / 2:
                        print('FORWARD')
                    desired_vel = MAX_SPEED / 2
            elif left_trigger > 0.1:
                if left_trigger > 0.8:
                    if desired_vel != -MAX_SPEED:
                        print('FAST BACKWARD')
                    desired_vel = -MAX_SPEED
                else:
                    if desired_vel != -MAX_SPEED / 2:
                        print('BACKWARD')
                    desired_vel = -MAX_SPEED / 2
            else:
                if desired_vel != 0.0:
                    print('STOP')
                    start_pos = (l_pos + r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS + np.sign(desired_vel)*0.2
                desired_vel = 0.0

            if abs(left_stick_x) > 0.2:
                if left_stick_x > 0:
                    if desired_yaw_rate != ANGULAR_SPEED:
                        print("RIGHT")
                    desired_yaw_rate = ANGULAR_SPEED
                else:
                    if desired_yaw_rate != -ANGULAR_SPEED:
                        print("LEFT")
                    desired_yaw_rate = -ANGULAR_SPEED
            else:
                if desired_yaw_rate != 0.0:
                    print('STOP!')
                    start_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*WHEEL_DIST)
                desired_yaw_rate = 0.0

            current_pitch = imu.robot_angle() 
            current_yaw_rate = -imu.gyro_RAW[2]
            current_pitch_rate = imu.gyro_RAW[0]
            try:
                l_vel = left_motor.get_speed_rpm()
                l_pos = left_motor.get_position_turns()
                r_vel = right_motor.get_speed_rpm()
                r_pos = right_motor.get_position_turns() 
                current_vel = (l_vel + r_vel) / 2 * RPM_TO_METERS_PER_SECOND
                current_pos = (l_pos + r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS - start_pos
                # current_yaw_rate = (l_vel - r_vel) * RPM_TO_METERS_PER_SECOND / (2*WHEEL_DIST)
                current_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*WHEEL_DIST) - start_yaw
                             
            except Exception as e:
                print('Motor controller error:', e)
                reset_and_initialize_motors()
                continue

            # Get current velocity

            current_time = time.time()


            # Store data for plotting
            times.append(current_time - start_plot_time)
            positions.append(current_pos)
            desired_positions.append(0)  # Always want position at 0
            velocities.append(current_vel)
            desired_velocities.append(desired_vel)
            pitches.append(current_pitch)
            desired_pitches.append(zero_angle)
            pitch_rates.append(current_pitch_rate)
            desired_pitch_rates.append(0)  # Want pitch rate at 0
            yaws.append(current_yaw)
            desired_yaws.append(0)  # Want yaw at 0
            yaw_rates.append(current_yaw_rate)
            desired_yaw_rates.append(desired_yaw_rate)

            current_state = np.array([
                current_pos, current_vel, current_pitch*np.pi/180, current_pitch_rate, current_yaw, current_yaw_rate
            ])
            
            desired_vel = max(min(desired_vel, MAX_SPEED), -MAX_SPEED)
            desired_state = np.array([
                0, desired_vel, zero_angle*np.pi/180, 0, 0, desired_yaw_rate
            ])

            state_error = (current_state - desired_state).reshape((6,1))
            if desired_vel != 0.0:
                state_error[0,0] = 0
                C = -K_drive @ state_error
            if desired_yaw_rate != 0:
                state_error[4,0] = 0  # 0 yaw error
                C = -K_drive @ state_error
            if desired_vel == 0 and desired_yaw_rate == 0:
                C = -K_balance @ state_error
            D = np.array([[0.5,0.5],[0.5,-0.5]])
            left_torque, right_torque = (D @ C).squeeze()

            # Limit torques to MAX_TORQUE
            left_torque = np.clip(left_torque, -MAX_TORQUE, MAX_TORQUE)
            right_torque = np.clip(right_torque, -MAX_TORQUE, MAX_TORQUE)

            try:
                # Apply torques
                left_motor.set_torque_nm(left_torque)
                right_motor.set_torque_nm(right_torque)
                # pass
            except Exception as e:
                print('Motor controller error:', e)
                reset_and_initialize_motors()
                continue

            loop_end_time = time.time()
            loop_duration = loop_end_time - loop_start_time
            if cycle_count % 50 == 0:
                # print(imu.robot_angle(), imu.robot_angle_RAW())
                print(f"Loop time: {loop_duration:.6f} sec, x=[{current_pos:.2f} | 0], v=[{current_vel:.2f} | {desired_vel:.2f}], θ=[{current_pitch:.2f} | {zero_angle:.2f}], ω=[{current_pitch_rate:.2f} | 0], δ=[{current_yaw:.2f} | 0], δ'=[{current_yaw_rate:.2f} | {desired_yaw_rate:.2f}]")

            time.sleep(max(0, Dt - (time.time() - current_time)))

    except KeyboardInterrupt:
        print("Balance stopped by user.")

    except Exception as e:
        print("An error occurred:")
        traceback.print_exc()  # This will print the stack trace

    finally:
        # Stop the motors after the loop
        left_motor.stop()
        right_motor.stop()
        jsdev.close()

        # Create the plots
        fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2, figsize=(15, 12))
        
        # Plot positions (x)
        ax1.plot(times, positions, label='Current Position')
        ax1.plot(times, desired_positions, label='Desired Position')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Position (m)')
        ax1.legend()
        ax1.grid(True)
        ax1.set_title('Position')
        
        # Plot velocities (v)
        ax2.plot(times, velocities, label='Current Velocity')
        ax2.plot(times, desired_velocities, label='Desired Velocity')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.legend()
        ax2.grid(True)
        ax2.set_title('Velocity')

        # Plot pitches
        ax3.plot(times, pitches, label='Current Pitch')
        ax3.plot(times, desired_pitches, label='Desired Pitch')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Pitch (deg)')
        ax3.legend()
        ax3.grid(True)
        ax3.set_title('Pitch')

        # Plot pitch rates
        ax4.plot(times, pitch_rates, label='Current Pitch Rate')
        ax4.plot(times, desired_pitch_rates, label='Desired Pitch Rate')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Pitch Rate (rad/s)')
        ax4.legend()
        ax4.grid(True)
        ax4.set_title('Pitch Rate')
        ax4.yaxis.set_major_locator(plt.MultipleLocator(np.pi/2))
        ax4.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x/np.pi:.1f}π'))

        # Plot yaws
        ax5.plot(times, yaws, label='Current Yaw')
        ax5.plot(times, desired_yaws, label='Desired Yaw')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Yaw (rad)')
        ax5.legend()
        ax5.grid(True)
        ax5.set_title('Yaw')
        ax5.yaxis.set_major_locator(plt.MultipleLocator(np.pi/2))
        ax5.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x/np.pi:.1f}π'))

        # Plot yaw rates
        ax6.plot(times, yaw_rates, label='Current Yaw Rate')
        ax6.plot(times, desired_yaw_rates, label='Desired Yaw Rate')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Yaw Rate (rad/s)')
        ax6.legend()
        ax6.grid(True)
        ax6.set_title('Yaw Rate')
        ax6.yaxis.set_major_locator(plt.MultipleLocator(np.pi/2))
        ax6.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x/np.pi:.1f}π'))

        plt.tight_layout()
        plt.savefig('plots.png')
        os.system('cursor plots.png')

if __name__ == "__main__":
    balance()
    # pass
