import time
import traceback
import numpy as np
import matplotlib.pyplot as plt
from imu import FilteredLSM6DS3
from odrive_uart import ODriveUART, reset_odrive
from lqr import LQR_gains
import os
import json

WHEEL_RADIUS = 6.5 * 0.0254 / 2  # 6.5 inches diameter converted to meters
WHEEL_DIST = 0.235  # 23.5 cm from wheel to center
YAW_RATE_TO_MOTOR_TORQUE = (WHEEL_DIST / WHEEL_RADIUS) * 0.1  # Assuming a rough torque constant
MOTOR_TURNS_TO_LINEAR_POS = WHEEL_RADIUS * 2 * np.pi
RPM_TO_METERS_PER_SECOND = WHEEL_RADIUS * 2 * np.pi / 60
MAX_TORQUE = 2.5  # Nm, adjust based on your motor's specifications
MAX_SPEED = 0.4  # m/s, set maximum linear speed

def balance():
    imu = FilteredLSM6DS3()
    
    K = LQR_gains(
        #     [x, v, θ, ω, δ, δ']
        Q_diag=[100,100,1,10,10,1],
        #     [pitch, yaw]
        R_diag=[1, 1]
    )
    print(K.round(2))
    Dt = 1./400.

    prev_time = time.time()
    
    zero_angle = -2.0
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
    
    motor_controller = ODriveUART(port='/dev/ttyAMA1', left_axis=1, right_axis=0, dir_left=left_dir, dir_right=right_dir)
    motor_controller.start_left()
    motor_controller.enable_torque_mode_left()
    motor_controller.start_right()
    motor_controller.enable_torque_mode_right()
    motor_controller.set_speed_rpm_left(0)
    motor_controller.set_speed_rpm_right(0)

    # Function to reset ODrive and re-initialize motors
    def reset_and_initialize_motors():
        nonlocal motor_controller
        reset_odrive()
        time.sleep(1)  # Give ODrive time to reset
        try:
            motor_controller = ODriveUART(port='/dev/ttyAMA1', left_axis=1, right_axis=0, dir_left=left_dir, dir_right=right_dir)
            motor_controller.clear_errors_left()
            motor_controller.clear_errors_right()
            motor_controller.enable_torque_mode_left()
            motor_controller.enable_torque_mode_right()
            motor_controller.start_left()
            motor_controller.start_right()
            print("Motors re-initialized successfully.")
        except Exception as e:
            print(f"Error re-initializing motors: {e}")

    # Record starting position
    try:
        l_pos = motor_controller.get_position_turns_left()
        r_pos = motor_controller.get_position_turns_right()
        start_pos = (l_pos + r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS
        start_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*WHEEL_DIST)
    except Exception as e:
        print(f"Error reading initial motor positions: {e}")
        reset_and_initialize_motors()
        # Try again after reset
        try:
            l_pos = motor_controller.get_position_turns_left()
            r_pos = motor_controller.get_position_turns_right()
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
        motor_controller.enable_watchdog_left()
        motor_controller.enable_watchdog_right()
        while True:
            loop_start_time = time.time()

            # Check for ODriveUART errors every 10 cycles
            if cycle_count % 20 == 0:
                try:
                    left_error_code, left_error = motor_controller.get_errors_left()
                    right_error_code, right_error = motor_controller.get_errors_right()
                    if left_error_code != 0 or right_error_code != 0:
                        print(f"Detected ODriveUART errors - Left: {left_error_code} ({left_error}), Right: {right_error_code} ({right_error})")
                        reset_and_initialize_motors()
                        continue
                except Exception as e:
                    print('Error checking motor errors:', e)
                    reset_and_initialize_motors()
                    continue

            cycle_count += 1

            pitch, roll, yaw = imu.get_orientation()
            # current_pitch = imu.robot_angle() 
            current_pitch = roll
            # print(f"current_pitch: {current_pitch:.2f}")
            current_yaw_rate = -imu.gyro_RAW[2]
            current_pitch_rate = imu.gyro_RAW[0]
            try:
                r_vel = motor_controller.get_speed_rpm_right() * RPM_TO_METERS_PER_SECOND
                l_vel = motor_controller.get_speed_rpm_left() * RPM_TO_METERS_PER_SECOND
                l_pos = motor_controller.get_position_turns_left()
                r_pos = motor_controller.get_position_turns_right()

                current_vel = (l_vel + r_vel) / 2
                current_pos = ((l_pos + r_pos) / 2) * MOTOR_TURNS_TO_LINEAR_POS - start_pos
                current_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*WHEEL_DIST) - start_yaw
                             
            except Exception as e:
                print('Motor controller error:', e)
                reset_and_initialize_motors()
                continue

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

            desired_state = np.array([0, 0, zero_angle*np.pi/180, 0, 0, 0])

            state_error = (current_state - desired_state).reshape((6,1))
            # print(f"State error: {state_error}")
            C = -K @ state_error
            D = np.array([[0.5,0.5],[0.5,-0.5]])
            left_torque, right_torque = (D @ C).squeeze()

            # Limit torques to MAX_TORQUE
            left_torque = np.clip(left_torque, -MAX_TORQUE, MAX_TORQUE)
            right_torque = np.clip(right_torque, -MAX_TORQUE, MAX_TORQUE)

            try:
                # Apply torques
                # print(f"left_torque: {left_torque:.2f}, right_torque: {right_torque:.2f}")
                motor_controller.set_torque_nm_left(left_torque)
                motor_controller.set_torque_nm_right(right_torque)
            except Exception as e:
                print('Motor controller error:', e)
                reset_and_initialize_motors()
                continue

            loop_end_time = time.time()
            loop_duration = loop_end_time - loop_start_time
            if cycle_count % 50 == 0:
                print(f"Loop time: {loop_duration:.6f} sec, x=[{current_pos:.2f} | 0], v=[{current_vel:.2f} | {desired_vel:.2f}], θ=[{current_pitch:.2f} | {zero_angle:.2f}], ω=[{current_pitch_rate:.2f} | 0], δ=[{current_yaw:.2f} | 0], δ'=[{current_yaw_rate:.2f} | {desired_yaw_rate:.2f}]")

            time.sleep(max(0, Dt - (time.time() - current_time)))

    except KeyboardInterrupt:
        print("Balance stopped by user.")

    except Exception as e:
        print("An error occurred:")
        traceback.print_exc()  # This will print the stack trace

    finally:
        # Stop the motors after the loop
        motor_controller.disable_watchdog_left()
        motor_controller.disable_watchdog_right()
        motor_controller.stop_left()
        motor_controller.stop_right()

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
