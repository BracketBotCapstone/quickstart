import time
import traceback
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread
from sshkeyboard import stop_listening, listen_keyboard
import logging

import nodes.utils.constants as CFG
from nodes.sensors.imu.imu import IMU
from nodes.sensors.imu.imu_ahrs_filter import FilteredIMU
from nodes.control.balance_control.odesc_uart import ODESC, reset_odrive
from nodes.control.balance_control.lqr import LQR_gains
from nodes.utils.logging_utils import Logger

YAW_RATE_TO_MOTOR_TORQUE = (CFG.ROBOT_WHEEL_DIST_M / CFG.ROBOT_WHEEL_RADIUS_M) * 0.1  # Assuming a rough torque constant
MOTOR_TURNS_TO_LINEAR_POS = CFG.ROBOT_WHEEL_RADIUS_M * 2 * np.pi
RPM_TO_METERS_PER_SECOND = CFG.ROBOT_WHEEL_RADIUS_M * 2 * np.pi / 60

class KeyboardThread(Thread):
    def __init__(self):
        super().__init__()
        self.pressed_key = None
        self.debounce_timer = None
        self.debounce_delay = 0.0
        self.desired_vel = 0
        self.desired_yaw_rate = 0
        self.zero_angle_adjustment = 0
    def press(self, key):
        if self.debounce_timer:
            self.debounce_timer.cancel()
            self.debounce_timer = None
        
        if key == self.pressed_key:
            return
        
        self.pressed_key = key
        if key == 'w':
            self.desired_vel = CFG.MOTOR_CONTROL_MAX_SPEED_MPS / 4
        elif key == 's':
            self.desired_vel = -CFG.MOTOR_CONTROL_MAX_SPEED_MPS / 4
        elif key == 'a':
            self.desired_yaw_rate = -0.5  # ANGULAR_SPEED
        elif key == 'd':
            self.desired_yaw_rate = 0.5  # -ANGULAR_SPEED
        elif key == 'up':
            self.zero_angle_adjustment = 0.025
        elif key == 'down':
            self.zero_angle_adjustment = -0.025
        elif key == 'q':
            print("Quit")
            stop_listening()
        # print(key.upper())
    def release(self, key):
        if key != self.pressed_key:
            return
        
        if self.debounce_timer:
            self.debounce_timer.cancel()
        
        if key in ['w', 's']:
            self.desired_vel = 0
        elif key in ['a', 'd']:
            self.desired_yaw_rate = 0
        elif key in ['up', 'down']:
            self.zero_angle_adjustment = 0
        
        self.pressed_key = None
    def run(self):
        try:
            print("Use WASD to control the robot. Up/Down arrows to adjust balance. Press 'q' to quit.")
            listen_keyboard(
                on_press=self.press,
                on_release=self.release,
                sequential=False,
                delay_second_char=0
            )
        except Exception as e:
            print(e)

class BalanceController:
    def __init__(self, standalone=True, enable_motor_control=True, enable_keyboard_control=False, logger=None):
        if logger is None:
            self.logger = Logger('balance_control', 'logs/balance_control.log', level=logging.INFO)
        else:
            self.logger = logger
        self.standalone = standalone
        self.enable_motor_control = enable_motor_control
        self.enable_keyboard_control = enable_keyboard_control
        self.imu = IMU() if standalone else None
        self.filtered_imu = FilteredIMU() if standalone else None

        # Initialize LQR gains
        self.K_balance = LQR_gains(
            #     [x, v, θ, ω, δ, δ']
            Q_diag=[100,10,100,1,10,1],
            #     [pitch, yaw]
            R_diag=[0.2, 1]
        )
        self.K_drive = LQR_gains(
            Q_diag=[1,100,1,1,1,10],
            R_diag=[0.1, 1]
        )

        print(f"K_balance: {self.K_balance.round(2)}")
        print(f"K_drive: {self.K_drive.round(2)}")
        
        # Control parameters
        self.Dt = 1./400.
        self.zero_angle = 0.0
        self.desired_yaw_rate = 0
        self.desired_vel = 0

        # For plotting
        self.times = []
        self.positions = []
        self.desired_positions = []
        self.velocities = []
        self.desired_velocities = []
        self.pitches = []
        self.desired_pitches = []
        self.pitch_rates = []
        self.desired_pitch_rates = []
        self.yaws = []
        self.desired_yaws = []
        self.yaw_rates = []
        self.desired_yaw_rates = []
        self.start_plot_time = time.time()

        # Initialize motors
        self.reset_odrive()
        time.sleep(1)
        self.motor_controller = ODESC(CFG.MOTOR_CONTROL_SERIAL_PORT, left_axis=CFG.MOTOR_CONTROL_LEFT_MOTOR_AXIS, right_axis=CFG.MOTOR_CONTROL_RIGHT_MOTOR_AXIS, dir_left=CFG.MOTOR_CONTROL_LEFT_MOTOR_DIR, dir_right=CFG.MOTOR_CONTROL_RIGHT_MOTOR_DIR)
        self.motor_controller.start_left()
        self.motor_controller.start_right()
        self.motor_controller.enable_torque_mode_left()
        self.motor_controller.enable_torque_mode_right()
        # self.motor_controller.enable_watchdog_left()
        # self.motor_controller.enable_watchdog_right()

        self.l_vel = 0
        self.r_vel = 0

        # Record starting position
        self.l_pos = 0
        self.r_pos = 0
        self.start_pos = 0
        self.start_yaw = 0
        try:    
            self.l_pos = self.motor_controller.get_position_turns_left()
            self.r_pos = self.motor_controller.get_position_turns_right()
            self.start_pos = (self.l_pos + self.r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS
            self.start_yaw = (self.l_pos - self.r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*CFG.ROBOT_WHEEL_DIST_M)
        except Exception as e:
            print(f"Error reading initial motor positions: {e}")
            self.reset_and_initialize_motors()
            # Try again after reset
            try:
                self.l_pos = self.motor_controller.get_position_turns_left()
                self.r_pos = self.motor_controller.get_position_turns_right()
                self.start_pos = (self.l_pos + self.r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS
                self.start_yaw = (self.l_pos - self.r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*CFG.ROBOT_WHEEL_DIST_M)
            except Exception as e:
                print(f"Error reading motor positions after reset: {e}")
                return
            
        print(f"Starting position: {self.start_pos}, Starting yaw: {self.start_yaw}")

        if self.standalone:
            accel, gyro, t = self.imu.calibrate()
            self.filtered_imu.calibrate(t, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2])
            if self.enable_keyboard_control:
                # Initialize keyboard thread
                self.keyboard_control = KeyboardThread()
                self.keyboard_control.start()

        self.cycle_count = 0
        self.is_pos_control = True

    # Function to reset ODrive and re-initialize motors
    def reset_and_initialize_motors(self):
        self.reset_odrive()
        time.sleep(1)  # Give ODrive time to reset
        try:
            self.motor_controller = ODESC(CFG.MOTOR_CONTROL_SERIAL_PORT, left_axis=CFG.MOTOR_CONTROL_LEFT_MOTOR_AXIS, right_axis=CFG.MOTOR_CONTROL_RIGHT_MOTOR_AXIS, dir_left=CFG.MOTOR_CONTROL_LEFT_MOTOR_DIR, dir_right=CFG.MOTOR_CONTROL_RIGHT_MOTOR_DIR)
            self.motor_controller.clear_errors_left()
            self.motor_controller.clear_errors_right()
            self.motor_controller.start_left()
            self.motor_controller.start_right()
            self.motor_controller.enable_torque_mode_left()
            self.motor_controller.enable_torque_mode_right()
            # self.motor_controller.enable_watchdog_left()
            # self.motor_controller.enable_watchdog_right()
            print("Motors re-initialized successfully.")
        except Exception as e:
            print(f"Error re-initializing motors: {e}")

    def reset_odrive(self):
        reset_odrive()
        time.sleep(1)

    def balance_step(self, gyro_filtered=None, imu_gyro_data=None, imu_accel_data=None, imu_timestamp=None, enable=True, velocity_target=0.0, yaw_rate_target=0.0):
        try:
            # Check for ODESC errors every 10 cycles
            if self.cycle_count % 20 == 0:
                try:
                    left_error_code, left_error = self.motor_controller.get_errors_left()
                    right_error_code, right_error = self.motor_controller.get_errors_right()
                    if left_error_code != 0 or right_error_code != 0:
                        print(f"Detected ODESC errors - Left: {left_error_code} ({left_error}), Right: {right_error_code} ({right_error})")
                        self.reset_and_initialize_motors()
                        return self.l_vel, self.r_vel
                except Exception as e:
                    print('Error checking motor errors:', e)
                    self.reset_and_initialize_motors()
                    return self.l_vel, self.r_vel

            # Get IMU data
            if self.standalone:
                if self.enable_keyboard_control:
                    self.desired_vel = self.keyboard_control.desired_vel
                    self.desired_yaw_rate = self.keyboard_control.desired_yaw_rate
                else:
                    # NOTE: for now set desired velocity and yaw rate to 0
                    self.desired_vel = 0.0
                    self.desired_yaw_rate = 0.0

                accel, gyro, gyro_raw, t = self.imu.update()
                self.filtered_imu.update(t, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2])
                pitch, roll, yaw = self.filtered_imu.get_orientation()
                current_pitch = roll
                # print('Current pitch:', current_pitch)
                # print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")
                current_yaw_rate = -gyro[2]
                current_pitch_rate = gyro[0]
                current_roll_rate = gyro[1]
                # print(f"Pitch: {current_pitch}, Yaw rate: {current_yaw_rate}, Pitch rate: {current_pitch_rate}, Roll rate: {current_roll_rate}")
            else:
                # NOTE: for now set desired velocity and yaw rate to 0
                self.desired_vel = velocity_target
                self.desired_yaw_rate = yaw_rate_target

                pitch, roll, yaw = gyro_filtered
                current_pitch = roll
                current_yaw_rate = -imu_gyro_data[2]
                current_pitch_rate = imu_gyro_data[0]

            was_pos_control = self.is_pos_control
            # is_pos_control = False
            self.is_pos_control = self.desired_vel == 0 and self.desired_yaw_rate == 0 and np.mean(np.abs([0]+self.velocities[-50:])) < 0.2
            if self.is_pos_control and not was_pos_control:
                self.start_pos = (self.l_pos + self.r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS
                self.start_yaw = (self.l_pos - self.r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*CFG.ROBOT_WHEEL_DIST_M)
            if self.desired_yaw_rate != 0:
                self.start_yaw = (self.l_pos - self.r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*CFG.ROBOT_WHEEL_DIST_M)

            self.cycle_count += 1

            # Get motor data
            try:
                self.l_vel, self.r_vel = self.get_wheel_velocities_mps()
                self.l_pos = self.motor_controller.get_position_turns_left()
                self.r_pos = self.motor_controller.get_position_turns_right()

                current_vel = (self.l_vel + self.r_vel) / 2
                current_pos = ((self.l_pos + self.r_pos) / 2) * MOTOR_TURNS_TO_LINEAR_POS - self.start_pos
                current_yaw = (self.l_pos - self.r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2*CFG.ROBOT_WHEEL_DIST_M) - self.start_yaw
            except Exception as e:
                print('Motor controller error:', e)
                self.reset_and_initialize_motors()
                return self.l_vel, self.r_vel
            
            if self.enable_keyboard_control and self.is_pos_control and abs(current_vel) < 0.01:
                self.zero_angle += 0.0002*np.sign(current_pitch-self.zero_angle)

            # Store data for plotting
            self.store_data_for_plotting(current_pos, current_vel, current_pitch, current_pitch_rate, current_yaw, current_yaw_rate)

            # Update control
            current_state = np.array([
                current_pos, current_vel, current_pitch*np.pi/180, 
                current_pitch_rate, current_yaw, current_yaw_rate
            ])

            # print(f"Current state: {current_state}")

            self.desired_vel = max(min(self.desired_vel, CFG.MOTOR_CONTROL_MAX_SPEED_MPS), -CFG.MOTOR_CONTROL_MAX_SPEED_MPS)
            desired_state = np.array([
                0, self.desired_vel, self.zero_angle*np.pi/180, 
                0, 0, self.desired_yaw_rate
            ])

            # LQR Control Logic
            state_error = (current_state - desired_state).reshape((6,1))

            if self.is_pos_control:
                # Position control
                C = -self.K_balance @ state_error
            else:
                # Velocity control
                state_error[0,0] = 0
                state_error[4,0] = 0
                C = -self.K_drive @ state_error

            # if self.desired_vel != 0.0:
            #     state_error[0,0] = 0
            #     C = -self.K_drive @ state_error
            # if self.desired_yaw_rate != 0:
            #     state_error[4,0] = 0
            #     C = -self.K_drive @ state_error
            # if self.desired_vel == 0 and self.desired_yaw_rate == 0:
            #     C = -self.K_balance @ state_error

            D = np.array([[0.5,0.5],[0.5,-0.5]])
            left_torque, right_torque = (D @ C).squeeze()

            # Limit torques to MAX_TORQUE
            left_torque = np.clip(left_torque, -CFG.MOTOR_CONTROL_MAX_TORQUE_NM, CFG.MOTOR_CONTROL_MAX_TORQUE_NM)
            right_torque = np.clip(right_torque, -CFG.MOTOR_CONTROL_MAX_TORQUE_NM, CFG.MOTOR_CONTROL_MAX_TORQUE_NM)

            # Apply torques if motor control is enabled
            # print(f"Setting left torque to {left_torque} and right torque to {right_torque}")
            if self.enable_motor_control and enable:
                try:
                    self.motor_controller.set_torque_nm_left(left_torque)
                    self.motor_controller.set_torque_nm_right(right_torque)
                except Exception as e:
                    print('Motor controller error:', e)
                    self.reset_and_initialize_motors()
                    return self.l_vel, self.r_vel

        except Exception as e:
            print("Balance step error:", e)
            traceback.print_exc()

        return self.l_vel, self.r_vel

    def get_wheel_velocities_mps(self):
        try:
            r_vel = self.motor_controller.get_speed_rpm_right() * RPM_TO_METERS_PER_SECOND
            l_vel = self.motor_controller.get_speed_rpm_left() * RPM_TO_METERS_PER_SECOND

            return l_vel, r_vel
        except Exception as e:
            print('Motor controller error:', e)
            return 0.0, 0.0

    def stop(self):
        if self.enable_keyboard_control:
            stop_listening()
            self.keyboard_control.join()
        self.motor_controller.disable_watchdog_left()
        self.motor_controller.disable_watchdog_right()
        self.motor_controller.stop_left()
        self.motor_controller.stop_right()

    def store_data_for_plotting(self, current_pos, current_vel, current_pitch, current_pitch_rate, current_yaw, current_yaw_rate):
        current_time = time.time()
        self.times.append(current_time - self.start_plot_time)
        self.positions.append(current_pos)
        self.desired_positions.append(0)  # Always want position at 0
        self.velocities.append(current_vel)
        self.desired_velocities.append(self.desired_vel)
        self.pitches.append(current_pitch)
        self.desired_pitches.append(self.zero_angle)
        self.pitch_rates.append(current_pitch_rate)
        self.desired_pitch_rates.append(0)  # Want pitch rate at 0
        self.yaws.append(current_yaw)
        self.desired_yaws.append(0)  # Want yaw at 0
        self.yaw_rates.append(current_yaw_rate)
        self.desired_yaw_rates.append(self.desired_yaw_rate)

    def plot_data(self):
        # Create the plots
        fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2, figsize=(15, 12))
        
        # Plot positions (x)
        ax1.plot(self.times, self.positions, label='Current Position')
        ax1.plot(self.times, self.desired_positions, label='Desired Position')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Position (m)')
        ax1.legend()
        ax1.grid(True)
        ax1.set_title('Position')
        
        # Plot velocities (v)
        ax2.plot(self.times, self.velocities, label='Current Velocity')
        ax2.plot(self.times, self.desired_velocities, label='Desired Velocity')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.legend()
        ax2.grid(True)
        ax2.set_title('Velocity')

        # Plot pitches
        ax3.plot(self.times, self.pitches, label='Current Pitch')
        ax3.plot(self.times, self.desired_pitches, label='Desired Pitch')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Pitch (deg)')
        ax3.legend()
        ax3.grid(True)
        ax3.set_title('Pitch')

        # Plot pitch rates
        ax4.plot(self.times, self.pitch_rates, label='Current Pitch Rate')
        ax4.plot(self.times, self.desired_pitch_rates, label='Desired Pitch Rate')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Pitch Rate (rad/s)')
        ax4.legend()
        ax4.grid(True)
        ax4.set_title('Pitch Rate')
        ax4.yaxis.set_major_locator(plt.MultipleLocator(np.pi/2))
        ax4.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x/np.pi:.1f}π'))

        # Plot yaws
        ax5.plot(self.times, self.yaws, label='Current Yaw')
        ax5.plot(self.times, self.desired_yaws, label='Desired Yaw')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Yaw (rad)')
        ax5.legend()
        ax5.grid(True)
        ax5.set_title('Yaw')
        ax5.yaxis.set_major_locator(plt.MultipleLocator(np.pi/2))
        ax5.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x/np.pi:.1f}π'))

        # Plot yaw rates
        ax6.plot(self.times, self.yaw_rates, label='Current Yaw Rate')
        ax6.plot(self.times, self.desired_yaw_rates, label='Desired Yaw Rate')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Yaw Rate (rad/s)')
        ax6.legend()
        ax6.grid(True)
        ax6.set_title('Yaw Rate')
        ax6.yaxis.set_major_locator(plt.MultipleLocator(np.pi/2))
        ax6.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x/np.pi:.1f}π'))

        plt.tight_layout()
        plt.savefig('plots.png')
        # os.system('cursor plots.png')

def balance(standalone=True, enable_motor_control=True, enable_keyboard_control=False):
    controller = BalanceController(standalone, enable_motor_control, enable_keyboard_control)
    if standalone:
        try:
            loop_time = 0
            while True:
                loop_start_time = time.time()
                controller.balance_step()
                loop_end_time = time.time()
                loop_duration = loop_end_time - loop_start_time
                Dt = 1/400
                time.sleep(max(0, Dt - (time.time() - loop_end_time)))
                # print(time.time() - loop_time)
                loop_time = time.time()
                # time.sleep(max(0, 1/400 - loop_duration))  # 400Hz control loop
        except KeyboardInterrupt:
            print("Balance stopped by user.")
        finally:
            controller.stop()
            controller.plot_data()
    return controller

if __name__ == "__main__":
    balance(standalone=True, enable_motor_control=True)
    # balance(standalone=True, enable_motor_control=True, enable_keyboard_control=True)
