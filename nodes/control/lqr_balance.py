import json
import logging
import os
import time
import traceback
from enum import Enum

import numpy as np

import utils.constants as CFG
from nodes.control.lqr import LQR_gains
from nodes.control.odrive_uart import ODriveUART, reset_odrive
from nodes.sensors.imu.imu_ahrs_filter import FilteredIMU
from utils.data_logger import DataLogger
from utils.logging_utils import Logger
from utils.messages.robot_extended_pose_msg import ROBOT_EXTENDED_POSE_MSG
from utils.messages.watchdog_status_msg import WATCHDOG_STATUS_MSG
from utils.mqtt_utils import MQTTPublisher
from utils.topic_to_message_type import TOPIC_WATCHDOG_STATUS

YAW_RATE_TO_MOTOR_TORQUE = (CFG.ROBOT_WHEEL_DIST_M / CFG.ROBOT_WHEEL_RADIUS_M) * 0.1  # Assuming a rough torque constant
MOTOR_TURNS_TO_LINEAR_POS = CFG.ROBOT_WHEEL_RADIUS_M * 2 * np.pi
RPM_TO_METERS_PER_SECOND = CFG.ROBOT_WHEEL_RADIUS_M * 2 * np.pi / 60

class CONTROL_STATE(Enum):
    IDLE = 0
    IDLE_WAIT = 1
    BALANCING = 2
    WAKING_UP = 3
    POSITION_CONTROL = 4

class BalanceController:
    def __init__(self, standalone=True, enable_motor_control=True, logger=None):
        """
        Initialize the BalanceController with optional standalone mode and motor control.

        :param standalone: If True, the controller operates independently.
        :param enable_motor_control: If True, motor control is enabled.
        :param logger: Optional logger instance.
        """
        if logger is None:
            self.logger = Logger('control', 'logs/control.log', level=logging.INFO)
        else:
            self.logger = logger
        self.enable_motor_control = enable_motor_control

        self.standalone = standalone

        # Initialize IMU
        self.filtered_imu = FilteredIMU() if standalone else None

        # Calibrate IMU
        if self.standalone:
            self.filtered_imu.calibrate()

        # Initialize LQR gains
        self.K_balance = LQR_gains(
            #     [x, v, θ, ω, δ, δ']
            Q_diag=[100,10,100,1,10,1],
            #     [pitch, yaw]
            R_diag=[0.2, 1]
        )
        self.K_drive = LQR_gains(
            Q_diag=[1,100,1,1,1,10],
            R_diag=[0.2, 1]
        )

        print(f"K_balance: {self.K_balance.round(2)}")
        print(f"K_drive: {self.K_drive.round(2)}")

        # Control parameters
        self.zero_angle_deg = -0.5
        self.desired_yaw_rate_rad_s = 0
        self.desired_vel_mps = 0
        self.last_significant_velocity_time = time.time()
        self.state: CONTROL_STATE = CONTROL_STATE.BALANCING
        self.t_idle_wait_start = None
        self.t_wakeup_start = None

        # For plotting and logging
        self.data_logger = DataLogger()
        self.start_plot_time = time.monotonic()

        try:
            self.left_motor_dir, self.right_motor_dir = get_motor_directions()
        except Exception as e:
            print(f"Error getting motor directions from JSON file, using constants from config instead: {e}")
            self.left_motor_dir, self.right_motor_dir = CFG.MOTOR_CONTROL_LEFT_MOTOR_DIR, CFG.MOTOR_CONTROL_RIGHT_MOTOR_DIR

        # Initialize motors
        self.reset_odrive()
        time.sleep(1)
        self.motor_controller = ODriveUART(CFG.MOTOR_CONTROL_SERIAL_PORT,
                                           left_axis=CFG.MOTOR_CONTROL_LEFT_MOTOR_AXIS,
                                           right_axis=CFG.MOTOR_CONTROL_RIGHT_MOTOR_AXIS,
                                           dir_left=self.left_motor_dir,
                                           dir_right=self.right_motor_dir)
        self.motor_controller.clear_errors_left()
        self.motor_controller.clear_errors_right()
        self.motor_controller.start_left()
        self.motor_controller.enable_torque_mode_left()
        self.motor_controller.start_right()
        self.motor_controller.enable_torque_mode_right()
        self.motor_controller.set_speed_rpm_left(0)
        self.motor_controller.set_speed_rpm_right(0)
        # self.motor_controller.enable_watchdog_left()
        # self.motor_controller.enable_watchdog_right()

        self.l_vel_mps = 0
        self.r_vel_mps = 0

        # Record starting position
        self.l_pos_m = 0
        self.r_pos_m = 0
        self.start_pos_m = 0
        self.start_yaw_rad = 0
        try:    
            self.l_pos_m = self.motor_controller.get_position_turns_left()
            self.r_pos_m = self.motor_controller.get_position_turns_right()
            self.start_pos_m = (self.l_pos_m + self.r_pos_m) / 2 * MOTOR_TURNS_TO_LINEAR_POS
            self.start_yaw_rad = (self.l_pos_m - self.r_pos_m) * MOTOR_TURNS_TO_LINEAR_POS / (2*CFG.ROBOT_WHEEL_DIST_M)
        except Exception as e:
            print(f"Error reading initial motor positions: {e}")
            self.reset_and_initialize_motors()
            return

        self.cycle_count = 0
        self.is_pos_control = True

        self.mqtt_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_WATCHDOG_STATUS: WATCHDOG_STATUS_MSG})

        self.mqtt_publisher.run(keep_alive=60)

    def reset_and_initialize_motors(self):
        """
        Reset the ODrive and re-initialize the motors.
        """
        self.reset_odrive()
        time.sleep(1)  # Give ODrive time to reset
        try:
            self.motor_controller = ODriveUART(CFG.MOTOR_CONTROL_SERIAL_PORT,
                                               left_axis=CFG.MOTOR_CONTROL_LEFT_MOTOR_AXIS,
                                               right_axis=CFG.MOTOR_CONTROL_RIGHT_MOTOR_AXIS,
                                               dir_left=self.left_motor_dir,
                                               dir_right=self.right_motor_dir)
            self.motor_controller.clear_errors_left()
            self.motor_controller.clear_errors_right()
            self.motor_controller.start_left()
            self.motor_controller.enable_torque_mode_left()
            self.motor_controller.start_right()
            self.motor_controller.enable_torque_mode_right()
            print("Motors re-initialized successfully.")
        except Exception as e:
            print(f"Error re-initializing motors: {e}")

    def reset_odrive(self):
        """
        Reset the ODrive controller.
        """
        reset_odrive()
        time.sleep(0.5)

    def balance_step(self, gyro_filtered=None, imu_gyro_data=None,
                     velocity_target_mps=0.0, yaw_rate_target_rad_s=0.0,
                     robot_pose: ROBOT_EXTENDED_POSE_MSG = None):
        """
        Perform a single balance control step.

        :param gyro_filtered: Filtered gyro data.
        :param imu_gyro_data: Raw IMU gyro data.
        :param velocity_target_mps: Target velocity in meters per second.
        :param yaw_rate_target_rad_s: Target yaw rate in radians per second.
        :param robot_pose: Current robot pose message.
        :return: Tuple of left and right wheel velocities in meters per second.
        """
        try:
            loop_start_time = time.monotonic()

            # Get motor data
            try:
                self.l_pos_m, self.l_vel_mps = self.motor_controller.get_pos_vel_left()
                self.r_pos_m, self.r_vel_mps = self.motor_controller.get_pos_vel_right()

                self.l_vel_mps = self.l_vel_mps * RPM_TO_METERS_PER_SECOND
                self.r_vel_mps = self.r_vel_mps * RPM_TO_METERS_PER_SECOND

                # if robot_pose is not None:
                #     current_pos_m = np.dot([robot_pose.r_a_b_x_m, robot_pose.r_a_b_y_m], [0,1]) - self.start_pos_m
                #     current_yaw_rad = yaw_rad - self.start_yaw_rad
                #     current_vel_mps = np.dot([robot_pose.v_a_b_x_mps, robot_pose.v_a_b_y_mps], [0,1])
                #     print(f"current_vel_mps: {current_vel_mps}")
                #     print(f"current_pos_m: {current_pos_m}")
                #     print(f"current_yaw_rad: {current_yaw_rad}")
                # else:
                current_vel_mps = (self.l_vel_mps + self.r_vel_mps) / 2
                current_pos_m = ((self.l_pos_m + self.r_pos_m) / 2) * MOTOR_TURNS_TO_LINEAR_POS - self.start_pos_m
                current_yaw_rad = (self.l_pos_m - self.r_pos_m) * MOTOR_TURNS_TO_LINEAR_POS / (2*CFG.ROBOT_WHEEL_DIST_M) - self.start_yaw_rad

            except Exception as e:
                print('Motor controller error:', e)
                self.reset_and_initialize_motors()
                return self.l_vel_mps, self.r_vel_mps

            # Check state transitions
            if self.state == CONTROL_STATE.BALANCING:
                # Feed the watchdog
                self.mqtt_publisher.publish_msg(TOPIC_WATCHDOG_STATUS, WATCHDOG_STATUS_MSG(loop_start_time))

                # Check if we've been idle for too long
                if loop_start_time - self.last_significant_velocity_time > CFG.BALANCE_IDLE_TIMEOUT_S:
                    self.logger.info("Entering IDLE_WAIT state")
                    self.state = CONTROL_STATE.IDLE_WAIT
                    self.t_idle_wait_start = loop_start_time
            elif self.state == CONTROL_STATE.IDLE_WAIT:
                # Do not feed the watchdog
                # Continue balancing
                if loop_start_time - self.t_idle_wait_start > CFG.BALANCE_IDLE_BALANCE_DURATION_S:
                    self.logger.info("Entering IDLE state")
                    self.state = CONTROL_STATE.IDLE
            elif self.state == CONTROL_STATE.IDLE:
                # Do not feed the watchdog
                # Do not balance
                if velocity_target_mps != 0 or yaw_rate_target_rad_s != 0:
                    self.logger.info("Entering WAKING_UP state")
                    self.state = CONTROL_STATE.WAKING_UP
                    self.t_wakeup_start = loop_start_time
                    self.last_significant_velocity_time = loop_start_time
            elif self.state == CONTROL_STATE.WAKING_UP:
                # Feed the watchdog
                # Do not balance
                self.mqtt_publisher.publish_msg(TOPIC_WATCHDOG_STATUS, WATCHDOG_STATUS_MSG(loop_start_time))

                velocity_target_mps = 0
                yaw_rate_target_rad_s = 0

                # Check if wake-up time is over (0.5s for watchdog, then 0.5s for balancing)
                if loop_start_time - self.t_wakeup_start > 1:
                    self.logger.info("Wake-up period over, entering BALANCING state")
                    self.state = CONTROL_STATE.BALANCING
                elif loop_start_time - self.t_wakeup_start <= 0.75:
                    # During first 0.5s, don't balance but keep feeding watchdog
                    try:
                        self.motor_controller.set_torque_nm_left(0)
                        self.motor_controller.set_torque_nm_right(0)
                    except Exception as e:
                        print('Motor controller error during wake-up:', e)
                    return self.l_vel_mps, self.r_vel_mps

            # Motor error checks (every 20 cycles)
            if self.cycle_count % 20 == 0:
                try:
                    if self.motor_controller.has_errors():
                        self.motor_controller.dump_errors()
                        self.reset_and_initialize_motors()
                        return self.l_vel_mps, self.r_vel_mps
                except Exception as e:
                    print('Error checking motor errors:', e)
                    self.reset_and_initialize_motors()
                    return self.l_vel_mps, self.r_vel_mps

            # Get IMU data
            if self.standalone:
                self.desired_vel_mps = 0.0
                self.desired_yaw_rate_rad_s = 0.0

                self.filtered_imu.update()
                pitch_deg, roll_deg, yaw_deg = self.filtered_imu.get_orientation()
                current_pitch_rad = -pitch_deg*np.pi/180

                print(f"current_pitch_rad: {current_pitch_rad}")

                gyro = self.filtered_imu.imu.get_gyro()
                current_yaw_rate_rad_s = -gyro[2]
                current_pitch_rate_rad_s = -gyro[1]
            else:
                self.desired_vel_mps = velocity_target_mps
                self.desired_yaw_rate_rad_s = yaw_rate_target_rad_s

                pitch_deg, roll_deg, yaw_deg = gyro_filtered
                current_pitch_rad = -1 * pitch_deg * np.pi/180
                current_yaw_rate_rad_s = -imu_gyro_data[2]
                current_pitch_rate_rad_s = -imu_gyro_data[1]

            # Clip the desired velocity to the maximum speed
            self.desired_vel_mps = max(min(self.desired_vel_mps, CFG.MOTOR_CONTROL_MAX_SPEED_MPS), -CFG.MOTOR_CONTROL_MAX_SPEED_MPS)
            
            if self.state == CONTROL_STATE.BALANCING and self.is_pos_control and abs(current_vel_mps) < 0.01:
                self.zero_angle_deg += 0.0002*np.sign(current_pitch_rad-self.zero_angle_deg*np.pi/180)

            # Retrieve the list of velocities from DataLogger
            velocities = self.data_logger.data.get('velocity', [])

            # Ensure there are enough velocities to calculate the mean
            if len(velocities) >= 50:
                recent_velocities = velocities[-50:]
            else:
                recent_velocities = velocities
    
            mean_abs_velocity = np.mean(np.abs(recent_velocities))
    
            # Control mode adjustments
            was_pos_control = self.is_pos_control
            self.is_pos_control = (
                self.desired_vel_mps == 0 and
                self.desired_yaw_rate_rad_s == 0 and
                mean_abs_velocity < 0.2
            )
            if self.is_pos_control and not was_pos_control:
                # if robot_pose is not None:
                #     self.start_pos_m = np.dot([robot_pose.r_a_b_x_m, robot_pose.r_a_b_y_m], [0,1])
                #     self.start_yaw_rad = robot_pose.phi_a_b_z_rad
                # else:
                self.start_pos_m = (self.l_pos_m + self.r_pos_m) / 2 * MOTOR_TURNS_TO_LINEAR_POS
                self.start_yaw_rad = (self.l_pos_m - self.r_pos_m) * MOTOR_TURNS_TO_LINEAR_POS / (2*CFG.ROBOT_WHEEL_DIST_M)
            if self.desired_yaw_rate_rad_s != 0:
                # if robot_pose is not None:
                #     self.start_yaw_rad = robot_pose.phi_a_b_z_rad
                # else:
                self.start_yaw_rad = (self.l_pos_m - self.r_pos_m) * MOTOR_TURNS_TO_LINEAR_POS / (2*CFG.ROBOT_WHEEL_DIST_M)

            # Update control
            current_state = np.array([
                current_pos_m,
                current_vel_mps, 
                current_pitch_rad, 
                current_pitch_rate_rad_s, 
                current_yaw_rad, 
                current_yaw_rate_rad_s
            ])

            desired_state = np.array([
                0,
                self.desired_vel_mps,
                self.zero_angle_deg * np.pi/180,
                0,
                0,
                self.desired_yaw_rate_rad_s
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

            D = np.array([[0.5, 0.5],
                          [0.5, -0.5]])
            left_torque, right_torque = (D @ C).squeeze()

            # Limit torques to MAX_TORQUE
            left_torque = np.clip(left_torque, -CFG.MOTOR_CONTROL_MAX_TORQUE_NM, CFG.MOTOR_CONTROL_MAX_TORQUE_NM)
            right_torque = np.clip(right_torque, -CFG.MOTOR_CONTROL_MAX_TORQUE_NM, CFG.MOTOR_CONTROL_MAX_TORQUE_NM)

            # Update last significant velocity time
            if (abs(current_vel_mps) > CFG.BALANCE_SIGNIFICANT_VELOCITY_MPS or 
                abs(self.desired_vel_mps) > CFG.BALANCE_SIGNIFICANT_VELOCITY_MPS or
                abs(current_yaw_rate_rad_s) > CFG.BALANCE_SIGNIFICANT_VELOCITY_MPS or 
                abs(self.desired_yaw_rate_rad_s) > CFG.BALANCE_SIGNIFICANT_VELOCITY_MPS):
                self.last_significant_velocity_time = loop_start_time
            
            # TODO: Remove this to re-enable states if legs are attached
            self.last_significant_velocity_time = loop_start_time

            # Apply torques if motor control is enabled
            if self.enable_motor_control and self.state in [CONTROL_STATE.BALANCING, CONTROL_STATE.IDLE_WAIT, CONTROL_STATE.WAKING_UP]:
                try:
                    self.motor_controller.set_torque_nm_left(left_torque)
                    self.motor_controller.set_torque_nm_right(right_torque)
                except Exception as e:
                    print('Motor controller error:', e)
                    self.reset_and_initialize_motors()
                    return self.l_vel_mps, self.r_vel_mps
            elif self.state == CONTROL_STATE.IDLE:
                try:
                    self.motor_controller.set_torque_nm_left(0)
                    self.motor_controller.set_torque_nm_right(0)
                except Exception as e:
                    print('Motor controller error:', e)
                    self.reset_and_initialize_motors()
                    return self.l_vel_mps, self.r_vel_mps

            # Log data using keyword arguments
            self.data_logger.log(
                time=loop_start_time - self.start_plot_time,
                position=current_pos_m,
                desired_position=0,  # Assuming desired position is zero
                velocity=current_vel_mps,
                desired_velocity=self.desired_vel_mps,
                pitch=current_pitch_rad,
                desired_pitch=self.zero_angle_deg,
                pitch_rate=current_pitch_rate_rad_s,
                desired_pitch_rate=0,
                yaw=current_yaw_rad,
                desired_yaw=0,  # Assuming desired yaw is zero
                yaw_rate=current_yaw_rate_rad_s,
                desired_yaw_rate=self.desired_yaw_rate_rad_s
            )

            # if self.cycle_count % 50 == 0:
            #     self.logger.info(
            #         f"Loop time: {time.monotonic() - loop_start_time:.6f} sec, "
            #         f"u=({float(left_torque):.2f}, {float(right_torque):.2f}), "
            #         f"x=[{current_pos_m:.2f} | 0], "
            #         f"v=[{current_vel_mps:.2f} | {self.desired_vel_mps:.2f}], "
            #         f"θ=[{current_pitch_rad:.2f} | {self.zero_angle_deg:.2f}], "
            #         f"ω=[{current_pitch_rate_rad_s:.2f} | 0], "
            #         f"δ=[{current_yaw_rad:.2f} | 0], "
            #         f"δ'=[{current_yaw_rate_rad_s:.2f} | {self.desired_yaw_rate_rad_s:.2f}], "
            #         f"state={current_state}"
            #     )
            #     self.data_logger.to_csv('logs.csv', max_num=6000)

            self.cycle_count += 1

        except Exception as e:
            print("Balance step error:", e)
            traceback.print_exc()

        return self.l_vel_mps, self.r_vel_mps

    def stop(self):
        """
        Stop the motors and disable the watchdog.
        """
        self.motor_controller.disable_watchdog_left()
        self.motor_controller.disable_watchdog_right()
        self.motor_controller.stop_left()
        self.motor_controller.stop_right()
        self.mqtt_publisher.stop()

def balance(standalone=True, enable_motor_control=True):
    """
    Start the balance controller in standalone mode or integrated mode.

    :param standalone: If True, the controller operates independently.
    :param enable_motor_control: If True, motor control is enabled.
    :return: The balance controller instance.
    """
    controller = BalanceController(standalone, enable_motor_control)
    if standalone:
        try:
            while True:
                loop_start_time = time.time()
                controller.balance_step()
                Dt = 1/200
                time.sleep(max(0, Dt - (time.time() - loop_start_time)))
        except KeyboardInterrupt:
            print("Balance stopped by user.")
        finally:
            controller.stop()
    return controller

def get_motor_directions():
    """
    Retrieve motor directions from a JSON file.

    :return: Tuple of left and right motor directions.
    :raises Exception: If there is an error reading the JSON file.
    """
    try:
        # json file is in the same directory as the script
        with open(os.path.join(os.path.dirname(__file__), 'motor_dir.json'), 'r') as f:
            motor_dir = json.load(f)
        return motor_dir['left'], motor_dir['right']
    except Exception as e:
        raise Exception(f"Error reading motor directions: {e}")

if __name__ == "__main__":
    balance(standalone=True, enable_motor_control=True)
