# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import time
import json
import traceback
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from threading import Event, Thread

import paho.mqtt.client as mqtt
from RPi import GPIO
from lib.imu import FilteredMPU6050
from lib.odrive_uart import ODriveUART, reset_odrive
from lib.lqr import LQR_gains
from lib.data_logger import DataLogger

# GPIO setup for resetting ODrive
GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.OUT)

# Constants from original file
WHEEL_RADIUS = 6.5 * 0.0254 / 2
WHEEL_DIST = 0.235
YAW_RATE_TO_MOTOR_TORQUE = (WHEEL_DIST / WHEEL_RADIUS) * 0.1
MOTOR_TURNS_TO_LINEAR_POS = WHEEL_RADIUS * 2 * np.pi
RPM_TO_METERS_PER_SECOND = WHEEL_RADIUS * 2 * np.pi / 60
MAX_TORQUE = 4.0
MAX_SPEED = 1.0


class MqttSubscriber(Thread):
    def __init__(self, broker_address="localhost", topic="robot/velocity"):
        super().__init__(daemon=True)
        self.broker_address = broker_address
        self.topic = topic
        self.desired_vel = 0
        self.desired_yaw_rate = 0
        self.client = mqtt.Client()
        self._stop_event = Event()

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        client.subscribe(self.topic)

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            self.desired_vel = payload.get("linear", 0)
            self.desired_yaw_rate = payload.get("angular", 0)
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")

    def stop(self):
        self._stop_event.set()
        self.client.disconnect()

    def run(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker_address)
        self.client.loop_start()
        self._stop_event.wait()  # Wait for stop event instead of sleeping
        self.client.loop_stop()


def balance():
    # Initialize keyboard thread
    mqtt_control = MqttSubscriber()
    mqtt_control.start()
    
    # Initialize MQTT client for watchdog with better connection handling
    watchdog_client = mqtt.Client()
    watchdog_client.connect("localhost", keepalive=60)  # Add keepalive
    watchdog_client.loop_start()  # Start background thread for MQTT

    #publish position and orientation
    orientation_client = mqtt.Client()
    orientation_client.connect("localhost")
    
    def safe_publish_watchdog(client, topic, message):
        try:
            result = client.publish(topic, message)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                print(f"Failed to publish watchdog message: {result.rc}")
                # Attempt to reconnect
                client.reconnect()
        except Exception as e:
            print(f"Error publishing watchdog: {e}")
            try:
                client.reconnect()
            except:
                pass
    
    # Initialize IMU and LQR gains
    imu = FilteredMPU6050()
    K_balance = LQR_gains(Q_diag=[100,10,100,1,10,1], R_diag=[0.2, 1])
    K_drive = LQR_gains(Q_diag=[1,100,1,1,1,10], R_diag=[0.2, 1])
    print(K_balance.round(2))
    Dt = 1./200.
    
    # Constants for idle mode
    SIGNIFICANT_VELOCITY = 0.2  # m/s
    IDLE_TIMEOUT = 5  # seconds
    IDLE_BALANCE_DURATION = 1  # seconds
    
    # Initialize variables
    zero_angle = 2.0
    start_plot_time = time.time()
    last_significant_velocity_time = start_plot_time
    state = 'BALANCING'
    t_idle_wait_start = None
    t_wakeup_start = None
    
    # Initialize data logger
    data_logger = DataLogger()
    
    # Reset ODrive and initialize motors
    reset_odrive()
    time.sleep(1)
    
    # Initialize motors
    try:
        with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'r') as f:
            motor_dirs = json.load(f)
            left_dir = motor_dirs['left']
            right_dir = motor_dirs['right']
    except Exception as e:
        raise Exception("Error reading motor_dir.json")
    
    motor_controller = ODriveUART(
        port='/dev/ttyAMA1',
        left_axis=0,
        right_axis=1,
        dir_left=left_dir,
        dir_right=right_dir
    )
    motor_controller.clear_errors_left()
    motor_controller.clear_errors_right()
    motor_controller.start_left()
    motor_controller.enable_torque_mode_left()
    motor_controller.start_right()
    motor_controller.enable_torque_mode_right()
    motor_controller.set_speed_rpm_left(0)
    motor_controller.set_speed_rpm_right(0)
    
    def reset_and_initialize_motors():
        nonlocal motor_controller
        reset_odrive()
        try:
            motor_controller = ODriveUART(
                port='/dev/ttyAMA1',
                left_axis=0,
                right_axis=1,
                dir_left=left_dir,
                dir_right=right_dir
            )
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
        start_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2 * WHEEL_DIST)
    except Exception as e:
        print(f"Error reading initial motor positions: {e}")
        return
    
    imu.calibrate()
    cycle_count = 0
    is_pos_control = True
    
    try:
        motor_controller.enable_watchdog_left()
        motor_controller.enable_watchdog_right()
    
        while True:
            loop_start_time = time.time()
            current_time = time.time()

            # Check state transitions
            if state == 'BALANCING':
                # Feed the watchdog
                safe_publish_watchdog(watchdog_client, "balance_watchdog", str(current_time))

                # Check for idle timeout
                if current_time - last_significant_velocity_time > IDLE_TIMEOUT:
                    print("Entering IDLE_WAIT state")
                    state = 'IDLE_WAIT'
                    t_idle_wait_start = current_time

            elif state == 'IDLE_WAIT':
                # Do not feed the watchdog
                # Continue balancing
                if current_time - t_idle_wait_start > IDLE_BALANCE_DURATION:
                    print("Entering IDLE state")
                    state = 'IDLE'
                    # Stop motors
                    try:
                        motor_controller.set_torque_nm_left(0)
                        motor_controller.set_torque_nm_right(0)
                    except Exception as e:
                        print('Motor controller error during idle:', e)

            elif state == 'IDLE':
                # Do not feed the watchdog
                # Do not balance
                # Check for new velocity commands
                if mqtt_control.desired_vel != 0 or mqtt_control.desired_yaw_rate != 0:
                    print("Received new velocity command, entering WAKING_UP state")
                    state = 'WAKING_UP'
                    t_wakeup_start = current_time
                    last_significant_velocity_time = current_time  # Reset velocity timer

            elif state == 'WAKING_UP':
                # Feed the watchdog immediately during wake-up
                safe_publish_watchdog(watchdog_client, "balance_watchdog", str(current_time))

                # Use zero desired velocities during wake-up
                desired_vel = 0
                desired_yaw_rate = 0

                # Check if wake-up time is over (0.5s for watchdog, then 0.5s for balancing)
                if current_time - t_wakeup_start > 1:
                    print("Wake-up period over, entering BALANCING state")
                    state = 'BALANCING'
                elif current_time - t_wakeup_start <= 0.75:
                    # During first 0.5s, don't balance but keep feeding watchdog
                    try:
                        motor_controller.set_torque_nm_left(0)
                        motor_controller.set_torque_nm_right(0)
                    except Exception as e:
                        print('Motor controller error during wake-up:', e)
                    continue

            # Get desired velocity and yaw rate
            if state == 'WAKING_UP':
                # Continue using zero velocities during wake-up
                desired_vel = 0
                desired_yaw_rate = 0
            else:
                # Get desired velocity and yaw rate from MQTT control
                desired_vel = mqtt_control.desired_vel
                desired_yaw_rate = mqtt_control.desired_yaw_rate
    
            # Motor error checks (every 20 cycles)
            if cycle_count % 20 == 0:
                try:
                    if motor_controller.has_errors():
                        motor_controller.dump_errors()
                        reset_and_initialize_motors()
                        continue
                except Exception as e:
                    print('Error checking motor errors:', e)
                    reset_and_initialize_motors()
                    continue
    
            # Get IMU data
            try:
                pitch, roll, yaw = imu.get_orientation()
            except:
                continue
            current_pitch = -pitch
            current_yaw_rate = -imu.gyro_RAW[2]
            current_pitch_rate = imu.gyro_RAW[0]
    
            # Get motor data
            try:
                l_pos, l_vel = motor_controller.get_pos_vel_left()
                r_pos, r_vel = motor_controller.get_pos_vel_right()
                current_vel = (l_vel + r_vel) / 2 * RPM_TO_METERS_PER_SECOND
                current_pos = (
                    (l_pos + r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS - start_pos
                )
                current_yaw = (
                    (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2 * WHEEL_DIST) - start_yaw
                )
            except Exception as e:
                print('Motor controller error:', e)
                reset_and_initialize_motors()
                continue
    
            # Zero angle adjustment
            if state == 'BALANCING' and is_pos_control and abs(current_vel) < 0.01:
                zero_angle += 0.0002 * np.sign(current_pitch - zero_angle)
    
            # Log data using keyword arguments
            data_logger.log(
                time=current_time - start_plot_time,
                position=current_pos,
                desired_position=0,  # Assuming desired position is zero
                velocity=current_vel,
                desired_velocity=desired_vel,
                pitch=current_pitch,
                desired_pitch=zero_angle,
                pitch_rate=current_pitch_rate,
                desired_pitch_rate=0,
                yaw=current_yaw,
                desired_yaw=0,  # Assuming desired yaw is zero
                yaw_rate=current_yaw_rate,
                desired_yaw_rate=desired_yaw_rate
            )
    
            # Retrieve the list of velocities from DataLogger
            velocities = data_logger.data.get('velocity', [])
    
            # Ensure there are enough velocities to calculate the mean
            if len(velocities) >= 50:
                recent_velocities = velocities[-50:]
            else:
                recent_velocities = velocities
    
            mean_abs_velocity = np.mean(np.abs(recent_velocities))
    
            # Control mode adjustments
            was_pos_control = is_pos_control
            is_pos_control = (
                desired_vel == 0 and
                desired_yaw_rate == 0 and
                mean_abs_velocity < 0.2
            )
            if is_pos_control and not was_pos_control:
                start_pos = (l_pos + r_pos) / 2 * MOTOR_TURNS_TO_LINEAR_POS
            if desired_yaw_rate != 0:
                start_yaw = (l_pos - r_pos) * MOTOR_TURNS_TO_LINEAR_POS / (2 * WHEEL_DIST)
    
            orientation_client.publish("robot/orientation", json.dumps({"position": current_pos, "velocity": current_vel, "angle": current_pitch}))

            # Calculate control outputs
            current_state = np.array([
                current_pos,
                current_vel,
                current_pitch * np.pi / 180,
                current_pitch_rate,
                current_yaw,
                current_yaw_rate
            ])
    
            desired_state = np.array([
                0,
                desired_vel,
                zero_angle * np.pi / 180,
                0,
                0,
                desired_yaw_rate
            ])
    
            state_error = (current_state - desired_state).reshape((6, 1))
    
            if is_pos_control:
                # Position control
                C = -K_balance @ state_error
            else:
                # Velocity control
                state_error[0, 0] = 0
                state_error[4, 0] = 0
                C = -K_drive @ state_error
    
            D = np.array([[0.5, 0.5], [0.5, -0.5]])
            left_torque, right_torque = (D @ C).squeeze()
    
            # Limit torques
            left_torque = np.clip(left_torque, -MAX_TORQUE, MAX_TORQUE)
            right_torque = np.clip(right_torque, -MAX_TORQUE, MAX_TORQUE)
    
            # Update last significant velocity time
            if (abs(current_vel) > SIGNIFICANT_VELOCITY or 
                abs(desired_vel) > SIGNIFICANT_VELOCITY or
                abs(current_yaw_rate) > SIGNIFICANT_VELOCITY or 
                abs(desired_yaw_rate) > SIGNIFICANT_VELOCITY):
                last_significant_velocity_time = current_time
    
            # Apply torques if balancing
            if state in ['BALANCING', 'IDLE_WAIT', 'WAKING_UP']:
                try:
                    motor_controller.set_torque_nm_left(left_torque)
                    motor_controller.set_torque_nm_right(right_torque)
                except Exception as e:
                    print('Motor controller error:', e)
                    reset_and_initialize_motors()
                    continue
            elif state == 'IDLE':
                # Ensure motors are stopped
                try:
                    motor_controller.set_torque_nm_left(0)
                    motor_controller.set_torque_nm_right(0)
                except Exception as e:
                    print('Motor controller error during idle:', e)
    
            if cycle_count % 50 == 0:
                print(
                    f"Loop time: {time.time() - loop_start_time:.6f} sec, "
                    f"u=({float(left_torque):.2f}, {float(right_torque):.2f}), "
                    f"x=[{current_pos:.2f} | 0], "
                    f"v=[{current_vel:.2f} | {desired_vel:.2f}], "
                    f"θ=[{current_pitch:.2f} | {zero_angle:.2f}], "
                    f"ω=[{current_pitch_rate:.2f} | 0], "
                    f"δ=[{current_yaw:.2f} | 0], "
                    f"δ'=[{current_yaw_rate:.2f} | {desired_yaw_rate:.2f}], "
                    f"state={state}"
                )
    
            cycle_count += 1
            time.sleep(max(0, Dt - (time.time() - current_time)))
    
    except KeyboardInterrupt:
        print("Balance stopped by user.")
    
    except Exception as e:
        print("An error occurred:")
        traceback.print_exc()
    
    finally:
        # Cleanup
        mqtt_control.stop()
        motor_controller.disable_watchdog_left()
        motor_controller.disable_watchdog_right()
        motor_controller.stop_left()
        motor_controller.stop_right()
        motor_controller.clear_errors_left()
        motor_controller.clear_errors_right()
        watchdog_client.loop_stop()
        watchdog_client.disconnect()
        orientation_client.disconnect()

        # Plot the data after the loop
        data_logger.plot(x_key='time')

if __name__ == "__main__":
    balance()