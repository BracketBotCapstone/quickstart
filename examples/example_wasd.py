# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from sshkeyboard import listen_keyboard, stop_listening
from lib.odrive_uart import ODriveUART
import json

# Load motor directions
with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'r') as f:
    motor_dirs = json.load(f)

# Initialize motor controller
motor = ODriveUART()

# Start motors
motor.start_left()
motor.start_right()
motor.enable_velocity_mode_left()
motor.enable_velocity_mode_right()
motor.disable_watchdog_left()
motor.disable_watchdog_right()
motor.clear_errors_left()
motor.clear_errors_right()

SPEED = 0.375

def press(key):
    """Handle key press"""
    if key.lower() == 'w':
        motor.set_speed_mps_left(SPEED)
        motor.set_speed_mps_right(SPEED)
    elif key.lower() == 's':
        motor.set_speed_mps_left(-SPEED)
        motor.set_speed_mps_right(-SPEED)
    elif key.lower() == 'a':
        motor.set_speed_mps_left(-SPEED)
        motor.set_speed_mps_right(SPEED)
    elif key.lower() == 'd':
        motor.set_speed_mps_left(SPEED)
        motor.set_speed_mps_right(-SPEED)
    elif key.lower() == 'q':
        stop_listening()

def release(key):
    """Stop on key release"""
    motor.set_speed_mps_left(0)
    motor.set_speed_mps_right(0)

try:
    print("WASD to control the robot:")
    print("  W - Forward")
    print("  S - Backward")
    print("  A - Turn Left")
    print("  D - Turn Right")
    print("  Q - Quit")
    
    listen_keyboard(
        on_press=press,
        on_release=release,
        delay_second_char=0.05,  # Reduced from 0.75s to 0.05s
        delay_other_chars=0.02,  # Reduced from 0.05s to 0.02s
        sequential=False,  # Allow concurrent key handling
        sleep=0.01  # Faster polling rate
    )

except Exception as e:
    print(f"Error: {e}")

finally:
    # Cleanup
    motor.set_speed_mps_left(0)
    motor.set_speed_mps_right(0)
    motor.clear_errors_left()
    motor.clear_errors_right()
    print("Shutdown complete.")
