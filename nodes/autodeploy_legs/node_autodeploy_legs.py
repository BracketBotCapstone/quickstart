import time

from rpi_hardware_pwm import HardwarePWM

from utils.messages.watchdog_status_msg import WATCHDOG_STATUS_MSG
from utils.mqtt_utils import MQTTSubscriber
from utils.topic_to_message_type import TOPIC_WATCHDOG_STATUS

# Initialize hardware PWM
pwm = HardwarePWM(pwm_channel=0, hz=50, chip=2)  # GPIO 12, 50 Hz frequency for servo

# Calculate duty cycles for angles:
# Legs out = 10.5% duty cycle  
# Legs in = 5% duty cycle
error_duty = 10.5    # Legs out position
ok_duty = 5.0       # Legs in position

# Track current servo state to avoid unnecessary updates
current_state = None  # Start with no state

# Initialize MQTT subscriber
mqtt_subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={TOPIC_WATCHDOG_STATUS: WATCHDOG_STATUS_MSG})
mqtt_subscriber.start()

def duty_cycle_to_pulse_width(duty_cycle):
    # Convert duty cycle percentage to pulse width in milliseconds
    pulse_width = (duty_cycle / 100) / 50 * 1000  # Convert to ms
    return pulse_width

def set_servo_positions(duty_cycle):
    global current_state
    if current_state != duty_cycle:
        # Only change position if needed
        pwm.change_duty_cycle(duty_cycle)
        current_state = duty_cycle
        pulse_width = duty_cycle_to_pulse_width(duty_cycle)
        print(f"Set servo to {duty_cycle}% = {pulse_width:.2f}ms pulse width")

def check_watchdog():
    current_time = time.monotonic()
    watchdog_status_msg: WATCHDOG_STATUS_MSG = mqtt_subscriber.get_latest_message(TOPIC_WATCHDOG_STATUS)

    if watchdog_status_msg is None or current_time - watchdog_status_msg.timestamp > 0.5:  # Half second delay before deploying legs
        set_servo_positions(error_duty)  # Legs out
    else:
        set_servo_positions(ok_duty)  # Legs in

# Start PWM and set initial position
pwm.start(0)
set_servo_positions(error_duty)

try:
    while True:
        check_watchdog()
        time.sleep(0.05)  # Check every 50ms
except KeyboardInterrupt:
    pwm.stop()
    mqtt_subscriber.stop()
    print("Watchdog stopped by user.")
