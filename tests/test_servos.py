import time

from rpi_hardware_pwm import HardwarePWM

# Initialize PWM on the GPIO pin at 50Hz (20ms period)
PWM_FREQ = 50  # Standard servo frequency
pwm1 = HardwarePWM(pwm_channel=0, hz=PWM_FREQ, chip=2)  # GPIO 12

# Define duty cycles we want to test
DUTY_CYCLE_1 = 5.0   # First test duty cycle
DUTY_CYCLE_2 = 10.5  # Second test duty cycle

def duty_cycle_to_pulse_width(duty_cycle):
    # Convert duty cycle percentage to pulse width in milliseconds
    pulse_width = (duty_cycle / 100) / PWM_FREQ * 1000  # Convert to ms
    return pulse_width

try:
    # Start PWM
    pwm1.start(0)
    
    # Test with 5.0% duty cycle
    pwm1.change_duty_cycle(DUTY_CYCLE_1)
    pulse_width = duty_cycle_to_pulse_width(DUTY_CYCLE_1)
    print(f"Duty cycle {DUTY_CYCLE_1}% = {pulse_width:.2f}ms pulse width")
    time.sleep(1)
    
    # Test with 10.5% duty cycle
    pwm1.change_duty_cycle(DUTY_CYCLE_2)
    pulse_width = duty_cycle_to_pulse_width(DUTY_CYCLE_2)
    print(f"Duty cycle {DUTY_CYCLE_2}% = {pulse_width:.2f}ms pulse width")
    time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    # Stop PWM
    # pwm1.stop()
    pass