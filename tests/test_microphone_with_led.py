# Adjustable Variables
CHUNK_DURATION = 0.1         # Duration (in seconds) to read audio chunks
MAX_LEDS = 15                # Maximum number of LEDs on your strip
LED_COLOR = (64, 64, 64)       # Grey color at quarter brightness (changed from blue)
LED_DEVICE_PATH = "/dev/spidev0.0"  # Path to LED device
LED_BAUDRATE = 800           # Baud rate for LED strip
VOLUME_SENSITIVITY = 100       # Added from tts version
RAMP_SPEED = 1               # Added from tts version

import threading
import queue
import os
import sounddevice as sd
import numpy as np
from pi5neo import Pi5Neo
import alsaaudio
import pyaudio

def set_alsa_volume(volume=70):
    """Set capture volume on the default ALSA mixer device (if available)."""
    try:
        mixer = alsaaudio.Mixer('Capture')  # Default capture mixer
        mixer.setvolume(volume)
        print(f"Set system capture volume to {volume}%")
    except alsaaudio.ALSAAudioError as e:
        print(f"Error setting volume: {e}")

def visualize_microphone_with_lights():
    # Initialize PyAudio
    p = pyaudio.PyAudio()
    
    # Print available audio devices
    print("\nAvailable Audio Input Devices:")
    for i in range(p.get_device_count()):
        dev_info = p.get_device_info_by_index(i)
        if dev_info['maxInputChannels'] > 0:  # Only show input devices
            print(f"Device {i}: {dev_info['name']}")
    
    # Get default input device info
    default_input = p.get_default_input_device_info()
    print(f"\nUsing default input device: {default_input['name']}")
    
    # Use the default device's sample rate
    sample_rate = int(default_input['defaultSampleRate'])

    # Variables for LED visualization
    chunk_samples = int(sample_rate * CHUNK_DURATION)

    # Use a queue size of 1 to act as a latest-value buffer
    spectrum_queue = queue.Queue(maxsize=1)

    # Event to signal the LED thread to stop
    stop_event = threading.Event()

    # Callback function for real-time audio input
    def audio_callback(indata, frames, time_info, status):
        if status:
            print(f"Audio Callback Status: {status}")

        current_chunk = indata[:, 0]  # Assuming single-channel input

        # Calculate volume using RMS like in the TTS version
        if len(current_chunk) > 0:
            avg_volume = np.sqrt(np.mean(current_chunk**2))
        else:
            avg_volume = 0

        # Put the latest volume into the queue
        try:
            if spectrum_queue.full():
                spectrum_queue.get_nowait()
            spectrum_queue.put_nowait(avg_volume)
        except queue.Full:
            pass

    # Function to update the LEDs in a separate thread
    def led_update_thread():
        neo = Pi5Neo(LED_DEVICE_PATH, MAX_LEDS, LED_BAUDRATE)
        current_led_count = 0.0  # Initialize as float to support fractional increments

        while not stop_event.is_set():
            try:
                # Get the most recent volume level
                avg_volume = spectrum_queue.get(timeout=CHUNK_DURATION)
            except queue.Empty:
                avg_volume = 0

            # Map the volume to the desired number of LEDs
            desired_led_count = avg_volume * MAX_LEDS * VOLUME_SENSITIVITY
            desired_led_count = min(MAX_LEDS, desired_led_count)

            # Smooth transition: adjust current_led_count towards desired_led_count
            if current_led_count < desired_led_count:
                current_led_count += RAMP_SPEED
                current_led_count = min(current_led_count, desired_led_count)
            elif current_led_count > desired_led_count:
                current_led_count -= RAMP_SPEED
                current_led_count = max(current_led_count, desired_led_count)

            # Update LEDs
            neo.clear_strip()
            for j in range(int(current_led_count)):
                neo.set_led_color(j, *LED_COLOR)
            neo.update_strip()

        # When stopped, clear the LEDs
        neo.clear_strip()
        neo.update_strip()

    # Start the LED update thread
    led_thread = threading.Thread(target=led_update_thread)
    led_thread.start()

    try:
        # Record audio with callback
        with sd.InputStream(
            samplerate=sample_rate,
            channels=1,
            callback=audio_callback,
            dtype='float32',
            latency='low',
        ):
            while not stop_event.is_set():
                sd.sleep(int(CHUNK_DURATION * 1000))
    except sd.PortAudioError as e:
        print(f"Error with audio input stream: {e}")
    finally:
        # Signal the LED thread to stop and wait for it to finish
        stop_event.set()
        led_thread.join()

if __name__ == "__main__":
    set_alsa_volume()
    visualize_microphone_with_lights()
