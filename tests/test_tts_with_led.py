# NOTE: for this to work well, i had to patch the pi5neo library, in its source for the update_led function, theres a time.sleep(0.1), i just made that 0.01

# Adjustable Variables
CHUNK_DURATION = 0.1         # Duration (in seconds) to average the audio for volume calculation
MAX_LEDS = 15                 # Maximum number of LEDs on your strip
LED_COLOR = (0, 0, 128)       # Reduced from (0, 0, 255) to half brightness
# Use the system's default output audio device
VOICE_NAME = "Brian"          # Voice name for ElevenLabs API
LED_DEVICE_PATH = "/dev/spidev0.0"  # Path to LED device
LED_BAUDRATE = 800            # Baud rate for LED strip
VOLUME_SENSITIVITY = 10       # Multiplier to adjust volume sensitivity for LEDs
RAMP_SPEED = 1              # Amount to change the LED count per update (higher = faster ramping)

import threading
import queue
import os
from elevenlabs.client import ElevenLabs
import sounddevice as sd
import soundfile as sf
from scipy import signal
from io import BytesIO
import dotenv
import alsaaudio
import numpy as np
from pi5neo import Pi5Neo

dotenv.load_dotenv()

def set_alsa_volume(volume=80):
    """Set volume on the default ALSA mixer device (if available)."""
    try:
        mixer = alsaaudio.Mixer()  # Use default mixer on the default sound card
        mixer.setvolume(volume)
        print(f"Set system volume to {volume}%")
    except alsaaudio.ALSAAudioError as e:
        print(f"Error setting volume: {e}")

def play_text_sound(text):
    api_key = os.getenv("ELEVENLABS_API_KEY")
    client = ElevenLabs(api_key=api_key)

    # Generate audio from text
    audio = client.generate(
        text=text,
        # voice=VOICE_NAME,
        voice="ceicSWVDzgXoWidth8WQ", #raphael
        model="eleven_multilingual_v2"
    )

    device_info = sd.query_devices(kind='output')  # Default output device
    device_id = device_info['index']
    device_sample_rate = int(device_info['default_samplerate'])

    audio_data = b''.join(audio)
    data, sample_rate = sf.read(BytesIO(audio_data), dtype='float32')

    # Resample if necessary
    if sample_rate != device_sample_rate:
        number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
        data = signal.resample(data, number_of_samples)
        sample_rate = device_sample_rate

    # Variables for LED visualization
    chunk_samples = int(sample_rate * CHUNK_DURATION)

    # Use a queue size of 1 to act as a latest-value buffer
    volume_queue = queue.Queue(maxsize=1)

    # Index to keep track of the position in the audio data
    index = 0

    # Event to signal the LED thread to stop
    stop_event = threading.Event()

    # Callback function for real-time audio playback
    def audio_callback(outdata, frames, time_info, status):
        nonlocal index
        if status:
            print(f"Audio Callback Status: {status}")

        # Compute the end index
        end_index = index + frames

        # Handle the case where the audio data ends
        if end_index > len(data):
            out_frames = len(data) - index
            outdata[:out_frames, 0] = data[index:index + out_frames]
            outdata[out_frames:, 0] = 0
            index += out_frames
            raise sd.CallbackStop()
        else:
            outdata[:, 0] = data[index:end_index]
            index += frames

        # Get the current chunk for volume calculation
        current_chunk = data[max(0, index - chunk_samples):index]

        # Calculate the average volume of the most recent chunk
        if len(current_chunk) > 0:
            avg_volume = np.sqrt(np.mean(current_chunk**2))
        else:
            avg_volume = 0

        # Put the latest volume into the queue, replacing any existing value
        try:
            # If the queue is full, remove the oldest item
            if volume_queue.full():
                volume_queue.get_nowait()
            volume_queue.put_nowait(avg_volume)
        except queue.Full:
            pass  # This should not happen due to prior check

    # Function to update the LEDs in a separate thread
    def led_update_thread():
        neo = Pi5Neo(LED_DEVICE_PATH, MAX_LEDS, LED_BAUDRATE)
        current_led_count = 0.0  # Initialize as float to support fractional increments

        while not stop_event.is_set():
            try:
                # Get the most recent volume level
                avg_volume = volume_queue.get(timeout=CHUNK_DURATION)
            except queue.Empty:
                avg_volume = 0  # Default to zero if no data received

            # Map the volume to the desired number of LEDs
            desired_led_count = avg_volume * MAX_LEDS * VOLUME_SENSITIVITY
            desired_led_count = min(MAX_LEDS, desired_led_count)  # Ensure we don't exceed MAX_LEDS

            # Smooth transition: adjust current_led_count towards desired_led_count
            if current_led_count < desired_led_count:
                current_led_count += RAMP_SPEED
                current_led_count = min(current_led_count, desired_led_count)  # Do not exceed desired
            elif current_led_count > desired_led_count:
                current_led_count -= RAMP_SPEED
                current_led_count = max(current_led_count, desired_led_count)  # Do not go below desired

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
        # Play audio with callback
        with sd.OutputStream(
            device=device_id,
            samplerate=sample_rate,
            channels=1,
            callback=audio_callback,
            dtype='float32',
            latency='low',
        ):
            sd.sleep(int(len(data) / sample_rate * 1000))
    except sd.PortAudioError as e:
        print(f"Error playing audio: {e}")
    finally:
        # Signal the LED thread to stop and wait for it to finish
        stop_event.set()
        led_thread.join()

if __name__ == "__main__":
    set_alsa_volume()
    play_text_sound("I would like to formally invite you, Jocelyn Murphy, to the official Bracket Bot Hackathon on February 7th to 9th.")
