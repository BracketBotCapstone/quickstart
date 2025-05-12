# Adjustable Variables
CHUNK_DURATION = 0.1         # Duration (in seconds) to average the audio for volume calculation
MAX_LEDS = 15                # Maximum number of LEDs on your strip
# Use the system's default output audio device
LED_DEVICE_PATH = "/dev/spidev0.0"  # Path to LED device
LED_BAUDRATE = 800           # Baud rate for LED strip

# Visualization tweaks
ALPHA = 0.4                  # Smoothing factor for exponential moving average
MIN_FREQ = 20                # Minimum frequency (Hz) to consider for log bands
MAX_FREQ = 20000             # Max frequency (Hz) to consider for log bands (or half the sample rate)
USE_COLOR_GRADIENT = True    # Whether to use a color gradient for each frequency band

import threading
import queue
import os
import sounddevice as sd
import soundfile as sf
from scipy import signal
import alsaaudio
import numpy as np
import colorsys
from pi5neo import Pi5Neo

def set_alsa_volume(volume=70):
    """Set volume on the default ALSA mixer device (if available)."""
    try:
        mixer = alsaaudio.Mixer()  # Use default mixer on the default sound card
        mixer.setvolume(volume)
        print(f"Set system volume to {volume}%")
    except alsaaudio.ALSAAudioError as e:
        print(f"Error setting volume: {e}")

def play_mp3_with_lights(mp3_path):
    device_info = sd.query_devices(kind='output')  # Default output device
    device_id = device_info['index']
    device_sample_rate = int(device_info['default_samplerate'])

    # Load the MP3 file
    data, sample_rate = sf.read(mp3_path, dtype='float32')

    # Handle stereo files by taking just the first channel
    if len(data.shape) > 1:
        data = data[:, 0]

    # Resample if necessary
    if sample_rate != device_sample_rate:
        number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
        data = signal.resample(data, number_of_samples)
        sample_rate = device_sample_rate

    # Variables for LED visualization
    chunk_samples = int(sample_rate * CHUNK_DURATION)

    # Prepare log-spaced frequency bin edges
    # We'll use rfft, so the maximum frequency is sample_rate/2
    max_possible_freq = sample_rate / 2.0
    actual_max_freq = min(MAX_FREQ, max_possible_freq)
    log_bin_edges = np.logspace(
        np.log10(MIN_FREQ),
        np.log10(actual_max_freq),
        num=MAX_LEDS + 1
    )

    # For smoothing over time
    smoothed_led_band_magnitudes = np.zeros(MAX_LEDS)

    # Use a queue size of 1 to act as a latest-value buffer
    spectrum_queue = queue.Queue(maxsize=1)

    # Index to keep track of the position in the audio data
    index = 0

    # Event to signal the LED thread to stop
    stop_event = threading.Event()

    # Callback function for real-time audio playback
    def audio_callback(outdata, frames, time_info, status):
        nonlocal index, smoothed_led_band_magnitudes

        if status:
            print(f"Audio Callback Status: {status}")

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

        # Get the current chunk
        current_chunk = data[max(0, index - chunk_samples):index]

        # Calculate the FFT in a more "musically meaningful" way
        if len(current_chunk) > 0:
            # Apply a window function (e.g., Hann) to reduce spectral leakage
            windowed_chunk = current_chunk * np.hanning(len(current_chunk))

            fft_data = np.fft.rfft(windowed_chunk)
            fft_magnitude = np.abs(fft_data)

            # Get frequencies corresponding to each FFT bin
            freqs = np.fft.rfftfreq(len(windowed_chunk), 1.0 / sample_rate)

            # Aggregate magnitude into logarithmic bands
            led_band_magnitudes = np.zeros(MAX_LEDS)
            for i in range(MAX_LEDS):
                start_freq = log_bin_edges[i]
                end_freq = log_bin_edges[i + 1]
                # Create a mask for frequencies within this band
                mask = (freqs >= start_freq) & (freqs < end_freq)
                if np.any(mask):
                    led_band_magnitudes[i] = np.mean(fft_magnitude[mask])
            
            # Normalize the magnitudes
            max_magnitude = np.max(led_band_magnitudes)
            if max_magnitude > 0:
                led_band_magnitudes /= max_magnitude
        else:
            led_band_magnitudes = np.zeros(MAX_LEDS)

        # Smooth results to reduce flicker (exponential moving average)
        smoothed_led_band_magnitudes = (
            ALPHA * led_band_magnitudes 
            + (1 - ALPHA) * smoothed_led_band_magnitudes
        )

        # Put the latest band magnitudes into the queue
        try:
            if spectrum_queue.full():
                spectrum_queue.get_nowait()
            spectrum_queue.put_nowait(smoothed_led_band_magnitudes)
        except queue.Full:
            pass

    # Function to get a color for a particular frequency band index
    def get_color_for_band(band_index, intensity):
        """
        Convert a band index to a color using HSV, then scale it by 'intensity'.
        Feel free to adjust hue range, saturation, value, etc.
        """
        if USE_COLOR_GRADIENT:
            # Hue goes from 0.0 to 1.0 across the LED bands
            hue = band_index / float(MAX_LEDS)
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
            return (
                int(r * 255 * intensity),
                int(g * 255 * intensity),
                int(b * 255 * intensity)
            )
        else:
            # Single color scaled by intensity (similar to your original approach)
            base_color = (0, 0, 255)  # or your choice
            return (
                int(base_color[0] * intensity),
                int(base_color[1] * intensity),
                int(base_color[2] * intensity)
            )

    # Function to update the LEDs in a separate thread
    def led_update_thread():
        neo = Pi5Neo(LED_DEVICE_PATH, MAX_LEDS, LED_BAUDRATE)
        while not stop_event.is_set():
            try:
                # Get the most recent LED band magnitudes
                led_band_magnitudes = spectrum_queue.get(timeout=CHUNK_DURATION)
            except queue.Empty:
                led_band_magnitudes = np.zeros(MAX_LEDS)

            # Update LEDs
            neo.clear_strip()
            for j in range(MAX_LEDS):
                intensity = led_band_magnitudes[j]
                color = get_color_for_band(j, intensity)
                neo.set_led_color(j, *color)
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
    play_mp3_with_lights("water.mp3")
