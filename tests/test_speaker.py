import os
import sounddevice as sd
import soundfile as sf
import numpy as np
from scipy import signal

# Use the system's default output audio device
device_info = sd.query_devices(kind='output')  # Default output device info
device_id = device_info['index']
device_sample_rate = device_info['default_samplerate']

# Load the MP3 file
mp3_path = "water.mp3"  # Replace with your MP3 file path
data, sample_rate = sf.read(mp3_path, dtype='float32')

# Resample if necessary
if sample_rate != device_sample_rate:
    print(f"Resampling from {sample_rate} to {device_sample_rate}")
    number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
    data = signal.resample(data, number_of_samples)
    sample_rate = device_sample_rate

# Increase the volume (optional)
volume_increase = 1.0
data = data * volume_increase

try:
    # Play the audio using the specified device
    sd.play(data, samplerate=sample_rate, device=device_id)
    sd.wait()
    print("Audio played successfully")
except sd.PortAudioError as e:
    print(f"Error playing audio: {e}")
    print(f"Supported sample rates for this device: {device_sample_rate}")