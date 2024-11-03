from elevenlabs.client import ElevenLabs
import os
from io import BytesIO
import sounddevice as sd
import soundfile as sf
import numpy as np
from scipy import signal  # Added for resampling

client = ElevenLabs(
    api_key="sk_4dd7ffc9d92c8b508debcd0de67f061b22057d8f31010815",  # Defaults to ELEVEN_API_KEY
)

audio = client.generate(
    text="""App development is just soooo information / knowledge intensive

Where you just need to know how things are implemented in a framework (learning the language)

often I feel stupid just using cursor for everything, but then need to remind myself that the thing itself is not that hard - but the implementation is often a question of how much stack overflow, js framework docs did you read""" ,
    # voice="0m2tDjDewtOfXrhxqgrJ",
    # voice="tdr2UZjCfSq8qpr5CfgU",
    voice="ceicSWVDzgXoWidth8WQ", #raphael
    model="eleven_multilingual_v2"
)

# Set the desired audio device
device_name = "UACDemoV1.0"  # Name of the USB audio device
device_info = sd.query_devices(device_name, 'output')
device_id = device_info['index']
device_sample_rate = device_info['default_samplerate']

# Prepare the audio data
audio_data = b''.join(audio)
data, sample_rate = sf.read(BytesIO(audio_data), dtype='float32')

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