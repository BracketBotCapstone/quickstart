import select
import sys
import time
import wave

import pyaudio

# Audio parameters
CHUNK = 2048  # Number of frames per buffer
FORMAT = pyaudio.paInt16  # Changed to Int16 for WAV compatibility
CHANNELS = 1
RATE = 44100  # Sampling rate in Hz
MAX_SECONDS = 15
OUTPUT_FILE = "test_recording.wav"

def record_audio():
    p = pyaudio.PyAudio()
    
    # Open input stream
    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=CHUNK
    )

    print("Starting recording... Press Enter to stop, or wait 15 seconds")
    
    # Record audio data
    frames = []
    start_time = time.time()
    
    while True:
        data = stream.read(CHUNK)
        frames.append(data)
        
        # Check if Enter was pressed
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = input()  # Read the Enter keypress
            break
                
        # Check if max time reached
        if time.time() - start_time >= MAX_SECONDS:
            break
        
    print("Recording finished!")
    
    # Stop and close the stream
    stream.stop_stream()
    stream.close()
    p.terminate()
    
    # Save the recorded data as a WAV file
    wf = wave.open(OUTPUT_FILE, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()
    
    print(f"Audio saved to {OUTPUT_FILE}")

if __name__ == "__main__":
    record_audio()