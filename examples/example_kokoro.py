"""
Test script for the Kokoro Text-to-Speech package.

pip install this package https://github.com/hexgrad/kokoro

i had to do this to install spacy "pip install --only-binary :all: spacy"
"""

from kokoro.pipeline import KPipeline
import numpy as np
import sounddevice as sd
import time

sample_text = "Hello World, im Bracket Bot, and I generated this audio entirely on my own."


def main():
    print("Initializing Kokoro pipeline...")
    pipeline = KPipeline(lang_code='a')

    print(f"Generating speech for: '{sample_text}'")

    start_time = time.time()

    audio_chunks = []
    for gs, ps, audio in pipeline(sample_text, voice='am_adam'):
        audio_chunks.append(audio.detach().cpu().numpy())

    audio = np.concatenate(audio_chunks) if len(audio_chunks) > 1 else audio_chunks[0]

    if hasattr(audio, 'detach'):
        audio = audio.detach().cpu().numpy()

    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Audio generation took {elapsed_time:.2f} seconds.")

    print("Playing audio...")
    sd.play(audio, samplerate=24000)
    sd.wait()
    print("Done! Audio played successfully.")

if __name__ == "__main__":
    main()
