from anthropic import Anthropic
from dotenv import load_dotenv
from pydantic import BaseModel
from transformers import pipeline
from transformers.pipelines.audio_utils import ffmpeg_microphone_live
from typing import Any, Dict, List

from enum import Enum

import instructor

import os
import sys


load_dotenv()

# Add error handling for model loading

class Transcriber:

    @property
    def transcriber_model(self):
        try:
            model = "openai/whisper-tiny.en"
            # model = "distil-whisper/distil-small.en"
            return pipeline(
                "automatic-speech-recognition",
                model=model,
                device='cpu',
                model_kwargs={"local_files_only": False}  # Allow downloading model files
            )
        except Exception as e:
            print(f"Error loading model: {e}")
            print("Make sure you have an internet connection and the required packages installed:")
            print("pip install transformers torch")
            sys.exit(1)


    def transcribe(self, chunk_length_s=5.0, stream_chunk_s=2.0):
        sampling_rate = self.transcriber_model.feature_extractor.sampling_rate

        mic = ffmpeg_microphone_live(
            sampling_rate=sampling_rate,
            chunk_length_s=chunk_length_s,
            stream_chunk_s=stream_chunk_s,
        )

        print("Start speaking...")
        speech_chunk = ""
        try:
            while True:
                for item in self.transcriber_model(mic, generate_kwargs={"max_new_tokens": 128}):
                    sys.stdout.write("\033[K")
                    print(item["text"], end="\r")
                    speech_chunk += item["text"]
                    if not item["partial"][0]:
                        print()  # Move to next line when chunk is complete
                        break
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            return speech_chunk
        