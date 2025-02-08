from enum import Enum

from anthropic import Anthropic
from dotenv import load_dotenv
from pydantic import BaseModel
from transformers import pipeline
from transformers.pipelines.audio_utils import ffmpeg_microphone_live
from typing import Any

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
        

class Command(BaseModel):
    intent: str
    description: str
    params: dict[str, float]
    fn: Any

    def __str__(self):
        return f"{self.intent}(parameters: {self.params}): {self.description}"


class Action(BaseModel):
    commands: list[Command]


class SpeechClassifier:
    
    def __init__(self, *commands: Command):
        self.commands = self.__build_commands(commands)

    def __build_commands(self, commands: list[Command]):
        return Enum("Commands", {
            c.intent: c for c in commands
        })

    @property
    def classifier_model(self):
        return Anthropic(
            api_key=os.environ.get("ANTHROPIC_API_KEY")
        )
    
    @property
    def system_prompt(self):
        return {
            "role": "system",
            "content": f"""\
            You are a robot action classifier. You will also be given a list of commands (atomic actions)
            which the robot can perform, and the parameters that action needs. You will then be given a string of text
            from a user, describing the action it wants the robot to take. Your job is to return a list of 
            commands which the robot should execute, and their parameters in order to execute the action the user
            describes. Do not return any explanation, just the list of commands.

            List of possible commands and parameters:
            {"\n".join(self.commands)}
            """,
        }
    
    def classify_action(self, text):
        message = self.classifier_model.messages.create(
            max_tokens=1024,
            messages=[
                self.system_prompt,
                {
                    "role": "user",
                    "content": text,
                }
            ],
            model="claude-3-5-sonnet-latest",
        )
        print(message.content)