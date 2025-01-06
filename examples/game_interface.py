import os
import subprocess

# this will livestream the camera to the web
# go to http://<robot_ip>:8080/game_interface.html to see the stream, control robot, and play sounds
# make sure to run setup_live_video.sh on the robot before running this script to install streaming dependencies
# you might also need to change the camera device /dev/video4, but sometimes it's /dev/video5
import sys
import threading
import time
from pathlib import Path

import alsaaudio
import paho.mqtt.client as mqtt
import pygame

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup.set_alsa_volume import set_alsa_volume

set_alsa_volume()


class GameInterface:
    def __init__(self, broker_address="localhost"):
        # MQTT setup
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.broker_address = broker_address
        
        # Sound setup
        pygame.mixer.init()
        self.current_sound_index = 0
        self.sounds_dir = Path(__file__).parent / "game_sounds"
        self.sound_files = [f.name for f in self.sounds_dir.glob("*.wav")]
        if not self.sound_files:
            raise FileNotFoundError("No WAV files found in game_sounds directory")
        self.sounds = [pygame.mixer.Sound(str(self.sounds_dir / f)) for f in self.sound_files]
        
        # Audio setup
        self.setup_audio()
        
        # Add stream management
        self.stream_process = None
        self.last_ping_time = 0
        
        # Start ping checker thread
        self.ping_checker = threading.Thread(target=self.check_pings, daemon=True)
        self.ping_checker.start()

    def setup_audio(self, volume=75):
        try:
            # Find the card number for UACDemoV1.0
            cards = alsaaudio.cards()
            card_num = None
            for i, card in enumerate(cards):
                if 'UACDemoV10' in card:
                    card_num = i
                    break
            
            if card_num is None:
                print("Could not find UACDemoV1.0 audio device")
                return
                
            mixer = alsaaudio.Mixer('PCM', cardindex=card_num)
            mixer.setvolume(volume)
            print(f"Set UACDemoV1.0 volume to {volume}%")
        except alsaaudio.ALSAAudioError as e:
            print(f"Error setting volume: {e}")

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        client.subscribe("robot/sound")
        client.subscribe("robot/ping")

    def on_message(self, client, userdata, msg):
        if msg.topic == "robot/sound":
            self.play_next_sound()
        elif msg.topic == "robot/ping":
            self.last_ping_time = time.time()
            if not self.stream_process:
                self.start_stream()

    def play_next_sound(self):
        # Stop any currently playing sounds
        pygame.mixer.stop()
        
        # Play the current sound
        self.sounds[self.current_sound_index].play()
        print(f"Playing sound: {self.sound_files[self.current_sound_index]}")
        
        # Move to next sound (loop back to start if at end)
        self.current_sound_index = (self.current_sound_index + 1) % len(self.sounds)

    def check_pings(self):
        while True:
            if self.stream_process and time.time() - self.last_ping_time > 20:
                self.stop_stream()
            time.sleep(5)  # Check every 5 seconds

    def start_stream(self):
        if not self.stream_process:
            try:
                self.stream_process = subprocess.Popen([
                    'mjpg_streamer',
                    '-i', 'input_uvc.so -d /dev/video4 -r 640x480 -f 15 -y',
                    '-o', 'output_http.so -w /usr/local/www -p 5001'
                ])
                print("Started video stream")
            except Exception as e:
                print(f"Error starting stream: {e}")

    def stop_stream(self):
        if self.stream_process:
            try:
                self.stream_process.terminate()
                self.stream_process = None
                print("Stopped video stream")
            except Exception as e:
                print(f"Error stopping stream: {e}")

    def run(self):
        try:
            self.client.connect(self.broker_address)
            self.client.loop_forever()
        except KeyboardInterrupt:
            print("Shutting down...")
            self.client.disconnect()
        except Exception as e:
            print(f"Error in main loop: {e}")

if __name__ == "__main__":
    interface = GameInterface()
    interface.run()