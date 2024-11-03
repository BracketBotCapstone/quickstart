#!/bin/bash

# Install system dependencies
sudo apt-get update
sudo apt-get install -y portaudio19-dev python3-dev libsndfile1-dev

# Install Python packages
pip3 install elevenlabs
pip3 install sounddevice
pip3 install soundfile
pip3 install scipy
pip3 install numpy

# Optional: Install ffmpeg for additional audio support
sudo apt-get install -y ffmpeg