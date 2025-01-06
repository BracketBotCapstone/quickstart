#!/bin/bash

# Exit on error
set -e

echo "Installing speaker dependencies..."

# Install system dependencies for audio
sudo apt-get update
sudo apt-get install -y python3-pyaudio libportaudio2 libportaudiocpp0 portaudio19-dev libsndfile1

# Update pip
pip3 install --upgrade pip
# Install Python audio and sound packages
pip3 install pyaudio pyalsaaudio elevenlabs sounddevice soundfile python-dotenv
python3 "$(dirname "$0")/set_alsa_volume.py"

echo "Speaker setup complete!"

# Test imports
python3 -c "import sounddevice; import soundfile; import elevenlabs; print('Speaker packages installation successful!')"