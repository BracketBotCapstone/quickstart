#!/bin/bash

# Exit on error
set -e

echo "Installing microphone dependencies..."

# Update package lists
sudo apt-get update

# Install portaudio development package
sudo apt-get install -y portaudio19-dev

# Install pyaudio using pip
pip3 install --upgrade pip
pip3 install pyaudio

echo "Microphone setup complete!"

# Test microphone installation
python3 -c "import pyaudio; print('PyAudio installation successful!')"