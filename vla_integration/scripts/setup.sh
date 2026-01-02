#!/bin/bash
# Setup script for VLA Integration Module

set -e  # Exit on any error

echo "Setting up VLA Integration Module..."

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python -m venv venv
fi

# Activate virtual environment
source venv/bin/activate  # On Windows, this would be: venv\Scripts\activate

# Upgrade pip
pip install --upgrade pip

# Install Python dependencies
echo "Installing Python dependencies..."
pip install openai
pip install rclpy
pip install pyaudio
pip install speechrecognition
pip install numpy
pip install requests

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "ROS2 is not installed or not in PATH. Please install ROS2 Humble Hawksbill."
    exit 1
else
    echo "ROS2 is available: $(ros2 --version)"
fi

# Setup ROS2 workspace if needed
if [ ! -d "ros2_ws/src" ]; then
    mkdir -p ros2_ws/src
    cd ros2_ws
    colcon build
    source install/setup.bash
    cd ..
fi

# Create necessary directories
mkdir -p logs
mkdir -p temp

# Create .env file if it doesn't exist
if [ ! -f ".env" ]; then
    echo "Creating .env file..."
    cat > .env << EOF
# VLA Integration Module Environment Variables
OPENAI_API_KEY=your_openai_api_key_here
AUDIO_SAMPLE_RATE=16000
AUDIO_CHUNK_SIZE=1024
MIN_TRANSCRIPTION_CONFIDENCE=0.5
LLM_MODEL=gpt-3.5-turbo
LOG_LEVEL=INFO
ENABLE_SIMULATION=true
EOF
    echo "Please update the .env file with your actual API keys."
fi

echo "Setup complete!"
echo "To activate the environment, run: source venv/bin/activate"