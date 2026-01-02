# Quickstart Guide: Vision-Language-Action (VLA) Integration Module

## Prerequisites

- Ubuntu 22.04 (or compatible Linux system)
- ROS 2 Humble Hawksbill installed
- Python 3.10 or higher
- OpenAI API key
- Docker and Docker Compose (optional, for containerized setup)

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-directory>
cd vla_integration
```

### 2. Install System Dependencies

```bash
# Install ROS 2 dependencies
sudo apt update
sudo apt install python3-rosdep python3-rosinstall python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

### 3. Set Up Python Environment

```bash
# Create virtual environment
python3 -m venv vla_env
source vla_env/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install Python dependencies
pip install openai pyaudio numpy speechrecognition
```

### 4. Configure Environment Variables

Create a `.env` file in the root directory:

```bash
# .env
OPENAI_API_KEY=your_openai_api_key_here
ROS_DOMAIN_ID=42
WHISPER_MODEL=base  # Options: tiny, base, small, medium, large
```

### 5. Install ROS 2 Packages

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Install required ROS 2 packages
sudo apt install ros-humble-teleop-twist-keyboard ros-humble-moveit
```

### 6. Build the Workspace

```bash
# Create ROS workspace
mkdir -p ~/vla_ws/src
cd ~/vla_ws

# Source ROS and build
source /opt/ros/humble/setup.bash
colcon build --packages-select vla_integration
source install/setup.bash
```

## Running the VLA Pipeline

### Option 1: Direct Execution

```bash
cd vla_integration
source ../vla_env/bin/activate  # Activate Python environment
source /opt/ros/humble/setup.bash  # Source ROS 2

python scripts/run_pipeline.py
```

### Option 2: Using Docker (Recommended)

```bash
# Build and run with Docker
docker-compose up --build
```

## Basic Usage

1. **Start the VLA Manager**:
   ```bash
   ros2 run vla_integration vla_manager
   ```

2. **Speak a Command**: Speak a command like "Move forward 1 meter" into your microphone

3. **Monitor Processing**: Watch the console for transcription and action execution status

4. **View Results**: Observe the robot action in the simulation environment

## Configuration

### Audio Settings
- Modify `config/audio_settings.yaml` to adjust microphone sensitivity
- Change `sample_rate` and `chunk_size` as needed

### LLM Parameters
- Adjust `llm_config.yaml` for different model parameters
- Modify `temperature` and `max_tokens` settings

### ROS 2 Parameters
- Update `config/ros2_params.yaml` for robot-specific settings
- Adjust action timeouts and feedback parameters

## Troubleshooting

### Audio Input Issues
- Check microphone permissions
- Verify audio device is correctly selected in `audio_input.py`

### OpenAI API Errors
- Verify your API key is correct in `.env`
- Check internet connectivity
- Confirm API usage limits haven't been exceeded

### ROS 2 Connection Issues
- Ensure ROS 2 environment is properly sourced
- Check that ROS_DOMAIN_ID matches across all nodes
- Verify correct ROS 2 distribution is installed

## Next Steps

- Review the simulation environment setup
- Configure your specific robot model
- Fine-tune LLM parameters for your use case
- Add custom action types to the pipeline