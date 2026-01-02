# VLA Integration Module

Vision-Language-Action (VLA) Integration for Robotics - A complete pipeline that converts voice commands into robot actions using AI and ROS2.

## Overview

The VLA Integration Module provides a complete solution for controlling robots using natural language voice commands. It integrates:
- **Vision**: Perception and environment understanding
- **Language**: Natural language processing and cognitive planning
- **Action**: Robot action execution and control

## Features

- Voice command recognition and transcription using OpenAI Whisper
- Cognitive planning with large language models (GPT-3.5/GPT-4)
- Multi-step task planning and execution
- ROS2 action execution for robot control
- Simulation environment for testing
- Real-time pipeline orchestration

## Architecture

```
Voice Input → Audio Processing → Transcription → LLM Planning → Action Execution → Robot Control
```

### Components

1. **Voice Processor**
   - Audio input capture
   - Whisper transcription service
   - Transcription validation

2. **LLM Planner**
   - Natural language understanding
   - Task planning and decomposition
   - Action generation

3. **ROS2 Executor**
   - Action client for ROS2
   - Robot controller interface
   - Execution feedback

4. **VLA Pipeline**
   - Pipeline orchestration
   - State management
   - Error handling

5. **Simulation Environment**
   - Robot simulation
   - Environment modeling
   - Testing framework

## Installation

### Prerequisites

- Python 3.8+
- ROS2 Humble Hawksbill
- OpenAI API key (for production use)

### Setup

1. Clone the repository:
```bash
git clone <repository-url>
cd <repository-name>
```

2. Install Python dependencies:
```bash
pip install -r vla_integration/requirements.txt
```

3. Set up environment variables:
```bash
export OPENAI_API_KEY=your_openai_api_key_here
```

4. Run the setup script:
```bash
cd vla_integration
python scripts/setup.sh
```

## Usage

### Running the System

```bash
cd vla_integration
python main.py
```

### Example Commands

- "Move forward 2 meters"
- "Go to the kitchen"
- "Pick up the red cup"
- "Turn left and wait for 5 seconds"
- "Say hello world"

### Configuration

The system can be configured via the `config/settings.py` file or environment variables:

- `OPENAI_API_KEY`: OpenAI API key
- `AUDIO_SAMPLE_RATE`: Audio sample rate (default: 16000)
- `MIN_TRANSCRIPTION_CONFIDENCE`: Minimum confidence for valid transcriptions (default: 0.5)
- `LLM_MODEL`: OpenAI model to use (default: gpt-3.5-turbo)
- `ENABLE_SIMULATION`: Enable simulation mode (default: true)

## Development

### Project Structure

```
vla_integration/
├── config/                 # Configuration files
├── voice_processor/        # Audio input and transcription
├── llm_planner/           # LLM-based planning
├── ros2_executor/         # ROS2 action execution
├── simulation/            # Simulation environment
├── vla_pipeline/          # Main pipeline orchestration
├── scripts/               # Utility scripts
└── tests/                 # Test files
```

### Running Tests

```bash
cd vla_integration
python test_basic_functionality.py
```

## Simulation Mode

The system can run in simulation mode without requiring an actual robot or OpenAI API key. In simulation mode:
- Audio input is simulated
- LLM responses are simulated
- Robot actions are executed in a simulated environment
- All pipeline stages are tested end-to-end

## API Reference

### VLA Manager

The main interface for the VLA system:

```python
from vla_integration.vla_pipeline.vla_manager import VLAManager

# Initialize the manager
vla_manager = VLAManager(whisper_api_key="...", openai_api_key="...")

# Execute a text command
result = vla_manager.execute_text_command("move forward 2 meters")

# Execute a voice command
result = vla_manager.execute_voice_command()
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.