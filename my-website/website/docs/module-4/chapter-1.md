---
title: "Voice-to-Action with OpenAI Whisper"
sidebar_label: "Chapter 1"
description: "Learn how to convert spoken commands into actionable robot instructions using OpenAI Whisper"
---

# Chapter 1: Voice-to-Action with OpenAI Whisper

## Overview

This chapter covers the foundational capability of converting spoken voice commands into actionable robot instructions using OpenAI Whisper. This is the essential first step in enabling human-robot interaction through speech, which provides intuitive robot control and forms the basis for more complex cognitive planning features.

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up OpenAI Whisper for voice recognition
- Process spoken commands and convert them to text
- Integrate voice processing with the ROS 2 action system
- Test voice-to-action translation with humanoid robots

## Prerequisites

Before starting this chapter, ensure you have:
- Ubuntu 22.04 (or compatible Linux system)
- ROS 2 Humble Hawksbill installed
- Python 3.10 or higher
- OpenAI API key
- Working microphone for voice input

## Setting Up Voice Processing

### 1. Install Required Dependencies

```bash
# Activate your Python environment
source vla_env/bin/activate

# Install Python dependencies for voice processing
pip install openai pyaudio numpy speechrecognition
```

### 2. Configure Audio Settings

Create an audio configuration file to handle microphone input:

```yaml
# config/audio_settings.yaml
audio:
  sample_rate: 16000
  chunk_size: 1024
  channels: 1
  format: int16
  device_index: null  # Use default microphone
```

### 3. Initialize Whisper Client

The voice processing system uses OpenAI Whisper to transcribe spoken commands:

```python
# voice_processor/whisper_client.py
import openai
import pyaudio
import wave
from config.settings import OPENAI_API_KEY

openai.api_key = OPENAI_API_KEY

class WhisperClient:
    def __init__(self):
        self.sample_rate = 16000
        self.chunk_size = 1024

    def transcribe_audio(self, audio_file_path):
        with open(audio_file_path, "rb") as audio_file:
            transcript = openai.Audio.transcribe("whisper-1", audio_file)
        return transcript.text
```

## Processing Voice Commands

### Voice Command Flow

The voice processing pipeline follows these steps:

1. **Audio Capture**: Capture spoken commands from the microphone
2. **Audio Processing**: Convert audio to the appropriate format
3. **Transcription**: Use OpenAI Whisper to transcribe speech to text
4. **Command Parsing**: Parse the transcribed text into actionable commands
5. **ROS 2 Integration**: Convert commands to ROS 2 action sequences

### Example Implementation

```python
# voice_processor/audio_input.py
import pyaudio
import wave
import time
from .transcription_handler import TranscriptionHandler

class AudioInput:
    def __init__(self):
        self.audio = pyaudio.PyAudio()
        self.transcriber = TranscriptionHandler()

    def record_audio(self, duration=5):
        # Record audio for specified duration
        stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )

        frames = []
        for _ in range(0, int(16000 / 1024 * duration)):
            data = stream.read(1024)
            frames.append(data)

        stream.stop_stream()
        stream.close()

        return frames

    def save_audio(self, frames, filename):
        wf = wave.open(filename, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(self.audio.get_sample_size(pyaudio.paInt16))
        wf.setframerate(16000)
        wf.writeframes(b''.join(frames))
        wf.close()
```

## Testing Voice-to-Action Translation

### Basic Test Scenario

1. **Given** a humanoid robot with voice recognition capabilities
2. **When** you speak a command like "Move forward 2 meters"
3. **Then** the robot should execute the corresponding movement action

### Test Implementation

```python
# tests/unit/test_voice_processor.py
import unittest
from voice_processor import AudioInput, WhisperClient

class TestVoiceProcessor(unittest.TestCase):
    def setUp(self):
        self.audio_input = AudioInput()
        self.whisper_client = WhisperClient()

    def test_voice_command_transcription(self):
        # Record a test command
        frames = self.audio_input.record_audio(duration=3)
        audio_file = "test_command.wav"
        self.audio_input.save_audio(frames, audio_file)

        # Transcribe the command
        transcription = self.whisper_client.transcribe_audio(audio_file)

        # Verify transcription quality
        self.assertIsNotNone(transcription)
        self.assertGreater(len(transcription.strip()), 0)
```

## Troubleshooting Common Issues

### Audio Input Issues
- **Problem**: No audio input detected
- **Solution**: Check microphone permissions and verify the correct device is selected

### Whisper API Errors
- **Problem**: Transcription fails with API error
- **Solution**: Verify your OpenAI API key is correct and you haven't exceeded usage limits

### Noise Interference
- **Problem**: Background noise affects transcription accuracy
- **Solution**: Use noise reduction techniques or move to a quieter environment

## Summary

In this chapter, you've learned how to set up and use OpenAI Whisper for voice-to-action translation. You've implemented the basic voice processing pipeline that will form the foundation for more complex cognitive planning in the next chapter. The voice-to-action capability enables intuitive human-robot interaction through natural language commands.

## Next Steps

In the next chapter, we'll explore how to use large language models to plan multi-step tasks from the transcribed voice commands, creating sophisticated action sequences for your humanoid robot.