"""
Whisper client for VLA Integration Module.
Handles transcription of audio using OpenAI Whisper API.
"""
import openai
import os
from typing import Optional
from pathlib import Path
import tempfile
import logging

from vla_integration.config.settings import config


class WhisperClient:
    """
    Client for interacting with OpenAI Whisper API for speech-to-text transcription.
    """

    def __init__(self, api_key: Optional[str] = None):
        """
        Initialize the Whisper client.

        Args:
            api_key: OpenAI API key. If not provided, will use config value or environment variable.
        """
        # Use provided API key, config value, or environment variable
        if api_key:
            self.api_key = api_key
        elif config.whisper_api_key:
            self.api_key = config.whisper_api_key
        elif os.getenv("OPENAI_API_KEY"):
            self.api_key = os.getenv("OPENAI_API_KEY")
        else:
            raise ValueError("OpenAI API key must be provided or set in configuration or OPENAI_API_KEY environment variable")

        openai.api_key = self.api_key

        # Setup logging
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(config.log_level)
        handler = logging.StreamHandler()
        formatter = logging.Formatter(config.log_format)
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)

    def transcribe_audio(self, audio_path: str, model: str = "whisper-1") -> str:
        """
        Transcribe audio file using OpenAI Whisper API.

        Args:
            audio_path: Path to the audio file to transcribe
            model: Whisper model to use (default: "whisper-1")

        Returns:
            Transcribed text
        """
        try:
            # Validate audio file exists
            if not os.path.exists(audio_path):
                raise FileNotFoundError(f"Audio file not found: {audio_path}")

            # Validate audio format
            if not self.validate_audio_format(audio_path):
                self.logger.warning(f"Audio format may not be supported: {audio_path}")

            # Open the audio file and transcribe
            with open(audio_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe(
                    model=model,
                    file=audio_file,
                    response_format="text",
                    language=config.transcription_language
                )

            result = transcript.strip()
            self.logger.info(f"Successfully transcribed audio: {audio_path}, length: {len(result)} characters")
            return result

        except Exception as e:
            self.logger.error(f"Error transcribing audio: {e}")
            raise

    def transcribe_audio_bytes(self, audio_bytes: bytes, file_extension: str = "wav", model: str = "whisper-1") -> str:
        """
        Transcribe audio from bytes using OpenAI Whisper API.

        Args:
            audio_bytes: Audio data as bytes
            file_extension: File extension for the temporary file (default: "wav")
            model: Whisper model to use (default: "whisper-1")

        Returns:
            Transcribed text
        """
        try:
            # Create a temporary file to hold the audio bytes
            with tempfile.NamedTemporaryFile(delete=False, suffix=f".{file_extension}") as temp_file:
                temp_file.write(audio_bytes)
                temp_file_path = temp_file.name

            # Transcribe the temporary file
            transcript = self.transcribe_audio(temp_file_path, model)

            # Clean up the temporary file
            os.unlink(temp_file_path)

            return transcript

        except Exception as e:
            self.logger.error(f"Error transcribing audio bytes: {e}")
            raise

    def validate_audio_format(self, audio_path: str) -> bool:
        """
        Validate if the audio file format is supported by Whisper.

        Args:
            audio_path: Path to the audio file

        Returns:
            True if format is supported, False otherwise
        """
        supported_formats = {'.mp3', '.mp4', '.mpeg', '.mpga', '.m4a', '.wav', '.webm'}
        file_ext = Path(audio_path).suffix.lower()
        return file_ext in supported_formats


# Example usage:
if __name__ == "__main__":
    # Initialize Whisper client
    # whisper_client = WhisperClient(api_key="your-api-key-here")
    print("Whisper client initialized.")