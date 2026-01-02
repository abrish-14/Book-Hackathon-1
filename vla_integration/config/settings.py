"""
Configuration settings for VLA Integration Module.
"""
import os
from typing import Optional


class VLAModuleConfig:
    """
    Configuration class for the VLA Integration Module.
    """

    def __init__(self):
        # API Keys
        self.whisper_api_key: Optional[str] = os.getenv("OPENAI_API_KEY")
        self.openai_api_key: Optional[str] = os.getenv("OPENAI_API_KEY")

        # Audio settings
        self.audio_sample_rate: int = int(os.getenv("AUDIO_SAMPLE_RATE", "16000"))
        self.audio_chunk_size: int = int(os.getenv("AUDIO_CHUNK_SIZE", "1024"))
        self.audio_format = "WAV"  # Format for audio processing
        self.max_audio_duration: float = float(os.getenv("MAX_AUDIO_DURATION", "10.0"))  # seconds

        # Transcription settings
        self.min_transcription_confidence: float = float(os.getenv("MIN_TRANSCRIPTION_CONFIDENCE", "0.5"))
        self.transcription_language: str = os.getenv("TRANSCRIPTION_LANGUAGE", "en")

        # LLM settings
        self.llm_model: str = os.getenv("LLM_MODEL", "gpt-3.5-turbo")
        self.llm_temperature: float = float(os.getenv("LLM_TEMPERATURE", "0.3"))
        self.llm_max_tokens: int = int(os.getenv("LLM_MAX_TOKENS", "1000"))

        # ROS2 settings
        self.ros2_domain_id: int = int(os.getenv("ROS2_DOMAIN_ID", "0"))
        self.ros2_action_timeout: float = float(os.getenv("ROS2_ACTION_TIMEOUT", "30.0"))

        # Pipeline settings
        self.pipeline_timeout: float = float(os.getenv("PIPELINE_TIMEOUT", "60.0"))
        self.max_concurrent_pipelines: int = int(os.getenv("MAX_CONCURRENT_PIPELINES", "5"))

        # Simulation settings
        self.enable_simulation: bool = os.getenv("ENABLE_SIMULATION", "true").lower() == "true"
        self.simulation_speed: float = float(os.getenv("SIMULATION_SPEED", "1.0"))

        # Logging settings
        self.log_level: str = os.getenv("LOG_LEVEL", "INFO")
        self.log_format: str = os.getenv("LOG_FORMAT", "%(asctime)s - %(name)s - %(levelname)s - %(message)s")

    def validate(self) -> bool:
        """
        Validate the configuration settings.

        Returns:
            True if configuration is valid, False otherwise
        """
        errors = []

        # Check API keys
        if not self.whisper_api_key and not self.enable_simulation:
            errors.append("Whisper API key is required when simulation is disabled")
        if not self.openai_api_key and not self.enable_simulation:
            errors.append("OpenAI API key is required when simulation is disabled")

        # Check audio settings
        if self.audio_sample_rate <= 0:
            errors.append("Audio sample rate must be positive")
        if self.audio_chunk_size <= 0:
            errors.append("Audio chunk size must be positive")
        if self.max_audio_duration <= 0:
            errors.append("Max audio duration must be positive")

        # Check transcription settings
        if not 0.0 <= self.min_transcription_confidence <= 1.0:
            errors.append("Min transcription confidence must be between 0 and 1")

        # Check LLM settings
        if self.llm_temperature < 0.0 or self.llm_temperature > 2.0:
            errors.append("LLM temperature must be between 0 and 2")
        if self.llm_max_tokens <= 0:
            errors.append("LLM max tokens must be positive")

        # Check ROS2 settings
        if self.ros2_domain_id < 0:
            errors.append("ROS2 domain ID must be non-negative")
        if self.ros2_action_timeout <= 0:
            errors.append("ROS2 action timeout must be positive")

        # Check pipeline settings
        if self.pipeline_timeout <= 0:
            errors.append("Pipeline timeout must be positive")
        if self.max_concurrent_pipelines <= 0:
            errors.append("Max concurrent pipelines must be positive")

        # Check simulation settings
        if self.simulation_speed <= 0:
            errors.append("Simulation speed must be positive")

        if errors:
            print("Configuration validation errors:")
            for error in errors:
                print(f"  - {error}")
            return False

        return True


# Global configuration instance
config = VLAModuleConfig()