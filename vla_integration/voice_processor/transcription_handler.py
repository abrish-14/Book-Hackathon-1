"""
Transcription handler for VLA Integration Module.
Validates and processes transcriptions from Whisper API.
"""
import re
import time
from typing import Optional, Dict, Any, List
from dataclasses import dataclass

from vla_integration.config.settings import config


@dataclass
class TranscriptionResult:
    """
    Data class to hold transcription results with metadata.
    """
    text: str
    confidence: float  # Confidence score (0.0 to 1.0)
    language: Optional[str] = None
    duration: Optional[float] = None  # Duration of audio in seconds
    timestamp: Optional[float] = None  # When transcription was processed


class TranscriptionHandler:
    """
    Handles validation and processing of transcriptions from Whisper API.
    """

    def __init__(self, min_confidence: Optional[float] = None):
        """
        Initialize the transcription handler.

        Args:
            min_confidence: Minimum confidence score for valid transcriptions (0.0 to 1.0).
                           Uses config value if None.
        """
        self.min_confidence = min_confidence or config.min_transcription_confidence

    def validate_transcription(self, text: str, confidence: float) -> bool:
        """
        Validate if a transcription meets quality requirements.

        Args:
            text: The transcribed text
            confidence: Confidence score from transcription service

        Returns:
            True if transcription is valid, False otherwise
        """
        # Check if text is not empty
        if not text or not text.strip():
            return False

        # Check if confidence is above minimum threshold
        if confidence < self.min_confidence:
            return False

        # Check for common transcription artifacts that indicate low quality
        if self._has_transcription_artifacts(text):
            return False

        return True

    def _has_transcription_artifacts(self, text: str) -> bool:
        """
        Check if the transcription contains common artifacts indicating low quality.

        Args:
            text: The transcribed text

        Returns:
            True if artifacts are detected, False otherwise
        """
        # Check for repeated characters (indicating unclear audio)
        if re.search(r'(.)\1{4,}', text):
            return True

        # Check for repeated words or phrases
        words = text.lower().split()
        if len(words) > 3:  # Only check for repetitions if there are enough words
            for i in range(len(words) - 3):
                # Check if the same 3-word sequence repeats
                if i + 6 <= len(words):
                    first_seq = ' '.join(words[i:i+3])
                    second_seq = ' '.join(words[i+3:i+6])
                    if first_seq == second_seq:
                        return True

        # Check for excessive punctuation or special characters
        special_char_ratio = len(re.findall(r'[^\w\s]', text)) / len(text) if text else 0
        if special_char_ratio > 0.3:  # If more than 30% are special characters
            return True

        return False

    def process_transcription(self, text: str, confidence: float = 1.0,
                           language: Optional[str] = None,
                           duration: Optional[float] = None) -> Optional[TranscriptionResult]:
        """
        Process and validate a transcription.

        Args:
            text: The transcribed text
            confidence: Confidence score from transcription service (default: 1.0)
            language: Detected language of the transcription
            duration: Duration of the original audio in seconds

        Returns:
            TranscriptionResult if valid, None if invalid
        """
        if self.validate_transcription(text, confidence):
            return TranscriptionResult(
                text=text.strip(),
                confidence=confidence,
                language=language,
                duration=duration,
                timestamp=time.time()
            )
        else:
            return None

    def normalize_text(self, text: str) -> str:
        """
        Normalize the transcribed text for consistent processing.

        Args:
            text: The raw transcribed text

        Returns:
            Normalized text
        """
        # Convert to lowercase for consistency
        text = text.lower()

        # Remove extra whitespace
        text = ' '.join(text.split())

        # Replace common transcription variations with standard forms
        replacements = {
            "what's": "what is",
            "don't": "do not",
            "doesn't": "does not",
            "can't": "cannot",
            "won't": "will not",
            "shouldn't": "should not",
            "couldn't": "could not",
            "wouldn't": "would not",
            "i'm": "i am",
            "you're": "you are",
            "we're": "we are",
            "they're": "they are",
            "isn't": "is not",
            "aren't": "are not",
            "wasn't": "was not",
            "weren't": "were not",
            "haven't": "have not",
            "hasn't": "has not",
            "hadn't": "had not",
            "i've": "i have",
            "you've": "you have",
            "we've": "we have",
            "they've": "they have",
            "i'll": "i will",
            "you'll": "you will",
            "we'll": "we will",
            "they'll": "they will",
            "i'd": "i would",
            "you'd": "you would",
            "we'd": "we would",
            "they'd": "they would"
        }

        for old, new in replacements.items():
            text = text.replace(old, new)

        # Remove punctuation for command processing (except for special cases)
        # Keep periods and commas as they might be important for complex commands
        text = re.sub(r'[!@#$%^&*()_+=\[\]{}|;:"<>?`~]', ' ', text)

        # Clean up extra spaces
        text = ' '.join(text.split())

        return text.strip()

    def extract_commands(self, text: str) -> List[str]:
        """
        Extract potential commands from transcribed text.

        Args:
            text: Normalized transcribed text

        Returns:
            List of potential commands extracted from the text
        """
        # Define common command patterns
        command_patterns = [
            r'\bmove\s+(forward|backward|left|right|up|down)\s*(\d*\.?\d+)\s*(meters?|cm|feet|steps?)?\b',
            r'\bgo\s+to\s+(.+?)\b',
            r'\bturn\s+(left|right|around)\b',
            r'\brotate\s+(left|right|clockwise|counterclockwise)\s*(\d*\.?\d+)\s*degrees?\b',
            r'\bpick\s+up\s+(.+?)\b',
            r'\bgrab\s+(.+?)\b',
            r'\btake\s+(.+?)\b',
            r'\bdrop\s+(.+?)\b',
            r'\bput\s+down\s+(.+?)\b',
            r'\bstop\b',
            r'\bhalt\b',
            r'\bwait\b',
            r'\bcontinue\b',
            r'\bstart\b',
            r'\brun\b',
            r'\bgoto\b',
            r'\bwalk\s+(forward|backward|left|right)\b',
            r'\brun\s+(forward|backward|left|right)\b',
            r'\bjump\b',
            r'\bsit\b',
            r'\bstand\b',
            r'\bstand\s+up\b',
            r'\bsit\s+down\b'
        ]

        commands = []
        for pattern in command_patterns:
            matches = re.findall(pattern, text, re.IGNORECASE)
            for match in matches:
                if isinstance(match, tuple):
                    # If the pattern has groups, join them to form the command
                    command = ' '.join([m for m in match if m]).strip()
                    if command:
                        commands.append(command.lower())
                else:
                    # If the pattern doesn't have groups, use the full match
                    commands.append(match.lower())

        # Add the entire normalized text as a potential command if no specific commands found
        if not commands and text.strip():
            commands.append(text.strip())

        return commands

    def calculate_confidence(self, text: str) -> float:
        """
        Calculate a confidence score based on text quality metrics.

        Args:
            text: The transcribed text

        Returns:
            Confidence score (0.0 to 1.0)
        """
        if not text or not text.strip():
            return 0.0

        # Start with a base confidence
        confidence = 1.0

        # Reduce confidence for very short texts (might be incomplete)
        if len(text.strip()) < 3:
            confidence *= 0.3

        # Reduce confidence for texts with many special characters
        special_chars = len(re.findall(r'[^\w\s]', text))
        if len(text) > 0:
            special_char_ratio = special_chars / len(text)
            if special_char_ratio > 0.2:  # More than 20% special characters
                confidence *= (1.0 - special_char_ratio)

        # Reduce confidence if text contains transcription artifacts
        if self._has_transcription_artifacts(text):
            confidence *= 0.1

        # Ensure confidence is within bounds
        return max(0.0, min(1.0, confidence))


# Example usage:
if __name__ == "__main__":
    handler = TranscriptionHandler(min_confidence=0.5)

    # Test text normalization
    raw_text = "Hey robot, can you move forward 2 meters please?"
    normalized = handler.normalize_text(raw_text)
    print(f"Raw: {raw_text}")
    print(f"Normalized: {normalized}")

    # Test command extraction
    commands = handler.extract_commands(normalized)
    print(f"Extracted commands: {commands}")

    # Test confidence calculation
    confidence = handler.calculate_confidence(normalized)
    print(f"Confidence: {confidence:.2f}")