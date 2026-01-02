"""
Audio input handler for VLA Integration Module.
Captures audio from microphone and provides it for transcription.
"""
import pyaudio
import wave
import numpy as np
import threading
import queue
import time
import os
from typing import Optional, Callable

from vla_integration.config.settings import config


class AudioInputHandler:
    """
    Handles audio input from microphone for voice command processing.
    """

    def __init__(self,
                 sample_rate: Optional[int] = None,
                 chunk_size: Optional[int] = None,
                 channels: int = 1,
                 format_type: int = pyaudio.paInt16,
                 max_duration: Optional[float] = None):
        """
        Initialize the audio input handler.

        Args:
            sample_rate: Audio sample rate (Hz). Uses config value if None.
            chunk_size: Size of audio chunks to process. Uses config value if None.
            channels: Number of audio channels (1 for mono)
            format_type: Audio format (pyaudio format constant)
            max_duration: Maximum recording duration in seconds. Uses config value if None.
        """
        self.sample_rate = sample_rate or config.audio_sample_rate
        self.chunk_size = chunk_size or config.audio_chunk_size
        self.channels = channels
        self.format_type = format_type
        self.max_duration = max_duration or config.max_audio_duration

        self.audio = pyaudio.PyAudio()
        self.is_recording = False
        self.recording_thread = None
        self.audio_queue = queue.Queue()
        self.start_time = None

    def start_recording(self, callback: Optional[Callable[[bytes], None]] = None):
        """
        Start recording audio from the microphone.

        Args:
            callback: Optional callback to process audio chunks as they're captured
        """
        if self.is_recording:
            return

        self.is_recording = True
        self.start_time = time.time()

        # Open audio stream
        try:
            self.stream = self.audio.open(
                format=self.format_type,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )
        except Exception as e:
            print(f"Error opening audio stream: {e}")
            self.is_recording = False
            raise

        # Start recording thread
        self.recording_thread = threading.Thread(target=self._record_audio, args=(callback,))
        self.recording_thread.start()

    def stop_recording(self):
        """
        Stop recording audio from the microphone.
        """
        if not self.is_recording:
            return

        self.is_recording = False

        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()

        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join()

    def _record_audio(self, callback: Optional[Callable[[bytes], None]] = None):
        """
        Internal method to handle audio recording in a separate thread.
        """
        while self.is_recording:
            try:
                # Check if we've exceeded the maximum duration
                if self.start_time and (time.time() - self.start_time) > self.max_duration:
                    print(f"Maximum recording duration ({self.max_duration}s) reached")
                    break

                # Read audio data
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)

                # Add to queue for processing
                self.audio_queue.put(data)

                # Call callback if provided
                if callback:
                    callback(data)

            except Exception as e:
                print(f"Error during audio recording: {e}")
                break

    def get_audio_chunk(self) -> Optional[bytes]:
        """
        Get the next available audio chunk from the queue.

        Returns:
            Audio data as bytes or None if no data available
        """
        try:
            return self.audio_queue.get_nowait()
        except queue.Empty:
            return None

    def capture_audio(self, duration: Optional[float] = None) -> bytes:
        """
        Capture audio for a specified duration and return as a single byte string.

        Args:
            duration: Duration to record in seconds. Uses max_duration if None.

        Returns:
            Audio data as bytes
        """
        if self.is_recording:
            raise RuntimeError("Already recording audio")

        record_duration = duration or self.max_duration
        all_audio_data = []

        def audio_callback(data):
            all_audio_data.append(data)

        self.start_recording(audio_callback)

        # Record for the specified duration
        time.sleep(record_duration)

        self.stop_recording()

        # Combine all audio chunks
        return b''.join(all_audio_data)

    def save_audio_to_file(self, audio_data: bytes, filename: str):
        """
        Save raw audio data to a WAV file.

        Args:
            audio_data: Raw audio data to save
            filename: Path to save the audio file
        """
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.audio.get_sample_size(self.format_type))
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_data)

    def __del__(self):
        """
        Cleanup audio resources when object is destroyed.
        """
        if hasattr(self, 'audio'):
            self.audio.terminate()


# Example usage:
if __name__ == "__main__":
    # Create audio input handler
    audio_handler = AudioInputHandler()

    # Start recording
    audio_handler.start_recording()

    # Record for a few seconds
    import time
    time.sleep(5)

    # Stop recording
    audio_handler.stop_recording()

    print("Audio input handler example completed.")