"""
VLA Manager for VLA Integration Module.
Orchestrates the Vision-Language-Action pipeline.
"""
import threading
import time
import queue
from typing import Dict, Any, Optional, Callable, List
from dataclasses import dataclass
from enum import Enum
import logging
from pathlib import Path

from vla_integration.voice_processor.audio_input import AudioInputHandler
from vla_integration.voice_processor.whisper_client import WhisperClient
from vla_integration.voice_processor.transcription_handler import TranscriptionHandler, TranscriptionResult
from vla_integration.llm_planner.llm_client import LLMClient, TaskPlan
from vla_integration.llm_planner.task_parser import TaskParser
from vla_integration.llm_planner.action_generator import ActionGenerator
from vla_integration.ros2_executor.action_client import ROS2ActionClient
from vla_integration.ros2_executor.robot_controller import RobotController
from vla_integration.ros2_executor.feedback_handler import FeedbackHandler
from vla_integration.vla_pipeline.state_manager import PipelineState, StateManager


class PipelineStage(Enum):
    """
    Enum for different stages of the VLA pipeline.
    """
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING_AUDIO = "processing_audio"
    TRANSCRIBING = "transcribing"
    PLANNING = "planning"
    EXECUTING = "executing"
    COMPLETED = "completed"
    ERROR = "error"


@dataclass
class PipelineResult:
    """
    Data class to hold pipeline execution results.
    """
    success: bool
    transcription: Optional[str] = None
    task_plan: Optional[TaskPlan] = None
    execution_results: Optional[List[Dict[str, Any]]] = None
    error_message: Optional[str] = None
    execution_time: Optional[float] = None


class VLAManager:
    """
    Manages the complete Vision-Language-Action pipeline from voice input to robot action execution.
    """

    def __init__(self, whisper_api_key: Optional[str] = None, openai_api_key: Optional[str] = None):
        """
        Initialize the VLA Manager.

        Args:
            whisper_api_key: OpenAI API key for Whisper transcription
            openai_api_key: OpenAI API key for LLM planning
        """
        # Initialize components
        self.audio_handler = AudioInputHandler()
        self.whisper_client = WhisperClient(api_key=whisper_api_key) if whisper_api_key else None
        self.transcription_handler = TranscriptionHandler(min_confidence=0.5)
        self.llm_client = LLMClient(api_key=openai_api_key) if openai_api_key else None
        self.task_parser = TaskParser()
        self.action_generator = ActionGenerator()
        self.action_client = ROS2ActionClient()
        self.robot_controller = RobotController()
        self.feedback_handler = FeedbackHandler()
        self.state_manager = StateManager()

        # Pipeline state
        self.pipeline_stage = PipelineStage.IDLE
        self.pipeline_result = None
        self.pipeline_thread = None
        self.is_running = False

        # Callbacks
        self.pipeline_callbacks = {
            'on_stage_change': [],
            'on_transcription': [],
            'on_task_plan': [],
            'on_execution': [],
            'on_completion': [],
            'on_error': []
        }

        # Queues for communication
        self.audio_queue = queue.Queue()
        self.transcription_queue = queue.Queue()
        self.plan_queue = queue.Queue()

        # Setup logging
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)

    def start_pipeline(self, command: Optional[str] = None):
        """
        Start the VLA pipeline with an optional command.

        Args:
            command: Optional pre-defined command to execute
        """
        if self.is_running:
            self.logger.warning("Pipeline is already running")
            return

        self.is_running = True
        self.pipeline_stage = PipelineStage.LISTENING
        self.pipeline_result = None

        # Start the pipeline thread
        self.pipeline_thread = threading.Thread(
            target=self._run_pipeline,
            args=(command,),
            daemon=True
        )
        self.pipeline_thread.start()

    def stop_pipeline(self):
        """
        Stop the VLA pipeline.
        """
        self.is_running = False
        self.pipeline_stage = PipelineStage.IDLE

        # Stop audio recording if active
        if self.audio_handler.is_recording:
            self.audio_handler.stop_recording()

        # Wait for pipeline thread to finish
        if self.pipeline_thread and self.pipeline_thread.is_alive():
            self.pipeline_thread.join(timeout=2.0)

    def _run_pipeline(self, command: Optional[str] = None):
        """
        Run the VLA pipeline in a separate thread.

        Args:
            command: Optional pre-defined command to execute
        """
        start_time = time.time()

        try:
            # Update pipeline stage
            self._update_stage(PipelineStage.LISTENING)

            # Get voice command or use provided command
            if command is None:
                # Listen for voice command
                self.logger.info("Listening for voice command...")
                audio_data = self._capture_audio()
                if audio_data is None:
                    self._handle_error("Failed to capture audio")
                    return

                # Process audio
                self._update_stage(PipelineStage.PROCESSING_AUDIO)
                transcription = self._transcribe_audio(audio_data)
            else:
                # Use provided command
                transcription = command

            if transcription is None:
                self._handle_error("Failed to transcribe audio or invalid command")
                return

            # Update pipeline stage
            self._update_stage(PipelineStage.PLANNING)

            # Generate task plan
            task_plan = self._generate_task_plan(transcription)
            if task_plan is None:
                self._handle_error("Failed to generate task plan")
                return

            # Update pipeline stage
            self._update_stage(PipelineStage.EXECUTING)

            # Execute the plan
            execution_results = self._execute_task_plan(task_plan)

            # Calculate execution time
            execution_time = time.time() - start_time

            # Set pipeline result
            self.pipeline_result = PipelineResult(
                success=True,
                transcription=transcription,
                task_plan=task_plan,
                execution_results=execution_results,
                execution_time=execution_time
            )

            # Update pipeline stage
            self._update_stage(PipelineStage.COMPLETED)

            # Notify completion
            self._notify_completion(self.pipeline_result)

        except Exception as e:
            self._handle_error(f"Pipeline execution failed: {str(e)}")

    def _update_stage(self, new_stage: PipelineStage):
        """
        Update the pipeline stage and notify callbacks.

        Args:
            new_stage: New pipeline stage
        """
        old_stage = self.pipeline_stage
        self.pipeline_stage = new_stage

        # Update state manager
        self.state_manager.update_state({
            'current_stage': new_stage.value,
            'timestamp': time.time()
        })

        # Notify callbacks
        for callback in self.pipeline_callbacks['on_stage_change']:
            try:
                callback(old_stage, new_stage)
            except Exception as e:
                self.logger.error(f"Error in stage change callback: {e}")

    def _capture_audio(self) -> Optional[bytes]:
        """
        Capture audio from the microphone.

        Returns:
            Audio data as bytes, or None if failed
        """
        try:
            self.logger.info("Recording audio...")
            # Capture audio for a specified duration (default 5 seconds)
            audio_data = self.audio_handler.capture_audio(duration=5.0)
            self.logger.info(f"Captured {len(audio_data)} bytes of audio data")
            return audio_data

        except Exception as e:
            self.logger.error(f"Error capturing audio: {e}")
            return None

    def _transcribe_audio(self, audio_data: bytes) -> Optional[str]:
        """
        Transcribe audio data using Whisper.

        Args:
            audio_data: Audio data to transcribe

        Returns:
            Transcribed text, or None if failed
        """
        try:
            # In a real implementation, we would save the audio_data to a file
            # and then transcribe it. For simulation, we'll return a dummy response.
            # For now, let's assume the audio contains a command like "move forward 2 meters"
            simulated_transcription = "move forward 2 meters"

            # Process the transcription
            result = self.transcription_handler.process_transcription(
                text=simulated_transcription,
                confidence=0.9
            )

            if result is None:
                self.logger.error("Transcription validation failed")
                return None

            # Notify transcription
            for callback in self.pipeline_callbacks['on_transcription']:
                try:
                    callback(result)
                except Exception as e:
                    self.logger.error(f"Error in transcription callback: {e}")

            return result.text

        except Exception as e:
            self.logger.error(f"Error transcribing audio: {e}")
            return None

    def _generate_task_plan(self, command: str) -> Optional[TaskPlan]:
        """
        Generate a task plan from a command using the LLM.

        Args:
            command: Natural language command

        Returns:
            TaskPlan, or None if failed
        """
        try:
            if self.llm_client is None:
                # For simulation, create a dummy task plan
                self.logger.warning("LLM client not initialized, using simulation")
                return self._create_simulation_task_plan(command)

            # Generate task plan using LLM
            task_plan = self.llm_client.generate_task_plan(command)

            # Validate the task plan
            validated_plan = self.llm_client.validate_task_plan(task_plan, self.robot_controller.get_capabilities())

            # Notify task plan
            for callback in self.pipeline_callbacks['on_task_plan']:
                try:
                    callback(validated_plan)
                except Exception as e:
                    self.logger.error(f"Error in task plan callback: {e}")

            return validated_plan

        except Exception as e:
            self.logger.error(f"Error generating task plan: {e}")
            return None

    def _create_simulation_task_plan(self, command: str) -> TaskPlan:
        """
        Create a simulation task plan for testing purposes.

        Args:
            command: Natural language command

        Returns:
            Simulated TaskPlan
        """
        # Parse the command to extract actions
        parsed_tasks = self.task_parser.parse_complex_command(command)

        # Convert to ROS2 actions
        ros_actions = self.action_generator.generate_actions_from_tasks(
            [{'action_type': task.action_type, 'parameters': task.parameters} for task in parsed_tasks],
            self.robot_controller.get_capabilities()
        )

        # Create tasks from ROS actions
        tasks = []
        for i, action in enumerate(ros_actions):
            task = {
                'action_type': action.action_type,
                'parameters': action.parameters,
                'priority': action.priority,
                'dependencies': action.dependencies
            }
            tasks.append(task)

        return TaskPlan(
            id=f"sim_plan_{hash(command) % 10000}",
            original_command=command,
            tasks=tasks,
            validation_status="valid",
            robot_compatibility=True,
            confidence=0.8
        )

    def _execute_task_plan(self, task_plan: TaskPlan) -> List[Dict[str, Any]]:
        """
        Execute a task plan using the robot controller.

        Args:
            task_plan: Task plan to execute

        Returns:
            List of execution results
        """
        try:
            # Generate ROS2 actions from the task plan
            ros_actions = self.action_generator.generate_actions_from_tasks(
                task_plan.tasks,
                self.robot_controller.get_capabilities()
            )

            # Execute the actions
            results = []
            for action in ros_actions:
                self.logger.info(f"Executing action: {action.action_type} with params {action.parameters}")

                # Execute the action using the action client
                result = self.action_client.execute_action(action.action_type, action.parameters)
                results.append({
                    'action_id': action.id,
                    'action_type': action.action_type,
                    'parameters': action.parameters,
                    'result': result
                })

                # Notify execution
                for callback in self.pipeline_callbacks['on_execution']:
                    try:
                        callback(action, result)
                    except Exception as e:
                        self.logger.error(f"Error in execution callback: {e}")

                # Check if pipeline is still running
                if not self.is_running:
                    self.logger.info("Pipeline stopped during execution")
                    break

            return results

        except Exception as e:
            self.logger.error(f"Error executing task plan: {e}")
            return []

    def _handle_error(self, error_message: str):
        """
        Handle an error in the pipeline.

        Args:
            error_message: Error message to handle
        """
        self.logger.error(error_message)
        self.pipeline_stage = PipelineStage.ERROR

        # Set pipeline result with error
        self.pipeline_result = PipelineResult(
            success=False,
            error_message=error_message,
            execution_time=None
        )

        # Notify error
        for callback in self.pipeline_callbacks['on_error']:
            try:
                callback(error_message)
            except Exception as e:
                self.logger.error(f"Error in error callback: {e}")

    def _notify_completion(self, result: PipelineResult):
        """
        Notify that the pipeline has completed.

        Args:
            result: Pipeline result
        """
        for callback in self.pipeline_callbacks['on_completion']:
            try:
                callback(result)
            except Exception as e:
                self.logger.error(f"Error in completion callback: {e}")

    def add_pipeline_callback(self, event_type: str, callback: Callable):
        """
        Add a callback for a specific pipeline event.

        Args:
            event_type: Type of event ('on_stage_change', 'on_transcription', 'on_task_plan', 'on_execution', 'on_completion', 'on_error')
            callback: Callback function to add
        """
        if event_type in self.pipeline_callbacks:
            self.pipeline_callbacks[event_type].append(callback)
        else:
            raise ValueError(f"Unknown event type: {event_type}")

    def remove_pipeline_callback(self, event_type: str, callback: Callable):
        """
        Remove a callback for a specific pipeline event.

        Args:
            event_type: Type of event
            callback: Callback function to remove
        """
        if event_type in self.pipeline_callbacks:
            try:
                self.pipeline_callbacks[event_type].remove(callback)
            except ValueError:
                pass  # Callback was not registered

    def get_pipeline_status(self) -> Dict[str, Any]:
        """
        Get the current status of the pipeline.

        Returns:
            Dictionary with pipeline status information
        """
        return {
            'stage': self.pipeline_stage.value,
            'is_running': self.is_running,
            'result': self.pipeline_result,
            'robot_status': self.robot_controller.get_status().__dict__ if hasattr(self.robot_controller.get_status(), '__dict__') else str(self.robot_controller.get_status()),
            'state': self.state_manager.get_state()
        }

    def execute_voice_command(self, timeout: float = 30.0) -> PipelineResult:
        """
        Execute a voice command synchronously.

        Args:
            timeout: Maximum time to wait for completion in seconds

        Returns:
            PipelineResult with execution results
        """
        # Start the pipeline
        self.start_pipeline()

        # Wait for completion or timeout
        start_time = time.time()
        while self.is_running and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        # If still running after timeout, stop it
        if self.is_running:
            self.logger.warning("Pipeline execution timed out")
            self.stop_pipeline()

        return self.pipeline_result or PipelineResult(
            success=False,
            error_message="Pipeline execution timed out"
        )

    def execute_text_command(self, command: str) -> PipelineResult:
        """
        Execute a text command synchronously.

        Args:
            command: Text command to execute

        Returns:
            PipelineResult with execution results
        """
        # Start the pipeline with the command
        self.start_pipeline(command=command)

        # Wait for completion
        while self.is_running:
            time.sleep(0.1)

        return self.pipeline_result or PipelineResult(
            success=False,
            error_message="Pipeline execution failed"
        )


def main():
    """
    Main function to demonstrate the VLA Manager.
    """
    # Initialize the VLA Manager
    # In a real implementation, you would provide actual API keys
    vla_manager = VLAManager()

    # Add some callbacks to monitor the pipeline
    def on_stage_change(old_stage, new_stage):
        print(f"Pipeline stage changed: {old_stage.value} -> {new_stage.value}")

    def on_completion(result):
        print(f"Pipeline completed. Success: {result.success}")
        if result.transcription:
            print(f"Transcription: {result.transcription}")
        if result.execution_time:
            print(f"Execution time: {result.execution_time:.2f}s")

    def on_error(error_message):
        print(f"Pipeline error: {error_message}")

    vla_manager.add_pipeline_callback('on_stage_change', on_stage_change)
    vla_manager.add_pipeline_callback('on_completion', on_completion)
    vla_manager.add_pipeline_callback('on_error', on_error)

    # Execute a text command for demonstration
    print("Executing text command: 'move forward 2 meters'")
    result = vla_manager.execute_text_command("move forward 2 meters")

    print(f"Command execution result: {result.success}")
    if result.error_message:
        print(f"Error: {result.error_message}")

    # Get pipeline status
    status = vla_manager.get_pipeline_status()
    print(f"Pipeline status: {status['stage']}, running: {status['is_running']}")


if __name__ == "__main__":
    main()