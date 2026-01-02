"""
Pipeline orchestrator for VLA Integration Module.
Coordinates the Vision-Language-Action pipeline execution.
"""
import threading
import time
import queue
from typing import Dict, Any, Optional, Callable, List, Tuple
from dataclasses import dataclass
from enum import Enum
import logging
import asyncio
from concurrent.futures import ThreadPoolExecutor, as_completed

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
from vla_integration.vla_pipeline.vla_manager import VLAManager, PipelineStage, PipelineResult


class OrchestratorState(Enum):
    """
    Enum for orchestrator states.
    """
    IDLE = "idle"
    INITIALIZING = "initializing"
    PROCESSING = "processing"
    WAITING = "waiting"
    EXECUTING = "executing"
    COMPLETED = "completed"
    ERROR = "error"


@dataclass
class OrchestratorConfig:
    """
    Configuration for the pipeline orchestrator.
    """
    whisper_api_key: Optional[str] = None
    openai_api_key: Optional[str] = None
    min_transcription_confidence: float = 0.5
    max_concurrent_tasks: int = 5
    pipeline_timeout: float = 60.0
    enable_vision: bool = False  # Whether to enable vision processing
    vision_processing_delay: float = 0.5  # Delay for vision processing simulation


class PipelineOrchestrator:
    """
    Orchestrates the complete Vision-Language-Action pipeline with coordination between components.
    """

    def __init__(self, config: OrchestratorConfig):
        """
        Initialize the pipeline orchestrator.

        Args:
            config: OrchestratorConfig with configuration settings
        """
        self.config = config

        # Initialize components
        self.audio_handler = AudioInputHandler()
        self.whisper_client = WhisperClient(api_key=config.whisper_api_key) if config.whisper_api_key else None
        self.transcription_handler = TranscriptionHandler(min_confidence=config.min_transcription_confidence)
        self.llm_client = LLMClient(api_key=config.openai_api_key) if config.openai_api_key else None
        self.task_parser = TaskParser()
        self.action_generator = ActionGenerator()
        self.action_client = ROS2ActionClient()
        self.robot_controller = RobotController()
        self.feedback_handler = FeedbackHandler()
        self.state_manager = StateManager()
        self.vla_manager = VLAManager(
            whisper_api_key=config.whisper_api_key,
            openai_api_key=config.openai_api_key
        )

        # Orchestrator state
        self.orchestrator_state = OrchestratorState.IDLE
        self.pipeline_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.executor = ThreadPoolExecutor(max_workers=config.max_concurrent_tasks)
        self.active_pipelines = {}
        self.pipeline_futures = {}

        # Callbacks
        self.orchestrator_callbacks = {
            'on_pipeline_start': [],
            'on_pipeline_complete': [],
            'on_pipeline_error': [],
            'on_state_change': [],
            'on_feedback': []
        }

        # Setup logging
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)

    def submit_pipeline(self, command: str, pipeline_id: Optional[str] = None) -> str:
        """
        Submit a command for pipeline processing.

        Args:
            command: Natural language command to process
            pipeline_id: Optional ID for the pipeline (generated if not provided)

        Returns:
            ID of the submitted pipeline
        """
        if pipeline_id is None:
            pipeline_id = f"pipeline_{int(time.time())}_{len(self.active_pipelines)}"

        # Create pipeline task
        pipeline_task = {
            'id': pipeline_id,
            'command': command,
            'submitted_at': time.time(),
            'status': 'submitted'
        }

        # Add to queue
        self.pipeline_queue.put(pipeline_task)
        self.active_pipelines[pipeline_id] = pipeline_task

        # Update state
        self._update_state(OrchestratorState.PROCESSING)

        # Notify pipeline start
        for callback in self.orchestrator_callbacks['on_pipeline_start']:
            try:
                callback(pipeline_id, command)
            except Exception as e:
                self.logger.error(f"Error in pipeline start callback: {e}")

        return pipeline_id

    def process_pipeline_queue(self, max_pipelines: int = 1):
        """
        Process pipelines from the queue up to max_pipelines.

        Args:
            max_pipelines: Maximum number of pipelines to process concurrently
        """
        processed = 0
        futures_to_check = []

        # Check for completed futures
        completed_futures = []
        for future in self.pipeline_futures.values():
            if future.done():
                completed_futures.append(future)

        # Process completed futures
        for future in completed_futures:
            try:
                result = future.result()
                # Handle result
                self.result_queue.put(result)
            except Exception as e:
                self.logger.error(f"Error in pipeline future: {e}")
            # Remove from active futures
            for pid, f in list(self.pipeline_futures.items()):
                if f == future:
                    del self.pipeline_futures[pid]
                    break

        # Submit new pipelines up to max_pipelines
        while not self.pipeline_queue.empty() and len(self.pipeline_futures) < max_pipelines and processed < max_pipelines:
            try:
                pipeline_task = self.pipeline_queue.get_nowait()
                pipeline_id = pipeline_task['id']
                command = pipeline_task['command']

                # Submit to executor
                future = self.executor.submit(self._execute_single_pipeline, pipeline_id, command)
                self.pipeline_futures[pipeline_id] = future
                processed += 1

            except queue.Empty:
                break
            except Exception as e:
                self.logger.error(f"Error submitting pipeline: {e}")
                break

    def _execute_single_pipeline(self, pipeline_id: str, command: str) -> Dict[str, Any]:
        """
        Execute a single pipeline.

        Args:
            pipeline_id: ID of the pipeline
            command: Command to execute

        Returns:
            Dictionary with pipeline result
        """
        try:
            self.logger.info(f"Executing pipeline {pipeline_id} with command: {command}")

            # Update pipeline status
            if pipeline_id in self.active_pipelines:
                self.active_pipelines[pipeline_id]['status'] = 'executing'
                self.active_pipelines[pipeline_id]['started_at'] = time.time()

            # Execute using VLA Manager
            result = self.vla_manager.execute_text_command(command)

            # Update pipeline status
            if pipeline_id in self.active_pipelines:
                self.active_pipelines[pipeline_id]['status'] = 'completed'
                self.active_pipelines[pipeline_id]['completed_at'] = time.time()
                self.active_pipelines[pipeline_id]['result'] = result

            # Notify completion
            for callback in self.orchestrator_callbacks['on_pipeline_complete']:
                try:
                    callback(pipeline_id, result)
                except Exception as e:
                    self.logger.error(f"Error in pipeline completion callback: {e}")

            return {
                'pipeline_id': pipeline_id,
                'command': command,
                'result': result,
                'success': result.success if result else False
            }

        except Exception as e:
            self.logger.error(f"Error executing pipeline {pipeline_id}: {e}")

            # Update pipeline status
            if pipeline_id in self.active_pipelines:
                self.active_pipelines[pipeline_id]['status'] = 'error'
                self.active_pipelines[pipeline_id]['error'] = str(e)

            # Notify error
            for callback in self.orchestrator_callbacks['on_pipeline_error']:
                try:
                    callback(pipeline_id, str(e))
                except Exception as e2:
                    self.logger.error(f"Error in pipeline error callback: {e2}")

            return {
                'pipeline_id': pipeline_id,
                'command': command,
                'result': None,
                'success': False,
                'error': str(e)
            }

    def process_multimodal_input(self, voice_command: Optional[str] = None,
                                vision_data: Optional[Dict[str, Any]] = None) -> str:
        """
        Process multimodal input combining voice and vision data.

        Args:
            voice_command: Optional voice command
            vision_data: Optional vision data

        Returns:
            ID of the submitted pipeline
        """
        # Combine voice and vision data
        combined_context = {}

        if vision_data:
            combined_context['vision'] = vision_data

        if voice_command:
            combined_context['voice_command'] = voice_command

        # For now, we'll use just the voice command
        # In a full implementation, we would integrate vision data with the command
        command = voice_command or "default command"

        # Submit pipeline with context
        pipeline_id = self.submit_pipeline(command)

        return pipeline_id

    def get_pipeline_status(self, pipeline_id: str) -> Dict[str, Any]:
        """
        Get the status of a specific pipeline.

        Args:
            pipeline_id: ID of the pipeline to check

        Returns:
            Dictionary with pipeline status
        """
        if pipeline_id in self.active_pipelines:
            pipeline = self.active_pipelines[pipeline_id]
            return {
                'id': pipeline_id,
                'status': pipeline['status'],
                'command': pipeline.get('command', ''),
                'submitted_at': pipeline.get('submitted_at'),
                'started_at': pipeline.get('started_at'),
                'completed_at': pipeline.get('completed_at'),
                'result': pipeline.get('result')
            }
        else:
            return {
                'id': pipeline_id,
                'status': 'not_found',
                'error': f'Pipeline {pipeline_id} not found'
            }

    def get_all_pipeline_statuses(self) -> Dict[str, Dict[str, Any]]:
        """
        Get the status of all active pipelines.

        Returns:
            Dictionary mapping pipeline IDs to their status
        """
        statuses = {}
        for pipeline_id in self.active_pipelines:
            statuses[pipeline_id] = self.get_pipeline_status(pipeline_id)
        return statuses

    def cancel_pipeline(self, pipeline_id: str) -> bool:
        """
        Cancel a running pipeline.

        Args:
            pipeline_id: ID of the pipeline to cancel

        Returns:
            True if pipeline was cancelled, False otherwise
        """
        if pipeline_id in self.pipeline_futures:
            future = self.pipeline_futures[pipeline_id]
            cancelled = future.cancel()

            if cancelled:
                # Remove from active pipelines
                if pipeline_id in self.active_pipelines:
                    self.active_pipelines[pipeline_id]['status'] = 'cancelled'
                    self.active_pipelines[pipeline_id]['cancelled_at'] = time.time()

                self.logger.info(f"Pipeline {pipeline_id} was cancelled")
            else:
                self.logger.warning(f"Failed to cancel pipeline {pipeline_id}")

            return cancelled
        else:
            self.logger.warning(f"Pipeline {pipeline_id} not found for cancellation")
            return False

    def _update_state(self, new_state: OrchestratorState):
        """
        Update the orchestrator state and notify callbacks.

        Args:
            new_state: New orchestrator state
        """
        old_state = self.orchestrator_state
        self.orchestrator_state = new_state

        # Notify callbacks
        for callback in self.orchestrator_callbacks['on_state_change']:
            try:
                callback(old_state, new_state)
            except Exception as e:
                self.logger.error(f"Error in state change callback: {e}")

    def add_orchestrator_callback(self, event_type: str, callback: Callable):
        """
        Add a callback for a specific orchestrator event.

        Args:
            event_type: Type of event ('on_pipeline_start', 'on_pipeline_complete', 'on_pipeline_error', 'on_state_change', 'on_feedback')
            callback: Callback function to add
        """
        if event_type in self.orchestrator_callbacks:
            self.orchestrator_callbacks[event_type].append(callback)
        else:
            raise ValueError(f"Unknown event type: {event_type}")

    def remove_orchestrator_callback(self, event_type: str, callback: Callable):
        """
        Remove a callback for a specific orchestrator event.

        Args:
            event_type: Type of event
            callback: Callback function to remove
        """
        if event_type in self.orchestrator_callbacks:
            try:
                self.orchestrator_callbacks[event_type].remove(callback)
            except ValueError:
                pass  # Callback was not registered

    def get_orchestrator_status(self) -> Dict[str, Any]:
        """
        Get the current status of the orchestrator.

        Returns:
            Dictionary with orchestrator status information
        """
        return {
            'state': self.orchestrator_state.value,
            'active_pipelines': len(self.active_pipelines),
            'queued_pipelines': self.pipeline_queue.qsize(),
            'completed_pipelines': len([p for p in self.active_pipelines.values() if p['status'] == 'completed']),
            'error_pipelines': len([p for p in self.active_pipelines.values() if p['status'] == 'error']),
            'config': self.config.__dict__,
            'timestamp': time.time()
        }

    def shutdown(self):
        """
        Shutdown the orchestrator and clean up resources.
        """
        self.logger.info("Shutting down pipeline orchestrator")

        # Cancel all active pipelines
        for pipeline_id in list(self.pipeline_futures.keys()):
            self.cancel_pipeline(pipeline_id)

        # Shutdown executor
        self.executor.shutdown(wait=True)

        # Update state
        self._update_state(OrchestratorState.IDLE)

        self.logger.info("Pipeline orchestrator shutdown complete")

    def run_continuous_processing(self, max_concurrent: int = 1):
        """
        Run continuous pipeline processing.

        Args:
            max_concurrent: Maximum number of pipelines to process concurrently
        """
        self.logger.info(f"Starting continuous pipeline processing (max {max_concurrent} concurrent)")

        try:
            while True:
                # Process the pipeline queue
                self.process_pipeline_queue(max_pipelines=max_concurrent)

                # Small delay to prevent busy waiting
                time.sleep(0.1)

        except KeyboardInterrupt:
            self.logger.info("Received interrupt signal, shutting down...")
            self.shutdown()


def main():
    """
    Main function to demonstrate the pipeline orchestrator.
    """
    # Create orchestrator configuration
    config = OrchestratorConfig(
        whisper_api_key=None,  # Would be set to actual API key in production
        openai_api_key=None,   # Would be set to actual API key in production
        min_transcription_confidence=0.5,
        max_concurrent_tasks=3,
        pipeline_timeout=60.0
    )

    # Initialize the orchestrator
    orchestrator = PipelineOrchestrator(config)

    # Add some callbacks to monitor the orchestrator
    def on_pipeline_start(pipeline_id, command):
        print(f"Pipeline {pipeline_id} started with command: {command}")

    def on_pipeline_complete(pipeline_id, result):
        print(f"Pipeline {pipeline_id} completed. Success: {result.success if result else False}")

    def on_pipeline_error(pipeline_id, error):
        print(f"Pipeline {pipeline_id} error: {error}")

    def on_state_change(old_state, new_state):
        print(f"Orchestrator state changed: {old_state.value} -> {new_state.value}")

    orchestrator.add_orchestrator_callback('on_pipeline_start', on_pipeline_start)
    orchestrator.add_orchestrator_callback('on_pipeline_complete', on_pipeline_complete)
    orchestrator.add_orchestrator_callback('on_pipeline_error', on_pipeline_error)
    orchestrator.add_orchestrator_callback('on_state_change', on_state_change)

    # Submit a few test pipelines
    print("Submitting test pipelines...")
    pipeline_ids = []
    for i, command in enumerate(["move forward 1 meter", "turn left", "speak hello"]):
        pid = orchestrator.submit_pipeline(command)
        pipeline_ids.append(pid)
        print(f"Submitted pipeline {pid} with command: {command}")

    # Process the pipelines
    print("\nProcessing pipelines...")
    orchestrator.process_pipeline_queue(max_pipelines=2)

    # Check statuses
    print("\nPipeline statuses:")
    for pid in pipeline_ids:
        status = orchestrator.get_pipeline_status(pid)
        print(f"  {pid}: {status['status']}")

    # Get orchestrator status
    print("\nOrchestrator status:")
    status = orchestrator.get_orchestrator_status()
    print(f"  State: {status['state']}")
    print(f"  Active pipelines: {status['active_pipelines']}")
    print(f"  Queued pipelines: {status['queued_pipelines']}")
    print(f"  Completed pipelines: {status['completed_pipelines']}")
    print(f"  Error pipelines: {status['error_pipelines']}")

    # Shutdown
    orchestrator.shutdown()


if __name__ == "__main__":
    main()