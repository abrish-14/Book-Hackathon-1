#!/usr/bin/env python3
"""
Main entry point for the VLA Integration Module.
Coordinates the complete Vision-Language-Action pipeline from voice input to robot action execution.
"""
import sys
import os
import signal
import time
import threading
from typing import Optional

# Add the project root to the Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from vla_integration.vla_pipeline.vla_manager import VLAManager
from vla_integration.vla_pipeline.pipeline_orchestrator import PipelineOrchestrator, OrchestratorConfig
from vla_integration.config.settings import config
from vla_integration.simulation.sim_environment import SimulationEnvironment


class VLAIntegrationSystem:
    """
    Main system class that integrates all VLA components into a cohesive pipeline.
    """

    def __init__(self, api_key: Optional[str] = None):
        """
        Initialize the VLA Integration System.

        Args:
            api_key: OpenAI API key for LLM and Whisper services
        """
        self.api_key = api_key
        self.vla_manager = None
        self.orchestrator = None
        self.sim_env = None
        self.running = False

        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """
        Handle shutdown signals gracefully.
        """
        print(f"\nReceived signal {signum}, shutting down gracefully...")
        self.shutdown()
        sys.exit(0)

    def initialize_components(self):
        """
        Initialize all VLA components.
        """
        print("Initializing VLA Integration System...")

        # Initialize VLA Manager
        print("  Initializing VLA Manager...")
        self.vla_manager = VLAManager(
            whisper_api_key=self.api_key,
            openai_api_key=self.api_key
        )

        # Add callbacks for monitoring
        def on_stage_change(old_stage, new_stage):
            print(f"    Pipeline stage: {old_stage.value} -> {new_stage.value}")

        def on_completion(result):
            print(f"    Pipeline completed. Success: {result.success}")
            if result.transcription:
                print(f"    Transcription: {result.transcription}")

        def on_error(error_message):
            print(f"    Pipeline error: {error_message}")

        self.vla_manager.add_pipeline_callback('on_stage_change', on_stage_change)
        self.vla_manager.add_pipeline_callback('on_completion', on_completion)
        self.vla_manager.add_pipeline_callback('on_error', on_error)

        # Initialize Pipeline Orchestrator
        print("  Initializing Pipeline Orchestrator...")
        orchestrator_config = OrchestratorConfig(
            whisper_api_key=self.api_key,
            openai_api_key=self.api_key,
            min_transcription_confidence=config.min_transcription_confidence,
            max_concurrent_tasks=config.max_concurrent_pipelines,
            pipeline_timeout=config.pipeline_timeout
        )
        self.orchestrator = PipelineOrchestrator(orchestrator_config)

        # Initialize Simulation Environment if enabled
        if config.enable_simulation:
            print("  Initializing Simulation Environment...")
            self.sim_env = SimulationEnvironment()
            self.sim_env.start_simulation()

        print("VLA Integration System initialized successfully!")

    def execute_voice_command(self, timeout: float = 30.0) -> bool:
        """
        Execute a voice command using the complete VLA pipeline.

        Args:
            timeout: Maximum time to wait for completion in seconds

        Returns:
            True if successful, False otherwise
        """
        if not self.vla_manager:
            print("Error: VLA Manager not initialized")
            return False

        print("\nListening for voice command...")
        print("Please speak your command now...")

        try:
            result = self.vla_manager.execute_voice_command(timeout=timeout)
            print(f"\nCommand execution result: {result.success}")
            if result.error_message:
                print(f"Error: {result.error_message}")
            return result.success
        except Exception as e:
            print(f"Error executing voice command: {e}")
            return False

    def execute_text_command(self, command: str) -> bool:
        """
        Execute a text command using the complete VLA pipeline.

        Args:
            command: Text command to execute

        Returns:
            True if successful, False otherwise
        """
        if not self.vla_manager:
            print("Error: VLA Manager not initialized")
            return False

        print(f"\nExecuting text command: '{command}'")

        try:
            result = self.vla_manager.execute_text_command(command)
            print(f"Command execution result: {result.success}")
            if result.error_message:
                print(f"Error: {result.error_message}")
            return result.success
        except Exception as e:
            print(f"Error executing text command: {e}")
            return False

    def run_continuous_mode(self):
        """
        Run the system in continuous mode, listening for commands.
        """
        print("\nStarting continuous mode...")
        print("Type 'help' for available commands, 'quit' to exit")

        self.running = True

        while self.running:
            try:
                user_input = input("\nEnter command (or 'quit' to exit): ").strip()

                if user_input.lower() in ['quit', 'exit', 'q']:
                    print("Shutting down...")
                    break
                elif user_input.lower() == 'help':
                    self._show_help()
                elif user_input.lower() == 'status':
                    self._show_status()
                elif user_input.lower() == 'voice':
                    self.execute_voice_command()
                elif user_input:
                    # Treat as text command
                    self.execute_text_command(user_input)

            except KeyboardInterrupt:
                print("\nShutting down...")
                break
            except EOFError:
                print("\nShutting down...")
                break

    def _show_help(self):
        """
        Show help information.
        """
        print("\nAvailable commands:")
        print("  [any text]     - Execute a text command (e.g., 'move forward 1 meter')")
        print("  voice          - Enter voice command mode")
        print("  status         - Show system status")
        print("  help           - Show this help message")
        print("  quit/exit/q    - Exit the system")

    def _show_status(self):
        """
        Show system status.
        """
        if self.vla_manager:
            status = self.vla_manager.get_pipeline_status()
            print(f"\nVLA Manager Status:")
            print(f"  Stage: {status['stage']}")
            print(f"  Running: {status['is_running']}")
            print(f"  Robot Status: {status['robot_status']}")

        if self.orchestrator:
            status = self.orchestrator.get_orchestrator_status()
            print(f"\nOrchestrator Status:")
            print(f"  State: {status['state']}")
            print(f"  Active Pipelines: {status['active_pipelines']}")
            print(f"  Queued Pipelines: {status['queued_pipelines']}")

        if self.sim_env:
            stats = self.sim_env.get_simulation_stats()
            print(f"\nSimulation Status:")
            print(f"  Running: {stats['is_running']}")
            print(f"  Objects: {stats['total_objects']}")
            print(f"  Battery: {stats['robot_battery_level']:.2%}")

    def shutdown(self):
        """
        Shutdown the system gracefully.
        """
        print("Shutting down VLA Integration System...")

        if self.orchestrator:
            self.orchestrator.shutdown()
            print("  Pipeline orchestrator shutdown complete")

        if self.sim_env:
            self.sim_env.stop_simulation()
            print("  Simulation environment shutdown complete")

        print("VLA Integration System shutdown complete")
        self.running = False


def main():
    """
    Main function to run the VLA Integration System.
    """
    print("VLA Integration Module - Vision-Language-Action for Robotics")
    print("=" * 60)

    # Check if we're running in simulation mode
    if config.enable_simulation:
        print("Running in SIMULATION mode (no API keys required)")
        api_key = None
    else:
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            print("Warning: OPENAI_API_KEY environment variable not set")
            print("Running in SIMULATION mode...")
            api_key = None

    # Create and initialize the system
    system = VLAIntegrationSystem(api_key=api_key)

    try:
        system.initialize_components()

        # Run in continuous mode
        system.run_continuous_mode()

    except Exception as e:
        print(f"Error running VLA system: {e}")
        import traceback
        traceback.print_exc()
    finally:
        system.shutdown()


if __name__ == "__main__":
    main()