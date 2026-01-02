#!/usr/bin/env python3
"""
Test script for basic voice command functionality with simulation environment.
"""
import sys
import os
import time

# Add the project root to the Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from vla_integration.vla_pipeline.vla_manager import VLAManager
from vla_integration.config.settings import config


def test_basic_voice_command():
    """
    Test basic voice command functionality using simulation.
    """
    print("=== Testing Basic Voice Command Functionality ===")

    # Create VLA Manager with simulation mode (no API keys needed)
    print("Initializing VLA Manager...")
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
        if result.execution_results:
            print(f"Execution results: {len(result.execution_results)} actions executed")

    def on_error(error_message):
        print(f"Pipeline error: {error_message}")

    vla_manager.add_pipeline_callback('on_stage_change', on_stage_change)
    vla_manager.add_pipeline_callback('on_completion', on_completion)
    vla_manager.add_pipeline_callback('on_error', on_error)

    # Test 1: Execute a text command
    print("\n--- Test 1: Text Command 'move forward 1 meter' ---")
    result = vla_manager.execute_text_command("move forward 1 meter")
    print(f"Command execution result: {result.success}")
    if result.error_message:
        print(f"Error: {result.error_message}")

    # Get pipeline status
    status = vla_manager.get_pipeline_status()
    print(f"Pipeline status: {status['stage']}, running: {status['is_running']}")

    # Test 2: Execute another text command
    print("\n--- Test 2: Text Command 'turn left' ---")
    result = vla_manager.execute_text_command("turn left")
    print(f"Command execution result: {result.success}")
    if result.error_message:
        print(f"Error: {result.error_message}")

    # Test 3: Execute a speak command
    print("\n--- Test 3: Text Command 'speak hello world' ---")
    result = vla_manager.execute_text_command("speak hello world")
    print(f"Command execution result: {result.success}")
    if result.error_message:
        print(f"Error: {result.error_message}")

    print("\n=== Basic voice command testing completed ===")


def test_simulation_environment():
    """
    Test the simulation environment directly.
    """
    print("\n=== Testing Simulation Environment ===")

    from vla_integration.simulation.sim_environment import SimulationEnvironment, SimActionType

    # Create simulation environment
    sim_env = SimulationEnvironment()
    sim_env.start_simulation()

    print("\n1. Current robot status:")
    status = sim_env.get_robot_status()
    print(f"  Position: {status['position']}")
    print(f"  Battery: {status['battery_level']:.2%}")

    print("\n2. Executing MOVE action:")
    move_result = sim_env.execute_action(
        SimActionType.MOVE,
        {'vector': [2.0, 0.0, 0.0], 'speed': 0.5, 'relative': True}
    )
    print(f"  Result: {move_result.success}, Message: {move_result.message}")
    print(f"  New position: {sim_env.get_robot_status()['position']}")

    print("\n3. Executing SPEAK action:")
    speak_result = sim_env.execute_action(
        SimActionType.SPEAK,
        {'text': 'Hello, I am testing the simulation environment.', 'volume': 0.8}
    )
    print(f"  Result: {speak_result.success}, Message: {speak_result.message}")

    print("\n4. Executing NAVIGATE action to kitchen:")
    navigate_result = sim_env.execute_action(
        SimActionType.NAVIGATE,
        {'location_name': 'kitchen', 'goal_tolerance': 0.5}
    )
    print(f"  Result: {navigate_result.success}, Message: {navigate_result.message}")
    print(f"  New position: {sim_env.get_robot_status()['position']}")

    # Stop the simulation
    sim_env.stop_simulation()
    print("\n=== Simulation environment testing completed ===")


if __name__ == "__main__":
    # Test the basic voice command functionality
    test_basic_voice_command()

    # Test the simulation environment
    test_simulation_environment()

    print("\nAll tests completed successfully!")