"""
State manager for VLA Integration Module.
Manages the state of the VLA pipeline.
"""
import threading
import time
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
from enum import Enum
import json
from datetime import datetime


class PipelineState(Enum):
    """
    Enum for different states of the VLA pipeline.
    """
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING_AUDIO = "processing_audio"
    TRANSCRIBING = "transcribing"
    PLANNING = "planning"
    EXECUTING = "executing"
    COMPLETED = "completed"
    ERROR = "error"
    PAUSED = "paused"
    CANCELLED = "cancelled"


@dataclass
class StateSnapshot:
    """
    Data class to hold a snapshot of the pipeline state.
    """
    state: PipelineState
    timestamp: float
    data: Dict[str, Any]
    stage_progress: float = 0.0


class StateManager:
    """
    Manages the state of the VLA pipeline, including history and transitions.
    """

    def __init__(self, max_history: int = 100):
        """
        Initialize the state manager.

        Args:
            max_history: Maximum number of state snapshots to keep in history
        """
        self._current_state = PipelineState.IDLE
        self._state_data: Dict[str, Any] = {}
        self._state_history: List[StateSnapshot] = []
        self._state_lock = threading.Lock()
        self._max_history = max_history
        self._transition_callbacks = []
        self._state_change_callbacks = []

        # Add initial state
        self._add_state_snapshot(self._current_state, self._state_data)

    def update_state(self, data: Optional[Dict[str, Any]] = None, state: Optional[PipelineState] = None):
        """
        Update the pipeline state with new data.

        Args:
            data: Optional dictionary with state data to update
            state: Optional new state to set (if not provided, current state is maintained)
        """
        with self._state_lock:
            old_state = self._current_state

            # Update state if provided
            if state is not None:
                self._current_state = state

            # Update data if provided
            if data is not None:
                self._state_data.update(data)

            # Add to history
            self._add_state_snapshot(self._current_state, self._state_data)

            # Notify callbacks
            self._notify_state_change_callbacks(old_state, self._current_state, self._state_data)
            if old_state != self._current_state:
                self._notify_transition_callbacks(old_state, self._current_state)

    def get_state(self) -> Dict[str, Any]:
        """
        Get the current pipeline state.

        Returns:
            Dictionary with current state information
        """
        with self._state_lock:
            return {
                'state': self._current_state.value,
                'data': self._state_data.copy(),
                'timestamp': time.time()
            }

    def get_current_state(self) -> PipelineState:
        """
        Get the current pipeline state enum.

        Returns:
            Current PipelineState
        """
        with self._state_lock:
            return self._current_state

    def set_state_data(self, key: str, value: Any):
        """
        Set a specific piece of state data.

        Args:
            key: Key for the data item
            value: Value to set
        """
        with self._state_lock:
            self._state_data[key] = value
            self._add_state_snapshot(self._current_state, self._state_data)

    def get_state_data(self, key: str, default: Any = None) -> Any:
        """
        Get a specific piece of state data.

        Args:
            key: Key for the data item
            default: Default value to return if key not found

        Returns:
            Value of the data item or default
        """
        with self._state_lock:
            return self._state_data.get(key, default)

    def get_state_history(self, limit: Optional[int] = None) -> List[StateSnapshot]:
        """
        Get the state history.

        Args:
            limit: Maximum number of snapshots to return (None for all)

        Returns:
            List of StateSnapshot objects
        """
        with self._state_lock:
            if limit is None:
                return self._state_history.copy()
            else:
                return self._state_history[-limit:].copy()

    def get_state_transitions(self) -> List[Dict[str, Any]]:
        """
        Get the sequence of state transitions.

        Returns:
            List of dictionaries with transition information
        """
        with self._state_lock:
            transitions = []
            for i in range(1, len(self._state_history)):
                from_state = self._state_history[i-1]
                to_state = self._state_history[i]

                if from_state.state != to_state.state:
                    transitions.append({
                        'from': from_state.state.value,
                        'to': to_state.state.value,
                        'timestamp': to_state.timestamp,
                        'duration': to_state.timestamp - from_state.timestamp,
                        'data': to_state.data
                    })

            return transitions

    def is_in_state(self, state: PipelineState) -> bool:
        """
        Check if the pipeline is currently in a specific state.

        Args:
            state: PipelineState to check

        Returns:
            True if in the specified state, False otherwise
        """
        with self._state_lock:
            return self._current_state == state

    def is_in_any_state(self, states: List[PipelineState]) -> bool:
        """
        Check if the pipeline is currently in any of the specified states.

        Args:
            states: List of PipelineState to check

        Returns:
            True if in any of the specified states, False otherwise
        """
        with self._state_lock:
            return self._current_state in states

    def wait_for_state(self, target_state: PipelineState, timeout: float = 10.0) -> bool:
        """
        Wait for the pipeline to reach a specific state.

        Args:
            target_state: PipelineState to wait for
            timeout: Maximum time to wait in seconds

        Returns:
            True if state was reached within timeout, False otherwise
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_in_state(target_state):
                return True
            time.sleep(0.1)
        return False

    def wait_for_state_change(self, timeout: float = 10.0) -> bool:
        """
        Wait for the pipeline state to change from the current state.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if state changed within timeout, False otherwise
        """
        initial_state = self.get_current_state()
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.get_current_state() != initial_state:
                return True
            time.sleep(0.1)
        return False

    def add_state_change_callback(self, callback: callable):
        """
        Add a callback to be called when the state changes.

        Args:
            callback: Function to call when state changes, with signature:
                     callback(old_state: PipelineState, new_state: PipelineState, data: Dict[str, Any])
        """
        with self._state_lock:
            self._state_change_callbacks.append(callback)

    def remove_state_change_callback(self, callback: callable):
        """
        Remove a state change callback.

        Args:
            callback: Function to remove
        """
        with self._state_lock:
            try:
                self._state_change_callbacks.remove(callback)
            except ValueError:
                pass  # Callback was not registered

    def add_transition_callback(self, callback: callable):
        """
        Add a callback to be called when the state transitions.

        Args:
            callback: Function to call when state transitions, with signature:
                     callback(from_state: PipelineState, to_state: PipelineState)
        """
        with self._state_lock:
            self._transition_callbacks.append(callback)

    def remove_transition_callback(self, callback: callable):
        """
        Remove a transition callback.

        Args:
            callback: Function to remove
        """
        with self._state_lock:
            try:
                self._transition_callbacks.remove(callback)
            except ValueError:
                pass  # Callback was not registered

    def _add_state_snapshot(self, state: PipelineState, data: Dict[str, Any]):
        """
        Add a state snapshot to the history.

        Args:
            state: Current state
            data: Current state data
        """
        snapshot = StateSnapshot(
            state=state,
            timestamp=time.time(),
            data=data.copy()
        )

        self._state_history.append(snapshot)

        # Maintain history size limit
        if len(self._state_history) > self._max_history:
            self._state_history.pop(0)

    def _notify_state_change_callbacks(self, old_state: PipelineState, new_state: PipelineState, data: Dict[str, Any]):
        """
        Notify all state change callbacks.

        Args:
            old_state: Previous state
            new_state: New state
            data: State data
        """
        for callback in self._state_change_callbacks:
            try:
                callback(old_state, new_state, data)
            except Exception as e:
                print(f"Error in state change callback: {e}")

    def _notify_transition_callbacks(self, from_state: PipelineState, to_state: PipelineState):
        """
        Notify all transition callbacks.

        Args:
            from_state: Previous state
            to_state: New state
        """
        for callback in self._transition_callbacks:
            try:
                callback(from_state, to_state)
            except Exception as e:
                print(f"Error in transition callback: {e}")

    def reset_state(self):
        """
        Reset the state to IDLE with empty data.
        """
        with self._state_lock:
            old_state = self._current_state
            self._current_state = PipelineState.IDLE
            self._state_data = {}

            self._add_state_snapshot(self._current_state, self._state_data)

            # Notify callbacks
            self._notify_state_change_callbacks(old_state, self._current_state, self._state_data)
            if old_state != self._current_state:
                self._notify_transition_callbacks(old_state, self._current_state)

    def get_state_duration(self, state: PipelineState) -> float:
        """
        Get the total duration spent in a specific state.

        Args:
            state: PipelineState to check

        Returns:
            Total duration in seconds spent in the state
        """
        with self._state_lock:
            total_duration = 0.0
            state_start_time = None

            for snapshot in self._state_history:
                if snapshot.state == state:
                    if state_start_time is None:
                        state_start_time = snapshot.timestamp
                else:
                    if state_start_time is not None:
                        total_duration += snapshot.timestamp - state_start_time
                        state_start_time = None

            # If still in the state, add time until now
            if state_start_time is not None and self._current_state == state:
                total_duration += time.time() - state_start_time

            return total_duration

    def get_state_summary(self) -> Dict[str, Any]:
        """
        Get a summary of state information.

        Returns:
            Dictionary with state summary information
        """
        with self._state_lock:
            summary = {
                'current_state': self._current_state.value,
                'current_state_duration': self.get_state_duration(self._current_state),
                'total_state_changes': len(self._state_history) - 1,  # -1 because first entry doesn't count as change
                'state_transitions_count': len(self.get_state_transitions()),
                'history_size': len(self._state_history),
                'max_history_size': self._max_history,
                'timestamp': time.time()
            }

            # Add duration for each state
            for state in PipelineState:
                summary[f'duration_{state.value}'] = self.get_state_duration(state)

            return summary

    def export_state_history(self) -> str:
        """
        Export the state history as a JSON string.

        Returns:
            JSON string representation of the state history
        """
        with self._state_lock:
            history_data = []
            for snapshot in self._state_history:
                history_data.append({
                    'state': snapshot.state.value,
                    'timestamp': snapshot.timestamp,
                    'data': snapshot.data,
                    'stage_progress': snapshot.stage_progress
                })

            return json.dumps(history_data, indent=2)

    def import_state_history(self, json_string: str):
        """
        Import state history from a JSON string.

        Args:
            json_string: JSON string representation of state history
        """
        with self._state_lock:
            history_data = json.loads(json_string)

            self._state_history = []
            for item in history_data:
                snapshot = StateSnapshot(
                    state=PipelineState(item['state']),
                    timestamp=item['timestamp'],
                    data=item['data'],
                    stage_progress=item.get('stage_progress', 0.0)
                )
                self._state_history.append(snapshot)

            # Update current state and data from the last snapshot
            if self._state_history:
                last_snapshot = self._state_history[-1]
                self._current_state = last_snapshot.state
                self._state_data = last_snapshot.data.copy()


# Example usage and testing
def example_state_change_callback(old_state: PipelineState, new_state: PipelineState, data: Dict[str, Any]):
    """
    Example callback for state changes.
    """
    print(f"State changed from {old_state.value} to {new_state.value}")
    if data:
        print(f"  Data: {data}")


def example_transition_callback(from_state: PipelineState, to_state: PipelineState):
    """
    Example callback for state transitions.
    """
    print(f"Transition: {from_state.value} -> {to_state.value}")


def main():
    """
    Main function to demonstrate the state manager functionality.
    """
    print("=== State Manager Demo ===")

    # Create state manager
    state_manager = StateManager(max_history=50)

    # Add callbacks
    state_manager.add_state_change_callback(example_state_change_callback)
    state_manager.add_transition_callback(example_transition_callback)

    # Update state with data
    print("\n1. Updating to LISTENING state...")
    state_manager.update_state(
        data={'message': 'Starting to listen for voice command', 'timestamp': time.time()},
        state=PipelineState.LISTENING
    )

    # Update with more data without changing state
    print("\n2. Updating data without changing state...")
    state_manager.set_state_data('volume_level', 0.8)
    state_manager.set_state_data('sensitivity', 'high')
    time.sleep(0.5)  # Small delay to see timestamp difference
    state_manager.update_state(data={'message': 'Still listening...'})

    # Move to processing state
    print("\n3. Moving to PROCESSING_AUDIO state...")
    state_manager.update_state(
        data={'message': 'Processing audio input', 'audio_length': 3.2, 'sample_rate': 16000},
        state=PipelineState.PROCESSING_AUDIO
    )

    # Move to transcribing state
    print("\n4. Moving to TRANSCRIBING state...")
    state_manager.update_state(
        data={'message': 'Transcribing with Whisper API', 'confidence': 0.85},
        state=PipelineState.TRANSCRIBING
    )

    # Move to planning state
    print("\n5. Moving to PLANNING state...")
    state_manager.update_state(
        data={'message': 'Generating task plan with LLM', 'command': 'move forward 2 meters'},
        state=PipelineState.PLANNING
    )

    # Get current state
    print("\n6. Current state:")
    current_state = state_manager.get_state()
    print(f"  State: {current_state['state']}")
    print(f"  Data: {current_state['data']}")

    # Get state history
    print("\n7. Recent state history (last 5):")
    history = state_manager.get_state_history(limit=5)
    for i, snapshot in enumerate(history):
        print(f"  {i+1}. {snapshot.state.value} at {datetime.fromtimestamp(snapshot.timestamp).strftime('%H:%M:%S')} - {snapshot.data}")

    # Get state transitions
    print("\n8. State transitions:")
    transitions = state_manager.get_state_transitions()
    for transition in transitions:
        print(f"  {transition['from']} -> {transition['to']} (duration: {transition['duration']:.2f}s)")

    # Get state summary
    print("\n9. State summary:")
    summary = state_manager.get_state_summary()
    for key, value in summary.items():
        print(f"  {key}: {value}")

    # Wait for state change example
    print("\n10. Waiting for state change (this will timeout)...")
    changed = state_manager.wait_for_state_change(timeout=1.0)
    print(f"  State changed: {changed}")

    # Export/import example
    print("\n11. Exporting and importing state history...")
    exported = state_manager.export_state_history()
    print(f"  Exported {len(exported)} characters of history data")

    # Create a new state manager and import
    new_state_manager = StateManager(max_history=50)
    new_state_manager.import_state_history(exported)
    print(f"  Imported state manager has {len(new_state_manager.get_state_history())} history entries")
    print(f"  Current state: {new_state_manager.get_current_state().value}")


if __name__ == "__main__":
    main()