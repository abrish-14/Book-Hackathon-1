"""
Feedback handler for VLA Integration Module.
Handles feedback from ROS 2 action execution.
"""
from typing import Dict, Any, Optional, List, Callable
from dataclasses import dataclass
from enum import Enum
import time
import threading
import json
from datetime import datetime


class FeedbackType(Enum):
    """
    Enum for different types of feedback.
    """
    ACTION_START = "action_start"
    ACTION_PROGRESS = "action_progress"
    ACTION_COMPLETE = "action_complete"
    ACTION_ERROR = "action_error"
    STATE_CHANGE = "state_change"
    SYSTEM_STATUS = "system_status"


@dataclass
class FeedbackMessage:
    """
    Data class to hold feedback message information.
    """
    message_id: str
    feedback_type: FeedbackType
    action_id: Optional[str]
    timestamp: float
    data: Dict[str, Any]
    source: str = "robot"
    severity: str = "info"  # info, warning, error, critical


class FeedbackHandler:
    """
    Handles feedback from ROS 2 action execution and system events.
    """

    def __init__(self):
        """
        Initialize the feedback handler.
        """
        self.feedback_queue = []
        self.action_feedback = {}
        self.subscribers = {}
        self._lock = threading.Lock()

        # Initialize action status tracking
        self.action_status = {}

    def publish_feedback(self, feedback: FeedbackMessage):
        """
        Publish a feedback message to the system.

        Args:
            feedback: FeedbackMessage object to publish
        """
        with self._lock:
            # Add to global feedback queue
            self.feedback_queue.append(feedback)

            # Update action-specific feedback if applicable
            if feedback.action_id:
                if feedback.action_id not in self.action_feedback:
                    self.action_feedback[feedback.action_id] = []
                self.action_feedback[feedback.action_id].append(feedback)

                # Update action status
                if feedback.feedback_type == FeedbackType.ACTION_START:
                    self.action_status[feedback.action_id] = 'running'
                elif feedback.feedback_type == FeedbackType.ACTION_COMPLETE:
                    self.action_status[feedback.action_id] = 'completed'
                elif feedback.feedback_type == FeedbackType.ACTION_ERROR:
                    self.action_status[feedback.action_id] = 'failed'

            # Notify subscribers
            self._notify_subscribers(feedback)

    def subscribe(self, feedback_type: FeedbackType, callback: Callable[[FeedbackMessage], None]):
        """
        Subscribe to specific feedback types.

        Args:
            feedback_type: Type of feedback to subscribe to
            callback: Function to call when feedback is received
        """
        if feedback_type not in self.subscribers:
            self.subscribers[feedback_type] = []
        self.subscribers[feedback_type].append(callback)

    def unsubscribe(self, feedback_type: FeedbackType, callback: Callable[[FeedbackMessage], None]):
        """
        Unsubscribe from specific feedback types.

        Args:
            feedback_type: Type of feedback to unsubscribe from
            callback: Function that was registered
        """
        if feedback_type in self.subscribers:
            try:
                self.subscribers[feedback_type].remove(callback)
            except ValueError:
                # Callback was not registered
                pass

    def _notify_subscribers(self, feedback: FeedbackMessage):
        """
        Notify all subscribers of a feedback message.

        Args:
            feedback: FeedbackMessage to notify subscribers about
        """
        # Notify specific type subscribers
        if feedback.feedback_type in self.subscribers:
            for callback in self.subscribers[feedback.feedback_type]:
                try:
                    callback(feedback)
                except Exception as e:
                    print(f"Error in feedback callback: {e}")

        # Notify all message subscribers
        if FeedbackType.ACTION_START in self.subscribers:
            for callback in self.subscribers[FeedbackType.ACTION_START]:
                try:
                    callback(feedback)
                except Exception as e:
                    print(f"Error in feedback callback: {e}")

    def get_latest_feedback(self, action_id: str) -> Optional[FeedbackMessage]:
        """
        Get the latest feedback for a specific action.

        Args:
            action_id: ID of the action to get feedback for

        Returns:
            Latest FeedbackMessage for the action, or None if not found
        """
        with self._lock:
            if action_id in self.action_feedback and self.action_feedback[action_id]:
                return self.action_feedback[action_id][-1]
            return None

    def get_action_status(self, action_id: str) -> Optional[str]:
        """
        Get the current status of an action.

        Args:
            action_id: ID of the action to get status for

        Returns:
            Current status string ('running', 'completed', 'failed', or None)
        """
        return self.action_status.get(action_id)

    def get_all_feedback(self, action_id: str) -> List[FeedbackMessage]:
        """
        Get all feedback for a specific action.

        Args:
            action_id: ID of the action to get feedback for

        Returns:
            List of FeedbackMessage objects for the action
        """
        with self._lock:
            return self.action_feedback.get(action_id, []).copy()

    def create_action_start_feedback(self, action_id: str, action_type: str, parameters: Dict[str, Any]) -> FeedbackMessage:
        """
        Create a feedback message for action start.

        Args:
            action_id: ID of the action
            action_type: Type of action
            parameters: Action parameters

        Returns:
            FeedbackMessage for action start
        """
        return FeedbackMessage(
            message_id=f"start_{action_id}_{int(time.time())}",
            feedback_type=FeedbackType.ACTION_START,
            action_id=action_id,
            timestamp=time.time(),
            data={
                'action_type': action_type,
                'parameters': parameters,
                'start_time': time.time()
            }
        )

    def create_action_progress_feedback(self, action_id: str, progress: float, details: Dict[str, Any] = None) -> FeedbackMessage:
        """
        Create a feedback message for action progress.

        Args:
            action_id: ID of the action
            progress: Progress percentage (0.0 to 1.0)
            details: Additional progress details

        Returns:
            FeedbackMessage for action progress
        """
        if details is None:
            details = {}

        return FeedbackMessage(
            message_id=f"progress_{action_id}_{int(time.time())}",
            feedback_type=FeedbackType.ACTION_PROGRESS,
            action_id=action_id,
            timestamp=time.time(),
            data={
                'progress': progress,
                'details': details
            }
        )

    def create_action_complete_feedback(self, action_id: str, result: Dict[str, Any]) -> FeedbackMessage:
        """
        Create a feedback message for action completion.

        Args:
            action_id: ID of the action
            result: Result data from the action

        Returns:
            FeedbackMessage for action completion
        """
        return FeedbackMessage(
            message_id=f"complete_{action_id}_{int(time.time())}",
            feedback_type=FeedbackType.ACTION_COMPLETE,
            action_id=action_id,
            timestamp=time.time(),
            data={
                'result': result,
                'completion_time': time.time()
            }
        )

    def create_action_error_feedback(self, action_id: str, error_message: str, error_code: Optional[str] = None) -> FeedbackMessage:
        """
        Create a feedback message for action error.

        Args:
            action_id: ID of the action
            error_message: Error message
            error_code: Optional error code

        Returns:
            FeedbackMessage for action error
        """
        return FeedbackMessage(
            message_id=f"error_{action_id}_{int(time.time())}",
            feedback_type=FeedbackType.ACTION_ERROR,
            action_id=action_id,
            timestamp=time.time(),
            data={
                'error_message': error_message,
                'error_code': error_code
            },
            severity='error'
        )

    def create_state_change_feedback(self, old_state: str, new_state: str, details: Dict[str, Any] = None) -> FeedbackMessage:
        """
        Create a feedback message for state change.

        Args:
            old_state: Previous state
            new_state: New state
            details: Additional details about the state change

        Returns:
            FeedbackMessage for state change
        """
        if details is None:
            details = {}

        return FeedbackMessage(
            message_id=f"state_{int(time.time())}",
            feedback_type=FeedbackType.STATE_CHANGE,
            action_id=None,
            timestamp=time.time(),
            data={
                'old_state': old_state,
                'new_state': new_state,
                'details': details
            }
        )

    def create_system_status_feedback(self, status_data: Dict[str, Any]) -> FeedbackMessage:
        """
        Create a feedback message for system status.

        Args:
            status_data: System status data

        Returns:
            FeedbackMessage for system status
        """
        return FeedbackMessage(
            message_id=f"status_{int(time.time())}",
            feedback_type=FeedbackType.SYSTEM_STATUS,
            action_id=None,
            timestamp=time.time(),
            data=status_data
        )

    def get_action_summary(self, action_id: str) -> Dict[str, Any]:
        """
        Get a summary of an action's execution.

        Args:
            action_id: ID of the action to summarize

        Returns:
            Dictionary with action summary information
        """
        with self._lock:
            feedback_list = self.action_feedback.get(action_id, [])

            if not feedback_list:
                return {
                    'action_id': action_id,
                    'status': 'unknown',
                    'start_time': None,
                    'end_time': None,
                    'duration': None,
                    'success': False,
                    'error': None
                }

            # Find start and end feedback
            start_feedback = None
            end_feedback = None
            error_feedback = None

            for feedback in feedback_list:
                if feedback.feedback_type == FeedbackType.ACTION_START and not start_feedback:
                    start_feedback = feedback
                elif feedback.feedback_type in [FeedbackType.ACTION_COMPLETE, FeedbackType.ACTION_ERROR]:
                    if not end_feedback or feedback.timestamp > end_feedback.timestamp:
                        end_feedback = feedback
                        if feedback.feedback_type == FeedbackType.ACTION_ERROR:
                            error_feedback = feedback

            # Calculate summary
            start_time = start_feedback.timestamp if start_feedback else None
            end_time = end_feedback.timestamp if end_feedback else None
            duration = (end_time - start_time) if start_time and end_time else None

            status = 'unknown'
            if end_feedback:
                if end_feedback.feedback_type == FeedbackType.ACTION_COMPLETE:
                    status = 'completed'
                elif end_feedback.feedback_type == FeedbackType.ACTION_ERROR:
                    status = 'failed'
            elif start_feedback:
                status = 'running'

            return {
                'action_id': action_id,
                'status': status,
                'start_time': start_time,
                'end_time': end_time,
                'duration': duration,
                'success': status == 'completed',
                'error': error_feedback.data.get('error_message', None) if error_feedback else None
            }

    def get_system_health(self) -> Dict[str, Any]:
        """
        Get overall system health based on feedback.

        Returns:
            Dictionary with system health information
        """
        with self._lock:
            # Count recent errors
            recent_errors = 0
            recent_feedback_count = 0
            current_time = time.time()

            for feedback in self.feedback_queue:
                if current_time - feedback.timestamp < 60:  # Last 60 seconds
                    recent_feedback_count += 1
                    if feedback.severity == 'error':
                        recent_errors += 1

            # Calculate health score
            health_score = 1.0  # Start with perfect health
            if recent_feedback_count > 0:
                error_rate = recent_errors / recent_feedback_count
                health_score = max(0.0, 1.0 - error_rate)

            # Check for active actions
            active_actions = []
            for action_id, status in self.action_status.items():
                if status == 'running':
                    active_actions.append(action_id)

            return {
                'health_score': health_score,
                'recent_feedback_count': recent_feedback_count,
                'recent_error_count': recent_errors,
                'active_actions': active_actions,
                'total_feedback_count': len(self.feedback_queue),
                'timestamp': current_time
            }

    def clear_feedback_for_action(self, action_id: str):
        """
        Clear all feedback for a specific action.

        Args:
            action_id: ID of the action to clear feedback for
        """
        with self._lock:
            if action_id in self.action_feedback:
                del self.action_feedback[action_id]

            # Also remove from global queue
            self.feedback_queue = [
                fb for fb in self.feedback_queue
                if fb.action_id != action_id
            ]

            # Remove from status tracking
            if action_id in self.action_status:
                del self.action_status[action_id]

    def serialize_feedback(self, action_id: str) -> str:
        """
        Serialize feedback for an action to JSON string.

        Args:
            action_id: ID of the action to serialize feedback for

        Returns:
            JSON string representation of the feedback
        """
        with self._lock:
            feedback_list = self.action_feedback.get(action_id, [])
            serialized_feedback = []

            for feedback in feedback_list:
                serialized_feedback.append({
                    'message_id': feedback.message_id,
                    'feedback_type': feedback.feedback_type.value,
                    'action_id': feedback.action_id,
                    'timestamp': feedback.timestamp,
                    'data': feedback.data,
                    'source': feedback.source,
                    'severity': feedback.severity
                })

            return json.dumps(serialized_feedback, indent=2)

    def deserialize_feedback(self, action_id: str, json_string: str):
        """
        Deserialize feedback from JSON string to the feedback store.

        Args:
            action_id: ID of the action to add feedback to
            json_string: JSON string representation of feedback
        """
        with self._lock:
            serialized_feedback = json.loads(json_string)

            for item in serialized_feedback:
                feedback = FeedbackMessage(
                    message_id=item['message_id'],
                    feedback_type=FeedbackType(item['feedback_type']),
                    action_id=item['action_id'],
                    timestamp=item['timestamp'],
                    data=item['data'],
                    source=item['source'],
                    severity=item['severity']
                )

                # Add to feedback store
                if action_id not in self.action_feedback:
                    self.action_feedback[action_id] = []
                self.action_feedback[action_id].append(feedback)

                # Update action status
                if feedback.feedback_type == FeedbackType.ACTION_START:
                    self.action_status[action_id] = 'running'
                elif feedback.feedback_type == FeedbackType.ACTION_COMPLETE:
                    self.action_status[action_id] = 'completed'
                elif feedback.feedback_type == FeedbackType.ACTION_ERROR:
                    self.action_status[action_id] = 'failed'


# Example usage and testing
class FeedbackTestHandler:
    """
    Test handler to demonstrate feedback subscription.
    """

    def __init__(self, name: str):
        self.name = name

    def handle_feedback(self, feedback: FeedbackMessage):
        """
        Handle incoming feedback messages.

        Args:
            feedback: FeedbackMessage to handle
        """
        print(f"[{self.name}] Received feedback: {feedback.feedback_type.value} for action {feedback.action_id}")
        print(f"  Data: {feedback.data}")
        print(f"  Timestamp: {datetime.fromtimestamp(feedback.timestamp)}")


def main():
    """
    Main function to demonstrate feedback handler functionality.
    """
    # Create feedback handler
    handler = FeedbackHandler()

    # Create test handlers
    test_handler1 = FeedbackTestHandler("Handler1")
    test_handler2 = FeedbackTestHandler("Handler2")

    # Subscribe to different feedback types
    handler.subscribe(FeedbackType.ACTION_START, test_handler1.handle_feedback)
    handler.subscribe(FeedbackType.ACTION_COMPLETE, test_handler1.handle_feedback)
    handler.subscribe(FeedbackType.ACTION_ERROR, test_handler2.handle_feedback)

    # Simulate some actions and feedback
    print("=== Simulating Actions and Feedback ===")

    # Create and publish action start feedback
    action_id = "action_001"
    start_feedback = handler.create_action_start_feedback(
        action_id=action_id,
        action_type="move",
        parameters={"x": 1.0, "y": 1.0, "speed": 0.5}
    )
    handler.publish_feedback(start_feedback)

    # Create and publish progress feedback
    progress_feedback = handler.create_action_progress_feedback(
        action_id=action_id,
        progress=0.5,
        details={"current_position": [0.5, 0.5, 0.0]}
    )
    handler.publish_feedback(progress_feedback)

    # Create and publish completion feedback
    complete_feedback = handler.create_action_complete_feedback(
        action_id=action_id,
        result={"success": True, "final_position": [1.0, 1.0, 0.0]}
    )
    handler.publish_feedback(complete_feedback)

    # Create another action with error
    error_action_id = "action_002"
    error_feedback = handler.create_action_error_feedback(
        action_id=error_action_id,
        error_message="Robot unable to reach target position",
        error_code="NAVIGATION_FAILED"
    )
    handler.publish_feedback(error_feedback)

    # Get action summaries
    print("\n=== Action Summaries ===")
    summary1 = handler.get_action_summary(action_id)
    print(f"Action {action_id} summary: {summary1}")

    summary2 = handler.get_action_summary(error_action_id)
    print(f"Action {error_action_id} summary: {summary2}")

    # Get system health
    print("\n=== System Health ===")
    health = handler.get_system_health()
    print(f"System health: {health}")


if __name__ == "__main__":
    main()