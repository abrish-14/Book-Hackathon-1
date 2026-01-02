"""
Action generator for VLA Integration Module.
Generates executable actions from parsed tasks and LLM plans.
"""
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import json


@dataclass
class ROS2Action:
    """
    Data class to represent a ROS 2 action that can be executed.
    """
    action_type: str
    parameters: Dict[str, Any]
    priority: int = 0
    dependencies: List[str] = None
    id: str = ""

    def __post_init__(self):
        """
        Initialize default values after object creation.
        """
        if self.dependencies is None:
            self.dependencies = []


class ActionGenerator:
    """
    Generates executable ROS 2 actions from parsed tasks and LLM plans.
    """

    def __init__(self):
        """
        Initialize the action generator.
        """
        # Define mapping from high-level actions to ROS 2 actions
        self.action_mapping = {
            'move': self._generate_move_action,
            'navigate': self._generate_navigate_action,
            'grasp': self._generate_grasp_action,
            'place': self._generate_place_action,
            'speak': self._generate_speak_action,
            'turn': self._generate_turn_action,
            'wait': self._generate_wait_action,
            'unknown': self._generate_unknown_action
        }

    def generate_actions_from_tasks(self, tasks: List[Dict[str, Any]],
                                   robot_capabilities: Optional[Dict[str, Any]] = None) -> List[ROS2Action]:
        """
        Generate ROS 2 actions from a list of parsed tasks.

        Args:
            tasks: List of parsed tasks from LLM or parser
            robot_capabilities: Dictionary of robot capabilities to consider

        Returns:
            List of ROS2Action objects ready for execution
        """
        if robot_capabilities is None:
            robot_capabilities = self._get_default_robot_capabilities()

        actions = []
        task_id_counter = 1

        for i, task in enumerate(tasks):
            action_type = task.get('action_type', 'unknown')
            parameters = task.get('parameters', {})

            # Generate action ID
            action_id = f"action_{task_id_counter:03d}"
            task_id_counter += 1

            # Get the appropriate generator function
            generator_func = self.action_mapping.get(action_type, self._generate_unknown_action)

            # Generate the ROS 2 action
            ros_action = generator_func(parameters, action_id, robot_capabilities)

            # Set priority based on original task priority if available
            if 'priority' in task:
                ros_action.priority = task['priority']
            else:
                ros_action.priority = i  # Default priority based on order

            # Set dependencies if available
            if 'dependencies' in task:
                ros_action.dependencies = task['dependencies']

            actions.append(ros_action)

        return actions

    def _get_default_robot_capabilities(self) -> Dict[str, Any]:
        """
        Get default robot capabilities.

        Returns:
            Dictionary of default robot capabilities
        """
        return {
            'max_speed': 1.0,  # m/s
            'arm_reach': 1.0,  # meters
            'max_payload': 5.0,  # kg
            'supported_actions': [
                'move', 'navigate', 'grasp', 'place', 'speak', 'turn', 'wait'
            ],
            'navigation_types': ['waypoint', 'goal_pose'],
            'gripper_types': ['parallel', 'suction']
        }

    def _generate_move_action(self, parameters: Dict[str, Any], action_id: str,
                             robot_capabilities: Dict[str, Any]) -> ROS2Action:
        """
        Generate a move action for the robot.

        Args:
            parameters: Parameters for the move action
            action_id: Unique ID for the action
            robot_capabilities: Robot capabilities to consider

        Returns:
            ROS2Action for moving
        """
        # Validate and adjust parameters based on robot capabilities
        direction = parameters.get('direction', 'forward')
        distance = parameters.get('distance', 1.0)

        # Limit speed based on capabilities
        max_speed = robot_capabilities.get('max_speed', 1.0)
        speed = min(parameters.get('speed', 1.0), max_speed)

        # Ensure distance is reasonable
        if distance <= 0:
            distance = 0.1  # Minimum distance
        elif distance > 10:  # Max 10 meters in one move
            distance = 10.0

        # Convert direction to movement vector
        direction_map = {
            'forward': [distance, 0, 0],
            'backward': [-distance, 0, 0],
            'left': [0, distance, 0],
            'right': [0, -distance, 0],
            'up': [0, 0, distance],
            'down': [0, 0, -distance]
        }

        movement_vector = direction_map.get(direction, [distance, 0, 0])

        ros_params = {
            'type': 'move',
            'vector': movement_vector,
            'speed': speed,
            'relative': True
        }

        return ROS2Action(
            action_type='move',
            parameters=ros_params,
            id=action_id
        )

    def _generate_navigate_action(self, parameters: Dict[str, Any], action_id: str,
                                robot_capabilities: Dict[str, Any]) -> ROS2Action:
        """
        Generate a navigation action for the robot.

        Args:
            parameters: Parameters for the navigation action
            action_id: Unique ID for the action
            robot_capabilities: Robot capabilities to consider

        Returns:
            ROS2Action for navigation
        """
        target_location = parameters.get('target_location', 'unknown')

        # For simulation, we'll use a simple coordinate system
        # In a real system, this would map to actual coordinates
        location_map = {
            'kitchen': [5.0, 0.0, 0.0],
            'living room': [0.0, 5.0, 0.0],
            'bedroom': [5.0, 5.0, 0.0],
            'office': [-5.0, 0.0, 0.0],
            'dining room': [0.0, -5.0, 0.0]
        }

        target_pose = location_map.get(target_location.lower(), [0.0, 0.0, 0.0])

        ros_params = {
            'type': 'navigate',
            'target_pose': target_pose,
            'frame_id': 'map',
            'goal_tolerance': 0.5
        }

        return ROS2Action(
            action_type='navigate',
            parameters=ros_params,
            id=action_id
        )

    def _generate_grasp_action(self, parameters: Dict[str, Any], action_id: str,
                              robot_capabilities: Dict[str, Any]) -> ROS2Action:
        """
        Generate a grasp action for the robot.

        Args:
            parameters: Parameters for the grasp action
            action_id: Unique ID for the action
            robot_capabilities: Robot capabilities to consider

        Returns:
            ROS2Action for grasping
        """
        object_name = parameters.get('object', 'unknown')

        ros_params = {
            'type': 'grasp',
            'object_name': object_name,
            'arm': 'right',  # Default to right arm
            'gripper_width': 0.1,  # Default gripper width
            'force': 50.0  # Default force in Newtons
        }

        return ROS2Action(
            action_type='grasp',
            parameters=ros_params,
            id=action_id
        )

    def _generate_place_action(self, parameters: Dict[str, Any], action_id: str,
                              robot_capabilities: Dict[str, Any]) -> ROS2Action:
        """
        Generate a place action for the robot.

        Args:
            parameters: Parameters for the place action
            action_id: Unique ID for the action
            robot_capabilities: Robot capabilities to consider

        Returns:
            ROS2Action for placing
        """
        # For placing, we might need location information
        location = parameters.get('location', 'default')

        ros_params = {
            'type': 'place',
            'location': location,
            'arm': 'right',  # Default to right arm
            'release_force': 10.0  # Force to release object
        }

        return ROS2Action(
            action_type='place',
            parameters=ros_params,
            id=action_id
        )

    def _generate_speak_action(self, parameters: Dict[str, Any], action_id: str,
                              robot_capabilities: Dict[str, Any]) -> ROS2Action:
        """
        Generate a speak action for the robot.

        Args:
            parameters: Parameters for the speak action
            action_id: Unique ID for the action
            robot_capabilities: Robot capabilities to consider

        Returns:
            ROS2Action for speaking
        """
        text = parameters.get('text', 'Hello')

        ros_params = {
            'type': 'speak',
            'text': text,
            'voice': 'default',
            'volume': 0.8,
            'language': 'en'
        }

        return ROS2Action(
            action_type='speak',
            parameters=ros_params,
            id=action_id
        )

    def _generate_turn_action(self, parameters: Dict[str, Any], action_id: str,
                             robot_capabilities: Dict[str, Any]) -> ROS2Action:
        """
        Generate a turn action for the robot.

        Args:
            parameters: Parameters for the turn action
            action_id: Unique ID for the action
            robot_capabilities: Robot capabilities to consider

        Returns:
            ROS2Action for turning
        """
        direction = parameters.get('direction', 'left')

        # Define turn parameters
        if direction in ['left', 'counterclockwise']:
            angle = 90.0  # degrees
        elif direction == 'right':
            angle = -90.0  # degrees
        elif direction == 'around':
            angle = 180.0  # degrees
        elif direction == 'clockwise':
            angle = -90.0  # degrees
        else:
            angle = 90.0  # default to left

        ros_params = {
            'type': 'turn',
            'angle': angle,
            'relative': True,
            'speed': 0.5
        }

        return ROS2Action(
            action_type='turn',
            parameters=ros_params,
            id=action_id
        )

    def _generate_wait_action(self, parameters: Dict[str, Any], action_id: str,
                             robot_capabilities: Dict[str, Any]) -> ROS2Action:
        """
        Generate a wait action for the robot.

        Args:
            parameters: Parameters for the wait action
            action_id: Unique ID for the action
            robot_capabilities: Robot capabilities to consider

        Returns:
            ROS2Action for waiting
        """
        duration = parameters.get('duration', 1.0)

        # Ensure duration is positive
        if duration <= 0:
            duration = 1.0

        # Limit duration to reasonable values
        if duration > 60:  # Max 60 seconds
            duration = 60.0

        ros_params = {
            'type': 'wait',
            'duration': duration
        }

        return ROS2Action(
            action_type='wait',
            parameters=ros_params,
            id=action_id
        )

    def _generate_unknown_action(self, parameters: Dict[str, Any], action_id: str,
                                robot_capabilities: Dict[str, Any]) -> ROS2Action:
        """
        Generate a default action for unknown action types.

        Args:
            parameters: Parameters for the action
            action_id: Unique ID for the action
            robot_capabilities: Robot capabilities to consider

        Returns:
            ROS2Action for unknown action (default to wait)
        """
        ros_params = {
            'type': 'unknown',
            'original_params': parameters,
            'duration': 1.0
        }

        return ROS2Action(
            action_type='wait',  # Default to wait for unknown actions
            parameters=ros_params,
            id=action_id
        )

    def validate_action_sequence(self, actions: List[ROS2Action]) -> Dict[str, Any]:
        """
        Validate a sequence of actions for logical consistency.

        Args:
            actions: List of ROS2Action objects to validate

        Returns:
            Dictionary with validation results
        """
        issues = []
        warnings = []

        for i, action in enumerate(actions):
            # Check for potential conflicts or issues
            if action.action_type == 'grasp' and i + 1 < len(actions):
                next_action = actions[i + 1]
                if next_action.action_type == 'grasp':
                    warnings.append(f"Potential conflict: grasp action at {i} followed by another grasp at {i + 1}")

            # Check for physically impossible actions
            if action.action_type == 'move':
                distance = action.parameters.get('vector', [0, 0, 0])
                distance_magnitude = sum([abs(x) for x in distance])
                if distance_magnitude > 20:  # Very large movement
                    warnings.append(f"Large movement detected in action {i}: {distance_magnitude}m")

        return {
            'valid': len(issues) == 0,
            'issues': issues,
            'warnings': warnings
        }

    def optimize_action_sequence(self, actions: List[ROS2Action]) -> List[ROS2Action]:
        """
        Optimize a sequence of actions for better execution.

        Args:
            actions: List of ROS2Action objects to optimize

        Returns:
            Optimized list of ROS2Action objects
        """
        if not actions:
            return actions

        # Create a copy of the actions list
        optimized_actions = actions.copy()

        # Merge consecutive wait actions
        i = 0
        while i < len(optimized_actions) - 1:
            current = optimized_actions[i]
            next_action = optimized_actions[i + 1]

            if current.action_type == 'wait' and next_action.action_type == 'wait':
                # Merge the wait times
                current.parameters['duration'] += next_action.parameters['duration']

                # Remove the next action
                optimized_actions.pop(i + 1)
            else:
                i += 1

        return optimized_actions

    def serialize_action_sequence(self, actions: List[ROS2Action]) -> str:
        """
        Serialize a sequence of actions to JSON string.

        Args:
            actions: List of ROS2Action objects to serialize

        Returns:
            JSON string representation of the action sequence
        """
        action_dicts = []
        for action in actions:
            action_dict = {
                'action_type': action.action_type,
                'parameters': action.parameters,
                'priority': action.priority,
                'dependencies': action.dependencies,
                'id': action.id
            }
            action_dicts.append(action_dict)

        return json.dumps(action_dicts, indent=2)

    def deserialize_action_sequence(self, json_string: str) -> List[ROS2Action]:
        """
        Deserialize a JSON string to a sequence of actions.

        Args:
            json_string: JSON string representation of action sequence

        Returns:
            List of ROS2Action objects
        """
        action_dicts = json.loads(json_string)
        actions = []

        for action_dict in action_dicts:
            action = ROS2Action(
                action_type=action_dict['action_type'],
                parameters=action_dict['parameters'],
                priority=action_dict['priority'],
                dependencies=action_dict['dependencies'],
                id=action_dict['id']
            )
            actions.append(action)

        return actions


# Example usage:
if __name__ == "__main__":
    generator = ActionGenerator()

    # Example tasks (these would normally come from the LLM)
    example_tasks = [
        {
            'action_type': 'move',
            'parameters': {'direction': 'forward', 'distance': 2.0},
            'priority': 1
        },
        {
            'action_type': 'grasp',
            'parameters': {'object': 'red cup'},
            'priority': 2
        },
        {
            'action_type': 'navigate',
            'parameters': {'target_location': 'kitchen'},
            'priority': 3
        }
    ]

    # Generate ROS 2 actions
    ros_actions = generator.generate_actions_from_tasks(example_tasks)

    print("Generated ROS 2 Actions:")
    for action in ros_actions:
        print(f"ID: {action.id}, Type: {action.action_type}, Params: {action.parameters}")

    # Validate the sequence
    validation_result = generator.validate_action_sequence(ros_actions)
    print(f"\nValidation: {validation_result}")

    # Optimize the sequence
    optimized_actions = generator.optimize_action_sequence(ros_actions)
    print(f"\nOptimized from {len(ros_actions)} to {len(optimized_actions)} actions")