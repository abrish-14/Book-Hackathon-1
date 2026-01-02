"""
Robot controller interface for VLA Integration Module.
Provides high-level control interface for the humanoid robot.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from typing import Dict, Any, Optional, List, Callable
from dataclasses import dataclass
from enum import Enum
import threading
import time
import math


class RobotState(Enum):
    """
    Enum for robot state.
    """
    IDLE = "idle"
    MOVING = "moving"
    GRASPING = "grasping"
    SPEAKING = "speaking"
    WAITING = "waiting"
    NAVIGATING = "navigating"
    ERROR = "error"


@dataclass
class RobotStatus:
    """
    Data class to hold robot status information.
    """
    state: RobotState
    position: List[float]  # [x, y, z]
    orientation: List[float]  # [roll, pitch, yaw]
    battery_level: float  # 0.0 to 1.0
    last_action_time: float
    active_gripper: Optional[str] = None  # 'left', 'right', or None


class RobotController(Node):
    """
    High-level controller interface for the humanoid robot.
    """

    def __init__(self, node_name: str = 'vla_robot_controller'):
        """
        Initialize the robot controller.

        Args:
            node_name: Name for the ROS 2 node
        """
        super().__init__(node_name)

        # Robot state
        self._state = RobotState.IDLE
        self._position = [0.0, 0.0, 0.0]
        self._orientation = [0.0, 0.0, 0.0]
        self._battery_level = 1.0
        self._last_action_time = time.time()
        self._active_gripper = None

        # Action clients
        self.action_clients = {}

        # Robot capabilities
        self.capabilities = {
            'max_speed': 1.0,  # m/s
            'arm_reach': 1.2,  # meters
            'max_payload': 5.0,  # kg
            'supported_actions': [
                'move', 'navigate', 'grasp', 'place', 'speak', 'turn', 'wait'
            ],
            'max_rotation_speed': 0.5,  # rad/s
            'max_acceleration': 0.5  # m/s^2
        }

        # Callbacks
        self._action_completed_callbacks = []
        self._state_change_callbacks = []

    def get_status(self) -> RobotStatus:
        """
        Get the current robot status.

        Returns:
            RobotStatus object with current status information
        """
        return RobotStatus(
            state=self._state,
            position=self._position.copy(),
            orientation=self._orientation.copy(),
            battery_level=self._battery_level,
            last_action_time=self._last_action_time,
            active_gripper=self._active_gripper
        )

    def move_to_position(self, x: float, y: float, z: float = 0.0,
                       speed: float = 0.5, relative: bool = False) -> bool:
        """
        Move the robot to a specific position.

        Args:
            x: Target x position
            y: Target y position
            z: Target z position (default: 0.0)
            speed: Movement speed (0.0 to 1.0)
            relative: If True, move relative to current position

        Returns:
            True if move initiated successfully, False otherwise
        """
        if self._state != RobotState.IDLE:
            self.get_logger().warn(f"Cannot move, robot is in {self._state.value} state")
            return False

        # Validate speed
        if speed > 1.0 or speed < 0.0:
            self.get_logger().error("Speed must be between 0.0 and 1.0")
            return False

        # Convert to actual speed in m/s
        actual_speed = speed * self.capabilities['max_speed']

        # Calculate target position
        if relative:
            target_x = self._position[0] + x
            target_y = self._position[1] + y
            target_z = self._position[2] + z
        else:
            target_x = x
            target_y = y
            target_z = z

        # Update state
        self._set_state(RobotState.MOVING)

        # Simulate movement (in real implementation, this would call actual ROS 2 actions)
        self.get_logger().info(f"Moving to position: [{target_x}, {target_y}, {target_z}] at speed {actual_speed}")

        # Calculate distance and time
        dx = target_x - self._position[0]
        dy = target_y - self._position[1]
        dz = target_z - self._position[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        if distance > 0 and actual_speed > 0:
            move_time = distance / actual_speed
            time.sleep(min(move_time, 3.0))  # Cap at 3 seconds for simulation

        # Update position
        self._position = [target_x, target_y, target_z]
        self._last_action_time = time.time()

        # Return to idle state
        self._set_state(RobotState.IDLE)

        return True

    def move_by_vector(self, vector: List[float], speed: float = 0.5) -> bool:
        """
        Move the robot by a relative vector.

        Args:
            vector: [x, y, z] vector to move by
            speed: Movement speed (0.0 to 1.0)

        Returns:
            True if move initiated successfully, False otherwise
        """
        if len(vector) != 3:
            self.get_logger().error("Vector must have 3 components [x, y, z]")
            return False

        return self.move_to_position(vector[0], vector[1], vector[2], speed, relative=True)

    def rotate(self, angle: float, axis: str = 'z', speed: float = 0.5) -> bool:
        """
        Rotate the robot around an axis.

        Args:
            angle: Rotation angle in radians
            axis: Axis to rotate around ('x', 'y', or 'z')
            speed: Rotation speed (0.0 to 1.0)

        Returns:
            True if rotation initiated successfully, False otherwise
        """
        if self._state != RobotState.IDLE:
            self.get_logger().warn(f"Cannot rotate, robot is in {self._state.value} state")
            return False

        # Validate speed
        if speed > 1.0 or speed < 0.0:
            self.get_logger().error("Speed must be between 0.0 and 1.0")
            return False

        # Convert to actual speed in rad/s
        actual_speed = speed * self.capabilities['max_rotation_speed']

        # Update state
        self._set_state(RobotState.MOVING)

        # Simulate rotation (in real implementation, this would call actual ROS 2 actions)
        self.get_logger().info(f"Rotating {angle} radians around {axis} axis at speed {actual_speed}")

        # Calculate rotation time
        if abs(angle) > 0 and actual_speed > 0:
            rotation_time = abs(angle) / actual_speed
            time.sleep(min(rotation_time, 3.0))  # Cap at 3 seconds for simulation

        # Update orientation (simplified - only update the relevant axis)
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        if axis in axis_map:
            idx = axis_map[axis]
            self._orientation[idx] += angle

        self._last_action_time = time.time()

        # Return to idle state
        self._set_state(RobotState.IDLE)

        return True

    def grasp_object(self, object_name: str, arm: str = 'right') -> bool:
        """
        Grasp an object with the specified arm.

        Args:
            object_name: Name or description of object to grasp
            arm: Which arm to use ('left' or 'right')

        Returns:
            True if grasp initiated successfully, False otherwise
        """
        if self._state != RobotState.IDLE:
            self.get_logger().warn(f"Cannot grasp, robot is in {self._state.value} state")
            return False

        if arm not in ['left', 'right']:
            self.get_logger().error("Arm must be 'left' or 'right'")
            return False

        # Check if gripper is already active
        if self._active_gripper is not None:
            self.get_logger().warn(f"Gripper {self._active_gripper} is already active")
            return False

        # Update state
        self._set_state(RobotState.GRASPING)
        self._active_gripper = arm

        # Simulate grasping (in real implementation, this would call actual ROS 2 actions)
        self.get_logger().info(f"Grasping object '{object_name}' with {arm} arm")

        # Simulate grasp time
        time.sleep(1.5)  # Simulate grasp time

        self._last_action_time = time.time()

        # Return to idle state
        self._set_state(RobotState.IDLE)

        return True

    def release_object(self, arm: str = 'right') -> bool:
        """
        Release an object from the specified arm.

        Args:
            arm: Which arm to release from ('left' or 'right')

        Returns:
            True if release initiated successfully, False otherwise
        """
        if self._state != RobotState.IDLE:
            self.get_logger().warn(f"Cannot release, robot is in {self._state.value} state")
            return False

        if arm not in ['left', 'right']:
            self.get_logger().error("Arm must be 'left' or 'right'")
            return False

        # Check if the specified gripper is active
        if self._active_gripper != arm:
            self.get_logger().warn(f"Arm {arm} is not currently grasping anything")
            return False

        # Update state
        self._set_state(RobotState.GRASPING)

        # Simulate releasing (in real implementation, this would call actual ROS 2 actions)
        self.get_logger().info(f"Releasing object from {arm} arm")

        # Simulate release time
        time.sleep(1.0)  # Simulate release time

        # Update gripper state
        self._active_gripper = None
        self._last_action_time = time.time()

        # Return to idle state
        self._set_state(RobotState.IDLE)

        return True

    def speak(self, text: str, volume: float = 0.8, language: str = 'en') -> bool:
        """
        Make the robot speak text.

        Args:
            text: Text to speak
            volume: Volume level (0.0 to 1.0)
            language: Language code (default: 'en')

        Returns:
            True if speaking initiated successfully, False otherwise
        """
        if self._state != RobotState.IDLE:
            self.get_logger().warn(f"Cannot speak, robot is in {self._state.value} state")
            return False

        if volume > 1.0 or volume < 0.0:
            self.get_logger().error("Volume must be between 0.0 and 1.0")
            return False

        # Update state
        self._set_state(RobotState.SPEAKING)

        # Simulate speaking (in real implementation, this would call actual ROS 2 actions)
        self.get_logger().info(f"Speaking: '{text}' (volume: {volume}, language: {language})")

        # Estimate speaking time based on text length
        words = len(text.split())
        speaking_time = words * 0.3  # Approximately 0.3 seconds per word
        time.sleep(min(speaking_time, 5.0))  # Cap at 5 seconds

        self._last_action_time = time.time()

        # Return to idle state
        self._set_state(RobotState.IDLE)

        return True

    def wait(self, duration: float) -> bool:
        """
        Make the robot wait for a specified duration.

        Args:
            duration: Duration to wait in seconds

        Returns:
            True if wait initiated successfully, False otherwise
        """
        if self._state != RobotState.IDLE:
            self.get_logger().warn(f"Cannot wait, robot is in {self._state.value} state")
            return False

        if duration <= 0:
            self.get_logger().error("Duration must be positive")
            return False

        # Update state
        self._set_state(RobotState.WAITING)

        # Simulate waiting
        self.get_logger().info(f"Waiting for {duration} seconds")

        # Actually wait
        time.sleep(min(duration, 10.0))  # Cap at 10 seconds for safety

        self._last_action_time = time.time()

        # Return to idle state
        self._set_state(RobotState.IDLE)

        return True

    def navigate_to_location(self, location_name: str) -> bool:
        """
        Navigate the robot to a named location.

        Args:
            location_name: Name of the location to navigate to

        Returns:
            True if navigation initiated successfully, False otherwise
        """
        if self._state != RobotState.IDLE:
            self.get_logger().warn(f"Cannot navigate, robot is in {self._state.value} state")
            return False

        # Location map (in real implementation, this would come from a map server)
        location_map = {
            'kitchen': [5.0, 0.0, 0.0],
            'living room': [0.0, 5.0, 0.0],
            'bedroom': [5.0, 5.0, 0.0],
            'office': [-5.0, 0.0, 0.0],
            'dining room': [0.0, -5.0, 0.0],
            'start': [0.0, 0.0, 0.0]
        }

        if location_name.lower() not in location_map:
            self.get_logger().error(f"Unknown location: {location_name}")
            return False

        target_pos = location_map[location_name.lower()]

        # Update state
        self._set_state(RobotState.NAVIGATING)

        # Simulate navigation (in real implementation, this would call actual ROS 2 navigation actions)
        self.get_logger().info(f"Navigating to {location_name} at position {target_pos}")

        # Simulate navigation time
        dx = target_pos[0] - self._position[0]
        dy = target_pos[1] - self._position[1]
        dz = target_pos[2] - self._position[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        navigation_time = distance / self.capabilities['max_speed'] if self.capabilities['max_speed'] > 0 else 0
        time.sleep(min(navigation_time, 10.0))  # Cap at 10 seconds for simulation

        # Update position
        self._position = target_pos.copy()
        self._last_action_time = time.time()

        # Return to idle state
        self._set_state(RobotState.IDLE)

        return True

    def execute_action_sequence(self, actions: List[Dict[str, Any]]) -> bool:
        """
        Execute a sequence of actions.

        Args:
            actions: List of action dictionaries to execute

        Returns:
            True if all actions completed successfully, False otherwise
        """
        for action in actions:
            action_type = action.get('action_type', 'unknown')
            parameters = action.get('parameters', {})

            success = False
            if action_type == 'move':
                x = parameters.get('x', 0.0)
                y = parameters.get('y', 0.0)
                z = parameters.get('z', 0.0)
                speed = parameters.get('speed', 0.5)
                relative = parameters.get('relative', False)
                success = self.move_to_position(x, y, z, speed, relative)
            elif action_type == 'navigate':
                location = parameters.get('location', 'start')
                success = self.navigate_to_location(location)
            elif action_type == 'grasp':
                object_name = parameters.get('object_name', 'object')
                arm = parameters.get('arm', 'right')
                success = self.grasp_object(object_name, arm)
            elif action_type == 'place':
                arm = parameters.get('arm', 'right')
                success = self.release_object(arm)
            elif action_type == 'speak':
                text = parameters.get('text', 'Hello')
                volume = parameters.get('volume', 0.8)
                success = self.speak(text, volume)
            elif action_type == 'wait':
                duration = parameters.get('duration', 1.0)
                success = self.wait(duration)
            else:
                self.get_logger().warn(f"Unknown action type: {action_type}")

            if not success:
                self.get_logger().error(f"Action {action_type} failed")
                self._set_state(RobotState.ERROR)
                return False

        return True

    def _set_state(self, new_state: RobotState):
        """
        Set the robot state and notify listeners.

        Args:
            new_state: New state to set
        """
        old_state = self._state
        self._state = new_state

        # Notify state change callbacks
        for callback in self._state_change_callbacks:
            callback(old_state, new_state)

    def add_state_change_callback(self, callback: Callable[[RobotState, RobotState], None]):
        """
        Add a callback to be called when the robot state changes.

        Args:
            callback: Function to call when state changes, with (old_state, new_state) parameters
        """
        self._state_change_callbacks.append(callback)

    def add_action_completed_callback(self, callback: Callable[[str, bool], None]):
        """
        Add a callback to be called when an action is completed.

        Args:
            callback: Function to call when action completes, with (action_type, success) parameters
        """
        self._action_completed_callbacks.append(callback)

    def get_capabilities(self) -> Dict[str, Any]:
        """
        Get the robot capabilities.

        Returns:
            Dictionary of robot capabilities
        """
        return self.capabilities.copy()

    def is_idle(self) -> bool:
        """
        Check if the robot is in idle state.

        Returns:
            True if robot is idle, False otherwise
        """
        return self._state == RobotState.IDLE

    def stop_all_actions(self) -> bool:
        """
        Stop all current actions and return to idle state.

        Returns:
            True if successfully stopped, False otherwise
        """
        # For simulation purposes, we'll just return to idle state
        if self._state != RobotState.IDLE:
            self.get_logger().info("Stopping all actions and returning to idle state")
            self._state = RobotState.IDLE
            return True
        return True


def main(args=None):
    """
    Main function to initialize and run the robot controller.
    """
    rclpy.init(args=args)

    # Create robot controller
    controller = RobotController()

    # Example usage
    controller.get_logger().info("VLA Robot Controller initialized")

    # Example: Move to position
    success = controller.move_to_position(1.0, 1.0, 0.0, speed=0.5)
    controller.get_logger().info(f"Move to position result: {success}")

    # Example: Speak
    success = controller.speak("Hello, I am your VLA robot assistant.")
    controller.get_logger().info(f"Speak result: {success}")

    # Example: Navigate to location
    success = controller.navigate_to_location('kitchen')
    controller.get_logger().info(f"Navigate to kitchen result: {success}")

    # Get robot status
    status = controller.get_status()
    controller.get_logger().info(f"Robot status: {status.state.value}")

    # Shutdown
    rclpy.shutdown()


if __name__ == '__main__':
    main()