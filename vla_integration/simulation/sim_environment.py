"""
Simulation environment for VLA Integration Module.
Provides a simulated environment for testing the VLA pipeline.
"""
import time
import threading
import random
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
from enum import Enum
import math
import json


class SimObjectType(Enum):
    """
    Enum for different types of objects in the simulation.
    """
    OBSTACLE = "obstacle"
    TARGET = "target"
    GRABBABLE = "grabbable"
    LOCATION = "location"


class SimActionType(Enum):
    """
    Enum for different types of actions in the simulation.
    """
    MOVE = "move"
    NAVIGATE = "navigate"
    GRASP = "grasp"
    PLACE = "place"
    SPEAK = "speak"
    TURN = "turn"
    WAIT = "wait"


@dataclass
class SimObject:
    """
    Data class to represent an object in the simulation.
    """
    id: str
    object_type: SimObjectType
    position: List[float]  # [x, y, z]
    orientation: List[float]  # [roll, pitch, yaw]
    properties: Dict[str, Any]  # Additional properties like name, size, etc.


@dataclass
class SimActionResult:
    """
    Data class to represent the result of a simulated action.
    """
    action_type: SimActionType
    success: bool
    message: str
    execution_time: float
    data: Dict[str, Any]


class SimulationEnvironment:
    """
    Simulates the environment for testing the VLA pipeline.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize the simulation environment.

        Args:
            config: Optional configuration dictionary for the simulation
        """
        self.objects: Dict[str, SimObject] = {}
        self.robot_position = [0.0, 0.0, 0.0]
        self.robot_orientation = [0.0, 0.0, 0.0]
        self.robot_battery = 1.0  # 100% battery
        self.active_gripper = None  # 'left', 'right', or None
        self.is_busy = False
        self.simulation_speed = 1.0  # Real-time simulation
        self.running = False
        self.sim_thread = None

        # Default configuration
        self.config = config or {
            'world_size': [10.0, 10.0, 3.0],  # [width, height, height]
            'gravity': 9.81,
            'robot_speed': 1.0,  # m/s
            'robot_rotation_speed': 0.5,  # rad/s
            'max_payload': 5.0,  # kg
            'arm_reach': 1.0,  # m
            'sim_step_duration': 0.1  # seconds per simulation step
        }

        # Set up initial environment
        self._setup_initial_environment()

    def _setup_initial_environment(self):
        """
        Set up the initial environment with default objects.
        """
        # Add some default locations
        self.add_object(SimObject(
            id="location_kitchen",
            object_type=SimObjectType.LOCATION,
            position=[5.0, 0.0, 0.0],
            orientation=[0.0, 0.0, 0.0],
            properties={"name": "kitchen", "size": [2.0, 2.0, 1.0]}
        ))

        self.add_object(SimObject(
            id="location_living_room",
            object_type=SimObjectType.LOCATION,
            position=[0.0, 5.0, 0.0],
            orientation=[0.0, 0.0, 0.0],
            properties={"name": "living_room", "size": [3.0, 3.0, 1.0]}
        ))

        self.add_object(SimObject(
            id="location_bedroom",
            object_type=SimObjectType.LOCATION,
            position=[5.0, 5.0, 0.0],
            orientation=[0.0, 0.0, 0.0],
            properties={"name": "bedroom", "size": [2.5, 2.5, 1.0]}
        ))

        # Add some grabbable objects
        self.add_object(SimObject(
            id="object_red_cup",
            object_type=SimObjectType.GRABBABLE,
            position=[5.2, 0.2, 0.0],
            orientation=[0.0, 0.0, 0.0],
            properties={"name": "red cup", "weight": 0.2, "color": "red"}
        ))

        self.add_object(SimObject(
            id="object_blue_box",
            object_type=SimObjectType.GRABBABLE,
            position=[0.2, 5.2, 0.0],
            orientation=[0.0, 0.0, 0.0],
            properties={"name": "blue box", "weight": 0.5, "color": "blue"}
        ))

    def add_object(self, sim_object: SimObject) -> bool:
        """
        Add an object to the simulation.

        Args:
            sim_object: SimObject to add

        Returns:
            True if successfully added, False otherwise
        """
        if sim_object.id in self.objects:
            return False

        self.objects[sim_object.id] = sim_object
        return True

    def remove_object(self, object_id: str) -> bool:
        """
        Remove an object from the simulation.

        Args:
            object_id: ID of the object to remove

        Returns:
            True if successfully removed, False otherwise
        """
        if object_id not in self.objects:
            return False

        del self.objects[object_id]
        return True

    def get_object_by_id(self, object_id: str) -> Optional[SimObject]:
        """
        Get an object by its ID.

        Args:
            object_id: ID of the object to retrieve

        Returns:
            SimObject if found, None otherwise
        """
        return self.objects.get(object_id)

    def get_objects_by_type(self, object_type: SimObjectType) -> List[SimObject]:
        """
        Get all objects of a specific type.

        Args:
            object_type: Type of objects to retrieve

        Returns:
            List of SimObjects of the specified type
        """
        return [obj for obj in self.objects.values() if obj.object_type == object_type]

    def get_objects_in_area(self, center: List[float], radius: float) -> List[SimObject]:
        """
        Get all objects within a certain radius of a center point.

        Args:
            center: Center point [x, y, z]
            radius: Radius to search within

        Returns:
            List of SimObjects within the specified area
        """
        objects_in_area = []
        for obj in self.objects.values():
            distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(obj.position, center)))
            if distance <= radius:
                objects_in_area.append(obj)

        return objects_in_area

    def start_simulation(self):
        """
        Start the simulation environment.
        """
        if self.running:
            return

        self.running = True
        self.sim_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self.sim_thread.start()

        print("Simulation environment started.")

    def stop_simulation(self):
        """
        Stop the simulation environment.
        """
        self.running = False
        if self.sim_thread:
            self.sim_thread.join()

        print("Simulation environment stopped.")

    def _simulation_loop(self):
        """
        Main simulation loop that runs in a separate thread.
        """
        while self.running:
            # Simulate passage of time
            time.sleep(self.config['sim_step_duration'] / self.simulation_speed)

            # Update robot battery level (slow drain when moving)
            if self.is_busy:
                self.robot_battery = max(0.0, self.robot_battery - 0.0001)  # Slow drain

    def execute_action(self, action_type: SimActionType, parameters: Dict[str, Any]) -> SimActionResult:
        """
        Execute an action in the simulation environment.

        Args:
            action_type: Type of action to execute
            parameters: Parameters for the action

        Returns:
            SimActionResult with the result of the action
        """
        start_time = time.time()
        self.is_busy = True

        try:
            if action_type == SimActionType.MOVE:
                result = self._execute_move_action(parameters)
            elif action_type == SimActionType.NAVIGATE:
                result = self._execute_navigate_action(parameters)
            elif action_type == SimActionType.GRASP:
                result = self._execute_grasp_action(parameters)
            elif action_type == SimActionType.PLACE:
                result = self._execute_place_action(parameters)
            elif action_type == SimActionType.SPEAK:
                result = self._execute_speak_action(parameters)
            elif action_type == SimActionType.TURN:
                result = self._execute_turn_action(parameters)
            elif action_type == SimActionType.WAIT:
                result = self._execute_wait_action(parameters)
            else:
                result = SimActionResult(
                    action_type=action_type,
                    success=False,
                    message=f"Unknown action type: {action_type}",
                    execution_time=time.time() - start_time,
                    data={}
                )

        finally:
            self.is_busy = False

        return result

    def _execute_move_action(self, parameters: Dict[str, Any]) -> SimActionResult:
        """
        Execute a move action in the simulation.

        Args:
            parameters: Parameters for the move action

        Returns:
            SimActionResult with the result of the move action
        """
        # Extract parameters
        vector = parameters.get('vector', [1.0, 0.0, 0.0])  # Default: move 1m forward
        speed = parameters.get('speed', self.config['robot_speed'])
        relative = parameters.get('relative', True)

        # Validate parameters
        if speed > self.config['robot_speed']:
            speed = self.config['robot_speed']

        # Calculate target position
        if relative:
            target_x = self.robot_position[0] + vector[0]
            target_y = self.robot_position[1] + vector[1]
            target_z = self.robot_position[2] + vector[2]
        else:
            target_x = vector[0]
            target_y = vector[1]
            target_z = vector[2]

        # Validate bounds
        world_size = self.config['world_size']
        if not (0 <= target_x <= world_size[0] and
                0 <= target_y <= world_size[1] and
                0 <= target_z <= world_size[2]):
            return SimActionResult(
                action_type=SimActionType.MOVE,
                success=False,
                message="Target position out of bounds",
                execution_time=0.0,
                data={'target_position': [target_x, target_y, target_z]}
            )

        # Check for obstacles in path
        obstacles = self.get_objects_by_type(SimObjectType.OBSTACLE)
        for obstacle in obstacles:
            distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(obstacle.position, [target_x, target_y, target_z])))
            if distance < 0.5:  # Collision threshold
                return SimActionResult(
                    action_type=SimActionType.MOVE,
                    success=False,
                    message=f"Path blocked by obstacle at {obstacle.position}",
                    execution_time=0.0,
                    data={'target_position': [target_x, target_y, target_z], 'blocking_obstacle': obstacle.id}
                )

        # Calculate distance and time
        dx = target_x - self.robot_position[0]
        dy = target_y - self.robot_position[1]
        dz = target_z - self.robot_position[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Move the robot
        if distance > 0:
            # Simulate movement time
            movement_time = distance / speed if speed > 0 else 0
            time.sleep(min(movement_time, 5.0))  # Cap simulation time

        # Update robot position
        self.robot_position = [target_x, target_y, target_z]

        return SimActionResult(
            action_type=SimActionType.MOVE,
            success=True,
            message="Move completed successfully",
            execution_time=time.time(),
            data={
                'from_position': self.robot_position,
                'to_position': [target_x, target_y, target_z],
                'distance': distance
            }
        )

    def _execute_navigate_action(self, parameters: Dict[str, Any]) -> SimActionResult:
        """
        Execute a navigation action in the simulation.

        Args:
            parameters: Parameters for the navigation action

        Returns:
            SimActionResult with the result of the navigation action
        """
        target_pose = parameters.get('target_pose', [0.0, 0.0, 0.0])
        frame_id = parameters.get('frame_id', 'map')
        goal_tolerance = parameters.get('goal_tolerance', 0.5)

        # Find the location by name if specified
        location_name = parameters.get('location_name')
        if location_name:
            for obj in self.objects.values():
                if (obj.object_type == SimObjectType.LOCATION and
                    obj.properties.get('name', '').lower() == location_name.lower()):
                    target_pose = obj.position
                    break

        # Calculate distance to target
        dx = target_pose[0] - self.robot_position[0]
        dy = target_pose[1] - self.robot_position[1]
        dz = target_pose[2] - self.robot_position[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Check if already at target
        if distance <= goal_tolerance:
            return SimActionResult(
                action_type=SimActionType.NAVIGATE,
                success=True,
                message="Already at destination",
                execution_time=0.0,
                data={'current_position': self.robot_position, 'target_pose': target_pose}
            )

        # Move toward the target
        speed = self.config['robot_speed']
        movement_time = distance / speed if speed > 0 else 0
        time.sleep(min(movement_time, 10.0))  # Cap simulation time

        # Update robot position
        self.robot_position = target_pose.copy()

        return SimActionResult(
            action_type=SimActionType.NAVIGATE,
            success=True,
            message="Navigation completed successfully",
            execution_time=movement_time,
            data={
                'from_position': self.robot_position,
                'to_position': target_pose,
                'distance': distance,
                'frame_id': frame_id
            }
        )

    def _execute_grasp_action(self, parameters: Dict[str, Any]) -> SimActionResult:
        """
        Execute a grasp action in the simulation.

        Args:
            parameters: Parameters for the grasp action

        Returns:
            SimActionResult with the result of the grasp action
        """
        object_name = parameters.get('object_name', 'unknown')
        arm = parameters.get('arm', 'right')
        position_threshold = 0.5  # Robot needs to be close to object to grasp

        # Find the object by name
        target_object = None
        for obj in self.objects.values():
            if (obj.object_type == SimObjectType.GRABBABLE and
                obj.properties.get('name', '').lower() == object_name.lower()):
                target_object = obj
                break

        if not target_object:
            return SimActionResult(
                action_type=SimActionType.GRASP,
                success=False,
                message=f"Object '{object_name}' not found in environment",
                execution_time=0.0,
                data={'requested_object': object_name}
            )

        # Check if robot is close enough to the object
        dx = target_object.position[0] - self.robot_position[0]
        dy = target_object.position[1] - self.robot_position[1]
        dz = target_object.position[2] - self.robot_position[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        if distance > position_threshold:
            return SimActionResult(
                action_type=SimActionType.GRASP,
                success=False,
                message=f"Robot is too far from object '{object_name}'. Distance: {distance:.2f}m",
                execution_time=0.0,
                data={
                    'robot_position': self.robot_position,
                    'object_position': target_object.position,
                    'distance': distance
                }
            )

        # Check if gripper is already active
        if self.active_gripper is not None:
            return SimActionResult(
                action_type=SimActionType.GRASP,
                success=False,
                message=f"Gripper {self.active_gripper} is already holding an object",
                execution_time=0.0,
                data={'active_gripper': self.active_gripper}
            )

        # Check payload
        object_weight = target_object.properties.get('weight', 0.1)
        if object_weight > self.config['max_payload']:
            return SimActionResult(
                action_type=SimActionType.GRASP,
                success=False,
                message=f"Object too heavy: {object_weight}kg (max: {self.config['max_payload']}kg)",
                execution_time=0.0,
                data={
                    'object_weight': object_weight,
                    'max_payload': self.config['max_payload']
                }
            )

        # Simulate grasping time
        grasp_time = 1.0
        time.sleep(grasp_time)

        # Update gripper state
        self.active_gripper = arm

        return SimActionResult(
            action_type=SimActionType.GRASP,
            success=True,
            message=f"Successfully grasped '{object_name}' with {arm} arm",
            execution_time=grasp_time,
            data={
                'grasped_object': object_name,
                'arm_used': arm,
                'object_properties': target_object.properties
            }
        )

    def _execute_place_action(self, parameters: Dict[str, Any]) -> SimActionResult:
        """
        Execute a place action in the simulation.

        Args:
            parameters: Parameters for the place action

        Returns:
            SimActionResult with the result of the place action
        """
        arm = parameters.get('arm', 'right')

        # Check if the specified gripper is active
        if self.active_gripper != arm:
            return SimActionResult(
                action_type=SimActionType.PLACE,
                success=False,
                message=f"Arm {arm} is not currently holding an object",
                execution_time=0.0,
                data={'active_gripper': self.active_gripper}
            )

        # Simulate placing time
        place_time = 1.0
        time.sleep(place_time)

        # Update gripper state
        self.active_gripper = None

        return SimActionResult(
            action_type=SimActionType.PLACE,
            success=True,
            message=f"Successfully placed object from {arm} arm",
            execution_time=place_time,
            data={'released_arm': arm}
        )

    def _execute_speak_action(self, parameters: Dict[str, Any]) -> SimActionResult:
        """
        Execute a speak action in the simulation.

        Args:
            parameters: Parameters for the speak action

        Returns:
            SimActionResult with the result of the speak action
        """
        text = parameters.get('text', 'Hello')
        voice = parameters.get('voice', 'default')
        volume = parameters.get('volume', 0.8)

        # Validate parameters
        if volume > 1.0 or volume < 0.0:
            return SimActionResult(
                action_type=SimActionType.SPEAK,
                success=False,
                message="Volume must be between 0.0 and 1.0",
                execution_time=0.0,
                data={'requested_volume': volume}
            )

        # Estimate speaking time based on text length
        words = len(text.split())
        speaking_time = words * 0.3  # Approximately 0.3 seconds per word
        time.sleep(min(speaking_time, 5.0))  # Cap at 5 seconds

        # In simulation, just simulate the speaking
        print(f"[ROBOT SPEAKS]: {text}")

        return SimActionResult(
            action_type=SimActionType.SPEAK,
            success=True,
            message="Spoke successfully",
            execution_time=speaking_time,
            data={
                'spoken_text': text,
                'voice_type': voice,
                'volume': volume
            }
        )

    def _execute_turn_action(self, parameters: Dict[str, Any]) -> SimActionResult:
        """
        Execute a turn action in the simulation.

        Args:
            parameters: Parameters for the turn action

        Returns:
            SimActionResult with the result of the turn action
        """
        angle = parameters.get('angle', 90.0)  # degrees
        axis = parameters.get('axis', 'z')
        speed = parameters.get('speed', self.config['robot_rotation_speed'])

        # Validate speed
        if speed > self.config['robot_rotation_speed']:
            speed = self.config['robot_rotation_speed']

        # Simulate turn time
        turn_time = abs(angle) / 90.0 * (90.0 / self.config['robot_rotation_speed'])  # Scale by desired speed
        time.sleep(min(turn_time, 5.0))  # Cap at 5 seconds

        # Update orientation (simplified - only update the relevant axis)
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        if axis in axis_map:
            idx = axis_map[axis]
            self.robot_orientation[idx] += math.radians(angle)

        return SimActionResult(
            action_type=SimActionType.TURN,
            success=True,
            message=f"Turned {angle} degrees around {axis} axis",
            execution_time=turn_time,
            data={
                'angle_degrees': angle,
                'axis': axis,
                'new_orientation': self.robot_orientation
            }
        )

    def _execute_wait_action(self, parameters: Dict[str, Any]) -> SimActionResult:
        """
        Execute a wait action in the simulation.

        Args:
            parameters: Parameters for the wait action

        Returns:
            SimActionResult with the result of the wait action
        """
        duration = parameters.get('duration', 1.0)

        # Validate parameters
        if duration <= 0:
            return SimActionResult(
                action_type=SimActionType.WAIT,
                success=False,
                message="Duration must be positive",
                execution_time=0.0,
                data={'requested_duration': duration}
            )

        # Cap wait time for simulation purposes
        if duration > 30:  # Don't wait for more than 30 seconds in simulation
            duration = 30.0

        # Actually wait
        time.sleep(duration)

        return SimActionResult(
            action_type=SimActionType.WAIT,
            success=True,
            message=f"Waited for {duration} seconds",
            execution_time=duration,
            data={'wait_duration': duration}
        )

    def execute_action_sequence(self, actions: List[Dict[str, Any]]) -> List[SimActionResult]:
        """
        Execute a sequence of actions in the simulation.

        Args:
            actions: List of action dictionaries to execute

        Returns:
            List of SimActionResult objects for each action
        """
        results = []

        for action in actions:
            action_type = action.get('action_type')
            parameters = action.get('parameters', {})

            if action_type is None:
                continue

            # Convert action type to enum
            try:
                action_enum = SimActionType(action_type)
            except ValueError:
                print(f"Unknown action type: {action_type}")
                continue

            # Execute the action
            result = self.execute_action(action_enum, parameters)
            results.append(result)

            # If action failed, stop execution sequence
            if not result.success:
                print(f"Action sequence stopped due to failure in {action_type}")
                break

        return results

    def get_robot_status(self) -> Dict[str, Any]:
        """
        Get the current status of the simulated robot.

        Returns:
            Dictionary with robot status information
        """
        return {
            'position': self.robot_position.copy(),
            'orientation': self.robot_orientation.copy(),
            'battery_level': self.robot_battery,
            'active_gripper': self.active_gripper,
            'is_busy': self.is_busy,
            'timestamp': time.time()
        }

    def get_environment_state(self) -> Dict[str, Any]:
        """
        Get the current state of the environment.

        Returns:
            Dictionary with environment state information
        """
        return {
            'objects': {obj_id: {
                'type': obj.object_type.value,
                'position': obj.position,
                'orientation': obj.orientation,
                'properties': obj.properties
            } for obj_id, obj in self.objects.items()},
            'robot_status': self.get_robot_status(),
            'config': self.config,
            'timestamp': time.time()
        }

    def get_visible_objects(self) -> List[SimObject]:
        """
        Get objects that are visible to the robot (within arm reach).

        Returns:
            List of SimObject that are within the robot's reach
        """
        visible_objects = []
        for obj in self.objects.values():
            dx = obj.position[0] - self.robot_position[0]
            dy = obj.position[1] - self.robot_position[1]
            dz = obj.position[2] - self.robot_position[2]
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)

            if distance <= self.config['arm_reach']:
                visible_objects.append(obj)

        return visible_objects

    def reset_environment(self):
        """
        Reset the simulation environment to its initial state.
        """
        # Clear all objects except the initial ones
        self.objects.clear()
        self.robot_position = [0.0, 0.0, 0.0]
        self.robot_orientation = [0.0, 0.0, 0.0]
        self.robot_battery = 1.0
        self.active_gripper = None
        self.is_busy = False

        # Re-setup initial environment
        self._setup_initial_environment()

    def get_simulation_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the simulation.

        Returns:
            Dictionary with simulation statistics
        """
        return {
            'total_objects': len(self.objects),
            'object_types': {
                obj_type.value: len(self.get_objects_by_type(obj_type))
                for obj_type in SimObjectType
            },
            'robot_battery_level': self.robot_battery,
            'is_running': self.running,
            'simulation_speed': self.simulation_speed,
            'timestamp': time.time()
        }


def main():
    """
    Main function to demonstrate the simulation environment.
    """
    print("=== Simulation Environment Demo ===")

    # Create simulation environment
    sim_env = SimulationEnvironment()

    # Start the simulation
    sim_env.start_simulation()

    # Demonstrate different actions
    print("\n1. Current robot status:")
    status = sim_env.get_robot_status()
    print(f"  Position: {status['position']}")
    print(f"  Battery: {status['battery_level']:.2%}")

    print("\n2. Visible objects:")
    visible_objects = sim_env.get_visible_objects()
    for obj in visible_objects:
        print(f"  - {obj.properties['name']} at {obj.position}")

    print("\n3. Executing MOVE action:")
    move_result = sim_env.execute_action(
        SimActionType.MOVE,
        {'vector': [2.0, 0.0, 0.0], 'speed': 0.5, 'relative': True}
    )
    print(f"  Result: {move_result.success}, Message: {move_result.message}")
    print(f"  New position: {sim_env.get_robot_status()['position']}")

    print("\n4. Executing NAVIGATE action to kitchen:")
    navigate_result = sim_env.execute_action(
        SimActionType.NAVIGATE,
        {'location_name': 'kitchen', 'goal_tolerance': 0.5}
    )
    print(f"  Result: {navigate_result.success}, Message: {navigate_result.message}")
    print(f"  New position: {sim_env.get_robot_status()['position']}")

    print("\n5. Executing GRASP action:")
    grasp_result = sim_env.execute_action(
        SimActionType.GRASP,
        {'object_name': 'red cup', 'arm': 'right'}
    )
    print(f"  Result: {grasp_result.success}, Message: {grasp_result.message}")
    print(f"  Active gripper: {sim_env.get_robot_status()['active_gripper']}")

    print("\n6. Executing SPEAK action:")
    speak_result = sim_env.execute_action(
        SimActionType.SPEAK,
        {'text': 'Hello, I have grasped the red cup.', 'volume': 0.8}
    )
    print(f"  Result: {speak_result.success}, Message: {speak_result.message}")

    print("\n7. Executing PLACE action:")
    place_result = sim_env.execute_action(
        SimActionType.PLACE,
        {'arm': 'right'}
    )
    print(f"  Result: {place_result.success}, Message: {place_result.message}")
    print(f"  Active gripper: {sim_env.get_robot_status()['active_gripper']}")

    print("\n8. Environment state:")
    env_state = sim_env.get_environment_state()
    print(f"  Total objects: {len(env_state['objects'])}")
    print(f"  Robot position: {env_state['robot_status']['position']}")

    print("\n9. Simulation stats:")
    stats = sim_env.get_simulation_stats()
    print(f"  Objects: {stats['total_objects']}")
    print(f"  Battery: {stats['robot_battery_level']:.2%}")
    print(f"  Running: {stats['is_running']}")

    # Stop the simulation
    sim_env.stop_simulation()


if __name__ == "__main__":
    main()