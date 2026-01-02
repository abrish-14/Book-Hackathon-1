---
title: "Cognitive Planning with LLMs & ROS 2 Action Sequences"
sidebar_label: "Chapter 2"
description: "Explore how LLMs can plan multi-step tasks and execute them using ROS 2"
---

# Chapter 2: Cognitive Planning with LLMs & ROS 2 Action Sequences

## Overview

This chapter covers the cognitive planning component of the Vision-Language-Action (VLA) system. You'll learn how to use large language models to decompose complex natural language commands into detailed action sequences that can be executed using ROS 2. This enables sophisticated robot behavior by allowing LLMs to plan multi-step tasks autonomously.

## Learning Objectives

By the end of this chapter, you will be able to:
- Integrate LLMs for cognitive planning
- Parse natural language commands into action sequences
- Generate ROS 2 action sequences from LLM output
- Handle multi-step task planning and execution
- Validate planned actions against robot capabilities

## Prerequisites

Before starting this chapter, ensure you have:
- Completed Chapter 1 (Voice-to-Action with OpenAI Whisper)
- ROS 2 Humble Hawksbill installed
- Python 3.10 or higher
- OpenAI API key
- Basic understanding of ROS 2 actions and services

## LLM Integration for Cognitive Planning

### Overview of LLM Planning

The LLM planning system uses large language models to decompose high-level commands into detailed action sequences. The system takes transcribed voice commands from Chapter 1 and generates appropriate ROS 2 action sequences.

### LLM Client Implementation

```python
# llm_planner/llm_client.py
import openai
from config.settings import OPENAI_API_KEY

openai.api_key = OPENAI_API_KEY

class LLMClient:
    def __init__(self, model="gpt-3.5-turbo"):
        self.model = model

    def plan_actions(self, command_text):
        """
        Plan actions based on natural language command
        """
        prompt = f"""
        Given the following command: "{command_text}"

        Generate a sequence of actions for a humanoid robot.
        Return the actions in JSON format with the following structure:
        {{
          "actions": [
            {{
              "action_type": "move_to",
              "parameters": {{"x": 1.0, "y": 2.0, "z": 0.0}},
              "description": "Move to specified coordinates"
            }}
          ]
        }}

        The possible action types are: move_to, pick_up, place, speak, wait, look_at
        """

        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.3
        )

        return response.choices[0].message.content
```

## Task Parsing and Action Generation

### Task Parser Implementation

The task parser converts LLM output into executable ROS 2 actions:

```python
# llm_planner/task_parser.py
import json
from .action_generator import ActionGenerator

class TaskParser:
    def __init__(self):
        self.action_generator = ActionGenerator()

    def parse_task(self, llm_output):
        """
        Parse LLM output and generate ROS 2 action sequence
        """
        try:
            # Extract JSON from LLM response
            start = llm_output.find('{')
            end = llm_output.rfind('}') + 1
            json_str = llm_output[start:end]
            task_data = json.loads(json_str)

            actions = []
            for action_data in task_data.get('actions', []):
                action = self.action_generator.create_action(
                    action_data['action_type'],
                    action_data['parameters'],
                    action_data['description']
                )
                actions.append(action)

            return actions
        except Exception as e:
            print(f"Error parsing task: {e}")
            return []
```

### Action Generator Implementation

```python
# llm_planner/action_generator.py
class ActionGenerator:
    def __init__(self):
        self.action_types = {
            'move_to': self._create_move_action,
            'pick_up': self._create_pickup_action,
            'place': self._create_place_action,
            'speak': self._create_speak_action,
            'wait': self._create_wait_action,
            'look_at': self._create_look_action
        }

    def create_action(self, action_type, parameters, description):
        """
        Create a ROS 2 action based on type and parameters
        """
        if action_type in self.action_types:
            return self.action_types[action_type](parameters, description)
        else:
            raise ValueError(f"Unknown action type: {action_type}")

    def _create_move_action(self, parameters, description):
        return {
            'type': 'move_to',
            'target_pose': {
                'position': {
                    'x': parameters.get('x', 0.0),
                    'y': parameters.get('y', 0.0),
                    'z': parameters.get('z', 0.0)
                },
                'orientation': {
                    'w': 1.0,
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0
                }
            },
            'description': description
        }

    def _create_pickup_action(self, parameters, description):
        return {
            'type': 'pick_up',
            'object_name': parameters.get('object_name', 'unknown'),
            'description': description
        }

    def _create_place_action(self, parameters, description):
        return {
            'type': 'place',
            'target_location': parameters.get('location', {}),
            'description': description
        }

    def _create_speak_action(self, parameters, description):
        return {
            'type': 'speak',
            'text': parameters.get('text', ''),
            'description': description
        }

    def _create_wait_action(self, parameters, description):
        return {
            'type': 'wait',
            'duration': parameters.get('duration', 1.0),
            'description': description
        }

    def _create_look_action(self, parameters, description):
        return {
            'type': 'look_at',
            'target': parameters.get('target', {}),
            'description': description
        }
```

## ROS 2 Action Integration

### Action Client for ROS 2 Execution

```python
# ros2_executor/action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

class ActionExecutorClient(Node):
    def __init__(self):
        super().__init__('action_executor_client')
        self.action_clients = {}

    def execute_action_sequence(self, actions):
        """
        Execute a sequence of actions using ROS 2
        """
        results = []
        for action in actions:
            result = self.execute_single_action(action)
            results.append(result)
        return results

    def execute_single_action(self, action):
        """
        Execute a single action based on its type
        """
        action_type = action['type']

        if action_type == 'move_to':
            return self._execute_move_to(action)
        elif action_type == 'pick_up':
            return self._execute_pick_up(action)
        elif action_type == 'place':
            return self._execute_place(action)
        elif action_type == 'speak':
            return self._execute_speak(action)
        elif action_type == 'wait':
            return self._execute_wait(action)
        elif action_type == 'look_at':
            return self._execute_look_at(action)
        else:
            self.get_logger().error(f"Unknown action type: {action_type}")
            return False

    def _execute_move_to(self, action):
        # Implementation for move_to action
        self.get_logger().info(f"Moving to position: {action['target_pose']['position']}")
        # Add actual ROS 2 move action implementation
        return True

    def _execute_pick_up(self, action):
        # Implementation for pick_up action
        self.get_logger().info(f"Picking up object: {action['object_name']}")
        # Add actual ROS 2 pick action implementation
        return True

    # Other action implementations would follow similar patterns
```

## Testing Multi-Step Task Planning

### Test Scenario: Complex Command Execution

1. **Given** a complex command like "Go to the kitchen, pick up the red cup, and bring it to the living room"
2. **When** the LLM processes this command
3. **Then** it should generate a sequence of discrete actions that accomplish the goal

### Test Implementation

```python
# tests/integration/test_llm_planning.py
import unittest
from llm_planner import LLMClient, TaskParser

class TestLLMPlanning(unittest.TestCase):
    def setUp(self):
        self.llm_client = LLMClient()
        self.task_parser = TaskParser()

    def test_complex_task_planning(self):
        command = "Go to the kitchen, pick up the red cup, and bring it to the living room"

        # Get LLM response
        llm_output = self.llm_client.plan_actions(command)

        # Parse the response into actions
        actions = self.task_parser.parse_task(llm_output)

        # Verify we have multiple actions
        self.assertGreater(len(actions), 1)

        # Verify action types
        action_types = [action['type'] for action in actions]
        expected_types = ['move_to', 'pick_up', 'move_to']
        for expected_type in expected_types:
            self.assertIn(expected_type, action_types)

    def test_action_validation(self):
        # Test that actions are properly formatted
        command = "Move forward 2 meters"
        llm_output = self.llm_client.plan_actions(command)
        actions = self.task_parser.parse_task(llm_output)

        # Verify action structure
        for action in actions:
            self.assertIn('type', action)
            self.assertIn('description', action)
```

## Error Handling and Validation

### Action Validation

```python
# llm_planner/action_validator.py
class ActionValidator:
    @staticmethod
    def validate_action(action, robot_capabilities):
        """
        Validate that an action is feasible given robot capabilities
        """
        action_type = action['type']

        if action_type == 'move_to':
            return ActionValidator._validate_move_action(action, robot_capabilities)
        elif action_type == 'pick_up':
            return ActionValidator._validate_pickup_action(action, robot_capabilities)
        # Add validation for other action types

        return True

    @staticmethod
    def _validate_move_action(action, robot_capabilities):
        # Check if target position is within robot's operational range
        target = action['target_pose']['position']
        max_range = robot_capabilities.get('max_movement_range', 10.0)

        distance = (target['x']**2 + target['y']**2 + target['z']**2)**0.5
        return distance <= max_range

    @staticmethod
    def _validate_pickup_action(action, robot_capabilities):
        # Check if robot has manipulation capabilities
        return robot_capabilities.get('has_manipulator', False)
```

## Summary

In this chapter, you've learned how to implement cognitive planning using large language models. You've created a system that takes natural language commands and generates appropriate ROS 2 action sequences. The LLM planning component enables sophisticated robot behavior by allowing the robot to decompose complex commands into detailed action sequences.

## Next Steps

In the final chapter, we'll integrate the voice processing and cognitive planning components into a complete VLA pipeline and demonstrate autonomous humanoid behavior in simulation.