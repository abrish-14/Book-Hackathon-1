"""
Task parser for VLA Integration Module.
Parses natural language commands and extracts structured information.
"""
import re
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
import json


@dataclass
class ParsedTask:
    """
    Data class to hold parsed task information.
    """
    command: str
    action_type: str
    parameters: Dict[str, Any]
    confidence: float  # 0.0 to 1.0


class TaskParser:
    """
    Parses natural language commands into structured task information.
    """

    def __init__(self):
        """
        Initialize the task parser with common patterns.
        """
        # Define action patterns with regex
        self.action_patterns = {
            'move': [
                r'move\s+(forward|backward|left|right|up|down)\s*(\d*\.?\d+)\s*(meters?|cm|feet|steps?)?',
                r'go\s+(forward|backward|left|right|up|down)\s*(\d*\.?\d+)\s*(meters?|cm|feet|steps?)?',
                r'walk\s+(forward|backward|left|right)\s*(\d*\.?\d+)\s*(meters?|cm|feet|steps?)?',
                r'go\s+(forward|backward|left|right)\s*(\d*\.?\d+)\s*(meters?|cm|feet|steps?)?',
            ],
            'navigate': [
                r'go\s+to\s+(.+?)(?:\s+and|$)',
                r'move\s+to\s+(.+?)(?:\s+and|$)',
                r'go\s+over\s+to\s+(.+?)(?:\s+and|$)',
                r'head\s+to\s+(.+?)(?:\s+and|$)',
                r'walk\s+to\s+(.+?)(?:\s+and|$)',
            ],
            'grasp': [
                r'pick\s+up\s+(.+?)(?:\s+and|$)',
                r'grab\s+(.+?)(?:\s+and|$)',
                r'take\s+(.+?)(?:\s+and|$)',
                r'pick\s+(.+?)(?:\s+and|$)',
                r'get\s+(.+?)(?:\s+and|$)',
            ],
            'place': [
                r'put\s+(.+?)\s+down',
                r'drop\s+(.+?)',
                r'place\s+(.+?)\s+on',
                r'put\s+(.+?)\s+on',
            ],
            'speak': [
                r'say\s+(.+?)(?:\s+and|$)',
                r'speak\s+(.+?)(?:\s+and|$)',
                r'tell\s+(.+?)(?:\s+and|$)',
            ],
            'turn': [
                r'turn\s+(left|right|around)',
                r'rotate\s+(left|right|clockwise|counterclockwise)',
                r'look\s+(left|right)',
            ],
            'wait': [
                r'wait\s*(\d*\.?\d+)\s*(seconds?|secs?|minutes?|mins?)?',
                r'pause\s*(\d*\.?\d+)\s*(seconds?|secs?|minutes?|mins?)?',
                r'stop\s+for\s*(\d*\.?\d+)\s*(seconds?|secs?|minutes?|mins?)?',
            ]
        }

    def parse_command(self, command: str) -> List[ParsedTask]:
        """
        Parse a natural language command into structured tasks.

        Args:
            command: Natural language command to parse

        Returns:
            List of ParsedTask objects containing structured information
        """
        tasks = []
        command = command.lower().strip()

        # Split compound commands (e.g., "Go to kitchen and pick up cup")
        sub_commands = self._split_compound_commands(command)

        for sub_command in sub_commands:
            task = self._parse_single_command(sub_command)
            if task:
                tasks.append(task)

        return tasks

    def _split_compound_commands(self, command: str) -> List[str]:
        """
        Split compound commands into individual commands.

        Args:
            command: Compound command string

        Returns:
            List of individual command strings
        """
        # Split on common conjunctions
        parts = re.split(r'\s+and\s+|\s+then\s+|\s+after\s+that\s+', command)
        return [part.strip() for part in parts if part.strip()]

    def _parse_single_command(self, command: str) -> Optional[ParsedTask]:
        """
        Parse a single command into structured information.

        Args:
            command: Single command string

        Returns:
            ParsedTask object or None if no match found
        """
        # Try to match each action type
        for action_type, patterns in self.action_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, command)
                if match:
                    # Extract parameters based on the matched pattern
                    params = self._extract_parameters(action_type, match, command)
                    confidence = self._calculate_confidence(action_type, match, command)

                    return ParsedTask(
                        command=command,
                        action_type=action_type,
                        parameters=params,
                        confidence=confidence
                    )

        # If no pattern matches, return a generic task
        return ParsedTask(
            command=command,
            action_type='unknown',
            parameters={'raw_command': command},
            confidence=0.1
        )

    def _extract_parameters(self, action_type: str, match: re.Match, command: str) -> Dict[str, Any]:
        """
        Extract parameters for a matched action.

        Args:
            action_type: Type of action that was matched
            match: Regex match object
            command: Original command string

        Returns:
            Dictionary of parameters
        """
        params = {}

        if action_type in ['move', 'navigate']:
            groups = match.groups()
            if len(groups) >= 1:
                if action_type == 'move':
                    params['direction'] = groups[0] if groups[0] else 'forward'
                    if len(groups) > 1 and groups[1]:  # distance
                        params['distance'] = float(groups[1])
                    if len(groups) > 2 and groups[2]:  # units
                        params['units'] = groups[2]
                elif action_type == 'navigate':
                    params['target_location'] = groups[0]

        elif action_type in ['grasp', 'place']:
            groups = match.groups()
            if len(groups) >= 1:
                params['object'] = groups[0]

        elif action_type == 'speak':
            groups = match.groups()
            if len(groups) >= 1:
                params['text'] = groups[0]

        elif action_type == 'turn':
            groups = match.groups()
            if len(groups) >= 1:
                params['direction'] = groups[0]

        elif action_type == 'wait':
            groups = match.groups()
            if len(groups) >= 1 and groups[0]:  # duration
                duration = float(groups[0])
                params['duration'] = duration
            if len(groups) > 1 and groups[1]:  # units
                params['units'] = groups[1]

        # Add additional context that might be useful
        params['original_command'] = command

        return params

    def _calculate_confidence(self, action_type: str, match: re.Match, command: str) -> float:
        """
        Calculate confidence score for the parsed task.

        Args:
            action_type: Type of action that was matched
            match: Regex match object
            command: Original command string

        Returns:
            Confidence score (0.0 to 1.0)
        """
        confidence = 0.8  # Base confidence for a match

        # Increase confidence for longer, more specific matches
        match_ratio = len(match.group(0)) / len(command) if command else 0
        confidence += (match_ratio * 0.1)

        # Check if the match is at the beginning or end of the command
        if match.start() == 0 or match.end() == len(command):
            confidence += 0.05

        # Cap the confidence at 1.0
        return min(1.0, confidence)

    def validate_task(self, task: ParsedTask) -> bool:
        """
        Validate if a parsed task is reasonable.

        Args:
            task: ParsedTask to validate

        Returns:
            True if task is valid, False otherwise
        """
        # Check confidence threshold
        if task.confidence < 0.3:
            return False

        # Validate action-specific parameters
        if task.action_type == 'move':
            if 'distance' in task.parameters:
                distance = task.parameters['distance']
                # Check if distance is reasonable (not too large or negative)
                if distance <= 0 or distance > 100:  # Max 100 meters for a single move
                    return False

        elif task.action_type == 'grasp':
            if 'object' not in task.parameters or not task.parameters['object'].strip():
                return False

        elif task.action_type == 'wait':
            if 'duration' in task.parameters:
                duration = task.parameters['duration']
                # Check if duration is reasonable (not negative)
                if duration < 0:
                    return False

        return True

    def normalize_command(self, command: str) -> str:
        """
        Normalize a command for consistent parsing.

        Args:
            command: Raw command string

        Returns:
            Normalized command string
        """
        # Convert to lowercase
        command = command.lower()

        # Remove extra whitespace
        command = ' '.join(command.split())

        # Replace common variations with standard forms
        replacements = {
            "robot": "",
            "please": "",
            "could you": "",
            "would you": "",
            "can you": "",
            "kindly": "",
            "if you would": "",
            "i would like you to": "",
            "now": "",
        }

        for old, new in replacements.items():
            command = command.replace(old, new)

        # Clean up extra spaces again after replacements
        command = ' '.join(command.split())

        return command.strip()

    def parse_complex_command(self, command: str) -> List[ParsedTask]:
        """
        Parse a complex command that might involve multiple steps.

        Args:
            command: Complex command string

        Returns:
            List of ParsedTask objects
        """
        # Normalize the command first
        normalized_command = self.normalize_command(command)

        # Parse the command
        tasks = self.parse_command(normalized_command)

        # Filter out low-confidence or invalid tasks
        valid_tasks = [task for task in tasks if self.validate_task(task)]

        return valid_tasks


# Example usage:
if __name__ == "__main__":
    parser = TaskParser()

    # Test commands
    test_commands = [
        "Move forward 2 meters",
        "Go to the kitchen and pick up the red cup",
        "Turn left and wait for 5 seconds",
        "Say hello world",
        "Grab the blue box",
        "Go to the living room"
    ]

    for cmd in test_commands:
        print(f"\nCommand: {cmd}")
        parsed_tasks = parser.parse_complex_command(cmd)
        for task in parsed_tasks:
            print(f"  Action: {task.action_type}")
            print(f"  Parameters: {task.parameters}")
            print(f"  Confidence: {task.confidence:.2f}")