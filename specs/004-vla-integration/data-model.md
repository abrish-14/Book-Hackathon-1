# Data Model: Vision-Language-Action (VLA) Integration Module

## Core Entities

### VoiceCommand
- **Fields**:
  - id: string (unique identifier)
  - audio_data: binary (raw audio input)
  - transcription: string (transcribed text from Whisper)
  - timestamp: datetime (when command was received)
  - confidence: float (Whisper transcription confidence score)
  - source: string (audio source identifier)

### ActionPlan
- **Fields**:
  - id: string (unique identifier)
  - command_id: string (reference to VoiceCommand)
  - llm_response: string (raw LLM output)
  - task_sequence: array of objects (ordered list of tasks)
  - validation_status: enum (valid, invalid, pending)
  - robot_compatibility: boolean (whether actions are compatible with robot capabilities)
  - created_at: datetime

### Task
- **Fields**:
  - id: string (unique identifier)
  - action_type: string (move, grasp, speak, etc.)
  - parameters: object (action-specific parameters)
  - priority: integer (execution priority)
  - dependencies: array of strings (other task IDs this task depends on)

### ROS2ActionSequence
- **Fields**:
  - id: string (unique identifier)
  - plan_id: string (reference to ActionPlan)
  - ros2_commands: array of objects (ROS 2 action commands)
  - execution_status: enum (pending, running, completed, failed)
  - feedback: object (execution feedback from ROS 2)
  - completed_at: datetime (optional)

### VisionInput
- **Fields**:
  - id: string (unique identifier)
  - image_data: binary (raw image/sensor data)
  - timestamp: datetime
  - source: string (camera/sensor identifier)
  - processed_data: object (processed vision data)

### VLAPipelineState
- **Fields**:
  - id: string (unique identifier)
  - current_stage: enum (listening, processing, planning, executing, completed)
  - active_command: string (current command ID being processed)
  - last_error: string (optional error message)
  - updated_at: datetime

## Relationships

- VoiceCommand → ActionPlan (one-to-many: one command may result in multiple plan attempts)
- ActionPlan → Task (one-to-many: one plan contains multiple tasks)
- ActionPlan → ROS2ActionSequence (one-to-one: each plan maps to one execution sequence)
- Task → ROS2ActionSequence (many-to-many through execution mapping)

## Validation Rules

- VoiceCommand.transcription must not be empty
- ActionPlan.task_sequence must contain at least one task
- Task.action_type must be from predefined list of supported actions
- ROS2ActionSequence.ros2_commands must be valid ROS 2 action formats
- ActionPlan.robot_compatibility must be validated before execution

## State Transitions

### VoiceCommand
- pending → processed (when transcription is complete)
- processed → planned (when LLM generates action plan)
- processed → failed (when transcription fails)

### ActionPlan
- pending → validated (when compatibility check passes)
- validated → executing (when sent to ROS 2 executor)
- executing → completed/failed (based on execution result)

### VLAPipelineState
- listening → processing (when voice command received)
- processing → planning (when transcription ready)
- planning → executing (when action plan ready)
- executing → completed/listening (when action sequence completed)