# Voice-to-Action: Enabling Conversational Robotics with Speech Recognition and ROS 2

## Learning Objectives

-   Understand the basics of speech recognition for robotics.
-   Explore how to integrate speech-to-text (STT) services (like OpenAI Whisper) with ROS 2.
-   Learn to trigger robot actions based on voice commands.

## Core Concepts

Giving robots the ability to understand and respond to human speech is a crucial step towards more intuitive and natural Human-Robot Interaction (HRI). This "Voice-to-Action" pipeline involves two main components:

1.  **Speech Recognition (Speech-to-Text - STT)**: This technology converts spoken language into written text. For robotics, this text then becomes the input for further processing to extract commands or intentions.
    -   **OpenAI Whisper**: A powerful general-purpose speech recognition model capable of transcribing audio in multiple languages and translating them into English. It can be run locally or via an API.
    -   **Other STT Engines**: Google Speech-to-Text, AssemblyAI, CMU Sphinx (local, open-source).

2.  **Natural Language Understanding (NLU) & Command Parsing**: Once speech is transcribed to text, the robot needs to understand the intent behind the words. Simple approaches involve keyword spotting (e.g., "move forward", "stop", "pick up"), while more advanced systems use NLU models to parse complex sentences and extract structured commands (e.g., "pick up the red cube from the table").

3.  **ROS 2 Action Mapping**: The parsed commands are then mapped to specific ROS 2 actions or services that the robot can execute. This creates a bridge between human language and robot capabilities.

**Why Voice Control is Important in Physical AI:**
-   **Natural Interface**: Voice is a natural and intuitive way for humans to interact.
-   **Hands-Free Operation**: Allows operators to control robots while performing other tasks.
-   **Accessibility**: Can improve accessibility for users with mobility impairments.
-   **Complex Task Initiation**: Can initiate complex sequences of actions with simple verbal cues.

## Step-by-Step Lab: Conceptualizing a Voice Command System with ROS 2

This lab will guide you conceptually through building a voice command system. We will outline the components and their interactions, but the full implementation will require external STT services or local models.

### Code Examples (Conceptual)

Imagine a scenario where a robot can respond to "Robot, move forward" or "Robot, stop."

1.  **Audio Capture Node (ROS 2 Python)**: This node would capture audio from a microphone and publish it to a ROS 2 topic.

```python
# Conceptual: audio_capture_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray # Or a custom audio message type
# ... (microphone interface code) ...

class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture_node')
        self.publisher_ = self.create_publisher(ByteMultiArray, 'audio_input', 10)
        # ... (setup microphone and timer for publishing audio chunks) ...

    def publish_audio(self, audio_chunk):
        msg = ByteMultiArray()
        msg.data = list(audio_chunk) # Convert bytes to list of ints
        self.publisher_.publish(msg)
    # ...

def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2.  **Speech-to-Text Node (Python, integrating Whisper)**: This node subscribes to the audio topic, sends it to a Whisper model (local or API), and publishes the transcribed text to another ROS 2 topic.

```python
# Conceptual: stt_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
# from openai import OpenAI # For API or local Whisper model
# ... (Whisper integration code) ...

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.subscription = self.create_subscription(
            ByteMultiArray, 'audio_input', self.audio_callback, 10)
        self.text_publisher_ = self.create_publisher(String, 'voice_command_text', 10)
        # self.whisper_client = OpenAI() # Or load local model

    def audio_callback(self, msg):
        audio_data = bytes(msg.data)
        # Conceptual: Call Whisper to transcribe audio_data
        # response = self.whisper_client.audio.transcriptions.create(
        #     model="whisper-1",
        #     file=audio_data # This needs to be a file-like object in reality
        # )
        # transcribed_text = response.text
        transcribed_text = "Robot, move forward" # Placeholder for actual transcription

        if transcribed_text:
            self.get_logger().info(f"Transcribed: '{transcribed_text}'")
            text_msg = String()
            text_msg.data = transcribed_text
            self.text_publisher_.publish(text_msg)
    # ...

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3.  **Command Interpreter Node (ROS 2 Python)**: This node subscribes to the transcribed text, parses it, and publishes commands (e.g., to a motor control topic or a navigation action goal).

```python
# Conceptual: command_interpreter_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from geometry_msgs.msg import Twist # For movement commands

class CommandInterpreterNode(Node):
    def __init__(self):
        super().__init__('command_interpreter_node')
        self.subscription = self.create_subscription(
            String, 'voice_command_text', self.text_callback, 10)
        # self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10) # For movement

    def text_callback(self, msg):
        command = msg.data.lower()
        if "move forward" in command:
            self.get_logger().info("Executing: Move Forward")
            # Publish Twist message or send navigation goal
        elif "stop" in command:
            self.get_logger().info("Executing: Stop")
            # Publish zero velocity Twist message
        else:
            self.get_logger().warn(f"Unknown command: '{command}'")
    # ...

def main(args=None):
    rclpy.init(args=args)
    node = CommandInterpreterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Hardware/Cloud Alternative

-   **Local Machine**: Running a local Whisper model can be resource-intensive, especially larger variants. A good CPU is recommended, and if using GPU-accelerated versions, an NVIDIA GPU.
-   **Cloud STT Services**: Using cloud-based STT APIs (like OpenAI, Google) offloads the computation but requires an internet connection and API keys.
-   **Jetson Devices**: NVIDIA Jetson devices are well-suited for running optimized STT models locally, especially with NVIDIA's Riva SDK for speech AI.

## Summary

Integrating voice control into robotics systems, through speech recognition and ROS 2, enables more natural human-robot interaction. By chaining an audio capture node, a speech-to-text node (using models like Whisper), and a command interpreter node, robots can respond to verbal commands, making them more accessible and user-friendly.

## Assessment / Mini Project

1.  **Question**: What are the main benefits and challenges of using voice commands to control a robot?
2.  **Question**: Why is it necessary to have a "Command Interpreter Node" after transcribing speech to text?
3.  **Mini-Project**: Research the NVIDIA Riva SDK. How could it be used on a Jetson device to create a high-performance, local speech-to-text system for a ROS 2 robot? Outline the conceptual steps and required components.
