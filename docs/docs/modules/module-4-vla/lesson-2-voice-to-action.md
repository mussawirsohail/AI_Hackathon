---
sidebar_position: 2
---

# Lesson 2: Voice-to-Action with OpenAI Whisper for Robotic Control

## Introduction to Voice Commands in Robotics

Natural voice commands provide an intuitive way for humans to interact with robots. Using OpenAI Whisper, a state-of-the-art speech recognition model, robots can understand and respond to spoken commands in real-time.

## OpenAI Whisper Overview

Whisper is a general-purpose speech recognition model that can transcribe audio in multiple languages. For robotics applications, Whisper can be used to convert voice commands into text that can be processed by robot reasoning systems.

### Whisper in Robotics Context

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import openai
import numpy as np
import pyaudio
import wave
import tempfile
import threading
from io import BytesIO

class WhisperVoiceNode(Node):
    def __init__(self):
        super().__init__('whisper_voice_node')
        
        # Initialize Whisper client
        # This assumes you have access to OpenAI API or are using a locally deployed version
        self.whisper_client = openai.OpenAI()  # For OpenAI API
        
        # Audio recording parameters
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024
        self.record_seconds = 5
        
        # Publisher for transcribed commands
        self.command_pub = self.create_publisher(String, '/robot_command', 10)
        
        # Initialize audio interface
        self.audio_interface = pyaudio.PyAudio()
        
        # Start recording thread
        self.recording = False
        self.record_thread = threading.Thread(target=self.continuous_recording)
        self.record_thread.start()
        
    def continuous_recording(self):
        """Continuously record audio and send to Whisper for transcription"""
        while rclpy.ok():
            # Record audio
            frames = []
            
            stream = self.audio_interface.open(
                format=self.audio_format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk
            )
            
            # Record for specified duration
            for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
                data = stream.read(self.chunk)
                frames.append(data)
            
            # Stop recording
            stream.stop_stream()
            stream.close()
            
            # Convert recorded audio to WAV format
            audio_data = self.save_audio_as_wav(frames)
            
            # Transcribe using Whisper
            try:
                transcription = self.transcribe_audio(audio_data)
                
                if transcription.strip():
                    self.publish_command(transcription)
                    
            except Exception as e:
                self.get_logger().error(f"Error in transcription: {e}")
                
            # Sleep before next recording
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))
    
    def save_audio_as_wav(self, frames):
        """Save recorded audio frames to a temporary WAV file"""
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
        
        with wave.open(temp_file.name, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.audio_interface.get_sample_size(self.audio_format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(frames))
            
        return temp_file.name
    
    def transcribe_audio(self, audio_file_path):
        """Transcribe audio file using OpenAI Whisper API"""
        with open(audio_file_path, "rb") as audio_file:
            response = self.whisper_client.audio.transcriptions.create(
                model="whisper-1",
                file=audio_file
            )
        
        return response.text
    
    def publish_command(self, transcription):
        """Publish transcribed command to robot command topic"""
        if self.is_robot_command(transcription):
            cmd_msg = String()
            cmd_msg.data = transcription
            self.command_pub.publish(cmd_msg)
            self.get_logger().info(f"Published command: {transcription}")
    
    def is_robot_command(self, text):
        """Check if the transcribed text contains robot commands"""
        # Simple heuristic - in practice, this could be more sophisticated
        robot_keywords = [
            'move', 'go', 'stop', 'turn', 'pick', 'place', 'clean',
            'bring', 'take', 'grasp', 'navigate', 'help', 'come'
        ]
        
        text_lower = text.lower()
        return any(keyword in text_lower for keyword in robot_keywords)

def main(args=None):
    rclpy.init(args=args)
    node = WhisperVoiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Whisper voice node...")
    finally:
        node.audio_interface.terminate()
        node.destroy_node()
        rclpy.shutdown()
```

## Local Whisper Deployment for Privacy and Latency

For real-time applications and privacy concerns, it's often better to deploy Whisper locally:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import torch
import whisper
import pyaudio
import wave
import numpy as np
import tempfile

class LocalWhisperNode(Node):
    def __init__(self):
        super().__init__('local_whisper_node')
        
        # Load Whisper model locally
        # Options: tiny, base, small, medium, large
        self.model = whisper.load_model("small")
        
        # Audio recording parameters
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024
        self.record_seconds = 5
        
        # Publisher for transcribed commands
        self.command_pub = self.create_publisher(String, '/robot_command', 10)
        
        # Initialize audio interface
        self.audio_interface = pyaudio.PyAudio()
        
        self.get_logger().info("Local Whisper node initialized with small model")
    
    def transcribe_audio_local(self, audio_file_path):
        """Transcribe audio file using local Whisper model"""
        result = self.model.transcribe(audio_file_path)
        return result["text"]
    
    def record_and_transcribe(self):
        """Record audio and transcribe it locally"""
        frames = []
        
        stream = self.audio_interface.open(
            format=self.audio_format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        self.get_logger().info("Recording...")
        
        # Record audio
        for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
            data = stream.read(self.chunk)
            frames.append(data)
        
        # Stop recording
        stream.stop_stream()
        stream.close()
        
        # Save to temporary file
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
        with wave.open(temp_file.name, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.audio_interface.get_sample_size(self.audio_format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(frames))
        
        # Transcribe
        transcription = self.transcribe_audio_local(temp_file.name)
        return transcription
```

## Speech Processing with VAD (Voice Activity Detection)

Using Voice Activity Detection (VAD) improves the efficiency of voice processing by only capturing when someone is talking:

```python
import webrtcvad
import collections

class VADWhisperNode(Node):
    def __init__(self):
        super().__init__('vad_whisper_node')
        
        # Initialize Whisper model
        self.model = whisper.load_model("tiny")
        
        # VAD parameters
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(3)  # Aggressive mode
        
        # Audio parameters
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk_duration = 30  # ms (must be 10, 20, or 30)
        self.chunk_size = int(self.rate * self.chunk_duration / 1000)
        
        # Ring buffer to hold audio frames
        self.ring_buffer = collections.deque(maxlen=int(30 * self.rate / 1000))  # 30 seconds buffer
        
        # Publisher for commands
        self.command_pub = self.create_publisher(String, '/robot_command', 10)
        
        # Audio interface
        self.audio_interface = pyaudio.PyAudio()
        
        # Start listening
        self.start_listening()
    
    def start_listening(self):
        stream = self.audio_interface.open(
            format=self.audio_format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )
        
        voiced_frames = []
        triggered = False
        
        while rclpy.ok():
            # Read audio chunk
            chunk = stream.read(self.chunk_size)
            
            # Convert to raw format for VAD
            is_speech = self.vad.is_speech(chunk, self.rate)
            
            if not triggered:
                # Not in a voice activity state
                if is_speech:
                    # Voice activity detected
                    triggered = True
                    voiced_frames.extend(self.ring_buffer)  # Add buffered frames
                    voiced_frames.append(chunk)
                    self.get_logger().info("Voice activity detected")
            else:
                # In a voice activity state
                if is_speech:
                    # Continue to add frames
                    voiced_frames.append(chunk)
                else:
                    # Voice activity ended
                    if len(voiced_frames) > 1:  # Only process if we have meaningful speech
                        self.process_voice_command(b''.join(voiced_frames))
                    
                    triggered = False
                    voiced_frames = []
            
            # Add chunk to ring buffer
            self.ring_buffer.append(chunk)
    
    def process_voice_command(self, audio_data):
        """Process recorded voice command"""
        # Convert raw audio to WAV
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
        
        with wave.open(temp_file.name, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.audio_interface.get_sample_size(self.audio_format))
            wf.setframerate(self.rate)
            wf.writeframes(audio_data)
        
        # Transcribe
        try:
            result = self.model.transcribe(temp_file.name)
            transcription = result["text"]
            
            if transcription.strip():
                self.get_logger().info(f"Transcribed: {transcription}")
                
                # Check if it's a robot command
                if self.is_robot_command(transcription):
                    cmd_msg = String()
                    cmd_msg.data = transcription
                    self.command_pub.publish(cmd_msg)
                    
        except Exception as e:
            self.get_logger().error(f"Error during transcription: {e}")
    
    def is_robot_command(self, text):
        """Simple command detection"""
        robot_keywords = [
            'move', 'go', 'stop', 'turn', 'pickup', 'place', 'clean',
            'bring', 'take', 'get', 'help', 'come here', 'follow me'
        ]
        
        text_lower = text.lower()
        return any(keyword in text_lower for keyword in robot_keywords)
```

## Integration with ROS 2 Speech Recognition Interface

To follow ROS 2 best practices, let's create a proper speech recognition action server:

```python
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from speech_recognition_msgs.action import RecognizeSpeech
import threading

class SpeechRecognitionActionServer(Node):
    def __init__(self):
        super().__init__('speech_recognition_action_server')
        
        # Initialize Whisper model
        self.model = whisper.load_model("base")
        
        # Use reentrant callback group since we'll have concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Create action server
        self._action_server = ActionServer(
            self,
            RecognizeSpeech,
            'recognize_speech',
            self.execute_callback,
            callback_group=self.callback_group
        )
        
        # Publisher for recognized text
        self.recognized_text_pub = self.create_publisher(String, '/recognized_text', 10)
        
        self.get_logger().info("Speech Recognition Action Server ready")
    
    def execute_callback(self, goal_handle):
        """Execute the speech recognition action"""
        self.get_logger().info('Executing speech recognition goal')
        
        # Get audio data from goal
        audio_data = goal_handle.request.audio
        
        # Transcribe the audio
        try:
            # Convert ROS AudioData to appropriate format for Whisper
            transcription = self.transcribe_audio_data(audio_data)
            
            # Create result
            result = RecognizeSpeech.Result()
            result.transcript = transcription
            result.confidence = 1.0  # Placeholder - in real system, use model's confidence
            
            # Publish the recognized text
            text_msg = String()
            text_msg.data = transcription
            self.recognized_text_pub.publish(text_msg)
            
            goal_handle.succeed()
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error in speech recognition: {e}")
            goal_handle.abort()
            result = RecognizeSpeech.Result()
            result.transcript = ""
            result.confidence = 0.0
            return result
    
    def transcribe_audio_data(self, audio_data_msg):
        """Transcribe ROS AudioData message using Whisper"""
        # This would convert the ROS audio message format to something
        # Whisper can process - implementation would depend on the audio format
        # and might require additional audio processing libraries
        
        # For now, showing the general approach
        # Convert audio_data_msg.data to WAV format
        # with appropriate sample rate and encoding
        pass
```

## Advanced: Wake Word Detection

For always-listening systems, it's important to implement wake word detection:

```python
import collections
import numpy as np

class WakeWordDetector:
    def __init__(self, wake_words=["robot", "hey robot", "assistant"]):
        self.wake_words = wake_words
        self.audio_buffer = collections.deque(maxlen=40000)  # 1 second buffer at 16kHz
        
    def is_wake_word_spotted(self, audio_chunk):
        """Detect if a wake word is present in the audio"""
        # This is a simplified approach
        # In practice, you'd use a dedicated wake word detection model like Porcupine
        # or implement keyword spotting using machine learning
        
        # Add chunk to buffer
        self.audio_buffer.extend(audio_chunk)
        
        # Convert to text and check for wake words
        # This is a placeholder - real implementation would use a lightweight
        # keyword detection model
        return False  # Placeholder return
```

## Performance Considerations

When implementing Whisper-based systems:

1. **Model Size**: Larger models are more accurate but slower. Choose based on your latency requirements.

2. **Latency vs. Accuracy**: Consider using faster models for real-time applications.

3. **Hardware Acceleration**: Use GPU acceleration if available to speed up processing.

4. **Offline vs. Online**: Local models eliminate network dependency and improve privacy.

Continue to [Lesson 3: Cognitive Planning with LLMs](./lesson-3-cognitive-planning.md) to learn how to use Large Language Models for translating natural language commands into sequences of robotic actions.