"""
OpenAI Integration Service

This service provides an interface to the OpenAI API for natural language processing
of robot commands. It handles translating natural language commands into actionable
robot instructions through prompt engineering and context management.

As per FR015: Command Dispatch via OpenAI API, this allows admins to send commands
to robots through an integrated chat interface powered by OpenAI API.
"""

import json
import logging
import os
from typing import Dict, List, Any, Optional

import openai
import requests
from pydantic import BaseModel

# Set up logging
logger = logging.getLogger(__name__)

# Set up OpenAI API client
openai.api_key = os.environ.get("OPENAI_API_KEY")


class RobotCommand(BaseModel):
    """Model for parsed robot command data"""
    command_type: str
    robot_id: str
    parameters: Dict[str, Any] = {}
    confidence: float
    raw_text: str


class OpenAIService:
    """
    Service for integrating with OpenAI API to process natural language commands
    """
    _instance = None
    _initialized = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(OpenAIService, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        
        # Get configuration from environment variables
        self.api_key = os.environ.get("OPENAI_API_KEY")
        self.model = os.environ.get("OPENAI_MODEL", "gpt-4")
        
        # Configure OpenAI client if API key is provided
        if self.api_key:
            openai.api_key = self.api_key
            self._has_api_key = True
        else:
            self._has_api_key = False
            logger.warning(
                "OpenAI API key not provided. Natural language command "
                "processing will not be available."
            )
        
        # Initialize conversation history
        self._conversation_history = {}  # user_id -> List[Dict]
        
        # List of supported robot commands
        self._supported_commands = [
            "move", "stop", "rotate", "pick", "place", "navigate_to",
            "set_speed", "collect_data", "take_photo", "scan_area",
            "follow_path", "return_to_base", "activate_sensor", 
            "deactivate_sensor", "start_recording", "stop_recording"
        ]
        
        self._initialized = True
        logger.info("OpenAIService initialized")

    def _build_system_prompt(self, robot_ids: List[str]) -> str:
        """
        Build the system prompt for the OpenAI model
        
        Args:
            robot_ids: List of available robot IDs
            
        Returns:
            String containing the system prompt
        """
        commands_str = ", ".join(self._supported_commands)
        robots_str = ", ".join(robot_ids)
        
        return (
            f"You are a ROS2 robot command translator. Your job is to convert natural language "
            f"instructions into structured commands for robots in the WiseVision system.\n\n"
            f"Available robots: {robots_str}\n"
            f"Available commands: {commands_str}\n\n"
            f"When given a natural language command, respond with a JSON object containing:\n"
            f"- command_type: The type of command to execute (one of the available commands)\n"
            f"- robot_id: The ID of the robot to execute the command\n"
            f"- parameters: Any parameters needed for the command (dictionary)\n\n"
            f"If you cannot determine a valid command, respond with a JSON object containing:\n"
            f"- error: A description of the error\n\n"
            f"Only respond with valid JSON. Do not include any other text in your response."
        )

    def process_command(
        self,
        user_id: str,
        command_text: str,
        available_robot_ids: List[str]
    ) -> RobotCommand:
        """
        Process a natural language command using OpenAI API
        
        Args:
            user_id: ID of the user sending the command
            command_text: Natural language command text
            available_robot_ids: List of available robot IDs
            
        Returns:
            RobotCommand object containing the parsed command
            
        Raises:
            ValueError: If OpenAI API key is not configured
            Exception: If there's an error processing the command
        """
        if not self._has_api_key:
            raise ValueError("OpenAI API key not configured")
        
        try:
            # Initialize conversation history for this user if needed
            if user_id not in self._conversation_history:
                self._conversation_history[user_id] = []
            
            # Build the conversation messages
            system_prompt = self._build_system_prompt(available_robot_ids)
            messages = [
                {"role": "system", "content": system_prompt},
                *self._conversation_history[user_id][-5:],  # Include last 5 messages for context
                {"role": "user", "content": command_text}
            ]
            
            # Call the OpenAI API
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=messages,
                temperature=0.2,  # Lower temperature for more deterministic outputs
                max_tokens=300,
                n=1,
                stop=None
            )
            
            # Get the response text
            response_text = response.choices[0].message['content'].strip()
            
            # Update conversation history
            self._conversation_history[user_id].append({"role": "user", "content": command_text})
            self._conversation_history[user_id].append({"role": "assistant", "content": response_text})
            
            # Parse the JSON response
            try:
                response_json = json.loads(response_text)
                
                # Check if the response contains an error
                if "error" in response_json:
                    raise ValueError(response_json["error"])
                
                # Validate the response
                if "command_type" not in response_json or "robot_id" not in response_json:
                    raise ValueError("Missing required fields in command response")
                
                # Check if the command type is supported
                if response_json["command_type"] not in self._supported_commands:
                    raise ValueError(f"Unsupported command type: {response_json['command_type']}")
                
                # Check if the robot ID is valid
                if response_json["robot_id"] not in available_robot_ids:
                    raise ValueError(f"Invalid robot ID: {response_json['robot_id']}")
                
                # Create the robot command
                command = RobotCommand(
                    command_type=response_json["command_type"],
                    robot_id=response_json["robot_id"],
                    parameters=response_json.get("parameters", {}),
                    confidence=response.choices[0].finish_reason == "stop" and 0.9 or 0.5,
                    raw_text=command_text
                )
                
                return command
                
            except json.JSONDecodeError as e:
                logger.error(f"Failed to parse OpenAI response as JSON: {e}")
                logger.debug(f"Response text: {response_text}")
                raise ValueError("Failed to parse OpenAI response as JSON")
            
        except Exception as e:
            logger.error(f"Error processing command: {e}")
            raise

    def get_conversation_history(self, user_id: str) -> List[Dict]:
        """
        Get the conversation history for a user
        
        Args:
            user_id: ID of the user
            
        Returns:
            List of conversation messages
        """
        return self._conversation_history.get(user_id, [])

    def clear_conversation_history(self, user_id: str) -> bool:
        """
        Clear the conversation history for a user
        
        Args:
            user_id: ID of the user
            
        Returns:
            True if successful, False otherwise
        """
        if user_id in self._conversation_history:
            self._conversation_history[user_id] = []
            return True
        return False

    def get_supported_commands(self) -> List[str]:
        """
        Get the list of supported robot commands
        
        Returns:
            List of supported command types
        """
        return self._supported_commands.copy()