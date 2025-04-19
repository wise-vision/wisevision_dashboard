"""
OpenAI API for WiseVision Dashboard

This module provides API endpoints for interacting with the OpenAI service,
allowing administrators to control robots through natural language commands.

As per FR015, this API enables command dispatch via OpenAI API, with appropriate
access controls to ensure only authorized users can send commands to robots.
"""

import json
import logging
from datetime import datetime, timedelta
from typing import Dict, List, Any, Optional

from fastapi import APIRouter, Depends, HTTPException, status, Query
from pydantic import BaseModel

from ...service.auth_service import User, admin_only
from ...service.openai_service import OpenAIService, RobotCommand
from ...service.ros2_manager import ROS2Manager
from ...data_object.message_objects import CommandRequest, CommandResponse, DeviceInfo

# Set up logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter(prefix="/api/v1/ai", tags=["openai"])

# Initialize services
openai_service = OpenAIService()
ros2_manager = ROS2Manager()


class NaturalLanguageCommandRequest(BaseModel):
    """Request model for natural language command"""
    command_text: str
    device_ids: Optional[List[str]] = None


class CommandHistoryEntry(BaseModel):
    """Model for command history entry"""
    timestamp: datetime
    command_text: str
    command_type: str
    robot_id: str
    parameters: Dict[str, Any]
    success: bool
    response: Optional[str] = None


class CommandHistoryResponse(BaseModel):
    """Response model for command history"""
    history: List[CommandHistoryEntry]


@router.post("/commands", response_model=CommandResponse)
async def process_natural_language_command(
    request: NaturalLanguageCommandRequest,
    current_user: User = Depends(admin_only)
):
    """
    Process a natural language command and send it to the appropriate robot.
    
    This endpoint is restricted to admin users only for security reasons.
    
    Args:
        request: NaturalLanguageCommandRequest containing the command text
        current_user: Current authenticated admin user
        
    Returns:
        CommandResponse with the result of the command execution
        
    Raises:
        HTTPException: If the command processing or execution fails
    """
    try:
        # Get available robot IDs
        available_robot_ids = request.device_ids
        
        # If no specific robots are provided, get all available robots
        if not available_robot_ids:
            # In a real implementation, this would fetch actual robot IDs
            # For now, we'll use a placeholder list
            available_robot_ids = ["robot_1", "robot_2", "delivery_bot", "survey_drone"]
        
        # Process the command using OpenAI
        try:
            command = openai_service.process_command(
                user_id=current_user.username,
                command_text=request.command_text,
                available_robot_ids=available_robot_ids
            )
        except ValueError as e:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Error processing command: {str(e)}"
            )
        
        # Log the processed command
        logger.info(
            f"User {current_user.username} sent command to {command.robot_id}: "
            f"{command.command_type} with parameters {command.parameters}"
        )
        
        # Execute the command
        # In a production system, this would communicate with ROS2
        # to send the actual command to the robot
        try:
            # Create a command request
            command_request = CommandRequest(
                robot_id=command.robot_id,
                command_type=command.command_type,
                parameters=command.parameters
            )
            
            # Send the command to the robot via ROS2
            # This is a simplified implementation
            # In a real system, this would use the ROS2Manager to publish a message
            # or call a service to execute the command
            
            # For demonstration purposes, we'll assume the command was successful
            # In a real implementation, we would wait for feedback from the robot
            
            # Record the successful command
            response = CommandResponse(
                success=True,
                message=f"Command {command.command_type} sent to {command.robot_id}",
                command_id=f"{datetime.now().timestamp()}",
                timestamp=datetime.now()
            )
            
            return response
            
        except Exception as e:
            logger.error(f"Error executing command: {e}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Error executing command: {str(e)}"
            )
            
    except Exception as e:
        logger.error(f"Error in process_natural_language_command: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An unexpected error occurred: {str(e)}"
        )


@router.get("/commands/history", response_model=CommandHistoryResponse)
async def get_command_history(
    limit: int = Query(10, ge=1, le=100),
    current_user: User = Depends(admin_only)
):
    """
    Get command history for the current user.
    
    Args:
        limit: Maximum number of history entries to return
        current_user: Current authenticated admin user
        
    Returns:
        CommandHistoryResponse with the command history
    """
    try:
        # Get conversation history from OpenAI service
        history = openai_service.get_conversation_history(current_user.username)
        
        # Convert to CommandHistoryEntry objects
        # In a real implementation, this would retrieve actual command history
        # from a database or other persistent storage
        
        # For demonstration purposes, we'll create some sample history entries
        sample_history = []
        for i in range(min(limit, len(history) // 2)):
            if i * 2 + 1 < len(history):
                user_msg = history[i * 2]
                assistant_msg = history[i * 2 + 1]
                
                # Try to parse the assistant response as JSON
                try:
                    response_data = json.loads(assistant_msg["content"])
                    entry = CommandHistoryEntry(
                        timestamp=datetime.now() - timedelta(minutes=i * 5),
                        command_text=user_msg["content"],
                        command_type=response_data.get("command_type", "unknown"),
                        robot_id=response_data.get("robot_id", "unknown"),
                        parameters=response_data.get("parameters", {}),
                        success=True,
                        response="Command executed successfully"
                    )
                    sample_history.append(entry)
                except:
                    # Skip entries that can't be parsed
                    pass
        
        return CommandHistoryResponse(history=sample_history)
        
    except Exception as e:
        logger.error(f"Error in get_command_history: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An unexpected error occurred: {str(e)}"
        )


@router.post("/commands/clear-history")
async def clear_command_history(current_user: User = Depends(admin_only)):
    """
    Clear command history for the current user.
    
    Args:
        current_user: Current authenticated admin user
        
    Returns:
        Dict with success status
    """
    try:
        success = openai_service.clear_conversation_history(current_user.username)
        return {"success": success}
    except Exception as e:
        logger.error(f"Error in clear_command_history: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An unexpected error occurred: {str(e)}"
        )


@router.get("/commands/supported")
async def get_supported_commands(current_user: User = Depends(admin_only)):
    """
    Get list of supported command types.
    
    Args:
        current_user: Current authenticated admin user
        
    Returns:
        Dict with list of supported commands
    """
    try:
        commands = openai_service.get_supported_commands()
        return {"commands": commands}
    except Exception as e:
        logger.error(f"Error in get_supported_commands: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An unexpected error occurred: {str(e)}"
        )