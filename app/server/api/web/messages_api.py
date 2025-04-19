"""
WiseVision Dashboard ROS 2 API Module

This module provides API endpoints for interacting with ROS 2 topics, services,
and actions. It handles real-time data streaming via WebSockets and REST endpoints
for querying historical data from InfluxDB via wisevision_data_black_box.
"""

import json
import logging
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any, Union

from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect, Query, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from ..service.ros2_manager import ROS2Manager
from ..service.data_black_box_client import DataBlackBoxClient
from ..service.auth_service import get_current_user, User, UserRole
from ..models.messages import TopicData, TopicInfo, ServiceInfo, ActionInfo

# Set up logging
logger = logging.getLogger(__name__)

# Create the router
router = APIRouter(prefix="/api/v1")

# Singleton instances for service managers
ros2_manager = ROS2Manager()
black_box_client = DataBlackBoxClient()

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: Dict[str, List[WebSocket]] = {}

    async def connect(self, websocket: WebSocket, topic: str):
        await websocket.accept()
        if topic not in self.active_connections:
            self.active_connections[topic] = []
        self.active_connections[topic].append(websocket)
        logger.info(f"Client connected to topic {topic}")

    def disconnect(self, websocket: WebSocket, topic: str):
        if topic in self.active_connections:
            if websocket in self.active_connections[topic]:
                self.active_connections[topic].remove(websocket)
                logger.info(f"Client disconnected from topic {topic}")
            # Clean up if no more connections for this topic
            if not self.active_connections[topic]:
                del self.active_connections[topic]

    async def broadcast_to_topic(self, topic: str, data: Any):
        if topic in self.active_connections:
            for connection in self.active_connections[topic]:
                try:
                    await connection.send_json(data)
                except Exception as e:
                    logger.error(f"Failed to send data to client: {e}")
                    # We'll handle disconnection separately

manager = ConnectionManager()

# Models for request/response
class TopicSubscriptionRequest(BaseModel):
    topic_name: str
    message_type: Optional[str] = None

class TopicPublishRequest(BaseModel):
    topic_name: str
    message_type: str
    data: Dict[str, Any]

class ServiceCallRequest(BaseModel):
    service_name: str
    service_type: str
    request_data: Dict[str, Any]

class ActionGoalRequest(BaseModel):
    action_name: str
    action_type: str
    goal: Dict[str, Any]

class QueryParams(BaseModel):
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    limit: Optional[int] = Field(default=100, gt=0, le=1000)
    filters: Optional[Dict[str, Any]] = None

# ROS 2 Topic WebSocket endpoint
@router.websocket("/ws/topics/{topic_name}")
async def websocket_topic(websocket: WebSocket, topic_name: str):
    await manager.connect(websocket, topic_name)
    
    try:
        # Subscribe to the ROS 2 topic
        subscription_id = ros2_manager.subscribe_topic(
            topic_name,
            lambda msg: manager.broadcast_to_topic(topic_name, msg)
        )
        
        # Keep the connection open and handle client messages
        while True:
            try:
                # Wait for messages from the client (could be used for filtering)
                data = await websocket.receive_text()
                client_data = json.loads(data)
                
                # Handle client commands like pausing, resuming, etc.
                if client_data.get("command") == "pause":
                    ros2_manager.pause_subscription(subscription_id)
                elif client_data.get("command") == "resume":
                    ros2_manager.resume_subscription(subscription_id)
                # Add more commands as needed
                
            except WebSocketDisconnect:
                manager.disconnect(websocket, topic_name)
                ros2_manager.unsubscribe_topic(subscription_id)
                break
            except Exception as e:
                logger.error(f"WebSocket error: {e}")
                break
    except Exception as e:
        logger.error(f"Failed to handle WebSocket connection: {e}")
        manager.disconnect(websocket, topic_name)

# REST API Endpoints

@router.get("/topics", response_model=List[TopicInfo])
async def get_topics(current_user: User = Depends(get_current_user)):
    """
    Get a list of all available ROS 2 topics.
    """
    try:
        topics = ros2_manager.get_topics()
        return topics
    except Exception as e:
        logger.error(f"Failed to retrieve topics: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to retrieve topics: {str(e)}")

@router.get("/topics/{topic_name}", response_model=TopicData)
async def get_topic_data(
    topic_name: str, 
    limit: int = Query(10, ge=1, le=100),
    current_user: User = Depends(get_current_user)
):
    """
    Get the latest messages from a specific ROS 2 topic.
    """
    try:
        data = ros2_manager.get_topic_data(topic_name, limit)
        return data
    except Exception as e:
        logger.error(f"Failed to retrieve data for topic {topic_name}: {e}")
        raise HTTPException(
            status_code=500, 
            detail=f"Failed to retrieve data for topic {topic_name}: {str(e)}"
        )

@router.post("/topics/publish")
async def publish_to_topic(
    request: TopicPublishRequest,
    current_user: User = Depends(get_current_user)
):
    """
    Publish a message to a ROS 2 topic.
    Only users with Admin or Operator roles can publish messages.
    """
    if current_user.role not in [UserRole.ADMIN, UserRole.OPERATOR]:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Insufficient permissions to publish messages"
        )
    
    try:
        success = ros2_manager.publish_message(
            request.topic_name,
            request.message_type,
            request.data
        )
        
        if success:
            return {"status": "success", "message": f"Message published to {request.topic_name}"}
        else:
            raise HTTPException(status_code=500, detail="Failed to publish message")
    except Exception as e:
        logger.error(f"Failed to publish to topic {request.topic_name}: {e}")
        raise HTTPException(
            status_code=500, 
            detail=f"Failed to publish message: {str(e)}"
        )

@router.get("/services", response_model=List[ServiceInfo])
async def get_services(current_user: User = Depends(get_current_user)):
    """
    Get a list of all available ROS 2 services.
    """
    try:
        services = ros2_manager.get_services()
        return services
    except Exception as e:
        logger.error(f"Failed to retrieve services: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to retrieve services: {str(e)}")

@router.post("/services/call")
async def call_service(
    request: ServiceCallRequest,
    current_user: User = Depends(get_current_user)
):
    """
    Call a ROS 2 service with the provided request data.
    Only users with Admin or Operator roles can call services.
    """
    if current_user.role not in [UserRole.ADMIN, UserRole.OPERATOR]:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Insufficient permissions to call services"
        )
    
    try:
        response = ros2_manager.call_service(
            request.service_name,
            request.service_type,
            request.request_data
        )
        
        return {"status": "success", "response": response}
    except Exception as e:
        logger.error(f"Failed to call service {request.service_name}: {e}")
        raise HTTPException(
            status_code=500, 
            detail=f"Failed to call service: {str(e)}"
        )

@router.get("/actions", response_model=List[ActionInfo])
async def get_actions(current_user: User = Depends(get_current_user)):
    """
    Get a list of all available ROS 2 actions.
    """
    try:
        actions = ros2_manager.get_actions()
        return actions
    except Exception as e:
        logger.error(f"Failed to retrieve actions: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to retrieve actions: {str(e)}")

@router.post("/actions/send_goal")
async def send_action_goal(
    request: ActionGoalRequest,
    current_user: User = Depends(get_current_user)
):
    """
    Send a goal to a ROS 2 action server.
    Only users with Admin or Operator roles can send action goals.
    """
    if current_user.role not in [UserRole.ADMIN, UserRole.OPERATOR]:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Insufficient permissions to send action goals"
        )
    
    try:
        goal_handle = ros2_manager.send_action_goal(
            request.action_name,
            request.action_type,
            request.goal
        )
        
        return {
            "status": "success", 
            "goal_id": goal_handle.goal_id,
            "accepted": goal_handle.accepted
        }
    except Exception as e:
        logger.error(f"Failed to send goal to action {request.action_name}: {e}")
        raise HTTPException(
            status_code=500, 
            detail=f"Failed to send action goal: {str(e)}"
        )

# Historical data endpoints using wisevision_data_black_box

@router.get("/historical/topics/{topic_name}")
async def get_historical_topic_data(
    topic_name: str,
    start_time: Optional[datetime] = None,
    end_time: Optional[datetime] = None,
    limit: int = Query(100, ge=1, le=1000),
    current_user: User = Depends(get_current_user)
):
    """
    Query historical data for a specific ROS 2 topic from the InfluxDB via wisevision_data_black_box.
    """
    # Default to the last 24 hours if no time range is specified
    if not start_time:
        start_time = datetime.now() - timedelta(days=1)
    if not end_time:
        end_time = datetime.now()
    
    try:
        data = black_box_client.query_topic_data(
            topic_name,
            start_time,
            end_time,
            limit
        )
        return {"topic": topic_name, "data": data, "count": len(data)}
    except Exception as e:
        logger.error(f"Failed to retrieve historical data for topic {topic_name}: {e}")
        raise HTTPException(
            status_code=500, 
            detail=f"Failed to retrieve historical data: {str(e)}"
        )

@router.get("/historical/metrics/{metric_name}")
async def get_historical_metrics(
    metric_name: str,
    start_time: Optional[datetime] = None,
    end_time: Optional[datetime] = None,
    aggregation: str = Query("mean", regex="^(mean|max|min|sum|count)$"),
    interval: str = Query("1h", regex="^[0-9]+[smhdw]$"),  # Valid InfluxDB time intervals
    current_user: User = Depends(get_current_user)
):
    """
    Query historical metrics with aggregation from the InfluxDB via wisevision_data_black_box.
    """
    # Default to the last 24 hours if no time range is specified
    if not start_time:
        start_time = datetime.now() - timedelta(days=1)
    if not end_time:
        end_time = datetime.now()
    
    try:
        data = black_box_client.query_metrics(
            metric_name,
            start_time,
            end_time,
            aggregation,
            interval
        )
        return {
            "metric": metric_name,
            "aggregation": aggregation,
            "interval": interval,
            "data": data
        }
    except Exception as e:
        logger.error(f"Failed to retrieve historical metrics for {metric_name}: {e}")
        raise HTTPException(
            status_code=500, 
            detail=f"Failed to retrieve historical metrics: {str(e)}"
        )

# Health check endpoint
@router.get("/health")
async def health_check():
    """
    Health check endpoint to verify the API is running.
    """
    ros2_status = ros2_manager.check_health()
    black_box_status = black_box_client.check_health()
    
    if ros2_status["status"] == "ok" and black_box_status["status"] == "ok":
        return {
            "status": "ok",
            "ros2": ros2_status,
            "data_black_box": black_box_status
        }
    else:
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={
                "status": "error",
                "ros2": ros2_status,
                "data_black_box": black_box_status
            }
        )


