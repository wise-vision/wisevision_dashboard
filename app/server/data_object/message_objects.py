#!/usr/bin/env python3
#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

"""
Message Objects for WiseVision Dashboard

This module defines the data models for the API endpoints, including
ROS2 topics, services, actions, and API request/response models.
"""

from datetime import datetime
from enum import Enum
from typing import Any, Dict, List, Optional, Union
from pydantic import BaseModel, Field


class ROS2TopicType(BaseModel):
    """ROS2 message type information"""
    name: str
    package: str
    type_name: str


class TopicInfo(BaseModel):
    """Information about a ROS2 topic"""
    name: str
    type: str
    num_publishers: Optional[int] = 0
    num_subscribers: Optional[int] = 0


class TopicMessage(BaseModel):
    """A message from a ROS2 topic"""
    data: Dict[str, Any]
    timestamp: datetime
    topic_name: str
    message_type: str


class TopicData(BaseModel):
    """Topic data response model"""
    topic_name: str
    msg_type: str
    messages: List[Dict[str, Any]]


class ROS2Topic(BaseModel):
    """ROS2 topic information"""
    name: str
    type: str


class ROS2Topics(BaseModel):
    """List of ROS2 topics"""
    topics: List[ROS2Topic]


class ServiceInfo(BaseModel):
    """Information about a ROS2 service"""
    name: str
    type: str


class ROS2Service(BaseModel):
    """ROS2 service information"""
    name: str
    type: str


class ROS2Services(BaseModel):
    """List of ROS2 services"""
    services: List[ROS2Service]


class ActionInfo(BaseModel):
    """Information about a ROS2 action"""
    name: str
    type: str


class ROS2Action(BaseModel):
    """ROS2 action information"""
    name: str
    type: str


class ROS2Actions(BaseModel):
    """List of ROS2 actions"""
    actions: List[ROS2Action]


class HealthStatus(BaseModel):
    """Health status information"""
    status: str
    message: Optional[str] = None
    version: Optional[str] = None
    timestamp: datetime


class TimeRange(BaseModel):
    """Time range for querying historical data"""
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None


class MetricsQuery(BaseModel):
    """Query parameters for metrics data"""
    metric_name: str
    time_range: TimeRange
    aggregation: str = "mean"
    interval: str = "1h"


class MetricDataPoint(BaseModel):
    """A single data point for a metric"""
    timestamp: datetime
    value: float
    tags: Optional[Dict[str, str]] = None


class MetricsResponse(BaseModel):
    """Response model for metrics query"""
    metric: str
    aggregation: str
    interval: str
    data: List[MetricDataPoint]


class CommandRequest(BaseModel):
    """Request model for sending a command to a robot"""
    robot_id: str
    command_type: str
    parameters: Dict[str, Any]


class CommandResponse(BaseModel):
    """Response model for a command request"""
    success: bool
    message: str
    command_id: Optional[str] = None
    timestamp: datetime


class AlarmLevel(str, Enum):
    """Alarm severity levels"""
    CRITICAL = "critical"
    WARNING = "warning"
    INFO = "info"


class AlarmDefinition(BaseModel):
    """Definition of a custom alarm"""
    name: str
    description: str
    level: AlarmLevel
    expression: str  # Logical expression to evaluate
    topic_name: str
    enabled: bool = True


class AlarmInstance(BaseModel):
    """An instance of an alarm that was triggered"""
    alarm_id: str
    name: str
    level: AlarmLevel
    message: str
    topic_name: str
    topic_value: Any
    timestamp: datetime
    acknowledged: bool = False
    acknowledged_by: Optional[str] = None
    acknowledged_at: Optional[datetime] = None


class DeviceType(str, Enum):
    """Types of devices"""
    ROBOT = "robot"
    SENSOR = "sensor"
    LORAWAN = "lorawan"
    DRONE = "drone"
    OTHER = "other"


class DeviceStatus(str, Enum):
    """Device status values"""
    ONLINE = "online"
    OFFLINE = "offline"
    WARNING = "warning"
    ERROR = "error"
    MAINTENANCE = "maintenance"


class DeviceLocation(BaseModel):
    """Location information for a device"""
    latitude: float
    longitude: float
    altitude: Optional[float] = None
    heading: Optional[float] = None
    accuracy: Optional[float] = None


class DeviceInfo(BaseModel):
    """Information about a connected device"""
    id: str
    name: str
    type: DeviceType
    status: DeviceStatus
    location: Optional[DeviceLocation] = None
    last_seen: Optional[datetime] = None
    attributes: Dict[str, Any] = {}


class DeviceCommand(BaseModel):
    """Command to send to a device"""
    device_id: str
    command: str
    parameters: Dict[str, Any] = {}


class DeviceCommandResult(BaseModel):
    """Result of a device command"""
    device_id: str
    command: str
    success: bool
    message: str
    timestamp: datetime
    result: Optional[Dict[str, Any]] = None
