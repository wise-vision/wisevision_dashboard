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


import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.executors import MultiThreadedExecutor
from concurrent.futures import Future
from ..data_object.message_objects import ROS2Topic, ROS2Topics, ROS2Service, ROS2Services
from dateutil import parser  
from rosidl_runtime_py.utilities import get_message, get_service
from rclpy.qos import QoSProfile
from collections import OrderedDict
import array
import numpy as np

def ros_message_to_dict(msg):
    if not hasattr(msg, '__slots__'):
        return msg

    result = {}
    for field_name in msg.__slots__:
        clean_field_name = field_name.lstrip('_')
        value = getattr(msg, field_name)

        if isinstance(value, (list, tuple)):
            result[clean_field_name] = [ros_message_to_dict(v) for v in value]
        elif hasattr(value, '__slots__'):
            result[clean_field_name] = ros_message_to_dict(value)
        else:
            result[clean_field_name] = value
    return result

"""
ROS2 Manager Service

This service provides a bridge between the dashboard and the ROS2 ecosystem.
It handles:
- Discovery of topics, services, and actions
- Subscribing to topics and converting messages to JSON
- Publishing messages to topics
- Calling services
- Sending goals to action servers
- Monitoring ROS2 node health

All interactions with the ROS2 system should go through this manager.
"""

import json
import logging
import threading
import time
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional, Tuple, Union, Set

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.task import Future
from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service, get_action

# Set up logging
logger = logging.getLogger(__name__)

class ROS2Manager:
    """
    Manager class for ROS2 interactions, designed as a singleton.
    Provides methods for discovering and interacting with ROS2 topics,
    services, and actions.
    """
    _instance = None
    _initialized = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ROS2Manager, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        # Initialize ROS2
        self._initialize_ros2()
        
        # Track subscriptions, publishers, etc.
        self._subscriptions = {}  # topic_name -> (subscription, callback_id)
        self._publishers = {}  # topic_name -> publisher
        self._service_clients = {}  # service_name -> client
        self._action_clients = {}  # action_name -> client
        self._subscription_callbacks = {}  # callback_id -> (user_callback, is_active)
        self._next_callback_id = 0
        
        # Topic cache for faster lookups
        self._topics_cache = None
        self._topics_cache_timestamp = None
        self._topic_types_cache = {}  # topic_name -> msg_type_str
        
        # Cache of message and service types (to avoid repeated imports)
        self._msg_type_cache = {}  # msg_type_name -> msg_class
        self._srv_type_cache = {}  # srv_type_name -> srv_class
        self._action_type_cache = {}  # action_type_name -> action_class
        
        # Thread safety
        self._lock = threading.RLock()
        
        self._initialized = True
        
        logger.info("ROS2Manager initialized successfully")

    def _initialize_ros2(self):
        """Initialize ROS2 node and executor"""
        try:
            # Initialize ROS2 if it hasn't been initialized already
            if not rclpy.ok():
                rclpy.init()
            
            # Create a node with a ReentrantCallbackGroup to allow callbacks from multiple threads
            self._node = rclpy.create_node(
                'wisevision_dashboard_bridge',
                namespace='',
                allow_undeclared_parameters=True,
                automatically_declare_parameters_from_overrides=True,
                parameter_overrides=[
                    Parameter('use_sim_time', Parameter.Type.BOOL, False)
                ],
                callback_group=ReentrantCallbackGroup()
            )
            
            # Create a multithreaded executor for handling callbacks
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            
            # Start a thread to spin the executor
            self._executor_thread = threading.Thread(
                target=self._spin_executor, 
                daemon=True
            )
            self._executor_thread.start()
            
            logger.info("ROS2 node and executor initialized")
        except Exception as e:
            logger.error(f"Failed to initialize ROS2: {e}")
            raise

    def _spin_executor(self):
        """Thread function to spin the ROS2 executor"""
        try:
            while rclpy.ok():
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            logger.error(f"Error in executor thread: {e}")
        finally:
            logger.info("Executor thread stopping")

    def _get_msg_type(self, msg_type_name: str):
        """Get a message class from its type name, with caching"""
        if msg_type_name not in self._msg_type_cache:
            try:
                self._msg_type_cache[msg_type_name] = get_message(msg_type_name)
            except (AttributeError, ModuleNotFoundError) as e:
                logger.error(f"Failed to get message type {msg_type_name}: {e}")
                raise ValueError(f"Unknown message type: {msg_type_name}")
        return self._msg_type_cache[msg_type_name]

    def _get_srv_type(self, srv_type_name: str):
        """Get a service class from its type name, with caching"""
        if srv_type_name not in self._srv_type_cache:
            try:
                self._srv_type_cache[srv_type_name] = get_service(srv_type_name)
            except (AttributeError, ModuleNotFoundError) as e:
                logger.error(f"Failed to get service type {srv_type_name}: {e}")
                raise ValueError(f"Unknown service type: {srv_type_name}")
        return self._srv_type_cache[srv_type_name]

    def _get_action_type(self, action_type_name: str):
        """Get an action class from its type name, with caching"""
        if action_type_name not in self._action_type_cache:
            try:
                self._action_type_cache[action_type_name] = get_action(action_type_name)
            except (AttributeError, ModuleNotFoundError) as e:
                logger.error(f"Failed to get action type {action_type_name}: {e}")
                raise ValueError(f"Unknown action type: {action_type_name}")
        return self._action_type_cache[action_type_name]

    def get_topics(self) -> List[Dict[str, str]]:
        """Get a list of all available ROS2 topics with their types"""
        # Use cached topics if they're fresh (less than 5 seconds old)
        current_time = time.time()
        if self._topics_cache is not None and current_time - self._topics_cache_timestamp < 5.0:
            return self._topics_cache
        
        topic_names_and_types = self._node.get_topic_names_and_types()
        topics = []
        
        for topic_name, type_list in topic_names_and_types:
            # Skip hidden topics
            if topic_name.startswith('/_'):
                continue
            
            for topic_type in type_list:
                topics.append({
                    'name': topic_name,
                    'type': topic_type
                })
                # Update the topic types cache
                self._topic_types_cache[topic_name] = topic_type
        
        # Cache the results
        self._topics_cache = topics
        self._topics_cache_timestamp = current_time
        
        return topics

    def get_topic_type(self, topic_name: str) -> str:
        """Get the message type of a topic"""
        # Try the cache first
        if topic_name in self._topic_types_cache:
            return self._topic_types_cache[topic_name]
        
        # If not in cache, refresh the topic list
        topics = self.get_topics()
        for topic in topics:
            if topic['name'] == topic_name:
                return topic['type']
        
        raise ValueError(f"Topic {topic_name} not found")

    def _msg_to_dict(self, msg) -> Dict[str, Any]:
        """Convert a ROS2 message to a dictionary, handling timestamps appropriately"""
        try:
            # Convert the message to a dictionary
            result = message_to_ordereddict(msg)
            
            # Add metadata
            result['_meta'] = {
                'timestamp': datetime.now().isoformat()
            }
            
            return result
        except Exception as e:
            logger.error(f"Error converting message to dictionary: {e}")
            return {'error': str(e)}

    def _dict_to_msg(self, msg_type_name: str, data: Dict[str, Any]):
        """Convert a dictionary to a ROS2 message"""
        try:
            # Get the message class
            msg_class = self._get_msg_type(msg_type_name)
            
            # Create an instance of the message
            msg = msg_class()
            
            # Remove metadata if present
            data_copy = data.copy()
            if '_meta' in data_copy:
                del data_copy['_meta']
            
            # Set the message fields
            set_message_fields(msg, data_copy)
            
            return msg
        except Exception as e:
            logger.error(f"Error converting dictionary to message: {e}")
            raise

    def subscribe_topic(self, topic_name: str, callback: Callable[[Dict[str, Any]], None]) -> int:
        """
        Subscribe to a ROS2 topic and call the callback when messages are received
        
        Args:
            topic_name: Name of the topic to subscribe to
            callback: Function to call with the received message as a dictionary
            
        Returns:
            Subscription ID (used for unsubscribing)
        """
        with self._lock:
            # Get the message type
            msg_type_name = self.get_topic_type(topic_name)
            msg_class = self._get_msg_type(msg_type_name)
            
            # Define QoS profile - default to best effort for faster delivery
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            # Assign a callback ID
            callback_id = self._next_callback_id
            self._next_callback_id += 1
            
            # Store the user callback
            self._subscription_callbacks[callback_id] = (callback, True)  # (callback, is_active)
            
            # Define the subscription callback
            def subscription_callback(msg):
                # If the subscription is not active, ignore the message
                if not self._subscription_callbacks.get(callback_id, (None, False))[1]:
                    return
                
                # Convert the message to a dictionary
                msg_dict = self._msg_to_dict(msg)
                
                # Add topic and type information
                msg_dict['_meta']['topic'] = topic_name
                msg_dict['_meta']['type'] = msg_type_name
                
                # Call the user callback
                try:
                    callback(msg_dict)
                except Exception as e:
                    logger.error(f"Error in topic callback for {topic_name}: {e}")
            
            # Create a subscription if one doesn't exist
            if topic_name not in self._subscriptions:
                subscription = self._node.create_subscription(
                    msg_class,
                    topic_name,
                    subscription_callback,
                    qos
                )
                self._subscriptions[topic_name] = (subscription, {callback_id})
            else:
                # Add the callback to the existing subscription
                subscription, callbacks = self._subscriptions[topic_name]
                callbacks.add(callback_id)
                self._subscriptions[topic_name] = (subscription, callbacks)
            
            logger.info(f"Subscribed to topic {topic_name} with ID {callback_id}")
            return callback_id

    def unsubscribe_topic(self, subscription_id: int) -> bool:
        """
        Unsubscribe from a topic callback
        
        Args:
            subscription_id: ID returned from subscribe_topic
            
        Returns:
            True if successful, False otherwise
        """
        with self._lock:
            if subscription_id not in self._subscription_callbacks:
                logger.warning(f"Subscription ID {subscription_id} not found")
                return False
            
            # Remove the callback
            del self._subscription_callbacks[subscription_id]
            
            # Find and update the subscription
            for topic_name, (subscription, callbacks) in list(self._subscriptions.items()):
                if subscription_id in callbacks:
                    callbacks.remove(subscription_id)
                    
                    # If no more callbacks, destroy the subscription
                    if not callbacks:
                        self._node.destroy_subscription(subscription)
                        del self._subscriptions[topic_name]
                    else:
                        self._subscriptions[topic_name] = (subscription, callbacks)
                    
                    logger.info(f"Unsubscribed from topic {topic_name} with ID {subscription_id}")
                    return True
            
            logger.warning(f"Subscription ID {subscription_id} not found in active subscriptions")
            return False

    def pause_subscription(self, subscription_id: int) -> bool:
        """
        Pause a topic subscription (messages will be received but callback won't be called)
        
        Args:
            subscription_id: ID returned from subscribe_topic
            
        Returns:
            True if successful, False otherwise
        """
        with self._lock:
            if subscription_id not in self._subscription_callbacks:
                return False
            
            callback, _ = self._subscription_callbacks[subscription_id]
            self._subscription_callbacks[subscription_id] = (callback, False)
            return True

    def resume_subscription(self, subscription_id: int) -> bool:
        """
        Resume a paused topic subscription
        
        Args:
            subscription_id: ID returned from subscribe_topic
            
        Returns:
            True if successful, False otherwise
        """
        with self._lock:
            if subscription_id not in self._subscription_callbacks:
                return False
            
            callback, _ = self._subscription_callbacks[subscription_id]
            self._subscription_callbacks[subscription_id] = (callback, True)
            return True

    def get_topic_data(self, topic_name: str, limit: int = 1) -> Dict[str, Any]:
        """
        Get the latest messages from a topic (blocks until messages are received)
        
        Args:
            topic_name: Name of the topic
            limit: Maximum number of messages to receive (default: 1)
            
        Returns:
            Dictionary with topic information and received messages
        """
        messages = []
        received_count = 0
        
        # Create a future to wait for messages
        future = Future()
        
        # Define the callback
        def callback(msg_dict):
            nonlocal received_count
            messages.append(msg_dict)
            received_count += 1
            if received_count >= limit:
                future.set_result(True)
        
        # Subscribe to the topic
        subscription_id = self.subscribe_topic(topic_name, callback)
        
        try:
            # Wait for the future with a timeout
            rclpy.spin_until_future_complete(
                self._node, 
                future, 
                timeout_sec=5.0
            )
        except Exception as e:
            logger.error(f"Error waiting for topic data: {e}")
        finally:
            # Clean up the subscription
            self.unsubscribe_topic(subscription_id)
        
        # Get the message type
        try:
            msg_type = self.get_topic_type(topic_name)
        except ValueError:
            msg_type = "Unknown"
        
        return {
            "topic_name": topic_name,
            "msg_type": msg_type,
            "messages": messages
        }

    def publish_message(self, topic_name: str, msg_type_name: str, data: Dict[str, Any]) -> bool:
        """
        Publish a message to a ROS2 topic
        
        Args:
            topic_name: Name of the topic to publish to
            msg_type_name: Type of the message (e.g. 'std_msgs/String')
            data: Dictionary containing the message data
            
        Returns:
            True if successful, False otherwise
        """
        with self._lock:
            # Create a publisher if one doesn't exist
            if topic_name not in self._publishers:
                msg_class = self._get_msg_type(msg_type_name)
                
                qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=10
                )
                
                publisher = self._node.create_publisher(
                    msg_class,
                    topic_name,
                    qos
                )
                self._publishers[topic_name] = (publisher, msg_type_name)
            else:
                publisher, existing_type = self._publishers[topic_name]
                if existing_type != msg_type_name:
                    logger.error(
                        f"Type mismatch for topic {topic_name}: "
                        f"expected {existing_type}, got {msg_type_name}"
                    )
                    return False
            
            try:
                # Convert the dictionary to a ROS2 message
                msg = self._dict_to_msg(msg_type_name, data)
                
                # Publish the message
                publisher.publish(msg)
                return True
            except Exception as e:
                logger.error(f"Error publishing message to {topic_name}: {e}")
                return False

    def get_services(self) -> List[Dict[str, str]]:
        """Get a list of all available ROS2 services with their types"""
        service_names_and_types = self._node.get_service_names_and_types()
        services = []
        
        for service_name, type_list in service_names_and_types:
            # Skip hidden services
            if service_name.startswith('/_'):
                continue
            
            for service_type in type_list:
                services.append({
                    'name': service_name,
                    'type': service_type
                })
        
        return services

    def call_service(
        self, 
        service_name: str, 
        service_type_name: str, 
        request_data: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Call a ROS2 service
        
        Args:
            service_name: Name of the service to call
            service_type_name: Type of the service (e.g. 'example_interfaces/srv/AddTwoInts')
            request_data: Dictionary containing the request data
            
        Returns:
            Dictionary containing the response data
        """
        with self._lock:
            try:
                # Create a client if one doesn't exist
                if service_name not in self._service_clients:
                    service_class = self._get_srv_type(service_type_name)
                    client = self._node.create_client(service_class, service_name)
                    self._service_clients[service_name] = client
                else:
                    client = self._service_clients[service_name]
                
                # Wait for the service to be available
                if not client.wait_for_service(timeout_sec=5.0):
                    raise TimeoutError(f"Service {service_name} not available")
                
                # Create the request
                request = client.srv_type.Request()
                set_message_fields(request, request_data)
                
                # Call the service
                future = client.call_async(request)
                
                # Wait for the response
                rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
                
                if future.done():
                    response = future.result()
                    if response is not None:
                        # Convert the response to a dictionary
                        return message_to_ordereddict(response)
                    else:
                        raise RuntimeError("Service call failed")
                else:
                    raise TimeoutError("Service call timed out")
            except Exception as e:
                logger.error(f"Error calling service {service_name}: {e}")
                raise

    def get_actions(self) -> List[Dict[str, str]]:
        """Get a list of all available ROS2 actions with their types"""
        try:
            from rclpy.action import get_action_names_and_types
            
            action_names_and_types = get_action_names_and_types(self._node)
            actions = []
            
            for action_name, type_list in action_names_and_types:
                # Skip hidden actions
                if action_name.startswith('/_'):
                    continue
                
                for action_type in type_list:
                    actions.append({
                        'name': action_name,
                        'type': action_type
                    })
            
            return actions
        except Exception as e:
            logger.error(f"Error getting actions: {e}")
            return []

    def send_action_goal(
        self,
        action_name: str,
        action_type_name: str,
        goal_data: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Send a goal to a ROS2 action server
        
        Args:
            action_name: Name of the action
            action_type_name: Type of the action (e.g. 'example_interfaces/action/Fibonacci')
            goal_data: Dictionary containing the goal data
            
        Returns:
            Dictionary containing information about the goal
        """
        with self._lock:
            try:
                from rclpy.action import ActionClient
                
                # Create a client if one doesn't exist
                if action_name not in self._action_clients:
                    action_class = self._get_action_type(action_type_name)
                    client = ActionClient(self._node, action_class, action_name)
                    self._action_clients[action_name] = client
                else:
                    client = self._action_clients[action_name]
                
                # Wait for the action server to be available
                if not client.wait_for_server(timeout_sec=5.0):
                    raise TimeoutError(f"Action server {action_name} not available")
                
                # Create the goal
                goal_msg = client._action_type.Goal()
                set_message_fields(goal_msg, goal_data)
                
                # Send the goal
                send_goal_future = client.send_goal_async(goal_msg)
                
                # Wait for the goal to be accepted
                rclpy.spin_until_future_complete(self._node, send_goal_future, timeout_sec=5.0)
                
                if send_goal_future.done():
                    goal_handle = send_goal_future.result()
                    
                    return {
                        "goal_id": str(goal_handle.goal_id),
                        "accepted": goal_handle.accepted
                    }
                else:
                    raise TimeoutError("Action goal send timed out")
            except Exception as e:
                logger.error(f"Error sending action goal to {action_name}: {e}")
                raise

    def check_health(self) -> Dict[str, Any]:
        """
        Check the health of the ROS2 node
        
        Returns:
            Dictionary with health status information
        """
        try:
            # Check if ROS2 is initialized
            if not rclpy.ok():
                return {
                    "status": "error",
                    "message": "ROS2 is not initialized"
                }
            
            # Check if the node is alive
            if not self._node.context.ok():
                return {
                    "status": "error",
                    "message": "ROS2 node is not alive"
                }
            
            # Get some basic stats
            topics_count = len(self.get_topics())
            services_count = len(self.get_services())
            actions_count = len(self.get_actions())
            
            return {
                "status": "ok",
                "node_name": self._node.get_name(),
                "topics_count": topics_count,
                "services_count": services_count,
                "actions_count": actions_count,
                "timestamp": datetime.now().isoformat()
            }
        except Exception as e:
            logger.error(f"Health check failed: {e}")
            return {
                "status": "error",
                "message": str(e),
                "timestamp": datetime.now().isoformat()
            }

    def shutdown(self):
        """Shutdown the ROS2 node and clean up resources"""
        logger.info("Shutting down ROS2Manager")
        
        with self._lock:
            # Clean up subscriptions
            for topic_name, (subscription, _) in self._subscriptions.items():
                self._node.destroy_subscription(subscription)
            self._subscriptions.clear()
            
            # Clean up publishers
            for topic_name, (publisher, _) in self._publishers.items():
                self._node.destroy_publisher(publisher)
            self._publishers.clear()
            
            # Clean up service clients
            self._service_clients.clear()
            
            # Clean up action clients
            self._action_clients.clear()
            
            # Shutdown ROS2 (only if we're the only node)
            if hasattr(self, '_node') and self._node is not None:
                self._node.destroy_node()
                self._node = None

ros2_manager = ROS2Manager()
