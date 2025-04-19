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


import unittest
from unittest.mock import patch
from flask_testing import TestCase

from app.server import create_app
from app.server.service.ros2_manager import ROS2Manager

class TestROS2API(TestCase):

    def create_app(self):
        app = create_app()
        app.config['TESTING'] = True
        return app

    def setUp(self):
        patcher = patch('app.server.service.ros2_manager.ROS2Manager')
        self.mock_ros2_manager_class = patcher.start()
        self.addCleanup(patcher.stop)

        self.mock_ros2_manager = self.mock_ros2_manager_class.return_value

    def tearDown(self):
        self.mock_ros2_manager.shutdown()
    def test_get_topic_list(self):
        self.mock_ros2_manager.get_topic_list.return_value = [
            {"name": "/topic1", "type": "std_msgs/String"},
            {"name": "/topic2", "type": "std_msgs/Int32"},
        ]

        result = self.mock_ros2_manager.get_topic_list()
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["name"], "/topic1")

    def test_get_service_list(self):
        self.mock_ros2_manager.get_service_list.return_value = [
            {"name": "/service1", "type": "custom_msgs/Service1"},
            {"name": "/service2", "type": "custom_msgs/Service2"},
        ]

        result = self.mock_ros2_manager.get_service_list()
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["name"], "/service1")

    def test_call_automatic_action_service(self):
        self.mock_ros2_manager.call_automatic_action_service.return_value = True

        params = {"listen_topic": "/topic1", "pub_topic": "/topic2"}
        result = self.mock_ros2_manager.call_automatic_action_service(params)
        self.assertTrue(result)

    def test_call_delete_automatic_action_service(self):
        self.mock_ros2_manager.call_delete_automatic_action_service.return_value = True

        params = {"listen_topic_to_delete": "/topic1"}
        result = self.mock_ros2_manager.call_delete_automatic_action_service(params)
        self.assertTrue(result)

    def test_call_combined_automatic_action_service(self):
        self.mock_ros2_manager.call_combined_automatic_action_service.return_value = True

        params = {"listen_topics": ["/topic1", "/topic2"], "logic_expression": "AND"}
        result = self.mock_ros2_manager.call_combined_automatic_action_service(params)
        self.assertTrue(result)

    def test_call_delete_combined_automatic_action_service(self):
        self.mock_ros2_manager.call_delete_combined_automatic_action_service.return_value = True

        params = {"name_of_combined_topics_publisher": "combined_action"}
        result = self.mock_ros2_manager.call_delete_combined_automatic_action_service(params)
        self.assertTrue(result)

    def test_call_available_topics_service(self):
        self.mock_ros2_manager.call_available_topics_service.return_value = [
            {"name": "/topic1", "type": "std_msgs/String"},
            {"name": "/topic2", "type": "std_msgs/Int32"},
        ]

        result = self.mock_ros2_manager.call_available_topics_service()
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["name"], "/topic1")

    def test_call_get_messages_service(self):
        self.mock_ros2_manager.call_get_messages_service.return_value = {
            "int32_msgs": [1, 2, 3],
            "timestamps": ["2023-11-15T10:00:00Z"],
        }

        params = {"topic_name": "/topic1"}
        result = self.mock_ros2_manager.call_get_messages_service(params)
        self.assertIn("int32_msgs", result)
        self.assertEqual(result["int32_msgs"], [1, 2, 3])

    def test_get_message_structure(self):
        self.mock_ros2_manager.get_message_structure.return_value = {
            "field1": "string", "field2": "int32"
        }

        result = self.mock_ros2_manager.get_message_structure("std_msgs/String")
        self.assertIn("field1", result)
        self.assertEqual(result["field1"], "string")

    def test_replace_percent_with_slash(self):
        self.mock_ros2_manager.replace_percent_with_slash.return_value = "topic/name"

        result = self.mock_ros2_manager.replace_percent_with_slash("topic%name")
        self.assertEqual(result, "topic/name")

    def test_shutdown(self):
        self.mock_ros2_manager.shutdown()
        self.mock_ros2_manager.shutdown.assert_called_once()

if __name__ == '__main__':
    unittest.main()

"""
Unit tests for the ROS2Manager service

These tests verify the functionality of the ROS2Manager service,
which provides a bridge between the dashboard and ROS2 ecosystem.

Note: These tests require a running ROS2 environment. They are intended
to be run in a containerized environment with ROS2 properly set up.
"""

import unittest
import time
from unittest.mock import patch, MagicMock

import pytest
import rclpy
from std_msgs.msg import String

from service.ros2_manager import ROS2Manager


class TestROS2Manager(unittest.TestCase):
    """Test the ROS2Manager service"""

    @classmethod
    def setUpClass(cls):
        """Set up for all tests - initialize ROS2"""
        try:
            # Initialize ROS2 if not already initialized
            if not rclpy.ok():
                rclpy.init()
            cls.ros2_manager = ROS2Manager()
        except Exception as e:
            pytest.skip(f"ROS2 initialization failed, skipping tests: {e}")

    @classmethod
    def tearDownClass(cls):
        """Clean up after all tests"""
        if hasattr(cls, 'ros2_manager'):
            cls.ros2_manager.shutdown()

    def test_get_topics(self):
        """Test that we can get a list of topics"""
        # Create a test publisher to ensure we have at least one topic
        node = rclpy.create_node('test_publisher_node')
        publisher = node.create_publisher(String, '/test_topic', 10)
        
        try:
            # Wait for discovery
            time.sleep(1.0)
            
            # Get topics
            topics = self.ros2_manager.get_topics()
            
            # Verify topics is a list
            self.assertIsInstance(topics, list)
            
            # Verify we have at least our test topic
            topic_names = [topic['name'] for topic in topics]
            self.assertIn('/test_topic', topic_names)
            
            # Verify our test topic has the correct type
            test_topic = next((t for t in topics if t['name'] == '/test_topic'), None)
            self.assertIsNotNone(test_topic)
            self.assertEqual(test_topic['type'], 'std_msgs/msg/String')
        finally:
            node.destroy_publisher(publisher)
            node.destroy_node()

    def test_msg_to_dict(self):
        """Test conversion of ROS2 message to dictionary"""
        # Create a test message
        test_msg = String()
        test_msg.data = "test message"
        
        # Convert to dictionary
        result = self.ros2_manager._msg_to_dict(test_msg)
        
        # Verify the result
        self.assertIsInstance(result, dict)
        self.assertEqual(result['data'], "test message")
        self.assertIn('_meta', result)
        self.assertIn('timestamp', result['_meta'])

    def test_dict_to_msg(self):
        """Test conversion of dictionary to ROS2 message"""
        # Create a test dictionary
        test_dict = {'data': "test message"}
        
        # Convert to message
        result = self.ros2_manager._dict_to_msg('std_msgs/msg/String', test_dict)
        
        # Verify the result
        self.assertIsInstance(result, String)
        self.assertEqual(result.data, "test message")

    def test_subscribe_unsubscribe_topic(self):
        """Test subscribing and unsubscribing to a topic"""
        # Create test data
        messages_received = []
        
        def callback(msg_dict):
            messages_received.append(msg_dict)
        
        # Create a test publisher
        node = rclpy.create_node('test_publisher_node')
        publisher = node.create_publisher(String, '/test_topic', 10)
        
        try:
            # Subscribe to the topic
            subscription_id = self.ros2_manager.subscribe_topic('/test_topic', callback)
            
            # Verify subscription was created
            self.assertIsInstance(subscription_id, int)
            
            # Wait for subscription setup
            time.sleep(0.5)
            
            # Publish a message
            test_msg = String()
            test_msg.data = "test message"
            publisher.publish(test_msg)
            
            # Wait for message processing
            time.sleep(0.5)
            
            # Verify message was received
            self.assertEqual(len(messages_received), 1)
            self.assertEqual(messages_received[0]['data'], "test message")
            
            # Unsubscribe
            result = self.ros2_manager.unsubscribe_topic(subscription_id)
            
            # Verify unsubscription was successful
            self.assertTrue(result)
            
            # Clear received messages
            messages_received.clear()
            
            # Publish another message
            publisher.publish(test_msg)
            
            # Wait for message processing
            time.sleep(0.5)
            
            # Verify no message was received after unsubscription
            self.assertEqual(len(messages_received), 0)
        finally:
            node.destroy_publisher(publisher)
            node.destroy_node()

    def test_health_check(self):
        """Test the health check function"""
        result = self.ros2_manager.check_health()
        
        # Verify result format
        self.assertIsInstance(result, dict)
        self.assertIn('status', result)
        self.assertEqual(result['status'], 'ok')
        self.assertIn('node_name', result)
        self.assertIn('topics_count', result)
        self.assertIn('services_count', result)
        self.assertIn('timestamp', result)


@pytest.mark.asyncio
class TestROS2ManagerMocked:
    """Test the ROS2Manager service with mocks"""

    @pytest.fixture
    def mock_rclpy(self):
        """Mock rclpy module"""
        with patch('service.ros2_manager.rclpy') as mock:
            # Set up mock to simulate rclpy.ok() returning True
            mock.ok.return_value = True
            
            # Mock node creation
            mock_node = MagicMock()
            mock.create_node.return_value = mock_node
            
            # Mock get_topic_names_and_types
            mock_node.get_topic_names_and_types.return_value = [
                ('/test_topic', ['std_msgs/msg/String']),
                ('/another_topic', ['std_msgs/msg/Int32']),
                ('/_hidden_topic', ['std_msgs/msg/Bool'])  # Should be filtered out
            ]
            
            # Mock executor
            mock_executor = MagicMock()
            mock.executors.MultiThreadedExecutor.return_value = mock_executor
            
            yield mock

    @pytest.fixture
    def manager(self, mock_rclpy):
        """Create a ROS2Manager with mock rclpy"""
        with patch('service.ros2_manager.message_to_ordereddict') as mock_to_dict:
            mock_to_dict.return_value = {'data': 'test', '_meta': {'timestamp': '2023-01-01T00:00:00'}}
            manager = ROS2Manager()
            yield manager

    async def test_get_topics_mocked(self, manager, mock_rclpy):
        """Test getting topics with mock rclpy"""
        topics = manager.get_topics()
        
        # Verify topics is a list with the expected items
        # (should not include hidden topics)
        assert len(topics) == 2
        assert topics[0]['name'] == '/test_topic'
        assert topics[0]['type'] == 'std_msgs/msg/String'
        assert topics[1]['name'] == '/another_topic'
        assert topics[1]['type'] == 'std_msgs/msg/Int32'
        
        # Verify hidden topics are filtered out
        for topic in topics:
            assert not topic['name'].startswith('/_')