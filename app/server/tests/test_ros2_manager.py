#!/usr/bin/env python3

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