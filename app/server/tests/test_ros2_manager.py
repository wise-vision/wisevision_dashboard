#!/usr/bin/env python3

import unittest
from flask_testing import TestCase
import sys
import os

# Ensure the root of your project is in the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))

from app.server import create_app  # Import the create_app function
from app.server.service.ros2_manager import ROS2Manager

class TestROS2API(TestCase):

    def create_app(self):
        app.config['TESTING'] = True
        return app

    def setUp(self):
        # Assuming you have a way to mock or initialize your ROS2Manager within the app
        self.ros2_manager = ROS2Manager()

    def tearDown(self):
        # Clean up / shut down your ROS2 Manager if necessary
        self.ros2_manager.shutdown()

    def test_get_topics(self):
        response = self.client.get('/api/topics')
        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data.decode())
        self.assertIsInstance(data, list)
        self.assertGreater(len(data), 0)  # Assuming there are topics available

    def test_get_services(self):
        response = self.client.get('/api/services')
        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data.decode())
        self.assertIsInstance(data, list)
        self.assertGreater(len(data), 0)  # Assuming there are services available

    def test_get_topic_data(self):
        # First, you would need to mock or simulate topic publishing
        topic_name = '/topic'
        topic_type = 'std_msgs/msg/String'
        # Mock data publishing
        # your_mock_publish_function(topic_name, topic_type, 'Hello World')
        
        response = self.client.get(f'/api/topic_echo/{topic_name}?type={topic_type}')
        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data.decode())
        self.assertEqual(data['message'], 'Hello World')

    def test_create_automatic_action(self):
        post_data = {
            "listen_topic": "/topic_input",
            "listen_message_type": "std_msgs/msg/Int32",
            "value": "data",
            "trigger_val": "50.0",
            "trigger_type": "LessThan",
            "pub_topic": "/topic_output",
            "pub_message_type": "std_msgs/msg/String",
            "trigger_text": "test",
            "data_validity_ms": 5000
        }
        response = self.client.post('/api/create_automatic_action', data=json.dumps(post_data), content_type='application/json')
        self.assertEqual(response.status_code, 200)
        result = json.loads(response.data.decode())
        self.assertTrue(result['success'])

if __name__ == '__main__':
    unittest.main()
