import sys
from pathlib import Path
import unittest
from unittest.mock import patch, MagicMock

project_root = Path(__file__).resolve().parents[3]
sys.path.append(str(project_root))
from app.server import create_app
from app.server.service.ros2_manager import ROS2Manager

class TestMessagesAPI(unittest.TestCase):
    def setUp(self):
        self.app = create_app()
        self.client = self.app.test_client()

        patcher = patch('app.server.api.web.messages_api.ros2_manager')
        self.mock_ros2_manager = patcher.start()
        self.addCleanup(patcher.stop)

    def test_list_topics(self):
        mocked_topics = MagicMock()
        mocked_topics.get_topics.return_value = [
            type('MockTopic', (), {'name': '/parameter_events', 'type': 'rcl_interfaces/msg/ParameterEvent'}),
            type('MockTopic', (), {'name': '/rosout', 'type': 'rcl_interfaces/msg/Log'})
        ]
        self.mock_ros2_manager.get_topic_list.return_value = mocked_topics

        response = self.client.get('/api/topics')

        self.assertEqual(response.status_code, 200)

        data = response.get_json()
        self.assertEqual(len(data), 2)
        self.assertEqual(data[0]['name'], '/parameter_events')
        self.assertEqual(data[1]['type'], 'rcl_interfaces/msg/Log')
    
    def test_list_services(self):
        mocked_services = MagicMock()
        mocked_services.get_services.return_value = [
            type('MockService', (), {'name': '/add_two_ints', 'type': 'example_interfaces/srv/AddTwoInts'}),
            type('MockService', (), {'name': '/get_parameter_types', 'type': 'rcl_interfaces/srv/ListParameterTypes'})
        ]
        self.mock_ros2_manager.get_service_list.return_value = mocked_services

        response = self.client.get('/api/services')

        self.assertEqual(response.status_code, 200)

        data = response.get_json()
        self.assertEqual(len(data), 2)
        self.assertEqual(data[0]['name'], '/add_two_ints')
        self.assertEqual(data[1]['type'], 'rcl_interfaces/srv/ListParameterTypes')

    ### Test for Automatic Action ###
    def test_create_automatic_action(self):
        self.mock_ros2_manager.call_automatic_action_service.return_value = True

        response = self.client.post('/api/create_automatic_action', json={
            "listen_topic": "topic_input",
            "listen_message_type": "std_msgs/msg/Int32",
            "value": "data",
            "trigger_val": "50.0",
            "trigger_type": "LessThan",
            "action_and_publisher_name": "topic_output",
            "pub_message_type": "std_msgs/msg/String",
            "trigger_text": "test",
            "data_validity_ms": 5000
        })
        self.assertEqual(response.status_code, 200)
        data = response.get_json()
        self.assertTrue(data['success'])

    def test_creaete_automatic_action_invalid_listen_topic(self):
        response = self.client.post('/api/create_automatic_action', json={
            "listen_topic": "123",
            "listen_message_type": "std_msgs/msg/Int32",
            "value": "data",
            "trigger_val": "50.0",
            "trigger_type": "LessThan",
            "action_and_publisher_name": "topic_output",
            "pub_message_type": "std_msgs/msg/String",
            "trigger_text": "test",
            "data_validity_ms": 5000
        })
        self.assertEqual(response.status_code, 400)
    
    def test_creaete_automatic_action_invalid_listen_msg_type(self):
        response = self.client.post('/api/create_automatic_action', json={
            "listen_topic": "listen_topic",
            "listen_message_type": "std_msgs//Int32",
            "value": "data",
            "trigger_val": "50.0",
            "trigger_type": "LessThan",
            "action_and_publisher_name": "topic_output",
            "pub_message_type": "std_msgs/msg/String",
            "trigger_text": "test",
            "data_validity_ms": 5000
        })
        self.assertEqual(response.status_code, 400)
    
    def test_delete_automatic_action(self):
        self.mock_ros2_manager.call_delete_automatic_action_service.return_value = True

        response = self.client.post('/api/delete_automatic_action', json={
            "listen_topic_to_delete": "topic_input"
        })
        self.assertEqual(response.status_code, 200)
        data = response.get_json()
        self.assertTrue(data['success'])
    
    def test_delete_automatic_action_invalid_listen_topic(self):
        response = self.client.post('/api/delete_automatic_action', json={
            "listen_topic_to_delete": "123"
        })
        self.assertEqual(response.status_code, 400)

    def test_change_automatic_action(self):
        self.mock_ros2_manager.call_change_automatic_action_service.return_value = True

        response = self.client.post('/api/change_automatic_action', json={
            "action_and_publisher_name_to_change": "topic_output",
            "listen_topic": "topic_input",
            "listen_message_type": "std_msgs/msg/Int32",
            "value": "data",
            "trigger_val": "50.0",
            "trigger_type": "LessThan",
            "new_action_and_publisher_name": "example_topic_2",
            "pub_message_type": "std_msgs/msg/String",
            "trigger_text": "test",
            "data_validity_ms": 5000,
            "pulication_method": 2
        })
        print(response.get_json())
        self.assertEqual(response.status_code, 200)
        data = response.get_json()
        self.assertTrue(data['success'])
    
    def test_change_automatic_action_invalid_action_and_publisher_name_to_change(self):
        response = self.client.post('/api/change_automatic_action', json={
            "action_and_publisher_name_to_change": "123",
            "listen_topic": "topic_input",
            "listen_message_type": "std_msgs/msg/Int32",
            "value": "data",
            "trigger_val": "50.0",
            "trigger_type": "LessThan",
            "new_action_and_publisher_name": "example_topic_2",
            "pub_message_type": "std_msgs/msg/String",
            "trigger_text": "test",
            "data_validity_ms": 5000,
            "pulication_method": 2
        })
        self.assertEqual(response.status_code, 400)

    def test_change_automatic_action_invalid_listen_msg_type(self):
        response = self.client.post('/api/change_automatic_action', json={
            "action_and_publisher_name_to_change": "topic_output",
            "listen_topic": "topic_input",
            "listen_message_type": "std_msgs//Int32",
            "value": "data",
            "trigger_val": "50.0",
            "trigger_type": "LessThan",
            "new_action_and_publisher_name": "example_topic_2",
            "pub_message_type": "std_msgs/msg/String",
            "trigger_text": "test",
            "data_validity_ms": 5000,
            "pulication_method": 2
        })
        self.assertEqual(response.status_code, 400)

    def test_change_automatic_action_invalid_new_action_and_publisher_name(self):
        response = self.client.post('/api/change_automatic_action', json={
            "action_and_publisher_name_to_change": "topic_output",
            "listen_topic": "topic_input",
            "listen_message_type": "std_msgs/msg/Int32",
            "value": "data",
            "trigger_val": "50.0",
            "trigger_type": "LessThan",
            "new_action_and_publisher_name": "123",
            "pub_message_type": "std_msgs/msg/String",
            "trigger_text": "test",
            "data_validity_ms": 5000,
            "pulication_method": 2
        })
        self.assertEqual(response.status_code, 400)

    def test_change_automatic_action_invalid_oub_msg_type(self):
        response = self.client.post('/api/change_automatic_action', json={
            "action_and_publisher_name_to_change": "topic_output",
            "listen_topic": "topic_input",
            "listen_message_type": "std_msgs/msg/Int32",
            "value": "data",
            "trigger_val": "50.0",
            "trigger_type": "LessThan",
            "new_action_and_publisher_name": "example_topic_2",
            "pub_message_type": "std_msgs//String",
            "trigger_text": "test",
            "data_validity_ms": 5000,
            "pulication_method": 2
        })
        self.assertEqual(response.status_code, 400)

    def test_available_topics(self):
        self.mock_ros2_manager.call_available_topics_service.return_value = [{'name': 'topic1', 'type': 'std_msgs/String'}]

        response = self.client.get('/api/available_topics')
        self.assertEqual(response.status_code, 200)
        data = response.get_json()
        self.assertIn('available_topics_with_parameters_and_time', data)
        self.assertEqual(len(data['available_topics_with_parameters_and_time']), 1)

    ### Test for Combined Automatic Action ###
    def test_create_combined_automatic_action(self):
        self.mock_ros2_manager.call_combined_automatic_action_service.return_value = True

        response = self.client.post('/api/create_combined_automatic_action', json={
            "listen_topics": ["topic1", "topic2", "topic3"],
            "logic_expression": "topic1 and topic2 or topic3",
            "action_and_publisher_name": "combined_action",
            "trigger_text": "example_text",
            "publication_method": 2
        })
        self.assertEqual(response.status_code, 200)
        
        data = response.get_json()
        self.assertTrue(data['success'])

    def test_create_combined_automatic_action_invalid_listen_topics(self):
        response = self.client.post('/api/create_combined_automatic_action', json={
            "listen_topics": ["/topic1", "/topic2", "123"],
            "logic_expression": "topic1 and topic2 or topic3",
            "action_and_publisher_name": "combined_action",
            "trigger_text": "example_text",
            "publication_method": 2
        })
        self.assertEqual(response.status_code, 400)

    def test_create_combined_automatic_action_invalid_logic_expression_without_operators(self):
        response = self.client.post('/api/create_combined_automatic_action', json={
            "listen_topics": ["/topic1", "/topic2", "/topic3"],
            "logic_expression": "topic1 topic2",
            "action_and_publisher_name": "combined_action",
            "trigger_text": "example_text",
            "publication_method": 2
        })
        self.assertEqual(response.status_code, 400)
    
    def test_create_combined_automatic_action_invalid_logic_expression_with_wrong_topics(self):
        response = self.client.post('/api/create_combined_automatic_action', json={
            "listen_topics": ["/topic1", "/topic2", "/topic3"],
            "logic_expression": "topic1 and topic4",
            "action_and_publisher_name": "combined_action",
            "trigger_text": "example_text",
            "publication_method": 2
        })
        self.assertEqual(response.status_code, 400)
    
    def test_create_combined_automatic_action_invalid_action_and_publisher_name(self):
        response = self.client.post('/api/create_combined_automatic_action', json={
            "listen_topics": ["/topic1", "/topic2", "/topic3"],
            "logic_expression": "topic1 and topic2 or topic3",
            "action_and_publisher_name": "123",
            "trigger_text": "example_text",
            "publication_method": 2
        })
        self.assertEqual(response.status_code, 400)
    
    def test_delete_combined_automatic_action(self):
        self.mock_ros2_manager.call_delete_combined_automatic_action_service.return_value = True

        response = self.client.post('/api/delete_combined_automatic_action', json={
            "name_of_combined_topics_publisher": "combined_publisher_1"
        })
        print(response.get_json())
        self.assertEqual(response.status_code, 200)
        data = response.get_json()
        self.assertTrue(data['success'])
    
    def test_delete_combined_automatic_action_invalid_name_of_combined_topics_publisher(self):
        response = self.client.post('/api/delete_combined_automatic_action', json={
            "name_of_combined_topics_publisher": "123"
        })
        self.assertEqual(response.status_code, 400)
    
    def test_change_combined_automatic_action(self):
        self.mock_ros2_manager.call_change_automatic_action_combined_service.return_value = True

        response = self.client.post('/api/change_automatic_action_combined', json={
            "action_and_publisher_name_to_change": "combined_publisher_1",
            "listen_topics": ["topic1", "topic2", "topic3"],
            "logic_expression": "topic1 and topic2 or topic3",
            "new_action_and_publisher_name": "combined_publisher_2",
            "trigger_text": "example_text",
            "publication_method": 2
        })
        self.assertEqual(response.status_code, 200)
        data = response.get_json()
        self.assertTrue(data['success'])

    def test_change_combined_automatic_action_invalid_action_and_publisher_name_to_change(self):
        response = self.client.post('/api/change_automatic_action_combined', json={
            "action_and_publisher_name_to_change": "123",
            "listen_topics": ["topic1", "topic2", "topic3"],
            "logic_expression": "topic1 and topic2 or topic3",
            "new_action_and_publisher_name": "combined_publisher_2",
            "trigger_text": "example_text",
            "publication_method": 2
        })
        self.assertEqual(response.status_code, 400)
    
    def test_change_combined_automatic_action_invalid_listen_topics(self):
        response = self.client.post('/api/change_automatic_action_combined', json={
            "action_and_publisher_name_to_change": "combined_publisher_1",
            "listen_topics": ["topic1", "topic2", "123"],
            "logic_expression": "topic1 and topic2 or topic3",
            "new_action_and_publisher_name": "combined_publisher_2",
            "trigger_text": "example_text",
            "publication_method": 2
        })
        self.assertEqual(response.status_code, 400)

    def test_change_combined_automatic_action_invalid_logic_expression_without_operators(self):
        response = self.client.post('/api/change_automatic_action_combined', json={
            "action_and_publisher_name_to_change": "combined_publisher_1",
            "listen_topics": ["topic1", "topic2", "topic3"],
            "logic_expression": "topic1 topic2",
            "new_action_and_publisher_name": "combined_publisher_2",
            "trigger_text": "example_text",
            "publication_method": 2
        })
        self.assertEqual(response.status_code, 400)

    def test_available_combined_topics(self):
        self.mock_ros2_manager.call_available_topics_combined_service.return_value = [
            {
                "action_and_publisher_name": "example_topic_test",
                "date_and_time": {
                    "day": 0,
                    "hour": 0,
                    "minute": 0,
                    "month": 0,
                    "nanosecond": 0,
                    "second": 0,
                    "year": 0
                },
                "listen_topics": ["/example_topic_1", "/example_topic_2"],
                "logic_expression": "/example_topic_1 and /example_topic_2",
                "publication_method": 2,
                "trigger_text": "Updated action triggered!"
            }
        ]
        response = self.client.get('/api/available_topics_combined')

        self.assertEqual(response.status_code, 200)
        expected_response = {
            "available_combined_topics_with_parameters_and_time": [
                {
                    "action_and_publisher_name": "example_topic_test",
                    "date_and_time": {
                        "day": 0,
                        "hour": 0,
                        "minute": 0,
                        "month": 0,
                        "nanosecond": 0,
                        "second": 0,
                        "year": 0
                    },
                    "listen_topics": ["/example_topic_1", "/example_topic_2"],
                    "logic_expression": "/example_topic_1 and /example_topic_2",
                    "publication_method": 2,
                    "trigger_text": "Updated action triggered!"
                }
            ]
        }
        self.assertEqual(response.get_json(), expected_response)


    ### Test for Echo Data Base ###
    def test_topic_echo_data_base(self):
        self.mock_ros2_manager.call_get_last_message_service.return_value = {
            'int32_msgs': [],
            'micro_publisher_data': [
                {
                    'sensors_data': [
                        {
                            'id': 111537764,
                            'tpb_value': {
                                'binary_value': True,
                                'pressure': 0,
                                'temperature': 0,
                                'value_type': 2
                            }
                        },
                        {
                            'id': 438792350,
                            'tpb_value': {
                                'binary_value': False,
                                'pressure': 25964,
                                'temperature': 0,
                                'value_type': 1
                            }
                        },
                        {
                            'id': 2142757034,
                            'tpb_value': {
                                'binary_value': False,
                                'pressure': 24588,
                                'temperature': 0,
                                'value_type': 1
                            }
                        },
                        {
                            'id': 155324914,
                            'tpb_value': {
                                'binary_value': False,
                                'pressure': 0,
                                'temperature': 0,
                                'value_type': 2
                            }
                        }
                    ]
                }
            ],
            'timestamps': [
                {
                    'year': 2024,
                    'month': 9,
                    'day': 4,
                    'hour': 9,
                    'minute': 24,
                    'second': 0,
                    'nanosecond': 893372345
                }
            ]
        }

        response = self.client.get(
        '/api/topic_echo_data_base/sensor_publisher',
        query_string={"type": "lora_msgs/MicroPublisher"}
)
        self.assertEqual(response.status_code, 200)

        data = response.get_json()
        self.assertIn('int32_msgs', data)
        self.assertIn('micro_publisher_data', data)
        self.assertIn('timestamps', data)

    def test_topic_echo_data_base_invalid_topic(self):
        response = self.client.get(
            '/api/topic_echo_data_base/123',
            query_string={"type": "lora_msgs/MicroPublisher"}
        )
        self.assertEqual(response.status_code, 400)

    ### Test for Echo Topic ###
    def test_echo_topic_message(self):
        self.mock_ros2_manager.get_topic_message.return_value = 'Hello World'

        response = self.client.get('/api/topic_echo/test_topic', query_string={'type': 'std_msgs/String'})
        self.assertEqual(response.status_code, 200)
        data = response.get_json()
        self.assertEqual(data['message'], 'Hello World')
    
    def test_echo_topic_message_invalid_topic(self):
        response = self.client.get('/api/topic_echo/123', query_string={'type': 'std_msgs/String'})
        self.assertEqual(response.status_code, 400)

    ### Test for Echo GPS Devices ###
    def test_add_gps_device(self):
        self.mock_ros2_manager.call_add_gps_device_service.return_value = True

        request_payload = {
            "device_name": "Test_Device",
            "device_eui": {
                "data": [0, 0, 0, 0, 0, 0, 0, 1]
            },
            "nav_value": {
                "latitude": 52.2297,
                "longitude": 21.0122,
                "altitude": 100.0
            },
            "is_moving": False
        }

        response = self.client.post('/api/add_gps_device', json=request_payload)
        self.assertEqual(response.status_code, 200)

        data = response.get_json()
        self.assertTrue(data['success'])

    def test_add_gps_device_invalid_device_name(self):
        response = self.client.post('/api/add_gps_device', json={
            "device_name": "",
            "device_eui": {
                "data": [0, 0, 0, 0, 0, 0, 0, 1]
            },
            "nav_value": {
                "latitude": 52.2297,
                "longitude": 21.0122,
                "altitude": 100.0
            },
            "is_moving": False
        })
        self.assertEqual(response.status_code, 400)

    def test_add_gps_device_invalid_device_eui(self):
        response = self.client.post('/api/add_gps_device', json={
            "device_name": "Test_Device",
            "device_eui": {
                "data": [0, 0, 0, 0, 0, 0, 0]
            },
            "nav_value": {
                "latitude": 52.2297,
                "longitude": 21.0122,
                "altitude": 100.0
            },
            "is_moving": False
        })
        self.assertEqual(response.status_code, 400)

    def test_add_gps_device_invalid_nav_value(self):
        response = self.client.post('/api/add_gps_device', json={
            "device_name": "Test_Device",
            "device_eui": {
                "data": [0, 0, 0, 0, 0, 0, 0, 1]
            },
            "nav_value": {
                "latitude": 52.2297,
                "longitude": 21.0122
            },
            "is_moving": False
        })
        self.assertEqual(response.status_code, 400)
    
    def test_delete_gps_device(self):
        self.mock_ros2_manager.call_delete_gps_device_service.return_value = True

        response = self.client.post('/api/delete_gps_device', json={
            "device_name": "Test_Device"
        })
        self.assertEqual(response.status_code, 200)

        data = response.get_json()
        self.assertTrue(data['success'])

    def test_modify_gps_device(self):
        self.mock_ros2_manager.call_modify_gps_device_service.return_value = True

        request_payload = {
            "device_name": "Test_Device",
            "device_eui": {
                "data": [0, 0, 0, 0, 0, 0, 0, 1]
            },
            "nav_value": {
                "latitude": 52.2297,
                "longitude": 21.0122,
                "altitude": 100.0
            }
        }

        response = self.client.post('/api/modify_gps_device', json=request_payload)
        self.assertEqual(response.status_code, 200)

        data = response.get_json()
        self.assertTrue(data['success'])
    
    def test_modify_gps_device_invalid_device_name(self):
        response = self.client.post('/api/modify_gps_device', json={
            "device_name": "",
            "device_eui": {
                "data": [0, 0, 0, 0, 0, 0, 0, 1]
            },
            "nav_value": {
                "latitude": 52.2297,
                "longitude": 21.0122,
                "altitude": 100.0
            }
        })
        self.assertEqual(response.status_code, 400)

    def test_modify_gps_device_invalid_device_eui(self):
        response = self.client.post('/api/modify_gps_device', json={
            "device_name": "Test_Device",
            "device_eui": {
                "data": [0, 0, 0, 0, 0, 0, 0]
            },
            "nav_value": {
                "latitude": 52.2297,
                "longitude": 21.0122,
                "altitude": 100.0
            }
        })
        self.assertEqual(response.status_code, 400)

    def test_topic_echo_gps_devices(self):
        self.mock_ros2_manager.get_gps_devices_message.return_value = [
            {
                "device_name": "Test_Device",
                "device_eui": {
                    "data": [0, 0, 0, 0, 0, 0, 0, 1]
                },
                "nav_value": {
                    "latitude": 52.2297,
                    "longitude": 21.0122,
                    "altitude": 100.0
                },
                "is_moving": False
            }
        ]

        response = self.client.get('/api/topic_echo_gps_devices')
        self.assertEqual(response.status_code, 200)

        data = response.get_json()
        self.assertIn('message', data)
        self.assertEqual(len(data['message']), 1)

        device = data['message'][0]
        self.assertEqual(device['device_name'], "Test_Device")
        self.assertEqual(device['device_eui']['data'], [0, 0, 0, 0, 0, 0, 0, 1])
        self.assertEqual(device['nav_value']['latitude'], 52.2297)
        self.assertEqual(device['nav_value']['longitude'], 21.0122)
        self.assertEqual(device['nav_value']['altitude'], 100.0)
        self.assertFalse(device['is_moving'])
    
    def test_message_type(self):
        self.mock_ros2_manager.get_topic_message_type.return_value = "notification_msgs/msg/Notification"

        response = self.client.get('/api/message_type/%mytopic')
        print(response.get_json())
        self.assertEqual(response.status_code, 200)
        data = response.get_json()

        message_type = data.get('message_type')
        self.assertEqual(message_type, 'notification_msgs/msg/Notification')
    
    def test_message_type_invalid_topic(self):
        response = self.client.get('/api/message_type/%123')
        self.assertEqual(response.status_code, 400)
        



if __name__ == '__main__':
    unittest.main()