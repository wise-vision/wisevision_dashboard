#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from ..data_object.message_objects import ROS2Topic, ROS2Topics, ROS2Service, ROS2Services
import importlib
from dateutil import parser  
from rosidl_runtime_py.utilities import get_message, get_service
from rclpy.qos import QoSProfile

class ROS2Manager:
    def __init__(self):
        rclpy.init()
        self.node = Node('ros2_topic_list_node')

    def get_topic_list(self):
        topics = self.node.get_topic_names_and_types()
        ros_topics = ROS2Topics()
        for name, types in topics:
            topic_type = types[0] if types else 'UnknownType'
            ros_topics.add_topic(ROS2Topic(name, topic_type))
        return ros_topics

    def get_service_list(self):
        services = self.node.get_service_names_and_types()
        ros_services = ROS2Services()
        for name, types in services:
            service_type = types[0] if types else 'UnknownType'
            ros_services.add_service(ROS2Service(name, service_type))
        return ros_services
    
    def replace_percent_with_slash(self, topic_name):
        return topic_name.replace('%', '/')

    def get_topic_message(self, topic_name, topic_type):  # get_new_topic_message from ros2 topic 
        msg_type = get_message(topic_type)
        topic_name = self.replace_percent_with_slash(topic_name)
        if not msg_type:
            raise ImportError(f"Could not find message type {topic_type}")

        message_received = None

        def callback(msg):
            nonlocal message_received
            message_received = msg

        subscription = self.node.create_subscription(msg_type, topic_name, callback, QoSProfile(depth=1))

        try:
            timeout_sec = 5.0
            end_time = self.node.get_clock().now().to_msg().sec + timeout_sec
            while rclpy.ok() and self.node.get_clock().now().to_msg().sec < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)
                if message_received is not None:
                    return str(message_received)
            return "No message arrived for 5s"
        finally:
            self.node.destroy_subscription(subscription)

    def call_automatic_action_service(self, params):
        service_type = get_service('lora_msgs/srv/AutomaticAction')
        if not service_type:
            raise ImportError("Service type not found for 'AutomaticAction'")

        client = self.node.create_client(service_type, '/create_automatic_action')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request(**params)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        return response.success if response else False
    
    def call_get_messages_service(self, params):
        service_type = get_service('lora_msgs/srv/GetMessages')
        if not service_type:
            raise ImportError("Service type not found for 'GetMessages'")
        
        client = self.node.create_client(service_type, '/get_messages')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")


        request = service_type.Request(**params)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        if response:
            return {
                'int32_msgs': response.int32_msgs,
                'micro_publisher_data': response.micro_publisher_data,
                'timestamps': response.timestamps
            }
        else:
            return None


    def call_get_last_message_service(self, params):
        
        service_type = get_service('lora_msgs/srv/GetMessages')
        if not service_type:
            raise ImportError("Service type not found for 'GetMessages'")

        client = self.node.create_client(service_type, '/get_messages')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request()
        request.topic_name = params.get('topic_name')
        print('topic_name:', request.topic_name)
        request.message_type = params.get('message_type')
        print('message_type:', request.message_type)
        request.number_of_msgs = 1

        future = client.call_async(request)
        print('params:')
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        
        if response:
            return {
                'int32_msgs': response.int32_msgs,
                'micro_publisher_data': response.micro_publisher_data,
                'timestamps': response.timestamps
            }
        else:
            self.get_logger().error('Error while retrieving messages from the service.')
            return None  

    def call_get_messages_service_any(self, params):
        service_type = get_service('lora_msgs/srv/GetMessages')
        if not service_type:
            raise ImportError("Service type not found for 'GetMessages'")

        client = self.node.create_client(service_type, '/get_messages')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request()
        topic_name = params.get('topic_name')
        topic_name = self.replace_percent_with_slash(topic_name)
        request.topic_name = topic_name
        request.message_type = 'any'
        request.number_of_msgs = params.get('number_of_msgs', 0)

        def parse_iso8601_to_fulldatetime(iso8601_str):
            FullDateTime = get_message('lora_msgs/msg/FullDateTime')

            dt = parser.isoparse(iso8601_str)

            full_datetime = FullDateTime()
            full_datetime.year = dt.year
            full_datetime.month = dt.month
            full_datetime.day = dt.day
            full_datetime.hour = dt.hour
            full_datetime.minute = dt.minute
            full_datetime.second = dt.second
            full_datetime.nanosecond = dt.microsecond * 1000 

            return full_datetime

        if 'time_start' in params:
            request.time_start = parse_iso8601_to_fulldatetime(params['time_start'])
        if 'time_end' in params:
            request.time_end = parse_iso8601_to_fulldatetime(params['time_end'])


        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        if response:
            try:
                MessageType = get_message(params.get('message_type'))
                messages = []
                data = response.data
                offset = 0

                while offset < len(data):
                    message_length = int.from_bytes(data[offset:offset + 4], byteorder='big')
                    offset += 4

                    message_data = bytes(data[offset:offset + message_length])
                    offset += message_length

                    message = deserialize_message(message_data, MessageType())
                    messages.append(message)

                def serialize_ros_message(msg):
                    result = {}
                    for field_name, field_type in msg.get_fields_and_field_types().items():
                        value = getattr(msg, field_name)

                        if hasattr(value, 'get_fields_and_field_types'):
                            result[field_name] = serialize_ros_message(value)
                        elif isinstance(value, list):
                            serialized_list = []
                            for item in value:
                                if hasattr(item, 'get_fields_and_field_types'):
                                    serialized_list.append(serialize_ros_message(item))
                                else:
                                    serialized_list.append(item)
                            result[field_name] = serialized_list
                        elif isinstance(value, (bytes, bytearray)):
                            result[field_name] = value.decode('utf-8', errors='ignore')
                        elif isinstance(value, (int, float, str, bool)):
                            result[field_name] = value
                        else:
                            print(f"Unsupported type for JSON serialization: {field_name} of type {type(value)}")
                            result[field_name] = str(value)

                    return result

                serialized_response = {
                    'timestamps': [serialize_ros_message(timestamp) for timestamp in response.timestamps],
                    'messages': [serialize_ros_message(msg) for msg in messages]
                }
                return serialized_response
            except Exception as e:
                raise Exception(f"Message deserialization error:” {e}")
        else:
            return None
       
    def shutdown(self):
        rclpy.shutdown()

ros2_manager = ROS2Manager()
