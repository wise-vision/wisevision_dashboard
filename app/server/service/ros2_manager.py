#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from ..data_object.message_objects import ROS2Topic, ROS2Topics, ROS2Service, ROS2Services
from dateutil import parser  
from rosidl_runtime_py.utilities import get_message, get_service
from rclpy.qos import QoSProfile
from collections import OrderedDict

import array
import numpy as np

def ros_message_to_dict(msg):
    """
    Rekurencyjnie konwertuje wiadomość ROS2 na słownik i usuwa prefiksy `_` z nazw pól.
    """
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

class ROS2Manager:
    def __init__(self):
        rclpy.init()
        self.node = Node('ros2_topic_list_node')
        self.subscriber_node = Node('ros2_subscriber_node')
        self.notification_subscriber = None
        self.subscription_created = False 

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
    
    
    def serialize_ros_message_sub(self, msg):
        result = {}
        for field_name, field_type in msg.get_fields_and_field_types().items():
            value = getattr(msg, field_name)

            if hasattr(value, 'get_fields_and_field_types'):
                result[field_name] = self.serialize_ros_message_sub(value)
            elif isinstance(value, list):
                serialized_list = []
                for item in value:
                    if hasattr(item, 'get_fields_and_field_types'):
                        serialized_list.append(self.serialize_ros_message_sub(item))
                    elif isinstance(item, (array.array, tuple)):
                        serialized_list.append(list(item))
                    else:
                        serialized_list.append(item)
                result[field_name] = serialized_list
            elif isinstance(value, (array.array, tuple)):
                result[field_name] = list(value)
            elif isinstance(value, np.ndarray):  # Handle numpy arrays
                result[field_name] = value.tolist()
            elif isinstance(value, (bytes, bytearray)):
                result[field_name] = value.decode('utf-8', errors='ignore')
            elif isinstance(value, (int, float, str, bool, type(None))):
                result[field_name] = value
            elif isinstance(value, dict):
                serialized_dict = {}
                for k, v in value.items():
                    serialized_dict[k] = self.serialize_ros_message_sub(v) if hasattr(v, 'get_fields_and_field_types') else v
                result[field_name] = serialized_dict
            else:
                print(f"Unsupported type for JSON serialization: {field_name} of type {type(value)}")
                result[field_name] = str(value)

        return result

    
    def get_topic_message(self, topic_name, topic_type):
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
                    serialized_message = self.serialize_ros_message_sub(message_received)
                    return serialized_message
            return {"error": "No message arrived for 5s"}
        finally:
            self.node.destroy_subscription(subscription)
    # Automatic Action services
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
    
    def call_delete_automatic_action_service(self, params):
        service_type = get_service('lora_msgs/srv/AutomaticActionDelete')
        if not service_type:
            raise ImportError("Service type not found for 'AutomaticActionDelete'")

        client = self.node.create_client(service_type, '/delete_automatic_action')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request(listen_topic_to_delete=params.get('listen_topic_to_delete'))

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        return response.success if response else False
    
    def call_combined_automatic_action_service(self, params):
        service_type = get_service('lora_msgs/srv/AutomaticActionConnection')
        if not service_type:
            raise ImportError("Service type not found for 'AutomaticActionConnection'")

        client = self.node.create_client(service_type, '/create_combined_automatic_action')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request(
            listen_topics=params.get('listen_topics', []),
            logic_expression=params.get('logic_expression', ''),
            action_and_publisher_name=params.get('action_and_publisher_name', ''),
            trigger_text=params.get('trigger_text', ''),
            publication_method=params.get('publication_method')
        )

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        return response.success if response else False
    
    def call_delete_combined_automatic_action_service(self, params):
        service_type = get_service('lora_msgs/srv/AutomaticActionCombinedDelete')
        if not service_type:
            raise ImportError("Service type not found for 'AutomaticActionCombinedDelete'")

        client = self.node.create_client(service_type, '/delete_combined_automatic_action')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request(
            name_of_combined_topics_publisher=params.get('name_of_combined_topics_publisher')
        )

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        return response.success if response else False
    
    def call_available_topics_service(self):
        service_type = get_service('lora_msgs/srv/AvailableTopics')
        if not service_type:
            raise ImportError("Service type not found for 'AvailableTopics'")

        client = self.node.create_client(service_type, '/available_topics')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request()

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        if response:
            return [ros_message_to_dict(topic) for topic in response.available_topics_with_parameters_and_time]
        else:
            return []
    def call_available_topics_combined_service(self):
        service_type = get_service('lora_msgs/srv/AvailableTopicsCombined')
        if not service_type:
            raise ImportError("Service type not found for 'AvailableTopicsCombined'")

        client = self.node.create_client(service_type, '/available_topics_combined')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request()

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        if response:
            return [ros_message_to_dict(topic) for topic in response.available_combined_topics_with_parameters_and_time]
        else:
            return []
        
    def call_change_automatic_action_service(self, params):
        service_type = get_service('lora_msgs/srv/ChangeAutomaticAction')
        if not service_type:
            raise ImportError("Service type not found for 'ChangeAutomaticAction'")

        client = self.node.create_client(service_type, '/change_automatic_action')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request(**params)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        return response.success if response else False
    
    def call_change_automatic_action_combined_service(self, params):
        service_type = get_service('lora_msgs/srv/ChangeAutomaticActionCombined')
        if not service_type:
            raise ImportError("Service type not found for 'ChangeAutomaticActionCombined'")

        client = self.node.create_client(service_type, '/change_combined_automatic_action')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request(**params)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        return response.success if response else False
    
    # END OF: Automatic Action services
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

    # GPS Devices services
    def get_gps_devices_message(self):
        msg_type = get_message('wisevision_msgs/msg/GpsDevicesPublisher')
        topic_name = self.replace_percent_with_slash('/gps_devices_data')
        if not msg_type:
            raise ImportError(f"Could not find message type {'wisevision_msgs/msg/GpsDevicesPublisher'}")

        message_received = None

        def callback(msg):
            nonlocal message_received
            message_received = msg

        subscription = self.node.create_subscription(msg_type, topic_name, callback, QoSProfile(depth=1))

        try:
            timeout_sec = 60.0
            end_time = self.node.get_clock().now().to_msg().sec + timeout_sec
            while rclpy.ok() and self.node.get_clock().now().to_msg().sec < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)
                if message_received is not None:
                    serialized_message = self.serialize_ros_message_sub(message_received)
                    return serialized_message
            return {"error": "No message arrived for 60s"}
        finally:
            self.node.destroy_subscription(subscription)

    def call_add_gps_device_service(self, params):
        service_type = get_service('wisevision_msgs/srv/AddGpsDevice')
        if not service_type:
            raise ImportError("Service type not found for 'AddGpsDevice'")

        client = self.node.create_client(service_type, '/add_gps_device')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")
        eui64_data = service_type.Request().device_eui
        eui64_data.data = params.get('device_eui', {}).get('data', [])
        nav_value_data = service_type.Request().nav_value
        nav_value_data.latitude = params.get('nav_value', {}).get('latitude', 0.0)
        nav_value_data.longitude = params.get('nav_value', {}).get('longitude', 0.0)
        nav_value_data.altitude = params.get('nav_value', {}).get('altitude', 0.0)
        request = service_type.Request(
            device_name=params.get('device_name'),
            device_eui=eui64_data,
            nav_value=nav_value_data,
            is_moving=params.get('is_moving')
        )

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        return response.success if response else False
    
    def call_delete_gps_device_service(self, params):
        service_type = get_service('wisevision_msgs/srv/DeleteGpsDevice')
        if not service_type:
            raise ImportError("Service type not found for 'DeleteGpsDevice'")

        client = self.node.create_client(service_type, '/delete_gps_device')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")
        eui64_data = service_type.Request().device_eui
        eui64_data.data = params.get('device_eui', {}).get('data', [])
        request = service_type.Request(device_eui=eui64_data)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        return response.success if response else False

    def call_modify_gps_device_service(self, params):
        service_type = get_service('wisevision_msgs/srv/ModifyGpsDevice')
        if not service_type:
            raise ImportError("Service type not found for 'ModifyGpsDevice'")

        client = self.node.create_client(service_type, '/modify_gps_device')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")
        eui64_data = service_type.Request().device_eui
        eui64_data.data = params.get('device_eui', {}).get('data', [])
        nav_value_data = service_type.Request().nav_value
        nav_value_data.latitude = params.get('nav_value', {}).get('latitude', 0.0)
        nav_value_data.longitude = params.get('nav_value', {}).get('longitude', 0.0)
        nav_value_data.altitude = params.get('nav_value', {}).get('altitude', 0.0)
        request = service_type.Request(
            device_name=params.get('device_name'),
            device_eui=eui64_data,
            nav_value=nav_value_data
        )

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        return response.success if response else False
    # END OF: GPS Devices services

    # Blackbox services

    def call_add_storage_service(self, params):
        service_type = get_service('wisevision_msgs/srv/AddStorageToDataBase')
        if not service_type:
            raise ImportError("Service type not found for 'AddStorageToDataBase'")

        client = self.node.create_client(service_type, '/add_storage_to_database')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request(
            storage_name=params.get('storage_name'),
        )

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        return response.success if response else False
    
    def call_create_database_service(self, params):
        service_type = get_service('wisevision_msgs/srv/CreateDataBase')
        if not service_type:
            raise ImportError("Service type not found for 'CreateDataBase'")

        client = self.node.create_client(service_type, '/create_database')
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = service_type.Request(
            key_expr=params.get('key_expr'),
            volume_id=params.get('volume_id'),
            db_name=params.get('db_name'),
            create_db=params.get('create_db')
        )

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        return response.success if response else False

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
        if future.done():
            print("Service call completed")
        else:
            print("Service call did not complete within the timeout")
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
                                elif isinstance(item, (array.array, tuple)):
                                    serialized_list.append(list(item))
                                else:
                                    serialized_list.append(item)
                            result[field_name] = serialized_list
                        elif isinstance(value, (array.array, tuple)):
                            result[field_name] = list(value)
                        elif isinstance(value, np.ndarray):
                            result[field_name] = value.tolist()
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
    
    # END OF: Blackbox services


    def get_topic_message_type(self, topic_name):
        topic_name = self.replace_percent_with_slash(topic_name)
        topics = self.node.get_topic_names_and_types()
        for name, types in topics:
            if name == topic_name:
                return types[0] if types else 'UnknownType'
        return None
        

    # Get nested message fields

    def get_message_structure(self, message_type_str):
        message_type_str = self.normalize_message_type(message_type_str)
        msg_class = get_message(message_type_str)
        if msg_class is None:
            raise Exception(f"Message type '{message_type_str}' not found")
        return self._get_message_structure_recursive(msg_class, processed_types=set())

    def normalize_message_type(self, type_str):
        if type_str.count('/') == 2:
            return type_str
        elif type_str.count('/') == 1:
            package_name, message_name = type_str.split('/')
            return f"{package_name}/msg/{message_name}"
        else:
            return type_str

    def _get_message_structure_recursive(self, msg_class, processed_types, depth=0):
        indent = '  ' * depth
        package_name = msg_class.__module__.split('.')[0]
        msg_type_name = f"{package_name}/msg/{msg_class.__name__}"

        if msg_type_name in processed_types:
            return msg_type_name  # Avoid infinite recursion

        processed_types.add(msg_type_name)
        structure = OrderedDict()

        for field_name, field_type_str in msg_class._fields_and_field_types.items():
            field_type = self._parse_field_type(field_type_str)

            if field_type['is_array']:
                element_type = field_type['type']
                if self._is_primitive_type(element_type):
                    structure[field_name] = [element_type]
                else:
                    nested_msg_class = get_message(element_type)
                    if nested_msg_class is not None:
                        structure[field_name] = [
                            self._get_message_structure_recursive(
                                nested_msg_class, processed_types, depth + 1
                            )
                        ]
                    else:
                        structure[field_name] = [element_type]
            else:
                if self._is_primitive_type(field_type['type']):
                    structure[field_name] = field_type['type']
                else:
                    nested_msg_class = get_message(field_type['type'])
                    if nested_msg_class is not None:
                        structure[field_name] = self._get_message_structure_recursive(
                            nested_msg_class, processed_types, depth + 1
                        )
                    else:
                        structure[field_name] = field_type['type']

        return structure

    def _parse_field_type(self, field_type_str):
        field_info = {'type': None, 'is_array': False}
        if field_type_str.startswith('sequence<'):
            field_info['is_array'] = True
            element_type = field_type_str[9:-1]
            field_info['type'] = self.normalize_message_type(element_type)
        elif '[' in field_type_str and field_type_str.endswith(']'):
            field_info['is_array'] = True
            element_type = field_type_str.split('[')[0]
            field_info['type'] = self.normalize_message_type(element_type)
        else:
            field_info['type'] = self.normalize_message_type(field_type_str)
        return field_info

    def _is_primitive_type(self, field_type):
        primitive_types = {
            'bool', 'boolean', 'byte', 'char',
            'float32', 'float64', 'float', 'double',
            'int8', 'uint8', 'int16', 'uint16',
            'int32', 'uint32', 'int64', 'uint64',
            'int', 'string', 'wstring'
        }
        return field_type in primitive_types
    # END OF: Get nested message fields
       
    def shutdown(self):
        rclpy.shutdown()

ros2_manager = ROS2Manager()
