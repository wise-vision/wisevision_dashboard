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
import time
from datetime import datetime, timezone
from ..data_object.fulldatatime_codec import FullDateTimeCodec as FDT

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

class ROS2Manager:
    def __init__(self):
        rclpy.init()
        self.node = Node('ros2_topic_list_node')
        self.subscriber_node = Node('ros2_subscriber_node')
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.subscriber_node)
        self.stop_requested = False

    def spin(self):
        try:
            print("Starting Executor...")
            while rclpy.ok() and not self.stop_requested:
                self.executor.spin_once(timeout_sec=0.1)
        except KeyboardInterrupt:
            print("Executor interrupted by KeyboardInterrupt.")
        finally:
            self.shutdown()

    def filter_topics(self, topics, default_filter=True, message_types=None, message_namespaces=None, name_contains=None):
        if message_types is None:
            message_types = []
        if message_namespaces is None:
            message_namespaces = []
        if name_contains is None:
            name_contains = []

        default_topics = {"/parameter_events", "/rosout", "/notifications"}

        filtered_topics = []
        for name, topic_type in topics:
            topic_type = topic_type.strip()

            if default_filter and name in default_topics:
                continue

            namespace = "/".join(name.split("/")[:-1])

            match_msg_type = not message_types or topic_type in message_types
            match_namespace = not message_namespaces or any(namespace.startswith(ns) for ns in message_namespaces)
            match_name = not name_contains or any(substring in name for substring in name_contains)

            if match_msg_type and match_namespace and match_name:
                filtered_topics.append((name, topic_type))

        return filtered_topics

    def get_topic_list(self, default_filter=True, message_types=None, message_namespaces=None, name_contains=None):
        topics = self.node.get_topic_names_and_types()
        topics = [(name, types[0] if types else "UnknownType") for name, types in topics]
        return self.filter_topics(topics, default_filter, message_types, message_namespaces, name_contains)
    
    
    def get_topics_types(self):
        topics = self.node.get_topic_names_and_types()
        topic_types = set()

        for _, types in topics:
            if types:
                topic_types.update(types)

        return list(topic_types) 
    
    def get_namespaces(self):
        """
        Retrieves unique namespaces of available topics in ROS2
        and returns them as a hierarchical dictionary structure.

        :return: A dictionary representing the hierarchical structure of namespaces, e.g.:
        {
            "ns_1": {
                "ns_2": {
                    "ns_3": {},
                    "ns_4": {}
                }
            }
        }
        """
        topics = self.node.get_topic_names_and_types()
        namespace_tree = {}

        for name, _ in topics:
            namespace_parts = name.strip("/").split("/")[:-1]
            
            if not namespace_parts:
                continue

            current_level = namespace_tree
            for part in namespace_parts:
                if part not in current_level:
                    current_level[part] = {} 
                current_level = current_level[part]

        return namespace_tree

    def get_service_list(self):
        services = self.node.get_service_names_and_types()
        ros_services = ROS2Services()
        for name, types in services:
            service_type = types[0] if types else 'UnknownType'
            ros_services.add_service(ROS2Service(name, service_type))
        return ros_services
    
    def replace_percent_with_slash(self, topic_name):
        return topic_name.replace('%2F', '/')
    
    
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
        if not msg_type:
            raise ImportError(f"Could not find message type {topic_type}")

        message_future = Future()

        def callback(msg):
            if not message_future.done():
                message_future.set_result(msg)

        subscription = self.node.create_subscription(msg_type, topic_name, callback, QoSProfile(depth=1))

        try:
            rclpy.spin_until_future_complete(self.node, message_future, timeout_sec=5.0)

            if message_future.done():
                serialized_message = self.serialize_ros_message_sub(message_future.result())
                return serialized_message
            else:
                return {"error": "No message arrived within 5 seconds"}

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
            raise ImportError(f"Could not find message type 'wisevision_msgs/msg/GpsDevicesPublisher'")

        message_future = Future()

        def callback(msg):
            if not message_future.done():
                message_future.set_result(msg)

        subscription = self.node.create_subscription(msg_type, topic_name, callback, QoSProfile(depth=1))

        try:
            rclpy.spin_until_future_complete(self.node, message_future, timeout_sec=60.0)

            if message_future.done():
                serialized_message = self.serialize_ros_message_sub(message_future.result())
                return serialized_message
            else:
                return {"error": "No message arrived within 60 seconds"}

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

    # wisevision_influxdb_ros2 services

    def _wait_service_or_timeout(self, node: Node, client, service_name: str, service_timeout: float, poll: float = 0.25):
        deadline = time.monotonic() + service_timeout
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=poll):
                return
            if not rclpy.ok():
                raise RuntimeError("ROS shutdown while waiting for service.")
        raise TimeoutError(f"Service {service_name} not available (timed out after {service_timeout}s).")

    def _wait_call_or_timeout(self, node: Node, future, service_name: str, call_timeout: float):
        """
        Wait for a future to complete using the executor that's already spinning.
        This allows multiple service calls to work concurrently without blocking the node.
        """
        deadline = time.monotonic() + call_timeout
        while time.monotonic() < deadline:
            if future.done():
                response = future.result()
                if response is None:
                    raise RuntimeError(f"Service {service_name} returned no response.")
                return response
            if not rclpy.ok():
                raise RuntimeError("ROS shutdown while waiting for service response.")
        raise TimeoutError(f"No response from {service_name} within {call_timeout}s.")



    def call_start_record_topics_service(self, params, service_timeout: float = 5.0, call_timeout: float = 15.0):
        """
        params:
        - bucket_name: str
        - topics_names: List[str]
        returns: bool (response.success)
        """
        service_name = '/start_record_topics'
        service_type = get_service('wisevision_msgs/srv/InfluxStartRecordTopics')
        if not service_type:
            raise ImportError("Service type not found for 'InfluxStartRecordTopics'")

        client = self.node.create_client(service_type, service_name)
        self._wait_service_or_timeout(self.node, client, service_name, service_timeout)

        request = service_type.Request(
            bucket_name=params.get('bucket_name', ''),
            topics_names=params.get('topics_names', []),
        )
        future = client.call_async(request)
        response = self._wait_call_or_timeout(self.node, future, service_name, call_timeout)
        print(response)
        return bool(response.success)


    def call_create_bucket_service(self, params, service_timeout: float = 5.0, call_timeout: float = 15.0):
        """
        params:
        - bucket_name: str
        - retention_days: int
        - description: str
        returns: bool (response.success)
        """
        service_name = '/create_bucket'
        service_type = get_service('wisevision_msgs/srv/InfluxCreateBucket')
        if not service_type:
            raise ImportError("Service type not found for 'InfluxCreateBucket'")

        client = self.node.create_client(service_type, service_name)
        self._wait_service_or_timeout(self.node, client, service_name, service_timeout)

        request = service_type.Request(
            bucket_name=params.get('bucket_name', ''),
            retention_days=params.get('retention_days', 0),
            description=params.get('description', ''),
        )
        future = client.call_async(request)
        response = self._wait_call_or_timeout(self.node, future, service_name, call_timeout)
        return bool(response.success)


    def call_get_influx_buckets_service(self, service_timeout: float = 5.0, call_timeout: float = 10.0):
        """
        returns: full response (response.success, response.influx_buckets, response.error_message)
        """
        service_name = '/get_influx_buckets'
        service_type = get_service('wisevision_msgs/srv/InfluxGetBuckets')
        if not service_type:
            raise ImportError("Service type not found for 'InfluxGetBuckets'")

        client = self.node.create_client(service_type, service_name)
        self._wait_service_or_timeout(self.node, client, service_name, service_timeout)

        request = service_type.Request()
        future = client.call_async(request)
        return self._wait_call_or_timeout(self.node, future, service_name, call_timeout)


    def call_get_currently_recording_topics_service(self, service_timeout: float = 5.0, call_timeout: float = 10.0):
        """
        returns: full response (response.success, response.topics, response.bucket_names, response.record_ids, response.error_message)
        """
        service_name = '/get_currently_recording_topics'
        service_type = get_service('wisevision_msgs/srv/InfluxGetCurrentlyRecordingTopics')
        if not service_type:
            raise ImportError("Service type not found for 'InfluxGetCurrentlyRecordingTopics'")

        client = self.node.create_client(service_type, service_name)
        self._wait_service_or_timeout(self.node, client, service_name, service_timeout)

        request = service_type.Request()
        future = client.call_async(request)
        return self._wait_call_or_timeout(self.node, future, service_name, call_timeout)


    def call_stop_record_topics_service(self, params, service_timeout: float = 5.0, call_timeout: float = 15.0):
        """
        params:
        - topics_names: List[str]
        returns: bool (response.success)
        """
        service_name = '/stop_record_topics'
        service_type = get_service('wisevision_msgs/srv/InfluxStopRecordTopics')
        if not service_type:
            raise ImportError("Service type not found for 'InfluxStopRecordTopics'")

        client = self.node.create_client(service_type, service_name)
        self._wait_service_or_timeout(self.node, client, service_name, service_timeout)

        request = service_type.Request(
            topics_names=params.get('topics_names', []),
        )
        future = client.call_async(request)
        response = self._wait_call_or_timeout(self.node, future, service_name, call_timeout)
        return bool(response.success)


    def call_get_recorded_bags_by_topic_service(self, params, service_timeout: float = 5.0, call_timeout: float = 10.0):
        service_name = '/get_recorded_bags_by_topic'
        service_type = get_service('wisevision_msgs/srv/InfluxGetRecordedBagsByTopic')
        if not service_type:
            raise ImportError("Service type not found for 'InfluxGetRecordedBagsByTopic'")

        client = self.node.create_client(service_type, service_name)
        self._wait_service_or_timeout(self.node, client, service_name, service_timeout)

        request = service_type.Request(
            topic_name=params.get('topic_name', ''),
        )
        future = client.call_async(request)
        response = self._wait_call_or_timeout(self.node, future, service_name, call_timeout)

        record_ids_raw = list(getattr(response, "record_ids", []))
        record_ids = [
            FDT.to_iso8601(x) if (hasattr(x, "year") and hasattr(x, "month") and (hasattr(x, "nanosecond") or hasattr(x, "nanosec")))
            else x
        for x in record_ids_raw]

        end_time_stamps = [
            FDT.to_iso8601(t) for t in getattr(response, "end_time_stamps", [])
        ]

        result = {
            "success": bool(response.success),
            "topic_name": getattr(response, "topic_name", ""),
            "bucket_names": list(getattr(response, "bucket_names", [])),
            "record_ids": record_ids,
            "end_time_stamps": end_time_stamps,
            "error": getattr(response, "error", ""),
        }
        return result


    def call_get_messages_service(self, params, service_timeout: float = 5.0, call_timeout: float = 20.0):
        """
        params:
        - topic_name: str
        - bucket_name: str (optional)
        - time_start: <FullDateTime> (optional)
        - time_end:   <FullDateTime> (optional)
        - number_of_msgs: int (optional)
        returns: full response (response.success, response.messages, response.timestamps, response.message_type, response.error_message)
        """
        service_name = '/get_messages'
        service_type = get_service('wisevision_msgs/srv/InfluxGetMessages')
        if not service_type:
            raise ImportError("Service type not found for 'InfluxGetMessages'")

        client = self.node.create_client(service_type, service_name)
        self._wait_service_or_timeout(self.node, client, service_name, service_timeout)

        request = service_type.Request(
            topic_name=params.get('topic_name', ''),
            bucket_name=params.get('bucket_name', ''),
            time_start=FDT.parse_iso8601(params.get('time_start')),
            time_end=FDT.parse_iso8601(params.get('time_end')),
            number_of_msgs=params.get('number_of_msgs', 0),
        )
        future = client.call_async(request)
        return self._wait_call_or_timeout(self.node, future, service_name, call_timeout)


    def call_get_recorded_topics_service(self, service_timeout: float = 5.0, call_timeout: float = 10.0):
        """
        returns: full response (response.success, response.topics, response.bucket_names, response.error_message)
        """
        service_name = '/get_recorded_topics'
        service_type = get_service('wisevision_msgs/srv/InfluxGetRecordedTopicsWithBucketsName')
        if not service_type:
            raise ImportError("Service type not found for 'InfluxGetRecordedTopicsWithBucketsName'")

        client = self.node.create_client(service_type, service_name)
        self._wait_service_or_timeout(self.node, client, service_name, service_timeout)

        request = service_type.Request()
        future = client.call_async(request)
        return self._wait_call_or_timeout(self.node, future, service_name, call_timeout)


    def call_play_recordings_service(self, params, service_timeout: float = 5.0, call_timeout: float = 10.0):
        """
        params:
        - bucket_name: str
        - topic_names: List[str]
        - record_ids:  List[str]
        returns: bool (response.success)
        """
        service_name = '/play_recordings'
        service_type = get_service('wisevision_msgs/srv/InfluxPlayRecordings')
        if not service_type:
            raise ImportError("Service type not found for 'InfluxPlayRecordings'")

        client = self.node.create_client(service_type, service_name)
        self._wait_service_or_timeout(self.node, client, service_name, service_timeout)

        request = service_type.Request(
            bucket_name=params.get('bucket_name', ''),
            topic_names=params.get('topic_names', []),
            record_ids=params.get('record_ids', []),
        )
        future = client.call_async(request)
        response = self._wait_call_or_timeout(self.node, future, service_name, call_timeout)
        return bool(response.success)


    def call_stop_playing_topics_service(self, params, service_timeout: float = 5.0, call_timeout: float = 15.0):
        """
        params:
        - topics_names: List[str]
        returns: bool (response.success)
        """
        service_name = '/stop_playing_topics'
        service_type = get_service('wisevision_msgs/srv/InfluxStopPlayingTopics')
        if not service_type:
            raise ImportError("Service type not found for 'InfluxStopPlayingTopics'")

        client = self.node.create_client(service_type, service_name)
        self._wait_service_or_timeout(self.node, client, service_name, service_timeout)

        request = service_type.Request(
            topics_names=params.get('topics_names', []),
        )
        future = client.call_async(request)
        response = self._wait_call_or_timeout(self.node, future, service_name, call_timeout)
        return bool(response.success)


    def call_get_currently_playing_topics_service(self, service_timeout: float = 5.0, call_timeout: float = 10.0):
        """
        returns: full response (response.success, response.topics, response.bucket_names, response.record_ids, response.error_message)
        """
        service_name = '/get_currently_playing_topics'
        service_type = get_service('wisevision_msgs/srv/InfluxGetCurrentlyPlayingTopics')
        if not service_type:
            raise ImportError("Service type not found for 'InfluxGetCurrentlyPlayingTopics'")

        client = self.node.create_client(service_type, service_name)
        self._wait_service_or_timeout(self.node, client, service_name, service_timeout)

        request = service_type.Request()
        future = client.call_async(request)
        return self._wait_call_or_timeout(self.node, future, service_name, call_timeout)
    
    def call_get_pending_recording_topics_service(self, service_timeout: float = 5.0, call_timeout: float = 10.0):
        """
        returns: full response (response.success, response.topics, response.error_message)
        """
        service_name = '/get_pending_recording_topics'
        service_type = get_service('wisevision_msgs/srv/InfluxGetPendingRecordingTopics')
        if not service_type:
            raise ImportError("Service type not found for 'InfluxGetPendingRecordingTopics'")

        client = self.node.create_client(service_type, service_name)
        self._wait_service_or_timeout(self.node, client, service_name, service_timeout)

        request = service_type.Request()
        future = client.call_async(request)
        return self._wait_call_or_timeout(self.node, future, service_name, call_timeout)
    
    
    # END OF: wisevision_influxdb_ros2 services

    def get_topic_message_type(self, topic_name):
        topics = self.node.get_topic_names_and_types()
        for name, types in topics:
            if name == topic_name:
                return types[0] if types else 'UnknownType'
        return None
    
    # Get unique field types from nested message fields 
    def get_message_field_types(self, message_structure):
        """
        Retrieves unique field types from a given ROS2 message structure.

        :param message_structure: The full message structure as a dictionary.
        :return: A list of unique field types.
        """
        unique_types = set()  # Using a set to avoid duplicates

        def recursive_extract(struct):
            """Recursively extract field types from nested message structures."""
            if isinstance(struct, dict):
                for value in struct.values():
                    if isinstance(value, dict) or isinstance(value, list):
                        recursive_extract(value)  # Recursively process nested structures
                    elif isinstance(value, str):  # Only store type strings
                        unique_types.add(value)
            elif isinstance(struct, list):
                for item in struct:
                    if isinstance(item, str):  # Lists of types (e.g., covariance arrays)
                        unique_types.add(item)

        recursive_extract(message_structure)
        return list(unique_types)  # Convert to list for JSON response
            

    # Get nested message fields

    def filter_message_structure(self, message_structure, include_types=None, exclude_types=None):
        """
        Filters the given message structure based on specified data types.

        :param message_structure: The original message structure as a dictionary.
        :param include_types: A list of data types to include (if provided, only these types will be kept).
        :param exclude_types: A list of data types to exclude (if provided, these types will be removed).
        :return: The filtered message structure.
        """
        if include_types is None:
            include_types = []
        if exclude_types is None:
            exclude_types = []

        def recursive_filter(struct):
            """Recursively filters the message structure."""
            if isinstance(struct, dict):
                filtered = {}
                for key, value in struct.items():
                    if isinstance(value, dict) or isinstance(value, list):
                        # Recursively filter nested structures
                        filtered_value = recursive_filter(value)
                        if filtered_value:  # Keep only non-empty values
                            filtered[key] = filtered_value
                    elif isinstance(value, str):  # Only check types if it's a string (type declaration)
                        if (include_types and value not in include_types) or (exclude_types and value in exclude_types):
                            continue  # Skip this field
                        filtered[key] = value
                return filtered
            elif isinstance(struct, list):
                return [recursive_filter(item) for item in struct if isinstance(item, str) and
                        ((not include_types or item in include_types) and (not exclude_types or item not in exclude_types))]
            return struct

        return recursive_filter(message_structure)

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
       
    def request_stop(self):
        print("Stop requested for ROS2 executor.")
        self.stop_requested = True

    def shutdown(self):
        print("Shutting down ROS2...")
        self.executor.shutdown()
        self.node.destroy_node()
        self.subscriber_node.destroy_node()
        rclpy.shutdown()
        print("ROS2 shutdown complete.")

ros2_manager = ROS2Manager()
