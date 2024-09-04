#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ..data_object.message_objects import ROS2Topic, ROS2Topics, ROS2Service, ROS2Services
import importlib
from rosidl_runtime_py.utilities import get_message, get_service
# from rosidl_runtime_py.utilities import get_message_names_and_types, get_service_names_and_types
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

    def get_topic_message(self, topic_name, topic_type):
        msg_type = get_message(topic_type)
        if not msg_type:
            raise ImportError(f"Could not find message type {topic_type}")

        message_received = None

        def callback(msg):
            nonlocal message_received
            message_received = msg

        subscription = self.node.create_subscription(msg_type, topic_name, callback, QoSProfile(depth=10))

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
        service_type = get_service('automatic_action_msgs/srv/AutomaticAction')
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

    def shutdown(self):
        rclpy.shutdown()

ros2_manager = ROS2Manager()
