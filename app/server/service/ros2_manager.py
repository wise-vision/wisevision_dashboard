#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ..data_object.message_objects import ROS2Topic, ROS2Topics, ROS2Service, ROS2Services
import importlib
from automatic_action_msgs.srv import AutomaticAction


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

    # Get topic message
    def get_topic_message(self, topic_name, topic_type):
        # Adjust topic_type from ROS2 format "std_msgs/msg/String" to Python module format "std_msgs.msg.String"
        if '/msg/' in topic_type:
            ros_module_name, ros_message_name = topic_type.replace('/msg/', '.msg.').rsplit('.', 1)
        else:
            # Fallback if the format does not strictly follow expected pattern
            try:
                ros_module_name, ros_message_name = topic_type.rsplit('.', 1)
            except ValueError:
                raise ValueError("Incorrect topic type format. Expected formats: 'package/msg/Type' or 'package.Type'")

        try:
            msg_module = importlib.import_module(ros_module_name)
            msg_type = getattr(msg_module, ros_message_name)
        except ModuleNotFoundError:
            raise ImportError(f"Could not find module {ros_module_name}")
        except AttributeError:
            raise ImportError(f"Could not find message type {ros_message_name} in {ros_module_name}")

        message_received = None

        def callback(msg):
            nonlocal message_received
            message_received = msg

        # Create a subscriber
        subscription = self.node.create_subscription(msg_type, topic_name, callback, 10)

        try:
            timeout_sec = 5.0
            end_time = self.node.get_clock().now().to_msg().sec + timeout_sec
            while rclpy.ok() and self.node.get_clock().now().to_msg().sec < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)
                if message_received is not None:
                    return str(message_received.data)
            return "No message arrived for 5s"
        finally:
            # Clean up the subscription after receiving the message or timing out
            self.node.destroy_subscription(subscription)
    
    def call_automatic_action_service(self, params):
        client = self.node.create_client(AutomaticAction, '/create_automatic_action')
        
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise Exception("Interrupted while waiting for the service. ROS shutdown.")

        request = AutomaticAction.Request(
            listen_topic=params['listen_topic'],
            listen_message_type=params['listen_message_type'],
            value=params['value'],
            trigger_val=params['trigger_val'],
            trigger_type=params['trigger_type'],
            pub_topic=params['pub_topic'],
            pub_message_type=params['pub_message_type'],
            trigger_text=params['trigger_text'],
            data_validity_ms=params['data_validity_ms']
        )

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        return response.success if response else False


    def shutdown(self):
        rclpy.shutdown()


ros2_manager = ROS2Manager()
