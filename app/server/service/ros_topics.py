#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# from data_object.message_objects import ROS2Topic, ROS2Topics
from ..data_object.message_objects import ROS2Topic, ROS2Topics

class ROS2Node(Node):
    def __init__(self):
        super().__init__('ros2_topic_list_node')

    def get_topic_list(self):
        topic_list = self.get_topic_names_and_types()
        return topic_list

def get_ros2_topics():
    rclpy.init()
    node = ROS2Node()
    try:
        topics_obj = ROS2Topics()
        topics_list = node.get_topic_list()
        for name, types in topics_list:
            topic_type = types[0] if types else 'UnknownType'
            topics_obj.add_topic(ROS2Topic(name, topic_type))

        rclpy.shutdown()
        return topics_obj
    except Exception as e:
        print(f"Failed to fetch topics: {str(e)}")
        rclpy.shutdown()
        return None

if __name__ == "__main__":
    topics = get_ros2_topics()
    if topics:
        print("List of ROS2 Topics:")
        print(topics)
    else:
        print("No topics found or error occurred.")
