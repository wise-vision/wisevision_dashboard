#!/usr/bin/env python3

class ROS2Topic:
    def __init__(self, name, type):
        self.name = name
        self.type = type

    def __str__(self):
        return f"{self.name} - {self.type}"

class ROS2Topics:
    def __init__(self):
        self.topics = []  # Instance variable to store topics

    def add_topic(self, topic):
        """Add a topic to the list."""
        self.topics.append(topic)

    def remove_topic(self, topic):
        """Remove a topic from the list."""
        self.topics.remove(topic)

    def get_topics(self):
        """Return a list of all topics."""
        return self.topics

    def __str__(self):
        return "\n".join(str(topic) for topic in self.topics)
