#!/usr/bin/env python3

class ROS2Topic:
    def __init__(self, name, type):
        self.name = name
        self.type = type

    def __str__(self):
        return f"{self.name} - {self.type}"

class ROS2Service:
    def __init__(self, name, type):
        self.name = name
        self.type = type

    def __str__(self):
        return f"{self.name} - {self.type}"

class ROS2Topics:
    def __init__(self):
        self.topics = []

    def add_topic(self, topic):
        self.topics.append(topic)

    def get_topics(self):
        return self.topics

    def __str__(self):
        return "\n".join(str(topic) for topic in self.topics)

class ROS2Services:
    def __init__(self):
        self.services = []

    def add_service(self, service):
        self.services.append(service)

    def get_services(self):
        return self.services

    def __str__(self):
        return "\n".join(str(service) for service in self.services)
