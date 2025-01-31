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
