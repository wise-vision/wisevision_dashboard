#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

import os
from typing import Dict, List, Union
from typing_extensions import TypedDict, Literal

class StdioConnection(TypedDict):
    command: str
    args: List[str]
    transport: Literal["stdio"]

class SSEConnection(TypedDict):
    url: str
    transport: Literal["sse"]

MCPConfig = Dict[str, Union[StdioConnection, SSEConnection]]

DEFAULT_MCP_CONFIG: MCPConfig = {
    "ros2": {
        "command": "docker",
        "args": [
            "run",
            "-i",
            "--rm",
            "--network=host",
            "--pid=host", 
            "--ipc=host",
            "-v", "/dev/shm:/dev/shm",
            "wisevision/mcp_server_ros_2:jazzy"
        ],
        "transport": "stdio",
    }
}