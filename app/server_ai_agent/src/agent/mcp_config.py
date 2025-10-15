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
    "math": {
        "command": "python",
        "args": [os.path.join(os.path.dirname(__file__), "servers", "math_server.py")],
        "transport": "stdio",
    },
    "ros2": {
        "command": "docker",
        "args": [
            "run",
            "-i",
            "--rm",
            "wisevision/mcp_server_ros_2:humble"
        ],
        "transport": "stdio",
    }
}