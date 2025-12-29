#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

"""
Tests for MCP configuration module
"""
import pytest
from agent.mcp_config import DEFAULT_MCP_CONFIG, StdioConnection, SSEConnection, MCPConfig


class TestMCPConfigConstants:
    """Tests for MCP configuration constants"""

    def test_default_mcp_config_exists(self):
        """Test that DEFAULT_MCP_CONFIG is defined"""
        assert DEFAULT_MCP_CONFIG is not None
        assert isinstance(DEFAULT_MCP_CONFIG, dict)

    def test_default_config_has_ros2(self):
        """Test that default config includes ROS2 server"""
        assert "ros2" in DEFAULT_MCP_CONFIG
        assert DEFAULT_MCP_CONFIG["ros2"]["transport"] == "stdio"

    def test_default_config_ros2_structure(self):
        """Test ROS2 configuration structure"""
        ros2_config = DEFAULT_MCP_CONFIG["ros2"]
        
        assert "command" in ros2_config
        assert "args" in ros2_config
        assert "transport" in ros2_config
        
        assert ros2_config["command"] == "docker"
        assert isinstance(ros2_config["args"], list)
        assert len(ros2_config["args"]) > 0

    def test_default_config_ros2_docker_args(self):
        """Test that ROS2 uses proper Docker arguments"""
        ros2_config = DEFAULT_MCP_CONFIG["ros2"]
        args = ros2_config["args"]
        
        # Should have basic docker run args
        assert "run" in args
        assert "-i" in args
        assert "--rm" in args
        assert "--network=host" in args


class TestStdioConnection:
    """Tests for StdioConnection TypedDict"""

    def test_stdio_connection_structure(self):
        """Test StdioConnection type structure"""
        stdio_conn: StdioConnection = {
            "command": "python",
            "args": ["-m", "mcp_server"],
            "transport": "stdio"
        }
        
        assert stdio_conn["command"] == "python"
        assert stdio_conn["args"] == ["-m", "mcp_server"]
        assert stdio_conn["transport"] == "stdio"

    def test_stdio_connection_with_docker(self):
        """Test StdioConnection with Docker command"""
        docker_conn: StdioConnection = {
            "command": "docker",
            "args": ["run", "-i", "--rm", "image:tag"],
            "transport": "stdio"
        }
        
        assert docker_conn["command"] == "docker"
        assert "run" in docker_conn["args"]
        assert docker_conn["transport"] == "stdio"

    def test_stdio_connection_with_empty_args(self):
        """Test StdioConnection with no arguments"""
        simple_conn: StdioConnection = {
            "command": "mcp-server",
            "args": [],
            "transport": "stdio"
        }
        
        assert simple_conn["args"] == []
        assert simple_conn["transport"] == "stdio"


class TestSSEConnection:
    """Tests for SSEConnection TypedDict"""

    def test_sse_connection_structure(self):
        """Test SSEConnection type structure"""
        sse_conn: SSEConnection = {
            "url": "http://localhost:8080",
            "transport": "sse"
        }
        
        assert sse_conn["url"] == "http://localhost:8080"
        assert sse_conn["transport"] == "sse"

    def test_sse_connection_with_https(self):
        """Test SSEConnection with HTTPS URL"""
        sse_conn: SSEConnection = {
            "url": "https://example.com:8443/mcp",
            "transport": "sse"
        }
        
        assert sse_conn["url"].startswith("https://")
        assert sse_conn["transport"] == "sse"

    def test_sse_connection_with_path(self):
        """Test SSEConnection with URL path"""
        sse_conn: SSEConnection = {
            "url": "http://localhost:8080/api/mcp/v1",
            "transport": "sse"
        }
        
        assert "/api/mcp/v1" in sse_conn["url"]


class TestMCPConfig:
    """Tests for MCPConfig type"""

    def test_mcp_config_with_multiple_servers(self):
        """Test MCPConfig with multiple server types"""
        config: MCPConfig = {
            "stdio_server": {
                "command": "python",
                "args": ["-m", "server1"],
                "transport": "stdio"
            },
            "sse_server": {
                "url": "http://localhost:8080",
                "transport": "sse"
            }
        }
        
        assert len(config) == 2
        assert "stdio_server" in config
        assert "sse_server" in config
        assert config["stdio_server"]["transport"] == "stdio"
        assert config["sse_server"]["transport"] == "sse"

    def test_mcp_config_empty(self):
        """Test empty MCP configuration"""
        config: MCPConfig = {}
        
        assert len(config) == 0
        assert isinstance(config, dict)

    def test_mcp_config_with_ros2_only(self):
        """Test MCPConfig with only ROS2 server"""
        config: MCPConfig = {
            "ros2": {
                "command": "docker",
                "args": ["run", "-i", "--rm", "ros2_image"],
                "transport": "stdio"
            }
        }
        
        assert len(config) == 1
        assert "ros2" in config
        assert config["ros2"]["command"] == "docker"


class TestDefaultConfigValidation:
    """Tests to validate the default configuration"""

    def test_default_config_is_valid_mcp_config(self):
        """Test that DEFAULT_MCP_CONFIG is a valid MCPConfig"""
        config: MCPConfig = DEFAULT_MCP_CONFIG
        
        # Should be a dictionary
        assert isinstance(config, dict)
        
        # Each server should have required fields
        for server_name, server_config in config.items():
            assert "transport" in server_config
            assert server_config["transport"] in ["stdio", "sse"]
            
            if server_config["transport"] == "stdio":
                assert "command" in server_config
                assert "args" in server_config
            elif server_config["transport"] == "sse":
                assert "url" in server_config

    def test_default_config_ros2_is_stdio(self):
        """Test that ROS2 default config uses stdio transport"""
        assert DEFAULT_MCP_CONFIG["ros2"]["transport"] == "stdio"
        assert "command" in DEFAULT_MCP_CONFIG["ros2"]
        assert "args" in DEFAULT_MCP_CONFIG["ros2"]

    def test_default_config_ros2_command_is_docker(self):
        """Test that ROS2 uses Docker as the command"""
        assert DEFAULT_MCP_CONFIG["ros2"]["command"] == "docker"

    def test_default_config_ros2_has_necessary_docker_flags(self):
        """Test that ROS2 Docker config has necessary flags for ROS2"""
        args = DEFAULT_MCP_CONFIG["ros2"]["args"]
        
        # Critical Docker flags for ROS2
        assert "--network=host" in args, "ROS2 needs host network"
        assert "--pid=host" in args, "ROS2 needs host PID"
        assert "--ipc=host" in args, "ROS2 needs host IPC"
        
        # Should mount shared memory
        shm_mount_found = False
        for i, arg in enumerate(args):
            if arg == "-v" and i + 1 < len(args):
                if "/dev/shm" in args[i + 1]:
                    shm_mount_found = True
                    break
        assert shm_mount_found, "ROS2 needs shared memory mount"

    def test_default_config_ros2_uses_interactive_mode(self):
        """Test that ROS2 Docker runs in interactive mode"""
        args = DEFAULT_MCP_CONFIG["ros2"]["args"]
        assert "-i" in args, "Docker should run in interactive mode for stdio"

    def test_default_config_ros2_uses_remove_flag(self):
        """Test that ROS2 Docker removes container after exit"""
        args = DEFAULT_MCP_CONFIG["ros2"]["args"]
        assert "--rm" in args, "Docker should remove container after exit"
