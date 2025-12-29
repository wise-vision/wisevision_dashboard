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
Tests for user MCP configuration management
"""
import pytest
import json
from pathlib import Path
from unittest.mock import patch, mock_open
from agent.user_mcp_config import (
    load_user_mcp_config,
    save_user_mcp_config,
    merge_with_defaults,
    get_config_file_path,
    config_file_exists,
)


class TestLoadUserMCPConfig:
    """Tests for loading user MCP configuration"""

    def test_load_config_file_not_exists(self, tmp_path, monkeypatch):
        """Test loading when config file doesn't exist"""
        # Mock the config file path to point to a non-existent file
        fake_file = tmp_path / "nonexistent.json"
        monkeypatch.setattr("agent.user_mcp_config.USER_CONFIG_FILE", fake_file)
        
        result = load_user_mcp_config()
        
        assert result == {}

    def test_load_config_valid_json(self, tmp_path, monkeypatch):
        """Test loading valid JSON configuration"""
        config_data = {
            "server1": {
                "command": "test_command",
                "args": ["arg1", "arg2"],
                "transport": "stdio"
            }
        }
        
        config_file = tmp_path / "config.json"
        config_file.write_text(json.dumps(config_data))
        monkeypatch.setattr("agent.user_mcp_config.USER_CONFIG_FILE", config_file)
        
        result = load_user_mcp_config()
        
        assert result == config_data
        assert "server1" in result
        assert result["server1"]["command"] == "test_command"

    def test_load_config_invalid_json(self, tmp_path, monkeypatch):
        """Test loading invalid JSON returns empty dict"""
        config_file = tmp_path / "config.json"
        config_file.write_text("{invalid json")
        monkeypatch.setattr("agent.user_mcp_config.USER_CONFIG_FILE", config_file)
        
        result = load_user_mcp_config()
        
        assert result == {}

    def test_load_config_permission_error(self, tmp_path, monkeypatch):
        """Test loading when file has permission issues"""
        config_file = tmp_path / "config.json"
        config_file.write_text("{}")
        config_file.chmod(0o000)  # Remove all permissions
        monkeypatch.setattr("agent.user_mcp_config.USER_CONFIG_FILE", config_file)
        
        result = load_user_mcp_config()
        
        assert result == {}
        
        # Restore permissions for cleanup
        config_file.chmod(0o644)


class TestSaveUserMCPConfig:
    """Tests for saving user MCP configuration"""

    def test_save_config_success(self, tmp_path, monkeypatch):
        """Test successful configuration save"""
        config_data = {
            "server1": {
                "command": "test",
                "args": ["arg1"],
                "transport": "stdio"
            }
        }
        
        config_file = tmp_path / "config.json"
        monkeypatch.setattr("agent.user_mcp_config.USER_CONFIG_FILE", config_file)
        monkeypatch.setattr("agent.user_mcp_config.CONFIG_DIR", tmp_path)
        
        result = save_user_mcp_config(config_data)
        
        assert result is True
        assert config_file.exists()
        
        # Verify saved content
        saved_data = json.loads(config_file.read_text())
        assert saved_data == config_data

    def test_save_config_creates_directory(self, tmp_path, monkeypatch):
        """Test that save creates directory if it doesn't exist"""
        config_dir = tmp_path / "new_dir"
        config_file = config_dir / "config.json"
        
        monkeypatch.setattr("agent.user_mcp_config.USER_CONFIG_FILE", config_file)
        monkeypatch.setattr("agent.user_mcp_config.CONFIG_DIR", config_dir)
        
        config_data = {"test": {"command": "cmd"}}
        result = save_user_mcp_config(config_data)
        
        assert result is True
        assert config_dir.exists()
        assert config_file.exists()

    def test_save_config_pretty_format(self, tmp_path, monkeypatch):
        """Test that saved config is pretty-formatted"""
        config_data = {
            "server1": {
                "command": "test",
                "args": ["arg1", "arg2"]
            }
        }
        
        config_file = tmp_path / "config.json"
        monkeypatch.setattr("agent.user_mcp_config.USER_CONFIG_FILE", config_file)
        monkeypatch.setattr("agent.user_mcp_config.CONFIG_DIR", tmp_path)
        
        save_user_mcp_config(config_data)
        
        content = config_file.read_text()
        # Check for indentation (pretty print)
        assert "  " in content or "\t" in content

    def test_save_config_write_error(self, tmp_path, monkeypatch):
        """Test handling of write errors"""
        import os
        
        # Skip this test if running as root (in Docker/CI) since root can write to read-only dirs
        if os.getuid() == 0:
            pytest.skip("Skipping permission test when running as root")
        
        config_file = tmp_path / "readonly_dir" / "config.json"
        readonly_dir = tmp_path / "readonly_dir"
        readonly_dir.mkdir()
        readonly_dir.chmod(0o444)  # Read-only directory
        
        monkeypatch.setattr("agent.user_mcp_config.USER_CONFIG_FILE", config_file)
        monkeypatch.setattr("agent.user_mcp_config.CONFIG_DIR", readonly_dir)
        
        config_data = {"test": "data"}
        result = save_user_mcp_config(config_data)
        
        assert result is False
        
        # Restore permissions for cleanup
        readonly_dir.chmod(0o755)


class TestMergeWithDefaults:
    """Tests for merging user config with defaults"""

    def test_merge_empty_user_config(self):
        """Test merging with empty user config returns only defaults"""
        default_config = {
            "server1": {
                "command": "default_cmd",
                "args": ["arg1"],
                "transport": "stdio"
            }
        }
        user_config = {}
        
        result = merge_with_defaults(user_config, default_config)
        
        assert result == default_config

    def test_merge_user_override(self):
        """Test that user config overrides defaults"""
        default_config = {
            "server1": {
                "command": "default_cmd",
                "args": ["arg1"],
                "transport": "stdio"
            }
        }
        user_config = {
            "server1": {
                "command": "user_cmd",
                "args": ["arg2"],
                "transport": "stdio"
            }
        }
        
        result = merge_with_defaults(user_config, default_config)
        
        assert result["server1"]["command"] == "user_cmd"
        assert result["server1"]["args"] == ["arg2"]

    def test_merge_adds_new_servers(self):
        """Test that user config can add new servers"""
        default_config = {
            "server1": {
                "command": "cmd1",
                "args": [],
                "transport": "stdio"
            }
        }
        user_config = {
            "server2": {
                "command": "cmd2",
                "args": ["arg"],
                "transport": "sse"
            }
        }
        
        result = merge_with_defaults(user_config, default_config)
        
        assert "server1" in result
        assert "server2" in result
        assert len(result) == 2

    def test_merge_filters_extra_fields(self):
        """Test that merge filters out non-MCP fields"""
        default_config = {
            "server1": {
                "command": "cmd",
                "args": [],
                "transport": "stdio"
            }
        }
        user_config = {
            "server1": {
                "command": "cmd",
                "args": [],
                "transport": "stdio",
                "enabled": True,  # UI field - should be filtered
                "name": "Server 1",  # UI field - should be filtered
                "is_default": False  # UI field - should be filtered
            }
        }
        
        result = merge_with_defaults(user_config, default_config)
        
        assert "enabled" not in result["server1"]
        assert "name" not in result["server1"]
        assert "is_default" not in result["server1"]
        assert "command" in result["server1"]
        assert "args" in result["server1"]
        assert "transport" in result["server1"]

    def test_merge_preserves_transport_types(self):
        """Test that different transport types are preserved"""
        default_config = {
            "stdio_server": {
                "command": "cmd",
                "args": [],
                "transport": "stdio"
            }
        }
        user_config = {
            "sse_server": {
                "command": "cmd",
                "args": [],
                "transport": "sse",
                "url": "http://example.com"
            }
        }
        
        result = merge_with_defaults(user_config, default_config)
        
        assert result["stdio_server"]["transport"] == "stdio"
        assert result["sse_server"]["transport"] == "sse"

    def test_merge_ros2_docker_injects_required_flags(self):
        default_config = {
            "ros2": {
                "command": "docker",
                "args": ["run", "-i", "--rm", "wisevision/ros2_mcp:jazzy"],
                "transport": "stdio",
            }
        }
        user_config = {
            "ros2": {
                "command": "docker",
                "args": ["run", "-i", "--rm", "wisevision/ros2_mcp:jazzy"],
                "transport": "stdio",
                "enabled": True,
                "name": "ROS2",
            }
        }

        import os
        os.environ["ROS_DOMAIN_ID"] = "0"
        result = merge_with_defaults(user_config, default_config)
        args = result["ros2"]["args"]

        assert "--network=host" in args
        assert "--pid=host" in args
        assert "--ipc=host" in args
        assert "-v" in args
        assert "/dev/shm:/dev/shm" in args
        assert "ROS_DOMAIN_ID=0" in args

    def test_merge_ros2_docker_can_target_specific_docker_network(self, monkeypatch):
        monkeypatch.setenv("ROS_DOMAIN_ID", "0")
        monkeypatch.setenv("ROS2_MCP_DOCKER_NETWORK", "wisevision_dashboard_default")

        default_config = {
            "ros2": {
                "command": "docker",
                "args": ["run", "-i", "--rm", "--network=host", "wisevision/ros2_mcp:jazzy"],
                "transport": "stdio",
            }
        }
        user_config = {
            "ros2": {
                "command": "docker",
                "args": ["run", "-i", "--rm", "--network=host", "wisevision/ros2_mcp:jazzy"],
                "transport": "stdio",
            }
        }

        result = merge_with_defaults(user_config, default_config)
        args = result["ros2"]["args"]
        assert "--network=wisevision_dashboard_default" in args
        assert "--network=host" not in args


class TestUtilityFunctions:
    """Tests for utility functions"""

    def test_get_config_file_path(self):
        """Test getting config file path"""
        path = get_config_file_path()
        
        assert isinstance(path, Path)
        assert path.name == ".mcp_user_config.json"

    def test_config_file_exists_true(self, tmp_path, monkeypatch):
        """Test config_file_exists when file exists"""
        config_file = tmp_path / "config.json"
        config_file.write_text("{}")
        monkeypatch.setattr("agent.user_mcp_config.USER_CONFIG_FILE", config_file)
        
        assert config_file_exists() is True

    def test_config_file_exists_false(self, tmp_path, monkeypatch):
        """Test config_file_exists when file doesn't exist"""
        config_file = tmp_path / "nonexistent.json"
        monkeypatch.setattr("agent.user_mcp_config.USER_CONFIG_FILE", config_file)
        
        assert config_file_exists() is False
