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
Tests for agent state module
"""
import pytest
from agent.state import AgentState


class TestAgentState:
    """Tests for AgentState TypedDict"""

    def test_agent_state_structure(self):
        """Test that AgentState has correct structure"""
        # Create a valid state
        state: AgentState = {
            "messages": [{"role": "user", "content": "Hello"}],
            "mcp_config": {"server": {"command": "test"}},
            "openai_api_key": "test-key"
        }
        
        assert "messages" in state
        assert "mcp_config" in state
        assert "openai_api_key" in state

    def test_agent_state_with_empty_messages(self):
        """Test AgentState with empty messages list"""
        state: AgentState = {
            "messages": [],
            "mcp_config": None,
            "openai_api_key": None
        }
        
        assert state["messages"] == []
        assert len(state["messages"]) == 0

    def test_agent_state_with_multiple_messages(self):
        """Test AgentState with multiple messages"""
        messages = [
            {"role": "user", "content": "First message"},
            {"role": "assistant", "content": "Response"},
            {"role": "user", "content": "Second message"}
        ]
        
        state: AgentState = {
            "messages": messages,
            "mcp_config": None,
            "openai_api_key": None
        }
        
        assert len(state["messages"]) == 3
        assert state["messages"][0]["role"] == "user"
        assert state["messages"][1]["role"] == "assistant"

    def test_agent_state_with_complex_mcp_config(self):
        """Test AgentState with complex MCP configuration"""
        mcp_config = {
            "ros2": {
                "command": "docker",
                "args": ["run", "-i", "--rm"],
                "transport": "stdio"
            },
            "filesystem": {
                "command": "mcp-server-filesystem",
                "args": ["/path/to/dir"],
                "transport": "stdio"
            }
        }
        
        state: AgentState = {
            "messages": [],
            "mcp_config": mcp_config,
            "openai_api_key": "test-key"
        }
        
        assert "ros2" in state["mcp_config"]
        assert "filesystem" in state["mcp_config"]
        assert state["mcp_config"]["ros2"]["command"] == "docker"

    def test_agent_state_with_none_values(self):
        """Test AgentState with None values for optional fields"""
        state: AgentState = {
            "messages": [{"role": "user", "content": "test"}],
            "mcp_config": None,
            "openai_api_key": None
        }
        
        assert state["mcp_config"] is None
        assert state["openai_api_key"] is None

    def test_agent_state_messages_format(self):
        """Test various message formats in AgentState"""
        messages = [
            {"role": "user", "content": "Simple text"},
            {"role": "assistant", "content": "Response", "tool_calls": []},
            {"role": "tool", "content": "Tool output", "tool_call_id": "123"}
        ]
        
        state: AgentState = {
            "messages": messages,
            "mcp_config": None,
            "openai_api_key": None
        }
        
        assert len(state["messages"]) == 3
        assert "tool_calls" in state["messages"][1]
        assert "tool_call_id" in state["messages"][2]

    def test_agent_state_api_key_types(self):
        """Test different API key values"""
        # String API key
        state1: AgentState = {
            "messages": [],
            "mcp_config": None,
            "openai_api_key": "sk-test123"
        }
        assert isinstance(state1["openai_api_key"], str)
        
        # None API key
        state2: AgentState = {
            "messages": [],
            "mcp_config": None,
            "openai_api_key": None
        }
        assert state2["openai_api_key"] is None

    def test_agent_state_mcp_config_transport_types(self):
        """Test different MCP transport types in config"""
        stdio_config = {
            "server1": {
                "command": "cmd",
                "args": ["arg1"],
                "transport": "stdio"
            }
        }
        
        sse_config = {
            "server2": {
                "url": "http://localhost:8080",
                "transport": "sse"
            }
        }
        
        state1: AgentState = {
            "messages": [],
            "mcp_config": stdio_config,
            "openai_api_key": None
        }
        assert state1["mcp_config"]["server1"]["transport"] == "stdio"
        
        state2: AgentState = {
            "messages": [],
            "mcp_config": sse_config,
            "openai_api_key": None
        }
        assert state2["mcp_config"]["server2"]["transport"] == "sse"
