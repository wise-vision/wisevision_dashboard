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
Tests for streaming runner module
"""
import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from agent.streaming_runner import stream_graph


@pytest.mark.asyncio
class TestStreamGraph:
    """Tests for stream_graph async generator"""

    async def test_stream_graph_basic_streaming(self):
        """Test basic streaming functionality"""
        messages = [{"role": "user", "content": "Hello"}]
        
        # Create mock events
        mock_events = [
            {
                "event": "on_chat_model_stream",
                "data": {
                    "chunk": MagicMock(content="Hello")
                }
            },
            {
                "event": "on_chat_model_stream",
                "data": {
                    "chunk": MagicMock(content=" world")
                }
            }
        ]
        
        async def mock_astream_events(*args, **kwargs):
            for event in mock_events:
                yield event
        
        with patch("agent.streaming_runner.graph") as mock_graph:
            mock_graph.astream_events = mock_astream_events
            
            events = []
            async for event in stream_graph(messages):
                events.append(event)
            
            # Should get token events plus done event
            token_events = [e for e in events if e["type"] == "token"]
            assert len(token_events) == 2
            assert token_events[0]["content"] == "Hello"
            assert token_events[1]["content"] == " world"
            
            # Should have done event at end
            assert events[-1]["type"] == "done"

    async def test_stream_graph_with_tool_calls(self):
        """Test streaming with tool calls"""
        messages = [{"role": "user", "content": "Test"}]
        
        mock_tool_call = MagicMock()
        mock_tool_call.name = "test_tool"
        mock_tool_call.args = {"param": "value"}
        mock_tool_call.id = "call_123"
        
        mock_events = [
            {
                "event": "on_chat_model_end",
                "data": {
                    "output": MagicMock(tool_calls=[mock_tool_call])
                }
            },
            {
                "event": "on_tool_start",
                "name": "test_tool",
                "data": {
                    "input": {"param": "value"}
                }
            },
            {
                "event": "on_tool_end",
                "name": "test_tool",
                "data": {
                    "output": "tool result"
                }
            }
        ]
        
        async def mock_astream_events(*args, **kwargs):
            for event in mock_events:
                yield event
        
        with patch("agent.streaming_runner.graph") as mock_graph:
            mock_graph.astream_events = mock_astream_events
            
            events = []
            async for event in stream_graph(messages):
                events.append(event)
            
            # Check tool_calls event
            tool_calls_events = [e for e in events if e["type"] == "tool_calls"]
            assert len(tool_calls_events) == 1
            assert len(tool_calls_events[0]["tool_calls"]) == 1
            assert tool_calls_events[0]["tool_calls"][0]["name"] == "test_tool"
            
            # Check tool execution events
            tool_start_events = [e for e in events if e["type"] == "tool_start"]
            assert len(tool_start_events) == 1
            assert tool_start_events[0]["tool_name"] == "test_tool"
            
            tool_end_events = [e for e in events if e["type"] == "tool_end"]
            assert len(tool_end_events) == 1
            assert tool_end_events[0]["tool_output"] == "tool result"

    async def test_stream_graph_with_mcp_config(self):
        """Test stream_graph with custom MCP config"""
        messages = [{"role": "user", "content": "Test"}]
        mcp_config = {"ros2": {"command": "test"}}
        
        async def mock_astream_events(*args, **kwargs):
            # Check that state was passed correctly
            state = args[0]
            assert state["mcp_config"] == mcp_config
            yield {"event": "done"}
        
        with patch("agent.streaming_runner.graph") as mock_graph:
            mock_graph.astream_events = mock_astream_events
            
            events = []
            async for event in stream_graph(messages, mcp_config=mcp_config):
                events.append(event)
            
            assert len(events) == 1  # Just the done event

    async def test_stream_graph_with_openai_key(self):
        """Test stream_graph with OpenAI API key"""
        messages = [{"role": "user", "content": "Test"}]
        api_key = "sk-test123"
        
        async def mock_astream_events(*args, **kwargs):
            state = args[0]
            assert state["openai_api_key"] == api_key
            yield {"event": "done"}
        
        with patch("agent.streaming_runner.graph") as mock_graph:
            mock_graph.astream_events = mock_astream_events
            
            events = []
            async for event in stream_graph(messages, openai_api_key=api_key):
                events.append(event)
            
            assert len(events) == 1

    async def test_stream_graph_with_thread_id(self):
        """Test stream_graph with custom thread ID"""
        messages = [{"role": "user", "content": "Test"}]
        thread_id = "custom-thread"
        
        async def mock_astream_events(*args, **kwargs):
            config = kwargs.get("config", {})
            assert config["configurable"]["thread_id"] == thread_id
            yield {"event": "done"}
        
        with patch("agent.streaming_runner.graph") as mock_graph:
            mock_graph.astream_events = mock_astream_events
            
            events = []
            async for event in stream_graph(messages, thread_id=thread_id):
                events.append(event)
            
            assert len(events) == 1

    async def test_stream_graph_require_approval(self):
        """Test stream_graph with require_approval flag"""
        messages = [{"role": "user", "content": "Test"}]
        
        mock_tool_call = MagicMock()
        mock_tool_call.name = "dangerous_tool"
        mock_tool_call.args = {}
        mock_tool_call.id = "call_123"
        
        mock_events = [
            {
                "event": "on_chat_model_end",
                "data": {
                    "output": MagicMock(tool_calls=[mock_tool_call])
                }
            }
        ]
        
        async def mock_astream_events(*args, **kwargs):
            for event in mock_events:
                yield event
        
        with patch("agent.streaming_runner.graph") as mock_graph:
            mock_graph.astream_events = mock_astream_events
            
            events = []
            async for event in stream_graph(messages, require_approval=True):
                events.append(event)
            
            # Check that require_approval is in the event
            tool_calls_events = [e for e in events if e["type"] == "tool_calls"]
            assert len(tool_calls_events) == 1
            assert tool_calls_events[0]["require_approval"] is True

    async def test_stream_graph_empty_content_filtered(self):
        """Test that empty content chunks are filtered"""
        messages = [{"role": "user", "content": "Test"}]
        
        mock_events = [
            {
                "event": "on_chat_model_stream",
                "data": {
                    "chunk": MagicMock(content="")  # Empty content
                }
            },
            {
                "event": "on_chat_model_stream",
                "data": {
                    "chunk": MagicMock(content="Valid content")
                }
            }
        ]
        
        async def mock_astream_events(*args, **kwargs):
            for event in mock_events:
                yield event
        
        with patch("agent.streaming_runner.graph") as mock_graph:
            mock_graph.astream_events = mock_astream_events
            
            events = []
            async for event in stream_graph(messages):
                events.append(event)
            
            # Should only get one token event (empty filtered out)
            token_events = [e for e in events if e["type"] == "token"]
            assert len(token_events) == 1
            assert token_events[0]["content"] == "Valid content"

    async def test_stream_graph_tool_call_dict_format(self):
        """Test handling tool calls in dict format"""
        messages = [{"role": "user", "content": "Test"}]
        
        mock_tool_call = {
            "name": "dict_tool",
            "args": {"key": "value"},
            "id": "call_456"
        }
        
        mock_events = [
            {
                "event": "on_chat_model_end",
                "data": {
                    "output": MagicMock(tool_calls=[mock_tool_call])
                }
            }
        ]
        
        async def mock_astream_events(*args, **kwargs):
            for event in mock_events:
                yield event
        
        with patch("agent.streaming_runner.graph") as mock_graph:
            mock_graph.astream_events = mock_astream_events
            
            events = []
            async for event in stream_graph(messages):
                events.append(event)
            
            tool_calls_events = [e for e in events if e["type"] == "tool_calls"]
            assert len(tool_calls_events) == 1
            assert tool_calls_events[0]["tool_calls"][0]["name"] == "dict_tool"
            assert tool_calls_events[0]["tool_calls"][0]["args"]["key"] == "value"

    async def test_stream_graph_always_ends_with_done(self):
        """Test that stream always ends with done event"""
        messages = [{"role": "user", "content": "Test"}]
        
        async def mock_astream_events(*args, **kwargs):
            yield {
                "event": "on_chat_model_stream",
                "data": {"chunk": MagicMock(content="test")}
            }
        
        with patch("agent.streaming_runner.graph") as mock_graph:
            mock_graph.astream_events = mock_astream_events
            
            events = []
            async for event in stream_graph(messages):
                events.append(event)
            
            # Last event should always be done
            assert events[-1]["type"] == "done"

    async def test_stream_graph_uses_default_config(self):
        """Test stream_graph uses DEFAULT_MCP_CONFIG when none provided"""
        messages = [{"role": "user", "content": "Test"}]
        
        async def mock_astream_events(*args, **kwargs):
            state = args[0]
            # Should have default config
            assert "mcp_config" in state
            yield {"event": "done"}
        
        with patch("agent.streaming_runner.graph") as mock_graph:
            with patch("agent.streaming_runner.DEFAULT_MCP_CONFIG", {"default": "config"}):
                mock_graph.astream_events = mock_astream_events
                
                events = []
                async for event in stream_graph(messages):
                    events.append(event)
                
                assert len(events) == 1
