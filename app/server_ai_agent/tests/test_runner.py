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
Tests for agent runner module
"""
import pytest
import os
from unittest.mock import AsyncMock, MagicMock, patch
from agent.runner import run_graph, _default_thread_id


class TestDefaultThreadId:
    """Tests for _default_thread_id function"""

    def test_default_thread_id_from_env(self, monkeypatch):
        """Test getting thread ID from environment variable"""
        monkeypatch.setenv("THREAD_ID", "custom-thread-123")
        
        thread_id = _default_thread_id()
        
        assert thread_id == "custom-thread-123"

    def test_default_thread_id_fallback(self, monkeypatch):
        """Test fallback thread ID when env var not set"""
        monkeypatch.delenv("THREAD_ID", raising=False)
        
        thread_id = _default_thread_id()
        
        assert thread_id == "local-test"


@pytest.mark.asyncio
class TestRunGraph:
    """Tests for run_graph function"""

    async def test_run_graph_basic_call(self):
        """Test basic run_graph call with minimal parameters"""
        messages = [{"role": "user", "content": "Hello"}]
        
        with patch("agent.runner.graph") as mock_graph:
            mock_graph.ainvoke = AsyncMock(return_value={
                "messages": messages + [{"role": "assistant", "content": "Hi"}]
            })
            
            result = await run_graph(messages)
            
            assert mock_graph.ainvoke.called
            assert "messages" in result

    async def test_run_graph_with_mcp_config(self):
        """Test run_graph with custom MCP configuration"""
        messages = [{"role": "user", "content": "Test"}]
        mcp_config = {
            "ros2": {
                "command": "docker",
                "args": ["run"],
                "transport": "stdio"
            }
        }
        
        with patch("agent.runner.graph") as mock_graph:
            mock_graph.ainvoke = AsyncMock(return_value={"messages": messages})
            
            result = await run_graph(messages, mcp_config=mcp_config)
            
            # Check that ainvoke was called with correct state
            call_args = mock_graph.ainvoke.call_args
            state = call_args[0][0]
            
            assert state["mcp_config"] == mcp_config
            assert state["messages"] == messages

    async def test_run_graph_with_thread_id(self):
        """Test run_graph with custom thread ID"""
        messages = [{"role": "user", "content": "Test"}]
        custom_thread_id = "my-thread-456"
        
        with patch("agent.runner.graph") as mock_graph:
            mock_graph.ainvoke = AsyncMock(return_value={"messages": messages})
            
            await run_graph(messages, thread_id=custom_thread_id)
            
            # Check config passed to ainvoke
            call_args = mock_graph.ainvoke.call_args
            config = call_args[0][1] if len(call_args[0]) > 1 else call_args[1]["config"]
            
            assert config["configurable"]["thread_id"] == custom_thread_id

    async def test_run_graph_with_openai_key(self):
        """Test run_graph with OpenAI API key"""
        messages = [{"role": "user", "content": "Test"}]
        api_key = "sk-test123"
        
        with patch("agent.runner.graph") as mock_graph:
            mock_graph.ainvoke = AsyncMock(return_value={"messages": messages})
            
            result = await run_graph(messages, openai_api_key=api_key)
            
            # Check state contains API key
            call_args = mock_graph.ainvoke.call_args
            state = call_args[0][0]
            
            assert state["openai_api_key"] == api_key

    async def test_run_graph_openai_key_from_env(self, monkeypatch):
        """Test run_graph gets OpenAI key from environment"""
        monkeypatch.setenv("OPENAI_API_KEY", "sk-env-key")
        messages = [{"role": "user", "content": "Test"}]
        
        with patch("agent.runner.graph") as mock_graph:
            mock_graph.ainvoke = AsyncMock(return_value={"messages": messages})
            
            await run_graph(messages)
            
            # Check state has API key from environment
            call_args = mock_graph.ainvoke.call_args
            state = call_args[0][0]
            
            assert state["openai_api_key"] == "sk-env-key"

    async def test_run_graph_uses_default_mcp_config(self):
        """Test run_graph uses DEFAULT_MCP_CONFIG when none provided"""
        messages = [{"role": "user", "content": "Test"}]
        
        with patch("agent.runner.graph") as mock_graph:
            with patch("agent.runner.DEFAULT_MCP_CONFIG", {"default": "config"}):
                mock_graph.ainvoke = AsyncMock(return_value={"messages": messages})
                
                await run_graph(messages, mcp_config=None)
                
                # Check state has default config
                call_args = mock_graph.ainvoke.call_args
                state = call_args[0][0]
                
                assert state["mcp_config"] == {"default": "config"}

    async def test_run_graph_returns_updated_messages(self):
        """Test run_graph returns updated messages from graph"""
        initial_messages = [{"role": "user", "content": "Hello"}]
        updated_messages = initial_messages + [
            {"role": "assistant", "content": "Hi there!"}
        ]
        
        with patch("agent.runner.graph") as mock_graph:
            mock_graph.ainvoke = AsyncMock(return_value={
                "messages": updated_messages
            })
            
            result = await run_graph(initial_messages)
            
            assert len(result["messages"]) == 2
            assert result["messages"][1]["role"] == "assistant"
            assert result["messages"][1]["content"] == "Hi there!"

    async def test_run_graph_empty_messages(self):
        """Test run_graph with empty messages list"""
        messages = []
        
        with patch("agent.runner.graph") as mock_graph:
            mock_graph.ainvoke = AsyncMock(return_value={"messages": []})
            
            result = await run_graph(messages)
            
            assert result["messages"] == []
            assert mock_graph.ainvoke.called

    async def test_run_graph_multiple_messages(self):
        """Test run_graph with conversation history"""
        messages = [
            {"role": "user", "content": "First message"},
            {"role": "assistant", "content": "First response"},
            {"role": "user", "content": "Second message"}
        ]
        
        with patch("agent.runner.graph") as mock_graph:
            mock_graph.ainvoke = AsyncMock(return_value={"messages": messages})
            
            result = await run_graph(messages)
            
            assert len(result["messages"]) == 3
            assert mock_graph.ainvoke.called

    async def test_run_graph_exception_propagation(self):
        """Test that exceptions from graph are propagated"""
        messages = [{"role": "user", "content": "Test"}]
        
        with patch("agent.runner.graph") as mock_graph:
            mock_graph.ainvoke = AsyncMock(side_effect=ValueError("Test error"))
            
            with pytest.raises(ValueError, match="Test error"):
                await run_graph(messages)
