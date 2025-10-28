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
Pytest configuration and shared fixtures for server_ai_agent tests
"""
import pytest
import os
import sys
from pathlib import Path
from unittest.mock import MagicMock, AsyncMock

# Add src directory to Python path so imports work
src_path = Path(__file__).parent.parent / "src"
if str(src_path) not in sys.path:
    sys.path.insert(0, str(src_path))


@pytest.fixture(autouse=True)
def mock_environment():
    """Mock environment variables for testing"""
    original_env = os.environ.copy()
    
    # Set default test environment variables
    os.environ.setdefault("OPENAI_API_KEY", "sk-test-key-for-testing")
    os.environ.setdefault("BRIDGE_HOST", "0.0.0.0")
    os.environ.setdefault("BRIDGE_PORT", "8089")
    
    yield
    
    # Restore original environment
    os.environ.clear()
    os.environ.update(original_env)


@pytest.fixture
def mock_openai_api_key():
    """Fixture to provide a mock OpenAI API key"""
    return "sk-test-mock-api-key-123456"


@pytest.fixture
def sample_messages():
    """Fixture providing sample conversation messages"""
    return [
        {"role": "user", "content": "Hello, how are you?"},
        {"role": "assistant", "content": "I'm doing well, thank you! How can I help you today?"},
        {"role": "user", "content": "Can you help me with a task?"}
    ]


@pytest.fixture
def sample_mcp_config():
    """Fixture providing a sample MCP configuration"""
    return {
        "ros2": {
            "command": "docker",
            "args": [
                "run",
                "-i",
                "--rm",
                "--network=host",
                "wisevision/mcp_server_ros_2:humble"
            ],
            "transport": "stdio"
        },
        "filesystem": {
            "command": "mcp-server-filesystem",
            "args": ["/tmp/test"],
            "transport": "stdio"
        }
    }


@pytest.fixture
def mock_langchain_message():
    """Fixture to create mock LangChain message objects"""
    def _create_message(role="ai", content="Test response", tool_calls=None):
        msg = MagicMock()
        msg.type = role
        msg.content = content
        if tool_calls is not None:
            msg.tool_calls = tool_calls
        return msg
    return _create_message


@pytest.fixture
def mock_tool_call():
    """Fixture to create mock tool call objects"""
    def _create_tool_call(name="test_tool", args=None, call_id="call_123"):
        tool_call = MagicMock()
        tool_call.name = name
        tool_call.args = args or {}
        tool_call.id = call_id
        return tool_call
    return _create_tool_call


@pytest.fixture
async def mock_mcp_client():
    """Fixture to create a mock MCP client"""
    client = MagicMock()
    client.get_tools = AsyncMock(return_value=[])
    client.get_prompt = AsyncMock(return_value=[])
    
    # Mock session context manager
    mock_session = MagicMock()
    mock_session.__aenter__ = AsyncMock(return_value=mock_session)
    mock_session.__aexit__ = AsyncMock(return_value=None)
    mock_session.list_prompts = AsyncMock(return_value=MagicMock(prompts=[]))
    
    client.session = MagicMock(return_value=mock_session)
    
    return client


@pytest.fixture
def temp_config_file(tmp_path):
    """Fixture providing a temporary configuration file path"""
    config_file = tmp_path / ".mcp_user_config.json"
    return config_file


@pytest.fixture
def mock_chat_model():
    """Fixture to mock ChatOpenAI model"""
    model = MagicMock()
    model.ainvoke = AsyncMock(return_value=MagicMock(
        content="Mocked response",
        type="ai"
    ))
    return model


@pytest.fixture
def mock_graph():
    """Fixture to mock LangGraph graph"""
    graph = MagicMock()
    graph.ainvoke = AsyncMock(return_value={
        "messages": [
            {"role": "user", "content": "Test"},
            {"role": "assistant", "content": "Response"}
        ]
    })
    
    async def mock_astream_events(*args, **kwargs):
        """Mock streaming events generator"""
        yield {
            "event": "on_chat_model_stream",
            "data": {"chunk": MagicMock(content="Test")}
        }
        yield {"event": "done"}
    
    graph.astream_events = mock_astream_events
    
    return graph


@pytest.fixture(autouse=True)
def reset_global_state():
    """Reset global state between tests"""
    # Import here to avoid circular imports
    try:
        from bridge.main import sessions, pending_approvals
        sessions.clear()
        pending_approvals.clear()
    except ImportError:
        pass
    
    yield
    
    # Clean up after test
    try:
        from bridge.main import sessions, pending_approvals
        sessions.clear()
        pending_approvals.clear()
    except ImportError:
        pass


@pytest.fixture
def sample_user_config():
    """Fixture providing sample user MCP configuration"""
    return {
        "custom_server": {
            "command": "python",
            "args": ["-m", "custom_mcp_server"],
            "transport": "stdio",
            "enabled": True,
            "name": "Custom Server",
            "is_default": False
        }
    }


# Pytest configuration
def pytest_configure(config):
    """Configure pytest with custom markers"""
    config.addinivalue_line(
        "markers", "asyncio: mark test as async"
    )
    config.addinivalue_line(
        "markers", "slow: mark test as slow running"
    )
    config.addinivalue_line(
        "markers", "integration: mark test as integration test"
    )


# Enable asyncio mode for pytest-asyncio
@pytest.fixture(scope="session")
def event_loop_policy():
    """Set event loop policy for async tests"""
    import asyncio
    return asyncio.get_event_loop_policy()
