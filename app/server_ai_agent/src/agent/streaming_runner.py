# src/agent/streaming_runner.py
from typing import Any, AsyncGenerator
from .graph import graph
from .mcp_config import DEFAULT_MCP_CONFIG
import os
import json

async def stream_graph(
    messages: list[dict[str, Any]],
    mcp_config: dict[str, Any] | None = None,
    thread_id: str | None = None,
    openai_api_key: str | None = None,
    require_approval: bool = False,
) -> AsyncGenerator[dict[str, Any], None]:
    """
    Stream graph execution with real-time events.
    Yields events as they happen.
    """
    final_api_key = openai_api_key or os.environ.get("OPENAI_API_KEY")
    
    state = {
        "messages": messages, 
        "mcp_config": mcp_config if mcp_config is not None else DEFAULT_MCP_CONFIG,
        "openai_api_key": final_api_key
    }
    cfg = {"configurable": {"thread_id": thread_id or "stream-session"}}
    
    # Stream events from the graph
    async for event in graph.astream_events(state, config=cfg, version="v2"):
        event_type = event.get("event")
        
        # Chat model events
        if event_type == "on_chat_model_stream":
            # Streaming token from LLM
            chunk = event.get("data", {}).get("chunk")
            if chunk and hasattr(chunk, 'content') and chunk.content:
                yield {
                    "type": "token",
                    "content": chunk.content
                }
        
        # Tool call events
        elif event_type == "on_chat_model_end":
            # Check if there are tool calls
            output = event.get("data", {}).get("output")
            if output and hasattr(output, 'tool_calls') and output.tool_calls:
                tool_calls_data = []
                for tc in output.tool_calls:
                    # Try different ways to extract tool info
                    tool_name = None
                    tool_args = {}
                    tool_id = None
                    
                    # Method 1: Direct attributes
                    if hasattr(tc, 'name'):
                        tool_name = tc.name
                    if hasattr(tc, 'args'):
                        tool_args = tc.args
                    if hasattr(tc, 'id'):
                        tool_id = tc.id
                    
                    # Method 2: Dict-like access
                    if tool_name is None and isinstance(tc, dict):
                        tool_name = tc.get('name')
                        tool_args = tc.get('args', {})
                        tool_id = tc.get('id')
                    
                    # Method 3: Check for 'function' attribute (OpenAI format)
                    if tool_name is None and hasattr(tc, 'function'):
                        tool_name = getattr(tc.function, 'name', None)
                        tool_args = getattr(tc.function, 'arguments', {})
                    
                    tool_call_info = {
                        "name": tool_name or 'unknown',
                        "args": tool_args or {},
                        "id": tool_id
                    }
                    tool_calls_data.append(tool_call_info)
                
                yield {
                    "type": "tool_calls",
                    "tool_calls": tool_calls_data,
                    "require_approval": require_approval
                }
        
        # Tool execution events
        elif event_type == "on_tool_start":
            tool_name = event.get("name", "unknown")
            tool_input = event.get("data", {}).get("input", {})
            yield {
                "type": "tool_start",
                "tool_name": tool_name,
                "tool_input": tool_input
            }
        
        elif event_type == "on_tool_end":
            tool_name = event.get("name", "unknown")
            tool_output = event.get("data", {}).get("output")
            yield {
                "type": "tool_end",
                "tool_name": tool_name,
                "tool_output": str(tool_output)
            }
    
    # Final event
    yield {
        "type": "done"
    }
