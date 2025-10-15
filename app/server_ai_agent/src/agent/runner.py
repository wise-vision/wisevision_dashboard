# src/agent/runner.py
from typing import Optional, Dict, Any, List
from .graph import graph
from .mcp_config import DEFAULT_MCP_CONFIG
import os

def _default_thread_id() -> str:
    return os.environ.get("THREAD_ID", "local-test")

async def run_graph(
    messages: List[Dict[str, Any]],
    mcp_config: Optional[Dict[str, Any]] = None,
    thread_id: Optional[str] = None,
    openai_api_key: Optional[str] = None,
):
    final_api_key = openai_api_key or os.environ.get("OPENAI_API_KEY")
    
    state = {
        "messages": messages, 
        "mcp_config": mcp_config if mcp_config is not None else DEFAULT_MCP_CONFIG,
        "openai_api_key": final_api_key
    }
    cfg = {"configurable": {"thread_id": thread_id or _default_thread_id()}}
    return await graph.ainvoke(state, config=cfg)