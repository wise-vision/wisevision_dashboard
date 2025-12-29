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
import json
from typing import Optional, Any
from typing_extensions import Literal
from langchain_openai import ChatOpenAI
from langgraph.graph import StateGraph, END
from langgraph.prebuilt import create_react_agent
from langgraph.checkpoint.memory import MemorySaver
from langgraph.types import Command

from .state import AgentState
from .mcp_config import DEFAULT_MCP_CONFIG
from .mcp_client_manager import mcp_client_manager

# Cache for react agents per session
# Key: f"{thread_id}_{mcp_config_fingerprint}_{api_key_fingerprint}"
_agent_cache: dict[str, Any] = {}

def _fingerprint_config(mcp_config: dict[str, Any]) -> str:
    try:
        return json.dumps(mcp_config, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    except TypeError:
        return json.dumps(mcp_config, default=str, sort_keys=True, separators=(",", ":"), ensure_ascii=True)

def _fingerprint_api_key(api_key: Optional[str]) -> str:
    """Create a fingerprint of the API key (just first 8 chars for security)"""
    if not api_key:
        return "none"
    return api_key[:8] if len(api_key) > 8 else api_key

async def chat_node(state: AgentState, _config: dict[str, Any] | None = None) -> Command[Literal["__end__"]]:
    mcp_config: Optional[dict[str, Any]] = state.get("mcp_config", DEFAULT_MCP_CONFIG)
    thread_id = ((_config or {}).get("configurable") or {}).get("thread_id") or "default-session"
    
    # Get API key from environment - try multiple sources
    api_key = (
        os.environ.get("OPENAI_API_KEY") or 
        state.get("openai_api_key") or
        (_config and _config.get("openai_api_key"))
    )
    
    if not api_key:
        raise ValueError("OpenAI API key not found. Please set OPENAI_API_KEY environment variable.")
    
    # Get tools from manager (cached per thread_id)
    tools = await mcp_client_manager.get_tools(thread_id, mcp_config or {})
    
    # Create cache key for this agent configuration
    mcp_fingerprint = _fingerprint_config(mcp_config or {})
    api_key_fingerprint = _fingerprint_api_key(api_key)
    cache_key = f"{thread_id}_{mcp_fingerprint}_{api_key_fingerprint}"
    
    # Check if we already have a cached agent for this configuration
    if cache_key not in _agent_cache:
        model = ChatOpenAI(model="gpt-4o", api_key=api_key)
        react_agent = create_react_agent(model, tools)
        _agent_cache[cache_key] = react_agent
    else:
        react_agent = _agent_cache[cache_key]
    
    resp = await react_agent.ainvoke({"messages": state["messages"]})
    updated = state["messages"] + resp.get("messages", [])
    return Command(goto=END, update={"messages": updated})

workflow = StateGraph(AgentState)
workflow.add_node("chat_node", chat_node)
workflow.set_entry_point("chat_node")
graph = workflow.compile(MemorySaver())
