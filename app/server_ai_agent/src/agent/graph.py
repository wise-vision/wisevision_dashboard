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
from typing import Optional, Dict, Any
from typing_extensions import Literal
from langchain_openai import ChatOpenAI
from langgraph.graph import StateGraph, END
from langgraph.prebuilt import create_react_agent
from langgraph.checkpoint.memory import MemorySaver
from langgraph.types import Command
from langchain_mcp_adapters.client import MultiServerMCPClient

from .state import AgentState
from .mcp_config import DEFAULT_MCP_CONFIG

async def chat_node(state: AgentState, _config: Dict[str, Any] | None = None) -> Command[Literal["__end__"]]:
    mcp_config: Optional[Dict[str, Any]] = state.get("mcp_config", DEFAULT_MCP_CONFIG)
    
    # Get API key from environment - try multiple sources
    api_key = (
        os.environ.get("OPENAI_API_KEY") or 
        state.get("openai_api_key") or
        (_config and _config.get("openai_api_key"))
    )
    
    if not api_key:
        raise ValueError("OpenAI API key not found. Please set OPENAI_API_KEY environment variable.")
    
    mcp_client = MultiServerMCPClient(mcp_config)
    tools = await mcp_client.get_tools()
    model = ChatOpenAI(model="gpt-4o", api_key=api_key)
    react_agent = create_react_agent(model, tools)
    resp = await react_agent.ainvoke({"messages": state["messages"]})
    updated = state["messages"] + resp.get("messages", [])
    return Command(goto=END, update={"messages": updated})

workflow = StateGraph(AgentState)
workflow.add_node("chat_node", chat_node)
workflow.set_entry_point("chat_node")
graph = workflow.compile(MemorySaver())