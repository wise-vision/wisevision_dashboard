#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

from typing import Optional, Any
from .graph import graph
from .mcp_config import DEFAULT_MCP_CONFIG
import os

def _default_thread_id() -> str:
    return os.environ.get("THREAD_ID", "local-test")

async def run_graph(
    messages: list[dict[str, Any]],
    mcp_config: Optional[dict[str, Any]] = None,
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
