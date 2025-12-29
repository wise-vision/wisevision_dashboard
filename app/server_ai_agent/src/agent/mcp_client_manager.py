#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

from __future__ import annotations

import asyncio
import json
import time
from dataclasses import dataclass
from typing import Any

import os

try:
    from langchain_mcp_adapters.client import MultiServerMCPClient
    from agent.persistent_stdio_session import persistent_stdio_manager
    
    # Monkeypatch langchain_mcp_adapters to use persistent stdio sessions
    import langchain_mcp_adapters.sessions
    from contextlib import asynccontextmanager
    
    # Store the original function
    _original_create_stdio_session = langchain_mcp_adapters.sessions._create_stdio_session
    
    # Replace with our persistent version
    @asynccontextmanager
    async def _persistent_create_stdio_session(*args, **kwargs):
        """Wrapper that uses persistent stdio manager instead of creating new sessions."""
        async with persistent_stdio_manager.get_session(*args, **kwargs) as session:
            yield session
    
    # Apply the monkeypatch
    langchain_mcp_adapters.sessions._create_stdio_session = _persistent_create_stdio_session
    print("[MCP_CLIENT_MANAGER] Applied monkeypatch for persistent stdio sessions")
    
except ModuleNotFoundError as e:
    _import_error = e

    class MultiServerMCPClient:  # type: ignore[no-redef]
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            raise RuntimeError(
                "Missing optional dependency `langchain_mcp_adapters`. "
                "Install server_ai_agent dependencies to use MCP tooling."
            ) from _import_error


async def _best_effort_close(client: Any) -> None:
    close_fn = getattr(client, "aclose", None) or getattr(client, "close", None)
    if close_fn is None:
        return
    res = close_fn()
    if asyncio.iscoroutine(res):
        await res


def _fingerprint_config(mcp_config: dict[str, Any]) -> str:
    try:
        return json.dumps(mcp_config, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    except TypeError:
        return json.dumps(mcp_config, default=str, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


@dataclass
class _ClientEntry:
    client: MultiServerMCPClient
    tools: Any | None
    config_fingerprint: str
    lock: asyncio.Lock
    last_used: float
    client_started: bool = False  # Track if client was started with __aenter__


class MCPClientManager:
    """
    Keeps MCP server processes (e.g. `docker run ... --rm`) alive per chat session/thread.

    Without this, a new MultiServerMCPClient (and thus a new docker container) is created
    on every request, which is expensive and breaks stateful MCP servers.
    """

    def __init__(self) -> None:
        self._entries: dict[str, _ClientEntry] = {}
        self._global_lock = asyncio.Lock()

    async def get_tools(self, thread_id: str, mcp_config: dict[str, Any]) -> Any:
        fingerprint = _fingerprint_config(mcp_config)
        
        # DEBUG: Log fingerprint and thread_id
        print(f"[MCP_CLIENT_MANAGER] get_tools called: thread_id={thread_id}, fingerprint={fingerprint[:50]}...")

        async with self._global_lock:
            entry = self._entries.get(thread_id)
            if entry is None:
                print(f"[MCP_CLIENT_MANAGER] Creating NEW client for thread_id={thread_id}")
                client = MultiServerMCPClient(mcp_config)
                
                entry = _ClientEntry(
                    client=client,
                    tools=None,
                    config_fingerprint=fingerprint,
                    lock=asyncio.Lock(),
                    last_used=time.time(),
                    client_started=False,
                )
                self._entries[thread_id] = entry
            elif entry.config_fingerprint != fingerprint:
                print(f"[MCP_CLIENT_MANAGER] Config CHANGED for thread_id={thread_id}")
                print(f"[MCP_CLIENT_MANAGER]   Old fingerprint: {entry.config_fingerprint[:50]}...")
                print(f"[MCP_CLIENT_MANAGER]   New fingerprint: {fingerprint[:50]}...")
                old_entry = self._entries.pop(thread_id)
                await _best_effort_close(old_entry.client)
                entry = _ClientEntry(
                    client=MultiServerMCPClient(mcp_config),
                    tools=None,
                    config_fingerprint=fingerprint,
                    lock=asyncio.Lock(),
                    last_used=time.time(),
                )
                self._entries[thread_id] = entry
            else:
                print(f"[MCP_CLIENT_MANAGER] Reusing EXISTING client for thread_id={thread_id}")

        async with entry.lock:
            entry.last_used = time.time()
            if entry.tools is None:
                print(f"[MCP_CLIENT_MANAGER] Tools are None, fetching from client...")
                entry.tools = await entry.client.get_tools()
                print(f"[MCP_CLIENT_MANAGER] Fetched {len(entry.tools)} tools")
            else:
                print(f"[MCP_CLIENT_MANAGER] Tools already cached ({len(entry.tools)} tools)")
            return entry.tools

    async def close(self, thread_id: str) -> None:
        async with self._global_lock:
            entry = self._entries.pop(thread_id, None)
        if entry is None:
            return
        async with entry.lock:
            await _best_effort_close(entry.client)

    async def close_all(self) -> None:
        async with self._global_lock:
            entries = list(self._entries.items())
            self._entries.clear()

        for _, entry in entries:
            async with entry.lock:
                await _best_effort_close(entry.client)
        
        # Also close all persistent stdio sessions
        await persistent_stdio_manager.close_all()
    
    def clear_agent_cache_for_thread(self, thread_id: str) -> None:
        """Clear cached agents for a specific thread when session is closed"""
        # Import here to avoid circular imports
        from .graph import _agent_cache
        
        # Remove all cache entries for this thread_id
        keys_to_remove = [k for k in _agent_cache.keys() if k.startswith(f"{thread_id}_")]
        for key in keys_to_remove:
            del _agent_cache[key]


mcp_client_manager = MCPClientManager()
