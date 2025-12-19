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

from langchain_mcp_adapters.client import MultiServerMCPClient


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

        async with self._global_lock:
            entry = self._entries.get(thread_id)
            if entry is None:
                entry = _ClientEntry(
                    client=MultiServerMCPClient(mcp_config),
                    tools=None,
                    config_fingerprint=fingerprint,
                    lock=asyncio.Lock(),
                    last_used=time.time(),
                )
                self._entries[thread_id] = entry
            elif entry.config_fingerprint != fingerprint:
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

        async with entry.lock:
            entry.last_used = time.time()
            if entry.tools is None:
                entry.tools = await entry.client.get_tools()
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


mcp_client_manager = MCPClientManager()

