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
Persistent stdio session manager for MCP.

This module provides a way to keep MCP stdio processes alive across multiple
tool invocations, avoiding the overhead of restarting containers.
"""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import Any, AsyncIterator
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


class PersistentStdioSessionManager:
    """Manager for persistent stdio sessions to MCP servers."""
    
    def __init__(self):
        self._sessions: dict[str, tuple[Any, Any, ClientSession]] = {}
        self._locks: dict[str, asyncio.Lock] = {}
    
    def _get_cache_key(self, server_params: StdioServerParameters) -> str:
        """Generate a cache key for the server parameters."""
        import json
        # Create a deterministic key from the server parameters
        key_data = {
            "command": server_params.command,
            "args": tuple(server_params.args) if server_params.args else (),
            "env": tuple(sorted(server_params.env.items())) if server_params.env else (),
            "cwd": str(server_params.cwd) if server_params.cwd else None,
        }
        return json.dumps(key_data, sort_keys=True)
    
    @asynccontextmanager
    async def get_session(
        self,
        *,
        command: str,
        args: list[str],
        env: dict[str, str] | None = None,
        cwd: str | Path | None = None,
        encoding: str = "utf-8",
        encoding_error_handler: str = "strict",
        session_kwargs: dict[str, Any] | None = None,
    ) -> AsyncIterator[ClientSession]:
        """Get or create a persistent session.
        
        This maintains the subprocess alive across multiple calls,
        avoiding the overhead of restarting containers.
        """
        server_params = StdioServerParameters(
            command=command,
            args=args,
            env=env,
            cwd=cwd,
            encoding=encoding,
            encoding_error_handler=encoding_error_handler,
        )
        
        cache_key = self._get_cache_key(server_params)
        
        # Ensure we have a lock for this cache key
        if cache_key not in self._locks:
            self._locks[cache_key] = asyncio.Lock()
        
        async with self._locks[cache_key]:
            # Check if we already have a session
            if cache_key in self._sessions:
                print(f"[PERSISTENT_STDIO] Reusing existing stdio session for {command}")
                _, _, session = self._sessions[cache_key]
                yield session
            else:
                print(f"[PERSISTENT_STDIO] Creating NEW stdio session for {command}")
                # Create a new session and cache it
                # Note: We don't use 'async with' here to keep it alive
                read_write_context = stdio_client(server_params)
                read, write = await read_write_context.__aenter__()
                
                session_context = ClientSession(read, write, **(session_kwargs or {}))
                session = await session_context.__aenter__()
                
                # Store the contexts and session so we can close them later
                self._sessions[cache_key] = (read_write_context, session_context, session)
                
                yield session
    
    async def close_session(self, cache_key: str) -> None:
        """Close a specific session."""
        if cache_key in self._sessions:
            print(f"[PERSISTENT_STDIO] Closing stdio session {cache_key}")
            read_write_context, session_context, _ = self._sessions.pop(cache_key)
            try:
                await session_context.__aexit__(None, None, None)
            except Exception as e:
                print(f"[PERSISTENT_STDIO] Error closing session context: {e}")
            try:
                await read_write_context.__aexit__(None, None, None)
            except Exception as e:
                print(f"[PERSISTENT_STDIO] Error closing read/write context: {e}")
    
    async def close_all(self) -> None:
        """Close all cached sessions."""
        print(f"[PERSISTENT_STDIO] Closing all {len(self._sessions)} stdio sessions")
        cache_keys = list(self._sessions.keys())
        for cache_key in cache_keys:
            await self.close_session(cache_key)


# Global manager instance
persistent_stdio_manager = PersistentStdioSessionManager()
