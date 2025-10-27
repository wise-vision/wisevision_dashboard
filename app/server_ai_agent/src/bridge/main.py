#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

import asyncio
import json
import os
from typing import Any, Optional

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import BaseModel

from agent.runner import run_graph
from agent.streaming_runner import stream_graph
from agent.mcp_config import DEFAULT_MCP_CONFIG
from agent.user_mcp_config import load_user_mcp_config, save_user_mcp_config, merge_with_defaults
from langchain_mcp_adapters.client import MultiServerMCPClient


class Settings(BaseSettings):
    bridge_host: str = "0.0.0.0"  # Listen on all interfaces
    bridge_port: int = 8089
    cors_origins: str = "" 
    openai_api_key: str = ""  # OpenAI API key

    # pydantic v2:
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        env_prefix="",
        extra="ignore",
    )

settings = Settings()

# Prepare CORS origins from environment or use defaults
cors_origins_list = []
if settings.cors_origins:
    cors_origins_list = [origin.strip() for origin in settings.cors_origins.split(",")]
else:
    # Default CORS origins for development
    cors_origins_list = [
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://172.16.14.133:3000",
        "http://0.0.0.0:3000",
        "*"  # Allow all origins for development
    ]

app = FastAPI(title="MCP Bridge")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins_list,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)

# Load user MCP configuration on startup
USER_MCP_CONFIG = load_user_mcp_config()

class Session:
    def __init__(self, cfg: dict[str, Any], openai_api_key: str | None = None):
        self.id = os.urandom(6).hex()
        self.mcp_config = cfg
        self.messages: list[dict[str, Any]] = []
        self.queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue()
        self.openai_api_key = openai_api_key

sessions: dict[str, Session] = {}

class CreateSessionBody(BaseModel):
    mcp_config: dict[str, Any] | None = None
    openai_api_key: str | None = None

@app.post("/session")
async def create_session(body: CreateSessionBody):
    cfg = body.mcp_config or DEFAULT_MCP_CONFIG
    s = Session(cfg, body.openai_api_key)
    sessions[s.id] = s
    return {"sessionId": s.id}

@app.get("/events/{session_id}")
async def events(session_id: str, request: Request):
    s = sessions.get(session_id)
    if not s:
        raise HTTPException(404, "Unknown session")

    async def gen():
        while True:
            if await request.is_disconnected():
                break
            evt = await s.queue.get()
            yield f"data: {json.dumps(evt)}\n\n"

    headers = {
        "Content-Type": "text/event-stream",
        "Cache-Control": "no-cache",
        "Connection": "keep-alive",
        "X-Accel-Buffering": "no",
    }
    return StreamingResponse(gen(), headers=headers)

@app.get("/tools/list")
async def tools_list(session_id: str):
    s = sessions.get(session_id)
    if not s:
        raise HTTPException(404, "Unknown session")
    # Create client (no context manager in 0.1.0+)
    mcp_client = MultiServerMCPClient(s.mcp_config)
    tools = await mcp_client.get_tools()
    return {"tools": [{"name": t.name, "description": t.description} for t in tools]}

@app.get("/mcp/prompts/list")
async def list_prompts():
    """Get available prompts from all configured MCP servers"""
    try:
        # Filter out disabled servers BEFORE merging
        enabled_user_config = {k: v for k, v in USER_MCP_CONFIG.items() if v.get("enabled", True)}
        
        # Merge with defaults (this will clean the config)
        mcp_config = merge_with_defaults(enabled_user_config, DEFAULT_MCP_CONFIG)
        
        if not mcp_config:
            return JSONResponse({
                "prompts": [],
                "count": 0,
                "ok": True,
                "message": "No MCP servers enabled"
            })
        
        # Create client (no context manager in 0.1.0+)
        mcp_client = MultiServerMCPClient(mcp_config)
        
        # Collect prompts from all servers
        all_prompts = []
        
        for server_name in mcp_config.keys():
            try:
                # Use session to access server-specific functionality
                async with mcp_client.session(server_name, auto_initialize=True) as session:
                    # List prompts from this server
                    prompts_result = await session.list_prompts()
                    
                    if hasattr(prompts_result, 'prompts'):
                        for prompt in prompts_result.prompts:
                            prompt_info = {
                                "name": getattr(prompt, 'name', ''),
                                "description": getattr(prompt, 'description', ''),
                                "server": server_name,
                                "arguments": []
                            }
                            
                            # Extract argument information if available
                            if hasattr(prompt, 'arguments'):
                                args_list = prompt.arguments if isinstance(prompt.arguments, list) else []
                                for arg in args_list:
                                    arg_info = {
                                        "name": getattr(arg, 'name', str(arg)),
                                        "description": getattr(arg, 'description', ''),
                                        "required": getattr(arg, 'required', False),
                                    }
                                    prompt_info["arguments"].append(arg_info)
                            
                            all_prompts.append(prompt_info)
            except Exception as e:
                # Server might not support prompts or might be offline
                # This is expected for servers like 'math' that don't have prompts
                error_msg = str(e)
                if "Unknown method: prompts/list" not in error_msg:
                    # Log unexpected errors
                    print(f"Unexpected error getting prompts from {server_name}: {error_msg}")
                continue
        
        return JSONResponse({
            "prompts": all_prompts,
            "count": len(all_prompts),
            "ok": True
        })
            
    except Exception as e:
        import traceback
        print(f"Error getting prompts: {str(e)}")
        print(traceback.format_exc())
        
        return JSONResponse({
            "prompts": [],
            "count": 0,
            "ok": False,
            "error": str(e)
        }, status_code=500)

class ExecutePromptBody(BaseModel):
    prompt_name: str
    server_name: str  # Added server name to know which server to use
    arguments: dict[str, Any] = {}

@app.post("/mcp/prompts/execute")
async def execute_prompt(body: ExecutePromptBody):
    """Execute a specific prompt with provided arguments"""
    try:
        # Filter out disabled servers BEFORE merging
        enabled_user_config = {k: v for k, v in USER_MCP_CONFIG.items() if v.get("enabled", True)}
        
        # Merge with defaults (this will clean the config)
        mcp_config = merge_with_defaults(enabled_user_config, DEFAULT_MCP_CONFIG)
        
        if not mcp_config:
            return JSONResponse({
                "ok": False,
                "error": "No MCP servers enabled"
            }, status_code=400)
        
        if body.server_name not in mcp_config:
            return JSONResponse({
                "ok": False,
                "error": f"Server '{body.server_name}' not found or not enabled"
            }, status_code=404)
        
        # Create client (no context manager in 0.1.0+)
        mcp_client = MultiServerMCPClient(mcp_config)
        
        # Use get_prompt method to execute the prompt
        result_messages = await mcp_client.get_prompt(
            server_name=body.server_name,
            prompt_name=body.prompt_name,
            arguments=body.arguments if body.arguments else None
        )
        
        # Convert messages to serializable format
        messages = []
        for msg in result_messages:
            if hasattr(msg, 'content'):
                messages.append({
                    "role": msg.type if hasattr(msg, 'type') else "assistant",
                    "content": msg.content
                })
            else:
                messages.append({
                    "role": "assistant",
                    "content": str(msg)
                })
        
        return JSONResponse({
            "ok": True,
            "messages": messages,
            "result": str(result_messages)
        })
            
    except Exception as e:
        import traceback
        print(f"Error executing prompt: {str(e)}")
        print(traceback.format_exc())
        
        return JSONResponse({
            "ok": False,
            "error": str(e)
        }, status_code=500)

class ChatBody(BaseModel):
    sessionId: str
    content: str
    openai_api_key: str | None = None

class SimpleChatBody(BaseModel):
    message: str
    history: Optional[list[dict[str, Any]]] = []
    openai_api_key: str | None = None
    use_mcp: bool | None = True  # Default to True for backward compatibility
    mcp_config: dict[str, Any] | None = None  # Custom MCP configuration from user

@app.post("/chat")
async def chat(body: ChatBody):
    s = sessions.get(body.sessionId)
    if not s:
        raise HTTPException(404, "Unknown session")
    s.messages.append({"role": "user", "content": body.content})
    await s.queue.put({"type": "chat.user", "content": body.content})
    
    # Use session's OpenAI API key if provided, otherwise fall back to settings
    api_key = body.openai_api_key or s.openai_api_key or settings.openai_api_key
    result = await run_graph(s.messages, mcp_config=s.mcp_config, openai_api_key=api_key)
    s.messages = result.get("messages", s.messages)
    
    # Find the last assistant message - handle both dict and LangChain message objects
    last = None
    for m in reversed(s.messages):
        if hasattr(m, 'type') and m.type in ('ai', 'assistant'):
            last = m
            break
        elif isinstance(m, dict) and m.get("role") in ("assistant", "ai"):
            last = m
            break
    
    # Extract content from the message
    if last:
        if hasattr(last, 'content'):
            content = last.content
        else:
            content = last.get("content", "")
    else:
        content = ""
    
    await s.queue.put({"type": "chat.assistant", "content": content})
    return JSONResponse({"ok": True})

@app.post("/simple-chat")
async def simple_chat(body: SimpleChatBody):
    """Simple chat endpoint that doesn't require session management"""
    try:
        # Check if OpenAI API key is available (either provided or in settings)
        api_key = body.openai_api_key or settings.openai_api_key
        if not api_key:
            return JSONResponse({
                "response": "⚠️ OpenAI API key is not set. Please configure it in the plugin settings or set the OPENAI_API_KEY environment variable.",
                "ok": False,
                "error_type": "missing_api_key"
            }, status_code=400)
        
        # Convert history to the format expected by run_graph
        messages = []
        for msg in body.history[-10:]:  # Only use last 10 messages for context
            messages.append({
                "role": msg.get("role", "user"),
                "content": msg.get("content", "")
            })
        
        # Add the new message
        messages.append({"role": "user", "content": body.message})
        
        # Determine which MCP config to use:
        # 1. If user provides custom config, use that (merged with defaults)
        # 2. Otherwise, use user's saved config merged with defaults
        # 3. If use_mcp is False, use empty config
        if body.mcp_config:
            # Filter out disabled servers BEFORE merging
            enabled_user_config = {k: v for k, v in body.mcp_config.items() if v.get("enabled", True)}
            # Merge with defaults (this will clean the config)
            mcp_config = merge_with_defaults(enabled_user_config, DEFAULT_MCP_CONFIG)
        elif body.use_mcp:
            # Filter out disabled servers BEFORE merging
            enabled_user_config = {k: v for k, v in USER_MCP_CONFIG.items() if v.get("enabled", True)}
            # Merge with defaults (this will clean the config)
            mcp_config = merge_with_defaults(enabled_user_config, DEFAULT_MCP_CONFIG)
        else:
            mcp_config = {}
        
        result = await run_graph(messages, mcp_config=mcp_config, openai_api_key=api_key)
        
        # Extract ALL messages to show the agent's reasoning and tool calls
        result_messages = result.get("messages", [])
        
        # Convert messages to a serializable format with type information
        formatted_messages = []
        for m in result_messages:
            msg_data = {"type": "unknown", "content": ""}
            
            # Handle LangChain message objects
            if hasattr(m, 'type'):
                msg_data["type"] = m.type  # 'human', 'ai', 'tool', 'system'
                
                # Extract content
                if hasattr(m, 'content'):
                    msg_data["content"] = m.content
                
                # Extract tool call information if present
                if hasattr(m, 'tool_calls') and m.tool_calls:
                    msg_data["tool_calls"] = []
                    for tc in m.tool_calls:
                        # Tool call might be a dict or an object
                        if isinstance(tc, dict):
                            tool_call_info = {
                                "name": tc.get('name', 'unknown'),
                                "args": tc.get('args', {}),
                                "id": tc.get('id', None)
                            }
                        else:
                            tool_call_info = {
                                "name": getattr(tc, 'name', getattr(tc, 'function', {}).get('name', 'unknown') if hasattr(tc, 'function') else 'unknown'),
                                "args": getattr(tc, 'args', getattr(tc, 'function', {}).get('arguments', {}) if hasattr(tc, 'function') else {}),
                                "id": getattr(tc, 'id', None)
                            }
                        msg_data["tool_calls"].append(tool_call_info)
                
                # Extract tool call ID if this is a tool message
                if msg_data["type"] == "tool" and hasattr(m, 'tool_call_id'):
                    msg_data["tool_call_id"] = m.tool_call_id
                    
                # Extract tool name if this is a tool message
                if msg_data["type"] == "tool" and hasattr(m, 'name'):
                    msg_data["tool_name"] = m.name
                    
            # Handle dict messages
            elif isinstance(m, dict):
                msg_data["type"] = m.get("type", m.get("role", "unknown"))
                msg_data["content"] = m.get("content", "")
                
            formatted_messages.append(msg_data)
        
        # Extract the last assistant message for backward compatibility
        last_message = None
        for m in reversed(result_messages):
            if hasattr(m, 'type') and m.type in ('ai', 'assistant'):
                last_message = m
                break
            elif isinstance(m, dict) and m.get("role") in ("assistant", "ai"):
                last_message = m
                break
        
        # Extract content from message object or dict
        if last_message:
            if hasattr(last_message, 'content'):
                response_content = last_message.content
            elif isinstance(last_message, dict):
                response_content = last_message.get("content", "I'm sorry, I couldn't generate a response.")
            else:
                response_content = str(last_message)
        else:
            response_content = "I'm sorry, I couldn't generate a response."
        
        return JSONResponse({
            "response": response_content,
            "messages": formatted_messages,  # Include all messages with types and tool calls
            "ok": True
        })
        
    except Exception as e:
        error_message = str(e)
        
        # Provide more helpful error messages for common issues
        if "invalid_api_key" in error_message.lower() or "incorrect api key" in error_message.lower():
            error_message = "🔑 Invalid OpenAI API key. Please check your API key at https://platform.openai.com/account/api-keys"
        elif "api_key" in error_message.lower():
            error_message = "⚠️ OpenAI API key issue. Please check that your OPENAI_API_KEY environment variable is set correctly."
        elif "connection" in error_message.lower() or "network" in error_message.lower():
            error_message = "🌐 Network connection issue. Please check your internet connection."
        elif "rate" in error_message.lower() and "limit" in error_message.lower():
            error_message = "⏱️ Rate limit exceeded. Please wait a moment and try again."
        else:
            error_message = f"❌ Error: {error_message}"
        
        return JSONResponse({
            "response": error_message,
            "ok": False,
            "error_type": "processing_error"
        }, status_code=500)

# Global dict to store pending tool approvals
# Format: {approval_id: {"approved": None/True/False, "tool_calls": [...]}}
pending_approvals: dict[str, dict[str, Any]] = {}

class StreamChatBody(BaseModel):
    message: str
    history: Optional[list[dict[str, Any]]] = []
    openai_api_key: str | None = None
    use_mcp: bool | None = True
    mcp_config: dict[str, Any] | None = None
    require_approval: bool | None = True  # Whether to require approval for tool calls

@app.post("/stream-chat")
async def stream_chat(body: StreamChatBody):
    """Streaming chat endpoint that sends events in real-time with tool approval support"""
    
    async def event_generator():
        try:
            # Check API key
            api_key = body.openai_api_key or settings.openai_api_key
            if not api_key:
                yield f"data: {json.dumps({'type': 'error', 'content': '⚠️ OpenAI API key is not set'})}\n\n"
                return
            
            # Send initial status
            yield f"data: {json.dumps({'type': 'status', 'content': 'Processing your request...'})}\n\n"
            
            # Prepare messages
            messages = []
            for msg in body.history[-10:]:
                messages.append({
                    "role": msg.get("role", "user"),
                    "content": msg.get("content", "")
                })
            messages.append({"role": "user", "content": body.message})
            
            # Determine MCP config
            if body.mcp_config:
                enabled_user_config = {k: v for k, v in body.mcp_config.items() if v.get("enabled", True)}
                mcp_config = merge_with_defaults(enabled_user_config, DEFAULT_MCP_CONFIG)
            elif body.use_mcp:
                enabled_user_config = {k: v for k, v in USER_MCP_CONFIG.items() if v.get("enabled", True)}
                mcp_config = merge_with_defaults(enabled_user_config, DEFAULT_MCP_CONFIG)
            else:
                mcp_config = {}
            
            # Use streaming runner
            pending_tool_approval = None  # Track pending approval with tool details
            
            async for event in stream_graph(
                messages=messages,
                mcp_config=mcp_config,
                openai_api_key=api_key,
                require_approval=body.require_approval
            ):
                event_type = event.get("type")
                
                # Handle token streaming (real-time AI output)
                if event_type == "token":
                    yield f"data: {json.dumps(event)}\n\n"
                
                # Handle tool calls (AI wants to use tools)
                elif event_type == "tool_calls":
                    tool_calls = event.get("tool_calls", [])
                    
                    if body.require_approval and tool_calls:
                        # Don't send approval yet - wait for tool_start to get actual tool name
                        # Just note that approval will be needed
                        pending_tool_approval = {
                            "tools": [],
                            "approval_id": f"approval_{id(event)}_{len(pending_approvals)}"
                        }
                    else:
                        # No approval required, just notify about tool calls
                        yield f"data: {json.dumps({'type': 'tool_calls_info', 'tool_calls': tool_calls})}\n\n"
                
                # Handle tool execution events
                elif event_type == "tool_start":
                    tool_name = event.get("tool_name", "unknown")
                    tool_input = event.get("tool_input", {})
                    
                    # If approval is pending, collect tool info and request approval
                    if pending_tool_approval is not None:
                        # Add tool to pending list
                        pending_tool_approval["tools"].append({
                            "name": tool_name,
                            "args": tool_input,
                            "id": None
                        })
                        
                        # Send approval request NOW that we have the real tool name
                        approval_id = pending_tool_approval["approval_id"]
                        tool_calls = pending_tool_approval["tools"]
                        
                        pending_approvals[approval_id] = {
                            "approved": None,
                            "tool_calls": tool_calls
                        }
                        
                        yield f"data: {json.dumps({'type': 'tool_approval_required', 'approval_id': approval_id, 'tool_calls': tool_calls})}\n\n"
                        
                        # Wait for approval
                        timeout = 120  # 2 minutes
                        waited = 0
                        while pending_approvals[approval_id]["approved"] is None and waited < timeout:
                            await asyncio.sleep(0.5)
                            waited += 0.5
                        
                        approval_status = pending_approvals[approval_id]["approved"]
                        del pending_approvals[approval_id]
                        pending_tool_approval = None  # Clear pending approval
                        
                        if approval_status == False:
                            yield f"data: {json.dumps({'type': 'tool_rejected', 'content': '❌ Tool call rejected by user'})}\n\n"
                            yield f"data: {json.dumps({'type': 'done'})}\n\n"
                            return
                        elif approval_status is None:
                            yield f"data: {json.dumps({'type': 'tool_timeout', 'content': '⏱️ Tool approval timeout'})}\n\n"
                            yield f"data: {json.dumps({'type': 'done'})}\n\n"
                            return
                        
                        # Approved - notify frontend and continue
                        yield f"data: {json.dumps({'type': 'tool_approved', 'content': '✅ Tool call approved'})}\n\n"
                    
                    # Send tool_start event
                    yield f"data: {json.dumps(event)}\n\n"
                
                elif event_type == "tool_end":
                    yield f"data: {json.dumps(event)}\n\n"
                
                # Handle completion
                elif event_type == "done":
                    yield f"data: {json.dumps(event)}\n\n"
            
        except Exception as e:
            import traceback
            error_trace = traceback.format_exc()
            print(f"Stream error: {error_trace}")
            yield f"data: {json.dumps({'type': 'error', 'content': f'Error: {str(e)}'})}\n\n"
    
    return StreamingResponse(
        event_generator(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        }
    )

class ToolApprovalBody(BaseModel):
    approval_id: str
    approved: bool

@app.post("/approve-tool")
async def approve_tool(body: ToolApprovalBody):
    """Approve or reject a pending tool call"""
    if body.approval_id not in pending_approvals:
        return JSONResponse({
            "ok": False,
            "error": "Approval ID not found or already processed"
        }, status_code=404)
    
    pending_approvals[body.approval_id]["approved"] = body.approved
    
    return JSONResponse({
        "ok": True,
        "approved": body.approved
    })

@app.get("/health")
async def health_check():
    """Simple health check endpoint"""
    has_api_key = bool(os.environ.get("OPENAI_API_KEY"))
    return JSONResponse({
        "status": "ok", 
        "message": "AI Agent Bridge is running",
        "host": settings.bridge_host,
        "port": settings.bridge_port,
        "openai_api_key_set": has_api_key,
        "ready_for_chat": has_api_key
    })

@app.get("/mcp/servers")
async def get_mcp_servers():
    """Get currently configured MCP servers (both default and user-configured)"""
    servers = []
    
    # Merge default and user configs
    all_configs = merge_with_defaults(USER_MCP_CONFIG, DEFAULT_MCP_CONFIG)
    
    for name, config in all_configs.items():
        # Check if this is a default server
        is_default = name in DEFAULT_MCP_CONFIG
        
        server_info = {
            "id": name,
            "name": config.get("name", name.upper() if name != "ros2" else "ROS2"),
            "transport": config.get("transport", "stdio"),
            "enabled": config.get("enabled", True),
            "is_default": is_default,
        }
        
        # Add command info for stdio transport
        if config.get("transport") == "stdio":
            server_info["command"] = config.get("command", "")
            server_info["args"] = config.get("args", [])
        # Add URL info for SSE transport
        elif config.get("transport") == "sse":
            server_info["url"] = config.get("url", "")
        
        servers.append(server_info)
    
    return JSONResponse({
        "servers": servers,
        "count": len(servers),
        "ok": True
    })

class UpdateMCPConfigBody(BaseModel):
    mcp_config: dict[str, Any]

@app.post("/mcp/servers/save")
async def save_mcp_servers(body: UpdateMCPConfigBody):
    """Save user's custom MCP server configuration to file"""
    global USER_MCP_CONFIG
    
    try:
        # Save to file
        success = save_user_mcp_config(body.mcp_config)
        
        if success:
            # Update in-memory config
            USER_MCP_CONFIG = body.mcp_config
            
            return JSONResponse({
                "ok": True,
                "message": "MCP configuration saved successfully",
                "servers_count": len(body.mcp_config)
            })
        else:
            return JSONResponse({
                "ok": False,
                "error": "Failed to save configuration to file"
            }, status_code=500)
            
    except Exception as e:
        return JSONResponse({
            "ok": False,
            "error": str(e)
        }, status_code=500)

@app.get("/debug")
async def debug_info():
    """Debug information endpoint"""
    return JSONResponse({
        "cors_origins": cors_origins_list,
        "sessions_count": len(sessions),
        "default_mcp_config": DEFAULT_MCP_CONFIG,
        "user_mcp_config": USER_MCP_CONFIG,
    })

def main():
    import uvicorn
    uvicorn.run("bridge.main:app",
                host=settings.bridge_host,
                port=settings.bridge_port,
                reload=bool(os.environ.get("DEV", "1") == "1"))

if __name__ == "__main__":
    main()