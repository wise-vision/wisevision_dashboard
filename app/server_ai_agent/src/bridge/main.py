import asyncio
import json
import os
from typing import Dict, Any, Optional

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import BaseModel

from agent.runner import run_graph
from agent.mcp_config import DEFAULT_MCP_CONFIG
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

class Session:
    def __init__(self, cfg: Dict[str, Any], openai_api_key: Optional[str] = None):
        self.id = os.urandom(6).hex()
        self.mcp_config = cfg
        self.messages: list[dict[str, Any]] = []
        self.queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue()
        self.openai_api_key = openai_api_key

sessions: Dict[str, Session] = {}

class CreateSessionBody(BaseModel):
    mcp_config: Optional[Dict[str, Any]] = None
    openai_api_key: Optional[str] = None

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
    async with MultiServerMCPClient(s.mcp_config) as c:
        tools = c.get_tools()
        return {"tools": [{"name": t.name, "description": t.description} for t in tools]}

class ChatBody(BaseModel):
    sessionId: str
    content: str
    openai_api_key: Optional[str] = None

class SimpleChatBody(BaseModel):
    message: str
    history: Optional[list[dict[str, Any]]] = []
    openai_api_key: Optional[str] = None
    use_mcp: Optional[bool] = True  # Default to True for backward compatibility

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
        
        # Use MCP config only if use_mcp is True
        mcp_config = DEFAULT_MCP_CONFIG if body.use_mcp else {}
        result = await run_graph(messages, mcp_config=mcp_config, openai_api_key=api_key)
        
        # Extract the last assistant message
        result_messages = result.get("messages", [])
        last_message = None
        
        # Look for the last AI/assistant message
        for m in reversed(result_messages):
            # Handle both dict and LangChain message objects
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

@app.get("/debug")
async def debug_info():
    """Debug information endpoint"""
    return JSONResponse({
        "cors_origins": cors_origins_list,
        "sessions_count": len(sessions),
        "default_mcp_config": DEFAULT_MCP_CONFIG,
    })

def main():
    import uvicorn
    uvicorn.run("bridge.main:app",
                host=settings.bridge_host,
                port=settings.bridge_port,
                reload=bool(os.environ.get("DEV", "1") == "1"))

if __name__ == "__main__":
    main()