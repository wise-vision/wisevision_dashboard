"""
MCP-compliant math server
"""
import sys
import json
import math
from typing import Any, Dict

def write_message(obj: Dict[str, Any]):
    message = json.dumps(obj) + "\n"
    sys.stdout.write(message)
    sys.stdout.flush()

def main():
    # Process messages from stdin
    for line in sys.stdin:
        try:
            line = line.strip()
            if not line:
                continue
                
            message = json.loads(line)
            
            # Handle initialize request
            if message.get("method") == "initialize":
                write_message({
                    "jsonrpc": "2.0",
                    "id": message.get("id"),
                    "result": {
                        "protocolVersion": "2024-11-05",
                        "capabilities": {
                            "tools": {}
                        },
                        "serverInfo": {
                            "name": "math-server",
                            "version": "1.0.0"
                        }
                    }
                })
            
            # Handle initialized notification
            elif message.get("method") == "notifications/initialized":
                # Just acknowledge, no response needed for notifications
                pass
            
            # Handle tools/list
            elif message.get("method") == "tools/list":
                write_message({
                    "jsonrpc": "2.0",
                    "id": message.get("id"),
                    "result": {
                        "tools": [{
                            "name": "evaluate",
                            "description": "Evaluate a mathematical expression",
                            "inputSchema": {
                                "type": "object",
                                "properties": {
                                    "expression": {
                                        "type": "string",
                                        "description": "Mathematical expression to evaluate"
                                    }
                                },
                                "required": ["expression"]
                            }
                        }]
                    }
                })
            
            # Handle tools/call
            elif message.get("method") == "tools/call":
                params = message.get("params", {})
                name = params.get("name")
                arguments = params.get("arguments", {})
                
                if name == "evaluate":
                    expression = arguments.get("expression", "")
                    try:
                        # UWAGA: w produkcji NIE używaj eval – to tylko demo!
                        safe_dict = {
                            "__builtins__": {},
                            "sqrt": math.sqrt,
                            "pow": pow,
                            "sin": math.sin,
                            "cos": math.cos,
                            "tan": math.tan,
                            "log": math.log,
                            "pi": math.pi,
                            "e": math.e
                        }
                        result = eval(expression, safe_dict, {})
                        
                        write_message({
                            "jsonrpc": "2.0",
                            "id": message.get("id"),
                            "result": {
                                "content": [{
                                    "type": "text",
                                    "text": f"Result: {result}"
                                }]
                            }
                        })
                    except Exception as e:
                        write_message({
                            "jsonrpc": "2.0",
                            "id": message.get("id"),
                            "error": {
                                "code": -1,
                                "message": f"Error evaluating expression: {str(e)}"
                            }
                        })
                else:
                    write_message({
                        "jsonrpc": "2.0",
                        "id": message.get("id"),
                        "error": {
                            "code": -1,
                            "message": f"Unknown tool: {name}"
                        }
                    })
            
            else:
                write_message({
                    "jsonrpc": "2.0",
                    "id": message.get("id"),
                    "error": {
                        "code": -1,
                        "message": f"Unknown method: {message.get('method')}"
                    }
                })
                
        except Exception as e:
            # Handle any parsing errors
            write_message({
                "jsonrpc": "2.0",
                "id": None,
                "error": {
                    "code": -1,
                    "message": f"Server error: {str(e)}"
                }
            })

if __name__ == "__main__":
    main()