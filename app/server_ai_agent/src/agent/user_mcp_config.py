"""
User MCP Configuration Management
Handles loading and saving user-defined MCP server configurations
"""
import json
import os
from pathlib import Path
from typing import Any
import logging

logger = logging.getLogger(__name__)

# Path to user configuration file (in the same directory as this file)
CONFIG_DIR = Path(__file__).parent.parent.parent  # Points to server_ai_agent root
USER_CONFIG_FILE = CONFIG_DIR / ".mcp_user_config.json"


def load_user_mcp_config() -> dict[str, Any]:
    """
    Load user's custom MCP configuration from file.
    Returns empty dict if file doesn't exist.
    """
    if not USER_CONFIG_FILE.exists():
        logger.info(f"User MCP config file not found at {USER_CONFIG_FILE}, using empty config")
        return {}
    
    try:
        with open(USER_CONFIG_FILE, 'r') as f:
            config = json.load(f)
            logger.info(f"Loaded user MCP config with {len(config)} servers")
            return config
    except json.JSONDecodeError as e:
        logger.error(f"Failed to parse user MCP config: {e}")
        return {}
    except Exception as e:
        logger.error(f"Failed to load user MCP config: {e}")
        return {}


def save_user_mcp_config(config: dict[str, Any]) -> bool:
    """
    Save user's custom MCP configuration to file.
    Returns True if successful, False otherwise.
    """
    try:
        # Create config directory if it doesn't exist
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        
        # Write config to file with pretty formatting
        with open(USER_CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=2)
        
        logger.info(f"Saved user MCP config with {len(config)} servers to {USER_CONFIG_FILE}")
        return True
    except Exception as e:
        logger.error(f"Failed to save user MCP config: {e}")
        return False


def merge_with_defaults(user_config: dict[str, Any], default_config: dict[str, Any]) -> dict[str, Any]:
    """
    Merge user configuration with default configuration.
    User config takes precedence for servers with the same name.
    Filters out extra fields that aren't needed by MCP client.
    """
    merged = {}
    
    # Add default configs first
    for name, config in default_config.items():
        # Only keep MCP-required fields
        clean_config = {
            "command": config.get("command"),
            "args": config.get("args", []),
            "transport": config.get("transport", "stdio"),
        }
        merged[name] = clean_config
    
    # Override with user configs
    for name, config in user_config.items():
        # Only keep MCP-required fields, ignore UI-only fields like 'name', 'enabled', 'is_default'
        clean_config = {
            "command": config.get("command"),
            "args": config.get("args", []),
            "transport": config.get("transport", "stdio"),
        }
        merged[name] = clean_config
    
    return merged


def get_config_file_path() -> Path:
    """Get the path to the user config file"""
    return USER_CONFIG_FILE


def config_file_exists() -> bool:
    """Check if user config file exists"""
    return USER_CONFIG_FILE.exists()
