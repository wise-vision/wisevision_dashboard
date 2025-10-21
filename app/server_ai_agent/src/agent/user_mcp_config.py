"""
User MCP Configuration Management
Handles loading and saving user-defined MCP server configurations
"""
import json
import os
from pathlib import Path
from typing import Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)

# Path to user configuration file (in the same directory as this file)
CONFIG_DIR = Path(__file__).parent.parent.parent  # Points to server_ai_agent root
USER_CONFIG_FILE = CONFIG_DIR / ".mcp_user_config.json"


def load_user_mcp_config() -> Dict[str, Any]:
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


def save_user_mcp_config(config: Dict[str, Any]) -> bool:
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


def merge_with_defaults(user_config: Dict[str, Any], default_config: Dict[str, Any]) -> Dict[str, Any]:
    """
    Merge user configuration with default configuration.
    User config takes precedence for servers with the same name.
    """
    merged = default_config.copy()
    merged.update(user_config)
    return merged


def get_config_file_path() -> Path:
    """Get the path to the user config file"""
    return USER_CONFIG_FILE


def config_file_exists() -> bool:
    """Check if user config file exists"""
    return USER_CONFIG_FILE.exists()
