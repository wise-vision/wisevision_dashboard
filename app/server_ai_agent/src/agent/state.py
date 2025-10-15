from typing import Optional, Dict, Any, List
from typing_extensions import TypedDict
# from copilotkit import CopilotKitState  # jeżeli nie używasz, możesz rozszerzyć zwykły TypedDict

class AgentState(TypedDict):
    messages: List[Dict[str, Any]]
    mcp_config: Optional[Dict[str, Any]]
    openai_api_key: Optional[str]