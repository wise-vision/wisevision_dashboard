#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

from typing import Optional, Dict, Any, List
from typing_extensions import TypedDict
# from copilotkit import CopilotKitState  # jeżeli nie używasz, możesz rozszerzyć zwykły TypedDict

class AgentState(TypedDict):
    messages: List[Dict[str, Any]]
    mcp_config: Optional[Dict[str, Any]]
    openai_api_key: Optional[str]