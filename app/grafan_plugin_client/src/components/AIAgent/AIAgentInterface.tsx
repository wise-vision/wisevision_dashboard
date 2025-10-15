import React, { useState } from 'react';
import { DualChatInterface } from './DualChatInterface';
import { SimpleSessionProvider } from './SimpleSessionProvider';
import { SimpleSessionManager } from '../services/SimpleSessionManager';

export function AIAgentInterface() {
  const [simpleSessionManager] = useState(() => new SimpleSessionManager());

  // Always show DualChatInterface with both Simple and MCP modes + settings
  return (
    <SimpleSessionProvider sessionManager={simpleSessionManager}>
      <DualChatInterface />
    </SimpleSessionProvider>
  );
}
