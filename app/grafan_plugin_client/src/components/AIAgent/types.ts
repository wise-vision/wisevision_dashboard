export interface MCPServerConfig {
  name: string;
  transport: 'stdio' | 'sse';
  command?: string;
  args?: string[];
  url?: string;
}

export interface SessionState {
  id: string | null;
  mcpConfig: Record<string, Omit<MCPServerConfig, 'name'>>;
  messages: ChatMessage[];
  isConnected: boolean;
}

export interface ChatMessage {
  role: 'user' | 'assistant' | 'system';
  content: string;
  timestamp: Date;
  id: string;
}

export interface ToolCall {
  name: string;
  args: any;
  status: 'running' | 'success' | 'error' | 'pending';
  result?: any;
}

export interface ServerEvent {
  type: 'chat.user' | 'chat.assistant' | 'tool.call' | 'error';
  content?: string;
  tool?: ToolCall;
  error?: string;
}
