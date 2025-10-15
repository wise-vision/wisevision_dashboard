import { SimpleChatMessage, SimpleSessionState } from './SimpleSessionManager';

export class MCPSessionManager {
  private sessionState: SimpleSessionState;
  private listeners: Set<(state: SimpleSessionState) => void> = new Set();

  constructor() {
    this.sessionState = {
      messages: [],
      connected: false,
      loading: false,
    };
  }

  subscribe(listener: (state: SimpleSessionState) => void) {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  private notifyListeners() {
    this.listeners.forEach(listener => listener({ ...this.sessionState }));
  }

  async connect() {
    // TODO: Connect to MCP bridge service
    this.sessionState.connected = true;
    this.notifyListeners();
  }

  async sendMessage(content: string) {
    if (!content.trim()) {
      return;
    }

    // Add user message
    const userMessage: SimpleChatMessage = {
      role: 'user',
      content: content.trim(),
      timestamp: new Date(),
      id: this.generateId(),
    };

    this.sessionState.messages.push(userMessage);
    this.sessionState.loading = true;
    this.notifyListeners();

    try {
      // Use Grafana's proxy to send to MCP bridge via AI agent backend
      const { getBackendSrv } = await import('@grafana/runtime');
      const { lastValueFrom } = await import('rxjs');
      
      const response = await lastValueFrom(
        getBackendSrv().fetch({
          url: '/api/plugin-proxy/wisevision-wiseos-app/agent_backend/simple-chat',
          method: 'POST',
          data: {
            message: content.trim(),
            history: this.sessionState.messages.slice(-10), // Send last 10 messages for context
            use_mcp: true, // Flag to indicate MCP usage
          }
        })
      );

      if (!response.ok) {
        const errorData = response.data as any || {};
        throw new Error(errorData.response || `Failed to send message: ${response.statusText}`);
      }

      const result = response.data as any;

      const assistantMessage: SimpleChatMessage = {
        role: 'assistant',
        content: result.response || result.message || 'No response received',
        timestamp: new Date(),
        id: this.generateId(),
      };

      this.sessionState.messages.push(assistantMessage);
    } catch (error) {
      console.error('MCP request failed:', error);
      
      let errorContent = '❌ MCP Bridge Error';
      
      if (error instanceof TypeError && error.message.includes('fetch')) {
        errorContent = '🔌 **Connection Error**\n\nCannot connect to the MCP bridge service.\n\n**Troubleshooting:**\n• Check if the AI agent backend is running (port 8089)\n• Verify the MCP bridge service is started\n• Check network connectivity';
      } else if (error instanceof Error) {
        errorContent = `❌ **MCP Error**\n\n${error.message}\n\n**Note:** Make sure the MCP bridge service is properly configured and running.`;
      }

      const errorMessage: SimpleChatMessage = {
        role: 'assistant',
        content: errorContent,
        timestamp: new Date(),
        id: this.generateId(),
      };
      
      this.sessionState.messages.push(errorMessage);
    } finally {
      this.sessionState.loading = false;
      this.notifyListeners();
    }
  }

  clearMessages() {
    this.sessionState.messages = [];
    this.notifyListeners();
  }

  getState(): SimpleSessionState {
    return { ...this.sessionState };
  }

  private generateId(): string {
    return Date.now().toString() + Math.random().toString(36).substr(2, 9);
  }
}
