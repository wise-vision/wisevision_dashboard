import { SimpleChatMessage, SimpleSessionState } from './SimpleSessionManager';

export interface MCPServerInfo {
  id: string;
  name: string;
  command?: string;
  args?: string[];
  url?: string;
  transport: 'stdio' | 'sse';
  enabled: boolean;
  is_default?: boolean;
}

export class MCPSessionManager {
  private sessionState: SimpleSessionState;
  private listeners: Set<(state: SimpleSessionState) => void> = new Set();
  private mcpServers: MCPServerInfo[] = [];
  private customMCPConfig: Record<string, any> = {};  // Store user's custom MCP config

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
    // Connect to MCP bridge service and fetch default servers
    try {
      await this.fetchMCPServers();
      this.sessionState.connected = true;
    } catch (error) {
      console.error('Failed to fetch MCP servers:', error);
      this.sessionState.connected = false;
    }
    this.notifyListeners();
  }

  async fetchMCPServers(): Promise<MCPServerInfo[]> {
    try {
      const { getBackendSrv } = await import('@grafana/runtime');
      const { lastValueFrom } = await import('rxjs');
      
      const response = await lastValueFrom(
        getBackendSrv().fetch({
          url: '/api/plugin-proxy/wisevision-wiseos-app/agent_backend/mcp/servers',
          method: 'GET',
        })
      );

      if (response.ok && response.data) {
        const result = response.data as any;
        this.mcpServers = result.servers || [];
        return this.mcpServers;
      }
    } catch (error) {
      console.error('Error fetching MCP servers:', error);
    }
    return [];
  }

  getMCPServers(): MCPServerInfo[] {
    return this.mcpServers;
  }

  setMCPServers(servers: MCPServerInfo[]) {
    this.mcpServers = servers;
    // Convert to backend MCP config format
    this.customMCPConfig = this.convertToMCPConfig(servers);
    // Save to backend
    this.saveMCPServersToBackend(this.customMCPConfig);
  }

  async saveMCPServersToBackend(config: Record<string, any>): Promise<boolean> {
    try {
      const { getBackendSrv } = await import('@grafana/runtime');
      const { lastValueFrom } = await import('rxjs');
      
      const response = await lastValueFrom(
        getBackendSrv().fetch({
          url: '/api/plugin-proxy/wisevision-wiseos-app/agent_backend/mcp/servers/save',
          method: 'POST',
          data: {
            mcp_config: config
          }
        })
      );

      if (response.ok) {
        console.log('MCP configuration saved successfully');
        return true;
      } else {
        console.error('Failed to save MCP configuration');
        return false;
      }
    } catch (error) {
      console.error('Error saving MCP configuration:', error);
      return false;
    }
  }

  private convertToMCPConfig(servers: MCPServerInfo[]): Record<string, any> {
    const config: Record<string, any> = {};
    
    servers.forEach(server => {
      // Include all servers in config (not just enabled ones)
      // The backend will filter based on enabled flag
      const serverConfig: any = {
        name: server.name,
        transport: server.transport,
        enabled: server.enabled
      };
      
      if (server.transport === 'stdio') {
        serverConfig.command = server.command || '';
        serverConfig.args = server.args || [];
      } else if (server.transport === 'sse') {
        serverConfig.url = server.url || '';
      }
      
      // Store whether it's a default server
      if (server.is_default) {
        serverConfig.is_default = true;
      }
      
      config[server.id] = serverConfig;
    });
    
    return config;
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
            mcp_config: Object.keys(this.customMCPConfig).length > 0 ? this.customMCPConfig : undefined, // Send custom config if available
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
