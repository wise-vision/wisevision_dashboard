import { getBackendSrv } from '@grafana/runtime';
import { lastValueFrom } from 'rxjs';

export interface SimpleChatMessage {
  role: 'user' | 'assistant' | 'system';
  content: string;
  timestamp: Date;
  id: string;
  // For tool approval messages
  toolApproval?: {
    approvalId: string;
    toolCalls: Array<{
      name: string;
      args: Record<string, any>;
      id?: string;
    }>;
    status?: 'pending' | 'approved' | 'rejected';
  };
}

export interface SimpleSessionState {
  messages: SimpleChatMessage[];
  connected: boolean;
  loading: boolean;
}

export class SimpleSessionManager {
  private baseUrl: string;
  private sessionState: SimpleSessionState;
  private listeners: Set<(state: SimpleSessionState) => void> = new Set();

  constructor(baseUrl?: string) {
    // Use Grafana's plugin proxy route (same pattern as StorageSettingsModal)
    if (baseUrl) {
      this.baseUrl = baseUrl;
    } else {
      // Use plugin proxy route: matches "agent_backend/*" -> "{{ .JsonData.apiUrlAgent }}"
      this.baseUrl = 'api/plugin-proxy/wisevision-wiseos-app/agent_backend';
    }
    
    console.log(`SimpleSessionManager initialized with baseUrl: ${this.baseUrl}`);
    
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
    try {
      console.log(`Attempting to connect to: ${this.baseUrl}/health`);
      
      // Use Grafana's backend service for proxy requests (same as StorageSettingsModal)
      const response = await lastValueFrom(
        getBackendSrv().fetch({
          url: `${this.baseUrl}/health`,
          method: 'GET'
        })
      ).catch(() => null);

      this.sessionState.connected = response?.ok || false;
      console.log(`Connection status: ${this.sessionState.connected ? 'Connected' : 'Disconnected'}`);
      this.notifyListeners();
      return this.sessionState.connected;
    } catch (error) {
      console.error('Failed to connect:', error);
      this.sessionState.connected = false;
      this.notifyListeners();
      return false;
    }
  }

  async sendMessage(content: string) {
    if (!content.trim()) {return;}

    try {
      this.sessionState.loading = true;
      
      // Add user message immediately
      const userMessage: SimpleChatMessage = {
        role: 'user',
        content: content.trim(),
        timestamp: new Date(),
        id: this.generateId(),
      };
      
      this.sessionState.messages.push(userMessage);
      this.notifyListeners();

      // OpenAI API key is now configured via environment variable in the AI agent backend
      console.log('OpenAI API key is configured via environment variable in AI agent backend');

      // Send to backend using Grafana's proxy (same pattern as StorageSettingsModal)
      console.log(`Sending message to: ${this.baseUrl}/simple-chat`);
      
      const response = await lastValueFrom(
        getBackendSrv().fetch({
          url: `${this.baseUrl}/simple-chat`,
          method: 'POST',
          data: {
            message: content.trim(),
            history: this.sessionState.messages.slice(-10), // Send last 10 messages for context
            use_mcp: false, // Disable MCP tools for simple mode
            // OpenAI API key is configured via environment variable in the AI agent backend
          }
        })
      );

      if (!response.ok) {
        const errorData = response.data as any || {};
        throw new Error(errorData.response || `Failed to send message: ${response.statusText}`);
      }

      const result = response.data as any;
      
      // Add assistant response
      const assistantMessage: SimpleChatMessage = {
        role: 'assistant',
        content: result.response || 'Sorry, I received an empty response.',
        timestamp: new Date(),
        id: this.generateId(),
      };

      this.sessionState.messages.push(assistantMessage);
      
    } catch (error) {
      console.error('Failed to send message:', error);
      
      // Add error message with more specific information
      let errorContent = 'Sorry, I encountered an error.';
      
      if (error instanceof TypeError && error.message.includes('fetch')) {
        errorContent = `Cannot connect to AI Agent backend at ${this.baseUrl}. Please ensure the backend is running and accessible.`;
      } else if (error instanceof Error) {
        errorContent = `Error: ${error.message}`;
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
    return Math.random().toString(36).substr(2, 9);
  }
}
