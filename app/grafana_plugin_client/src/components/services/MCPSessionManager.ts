/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

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

      // Check if we have detailed messages with tool calls
      if (result.messages && Array.isArray(result.messages)) {
        // Process all messages to show AI reasoning and tool usage
        for (const msg of result.messages) {
          // Skip messages that are already in history (user input)
          if (msg.type === 'human' || msg.type === 'user') {
            continue;
          }
          
          // Format AI messages with tool calls
          if (msg.type === 'ai' || msg.type === 'assistant') {
            let content = msg.content || '';
            
            // Add tool call information if present
            if (msg.tool_calls && msg.tool_calls.length > 0) {
              content += '\n\n🔧 **Calling tools:**\n';
              for (const tc of msg.tool_calls) {
                content += `- **${tc.name}**`;
                if (tc.args && Object.keys(tc.args).length > 0) {
                  content += `\n  \`\`\`json\n  ${JSON.stringify(tc.args, null, 2)}\n  \`\`\``;
                }
                content += '\n';
              }
            }
            
            // Only add if there's actual content
            if (content.trim()) {
              const aiMessage: SimpleChatMessage = {
                role: 'assistant',
                content: content,
                timestamp: new Date(),
                id: this.generateId(),
              };
              this.sessionState.messages.push(aiMessage);
            }
          }
          
          // Format tool response messages
          else if (msg.type === 'tool') {
            const toolContent = `**🛠️ Tool: ${msg.tool_name || 'Unknown'}**\n\n\`\`\`\n${msg.content}\n\`\`\``;
            const toolMessage: SimpleChatMessage = {
              role: 'assistant',
              content: toolContent,
              timestamp: new Date(),
              id: this.generateId(),
            };
            this.sessionState.messages.push(toolMessage);
          }
        }
      } else {
        // Fallback to simple response (backward compatibility)
        const assistantMessage: SimpleChatMessage = {
          role: 'assistant',
          content: result.response || result.message || 'No response received',
          timestamp: new Date(),
          id: this.generateId(),
        };
        this.sessionState.messages.push(assistantMessage);
      }
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

  getSessionState(): SimpleSessionState {
    return { ...this.sessionState };
  }

  addMessage(message: SimpleChatMessage) {
    this.sessionState.messages.push(message);
    this.notifyListeners();
  }

  updateLastMessage(content: string) {
    if (this.sessionState.messages.length > 0) {
      const lastMsg = this.sessionState.messages[this.sessionState.messages.length - 1];
      if (lastMsg.role === 'assistant') {
        lastMsg.content = content;
        this.notifyListeners();
      }
    }
  }

  async streamMessage(
    content: string,
    requireApproval = false,
    onToolApproval?: (approvalId: string, toolCalls: any[]) => Promise<boolean>
  ) {
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

    // Create placeholder for AI response
    let currentAIMessage: SimpleChatMessage = {
      role: 'assistant',
      content: '',
      timestamp: new Date(),
      id: this.generateId(),
    };
    this.sessionState.messages.push(currentAIMessage);
    this.notifyListeners();

    try {
      // Construct the URL
      const baseUrl = '/api/plugin-proxy/wisevision-wiseos-app/agent_backend/stream-chat';
      const requestBody = {
        message: content.trim(),
        history: this.sessionState.messages.slice(-12, -2), // Exclude user msg and placeholder
        use_mcp: true,
        require_approval: requireApproval,
        mcp_config: Object.keys(this.customMCPConfig).length > 0 ? this.customMCPConfig : undefined,
      };

      // Use fetch directly for EventSource streaming
      const response = await fetch(baseUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'text/event-stream',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`Failed to connect to stream: ${response.statusText}`);
      }

      const reader = response.body?.getReader();
      if (!reader) {
        throw new Error('Failed to get response reader');
      }

      const decoder = new TextDecoder();
      let buffer = '';

      while (true) {
        const { done, value } = await reader.read();
        
        if (done) {
          break;
        }

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop() || '';

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            const data = JSON.parse(line.slice(6));
            
            switch (data.type) {
              case 'status':
                // Skip status messages - they're not needed in the UI
                // (These are "Processing your request..." messages)
                break;

              case 'token':
                // Append token to current message
                currentAIMessage.content += data.content;
                this.notifyListeners();
                break;

              case 'tool_calls_info':
                // Show tool being called (no approval needed)
                const toolsInfo = data.tool_calls.map((tc: any) => 
                  `🔧 **${tc.name}**\n\`\`\`json\n${JSON.stringify(tc.args, null, 2)}\n\`\`\``
                ).join('\n\n');
                
                // Add tool info as separate message
                const toolInfoMsg: SimpleChatMessage = {
                  role: 'assistant',
                  content: toolsInfo,
                  timestamp: new Date(),
                  id: this.generateId(),
                };
                this.sessionState.messages.push(toolInfoMsg);
                this.notifyListeners();
                break;

              case 'tool_approval_required':
                // Add approval message to chat in chronological order
                const approvalMessage: SimpleChatMessage = {
                  role: 'assistant',
                  content: '⚠️ Tool approval required',
                  timestamp: new Date(),
                  id: this.generateId(),
                  toolApproval: {
                    approvalId: data.approval_id,
                    toolCalls: data.tool_calls,
                    status: 'pending',
                  },
                };
                
                // Remove the current AI placeholder temporarily
                const aiPlaceholderIndex = this.sessionState.messages.findIndex(msg => msg.id === currentAIMessage.id);
                if (aiPlaceholderIndex !== -1) {
                  this.sessionState.messages.splice(aiPlaceholderIndex, 1);
                }
                
                // Add approval message at the end
                this.sessionState.messages.push(approvalMessage);
                
                // Add the AI placeholder back at the end
                this.sessionState.messages.push(currentAIMessage);
                
                console.log('Added approval, total messages:', this.sessionState.messages.length);
                this.notifyListeners();
                
                // If callback provided (for programmatic approval), use it
                if (onToolApproval) {
                  const approved = await onToolApproval(data.approval_id, data.tool_calls);
                  
                  // Update approval message status
                  approvalMessage.toolApproval!.status = approved ? 'approved' : 'rejected';
                  this.notifyListeners();
                  
                  // Send approval response
                  await fetch('/api/plugin-proxy/wisevision-wiseos-app/agent_backend/approve-tool', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                      approval_id: data.approval_id,
                      approved: approved,
                    }),
                  });
                }
                break;

              case 'tool_approved':
                // Create separate message for approval confirmation
                const approvedMsg: SimpleChatMessage = {
                  role: 'assistant',
                  content: `${data.content}`,
                  timestamp: new Date(),
                  id: this.generateId(),
                };
                // Remove AI placeholder, add this message, re-add placeholder
                const approvedIdx = this.sessionState.messages.findIndex(msg => msg.id === currentAIMessage.id);
                if (approvedIdx !== -1) {
                  this.sessionState.messages.splice(approvedIdx, 1);
                }
                this.sessionState.messages.push(approvedMsg);
                this.sessionState.messages.push(currentAIMessage);
                this.notifyListeners();
                break;

              case 'tool_rejected':
                // Create separate message for rejection
                const rejectedMsg: SimpleChatMessage = {
                  role: 'assistant',
                  content: `${data.content}`,
                  timestamp: new Date(),
                  id: this.generateId(),
                };
                // Remove AI placeholder, add this message, re-add placeholder
                const rejectedIdx = this.sessionState.messages.findIndex(msg => msg.id === currentAIMessage.id);
                if (rejectedIdx !== -1) {
                  this.sessionState.messages.splice(rejectedIdx, 1);
                }
                this.sessionState.messages.push(rejectedMsg);
                this.sessionState.messages.push(currentAIMessage);
                this.notifyListeners();
                break;

              case 'tool_start':
                // Create separate message for tool execution start
                const startMsg: SimpleChatMessage = {
                  role: 'assistant',
                  content: `⚙️ Executing **${data.tool_name}**...`,
                  timestamp: new Date(),
                  id: this.generateId(),
                };
                // Remove AI placeholder, add this message, re-add placeholder
                const startIdx = this.sessionState.messages.findIndex(msg => msg.id === currentAIMessage.id);
                if (startIdx !== -1) {
                  this.sessionState.messages.splice(startIdx, 1);
                }
                this.sessionState.messages.push(startMsg);
                this.sessionState.messages.push(currentAIMessage);
                this.notifyListeners();
                break;

              case 'tool_end':
                // Create separate message for tool completion
                const endMsg: SimpleChatMessage = {
                  role: 'assistant',
                  content: `**${data.tool_name}** completed`,
                  timestamp: new Date(),
                  id: this.generateId(),
                };
                // Remove AI placeholder, add this message, re-add placeholder
                const endIdx = this.sessionState.messages.findIndex(msg => msg.id === currentAIMessage.id);
                if (endIdx !== -1) {
                  this.sessionState.messages.splice(endIdx, 1);
                }
                this.sessionState.messages.push(endMsg);
                this.sessionState.messages.push(currentAIMessage);
                this.notifyListeners();
                break;

              case 'done':
                // Stream completed
                break;

              case 'error':
                // Show error
                currentAIMessage.content = `❌ Error: ${data.content}`;
                this.notifyListeners();
                break;
            }
          }
        }
      }

    } catch (error) {
      console.error('Stream request failed:', error);
      
      const errorContent = error instanceof Error 
        ? `❌ **Streaming Error**\n\n${error.message}`
        : '❌ **Streaming Error**\n\nFailed to connect to streaming endpoint';
      
      currentAIMessage.content = errorContent;
      this.notifyListeners();
    } finally {
      this.sessionState.loading = false;
      this.notifyListeners();
    }
  }

  async handleToolApproval(approvalId: string, approved: boolean) {
    // Find the approval message
    const approvalMsg = this.sessionState.messages.find(
      msg => msg.toolApproval?.approvalId === approvalId && msg.toolApproval?.status === 'pending'
    );
    
    if (!approvalMsg || !approvalMsg.toolApproval) {
      console.error('Approval message not found:', approvalId);
      return;
    }
    
    // Update status
    approvalMsg.toolApproval.status = approved ? 'approved' : 'rejected';
    this.notifyListeners();
    
    // Send approval to backend
    try {
      await fetch('/api/plugin-proxy/wisevision-wiseos-app/agent_backend/approve-tool', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          approval_id: approvalId,
          approved: approved,
        }),
      });
    } catch (error) {
      console.error('Failed to send approval:', error);
    }
  }

  private generateId(): string {
    return Date.now().toString() + Math.random().toString(36).substr(2, 9);
  }
}
