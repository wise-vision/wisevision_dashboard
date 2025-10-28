/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import { MCPServerConfig, SessionState, ChatMessage, ServerEvent } from '../AIAgent/types';
import { getBackendSrv } from '@grafana/runtime';
import { lastValueFrom } from 'rxjs';

export class SessionManager {
  private baseUrl: string;
  private sessionState: SessionState;
  private eventSource: EventSource | null = null;
  private listeners: Set<(state: SessionState) => void> = new Set();

  constructor(baseUrl?: string) {
    // Use provided baseUrl or auto-detect based on current hostname
    if (baseUrl) {
      this.baseUrl = baseUrl;
    } else {
      const hostname = window.location.hostname;
      if (hostname === 'localhost' || hostname === '127.0.0.1') {
        this.baseUrl = 'http://localhost:8089';
      } else {
        // Use the same hostname as frontend but different port
        this.baseUrl = `http://${hostname}:8089`;
      }
    }
    
    console.log(`SessionManager initialized with baseUrl: ${this.baseUrl}`);
    
    this.sessionState = {
      id: null,
      mcpConfig: {},
      messages: [],
      isConnected: false,
    };
  }

  subscribe(listener: (state: SessionState) => void) {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  private notifyListeners() {
    this.listeners.forEach(listener => listener(this.sessionState));
  }

  private async getOpenAIApiKey(): Promise<string | null> {
    try {
      // Get plugin settings from Grafana
      const response = await lastValueFrom(
        getBackendSrv().fetch({
          url: '/api/plugins/wisevision-wiseos-app/settings',
          method: 'GET'
        })
      );
      
      // Check if OpenAI API key is configured in plugin settings
      const data = response.data as any;
      if (data?.secureJsonData?.openaiApiKey) {
        return data.secureJsonData.openaiApiKey;
      }
      
      return null;
    } catch (error) {
      console.warn('Could not get OpenAI API key from plugin settings:', error);
      return null;
    }
  }

  async createSession(mcpConfig: Record<string, Omit<MCPServerConfig, 'name'>>) {
    try {
      console.log(`Attempting to create session at: ${this.baseUrl}/session`);
      
      // Get OpenAI API key from plugin settings
      const openaiApiKey = await this.getOpenAIApiKey();
      console.log('OpenAI API key from plugin settings:', openaiApiKey ? '✅ Found' : '❌ Not found');
      
      const response = await fetch(`${this.baseUrl}/session`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json',
        },
        body: JSON.stringify({ 
          mcp_config: mcpConfig,
          openai_api_key: openaiApiKey 
        }),
      });

      if (!response.ok) {
        const errorText = await response.text().catch(() => 'Unknown error');
        throw new Error(`Failed to create session (${response.status}): ${errorText}`);
      }

      const result = await response.json();
      this.sessionState = {
        ...this.sessionState,
        id: result.sessionId,
        mcpConfig,
        messages: [],
        isConnected: true,
      };

      this.connectToEvents();
      this.notifyListeners();
      return result.sessionId;
    } catch (error) {
      console.error('Failed to create session:', error);
      // Check if it's a network error
      if (error instanceof TypeError && error.message.includes('fetch')) {
        throw new Error(`Cannot connect to AI Agent backend at ${this.baseUrl}. Please ensure the backend is running.`);
      }
      throw error;
    }
  }

  private connectToEvents() {
    if (!this.sessionState.id) {return;}

    this.eventSource?.close();
    this.eventSource = new EventSource(`${this.baseUrl}/events/${this.sessionState.id}`);

    this.eventSource.onmessage = (event) => {
      try {
        const data: ServerEvent = JSON.parse(event.data);
        this.handleServerEvent(data);
      } catch (error) {
        console.error('Failed to parse server event:', error);
      }
    };

    this.eventSource.onerror = () => {
      this.sessionState.isConnected = false;
      this.notifyListeners();
    };
  }

  private handleServerEvent(event: ServerEvent) {
    switch (event.type) {
      case 'chat.assistant':
        if (event.content) {
          const message: ChatMessage = {
            role: 'assistant',
            content: event.content,
            timestamp: new Date(),
            id: this.generateId(),
          };
          this.sessionState.messages.push(message);
          this.notifyListeners();
        }
        break;
      
      case 'chat.user':
        if (event.content) {
          const message: ChatMessage = {
            role: 'user',
            content: event.content,
            timestamp: new Date(),
            id: this.generateId(),
          };
          this.sessionState.messages.push(message);
          this.notifyListeners();
        }
        break;

      case 'error':
        console.error('Server error:', event.error);
        break;
    }
  }

  async sendMessage(content: string) {
    if (!this.sessionState.id) {
      throw new Error('No active session');
    }

    try {
      // Get OpenAI API key from plugin settings
      const openaiApiKey = await this.getOpenAIApiKey();
      
      const response = await fetch(`${this.baseUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          sessionId: this.sessionState.id,
          content,
          openai_api_key: openaiApiKey,
        }),
      });

      if (!response.ok) {
        throw new Error(`Failed to send message: ${response.statusText}`);
      }

      // User message will be added via server event
    } catch (error) {
      console.error('Failed to send message:', error);
      throw error;
    }
  }

  async getAvailableTools() {
    if (!this.sessionState.id) {
      throw new Error('No active session');
    }

    try {
      const response = await fetch(`${this.baseUrl}/tools/list?session_id=${this.sessionState.id}`);
      
      if (!response.ok) {
        throw new Error(`Failed to get tools: ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Failed to get available tools:', error);
      throw error;
    }
  }

  getState(): SessionState {
    return { ...this.sessionState };
  }

  disconnect() {
    this.eventSource?.close();
    this.eventSource = null;
    this.sessionState.isConnected = false;
    this.notifyListeners();
  }

  private generateId(): string {
    return Math.random().toString(36).substr(2, 9);
  }
}
