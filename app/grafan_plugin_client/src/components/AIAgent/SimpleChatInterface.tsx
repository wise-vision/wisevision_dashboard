/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import React, { useState, useRef, useEffect } from 'react';
import { css } from '@emotion/css';
import { useTheme2, Button, Input } from '@grafana/ui';
import { GrafanaTheme2 } from '@grafana/data';
import { useSimpleSession } from './SimpleSessionProvider';

const getStyles = (theme: GrafanaTheme2) => ({
  container: css`
    flex: 1;
    display: flex;
    flex-direction: column;
    height: 100%;
  `,
  messagesContainer: css`
    flex: 1;
    overflow-y: auto;
    padding: ${theme.spacing(2)};
    min-height: 0;
  `,
  message: css`
    margin-bottom: ${theme.spacing(2)};
    max-width: 80%;
  `,
  userMessage: css`
    margin-left: auto;
    background: ${theme.colors.primary.main};
    color: ${theme.colors.primary.contrastText};
    padding: ${theme.spacing(1)} ${theme.spacing(2)};
    border-radius: ${theme.shape.radius.default};
  `,
  assistantMessage: css`
    background: ${theme.colors.background.canvas};
    border: 1px solid ${theme.colors.border.weak};
    padding: ${theme.spacing(1)} ${theme.spacing(2)};
    border-radius: ${theme.shape.radius.default};
  `,
  messageContent: css`
    white-space: pre-wrap;
    line-height: 1.4;
    margin: 0;
  `,
  timestamp: css`
    font-size: ${theme.typography.bodySmall.fontSize};
    color: ${theme.colors.text.secondary};
    margin-top: ${theme.spacing(0.5)};
  `,
  inputContainer: css`
    padding: ${theme.spacing(2)};
    border-top: 1px solid ${theme.colors.border.weak};
    background: ${theme.colors.background.canvas};
  `,
  inputRow: css`
    display: flex;
    gap: ${theme.spacing(1)};
  `,
  input: css`
    flex: 1;
  `,
  emptyState: css`
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    height: 100%;
    color: ${theme.colors.text.secondary};
    text-align: center;
  `,
  emptyIcon: css`
    width: 48px;
    height: 48px;
    margin-bottom: ${theme.spacing(2)};
    opacity: 0.5;
  `,
  connectionStatus: css`
    padding: ${theme.spacing(1)} ${theme.spacing(2)};
    text-align: center;
    font-size: ${theme.typography.bodySmall.fontSize};
  `,
  connected: css`
    background: ${theme.colors.success.main};
    color: ${theme.colors.success.contrastText};
  `,
  disconnected: css`
    background: ${theme.colors.warning.main};
    color: ${theme.colors.warning.contrastText};
  `,
  toolbarContainer: css`
    padding: ${theme.spacing(1)} ${theme.spacing(2)};
    border-bottom: 1px solid ${theme.colors.border.weak};
    background: ${theme.colors.background.canvas};
    display: flex;
    justify-content: between;
    align-items: center;
  `,
  clearButton: css`
    margin-left: auto;
  `,
});

export function SimpleChatInterface() {
  const theme = useTheme2();
  const styles = getStyles(theme);
  const { sessionManager, sessionState } = useSimpleSession();
  const [input, setInput] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [sessionState.messages]);

  const handleSendMessage = async () => {
    if (!input.trim() || sessionState.loading) {
      return;
    }

    await sessionManager.sendMessage(input.trim());
    setInput('');
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleClearMessages = () => {
    sessionManager.clearMessages();
  };

  const formatTimestamp = (date: Date) => {
    return date.toLocaleTimeString('en-US', {
      hour: '2-digit',
      minute: '2-digit',
    });
  };

  return (
    <div className={styles.container}>
      {/* Connection Status */}
      <div className={`${styles.connectionStatus} ${sessionState.connected ? styles.connected : styles.disconnected}`}>
        {sessionState.connected ? '🟢 Connected to AI Agent' : '🔴 Disconnected - Check if backend is running'}
      </div>

      {/* Toolbar */}
      <div className={styles.toolbarContainer}>
        <span>{sessionState.messages.length} messages</span>
        {sessionState.messages.length > 0 && (
          <Button
            variant="secondary"
            size="sm"
            icon="trash-alt"
            className={styles.clearButton}
            onClick={handleClearMessages}
          >
            Clear
          </Button>
        )}
      </div>

      {/* Messages */}
      <div className={styles.messagesContainer}>
        {sessionState.messages.length === 0 ? (
          <div className={styles.emptyState}>
            <svg
              className={styles.emptyIcon}
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={1}
                d="M8 12h.01M12 12h.01M16 12h.01M21 12c0 4.418-4.03 8-9 8a9.863 9.863 0 01-4.255-.949L3 20l1.395-3.72C3.512 15.042 3 13.574 3 12c0-4.418 4.03-8 9-8s9 3.582 9 8z"
              />
            </svg>
            <h3>Ready to chat!</h3>
            <p>
              Start a conversation with your AI agent.
              <br />
              No configuration needed - just type and send!
            </p>
          </div>
        ) : (
          sessionState.messages.map((message) => (
            <div
              key={message.id}
              className={`${styles.message} ${
                message.role === 'user' ? styles.userMessage : styles.assistantMessage
              }`}
            >
              <div className={styles.messageContent}>{message.content}</div>
              <div className={styles.timestamp}>
                {formatTimestamp(message.timestamp)}
              </div>
            </div>
          ))
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <div className={styles.inputContainer}>
        <div className={styles.inputRow}>
          <Input
            className={styles.input}
            value={input}
            onChange={(e) => setInput(e.currentTarget.value)}
            onKeyPress={handleKeyPress}
            placeholder={
              sessionState.connected 
                ? "Type your message here..." 
                : "Backend not connected - please start the AI agent service"
            }
            disabled={!sessionState.connected || sessionState.loading}
          />
          <Button
            variant="primary"
            icon="arrow-right"
            onClick={handleSendMessage}
            disabled={!input.trim() || !sessionState.connected || sessionState.loading}
          >
            {sessionState.loading ? 'Sending...' : 'Send'}
          </Button>
        </div>
      </div>
    </div>
  );
}
