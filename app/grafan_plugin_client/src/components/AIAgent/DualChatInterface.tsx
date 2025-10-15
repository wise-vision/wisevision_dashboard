import React, { useState, useRef, useEffect } from 'react';
import { css } from '@emotion/css';
import { useTheme2, Button, Input, Select, Alert, Badge } from '@grafana/ui';
import { GrafanaTheme2, SelectableValue } from '@grafana/data';
import { useSimpleSession } from './SimpleSessionProvider';
import { MCPSessionManager } from '../services/MCPSessionManager';
import { MCPSettingsModal, MCPServer } from './MCPSettingsModal';

type ChatMode = 'simple' | 'mcp';

const chatModeOptions = [
  { label: '💬 Simple Chat', value: 'simple' as ChatMode, description: 'Direct OpenAI API calls' },
  { label: '🔧 MCP Chat', value: 'mcp' as ChatMode, description: 'AI Agent with tools and context' },
];

const getStyles = (theme: GrafanaTheme2) => ({
  container: css`
    flex: 1;
    display: flex;
    flex-direction: column;
    height: 100%;
  `,
  header: css`
    padding: ${theme.spacing(2)};
    border-bottom: 1px solid ${theme.colors.border.weak};
    background: ${theme.colors.background.secondary};
    display: flex;
    align-items: center;
    gap: ${theme.spacing(2)};
  `,
  modeSelector: css`
    min-width: 200px;
  `,
  sessionInfo: css`
    flex: 1;
    display: flex;
    align-items: center;
    gap: ${theme.spacing(1)};
    font-size: ${theme.typography.bodySmall.fontSize};
    color: ${theme.colors.text.secondary};
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
    display: flex;
    gap: ${theme.spacing(1)};
  `,
  messageInput: css`
    flex: 1;
  `,
  sendButton: css`
    white-space: nowrap;
  `,
  modeAlert: css`
    margin: ${theme.spacing(2)};
    margin-bottom: 0;
  `,
  loadingIndicator: css`
    text-align: center;
    padding: ${theme.spacing(2)};
    color: ${theme.colors.text.secondary};
    font-style: italic;
  `,
});

export const DualChatInterface: React.FC = () => {
  const theme = useTheme2();
  const styles = getStyles(theme);
  const [chatMode, setChatMode] = useState<ChatMode>('simple');
  const [message, setMessage] = useState('');
  const [showModeChangeAlert, setShowModeChangeAlert] = useState(false);
  const [showMCPSettings, setShowMCPSettings] = useState(false);
  const [mcpServers, setMcpServers] = useState<MCPServer[]>([]);
  const [mcpSessionManager] = useState(() => new MCPSessionManager());
  const [mcpSessionState, setMcpSessionState] = useState(() => mcpSessionManager.getState());
  const messagesEndRef = useRef<HTMLDivElement>(null);
  
  // Use the existing simple session for simple mode
  const { sessionManager: simpleSessionManager, sessionState: simpleSessionState } = useSimpleSession();
  
  // Choose the appropriate session based on mode
  const currentSessionManager = chatMode === 'simple' ? simpleSessionManager : mcpSessionManager;
  const currentSessionState = chatMode === 'simple' ? simpleSessionState : mcpSessionState;

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Subscribe to MCP session updates
  useEffect(() => {
    const unsubscribe = mcpSessionManager.subscribe(setMcpSessionState);
    mcpSessionManager.connect();
    return () => {
      unsubscribe();
    };
  }, [mcpSessionManager]);

  useEffect(() => {
    scrollToBottom();
  }, [currentSessionState.messages]);

  const handleModeChange = (selection: SelectableValue<ChatMode>) => {
    if (selection.value && selection.value !== chatMode) {
      // Show alert before changing mode
      setShowModeChangeAlert(true);
      setTimeout(() => {
        setChatMode(selection.value!);
        currentSessionManager.clearMessages(); // Restart session when changing modes
        setShowModeChangeAlert(false);
        console.log(`Switched to ${selection.value} mode and cleared session`);
      }, 1500);
    }
  };

  const handleSendMessage = async () => {
    if (!message.trim() || currentSessionState.loading) {
      return;
    }

    if (chatMode === 'simple') {
      // Use existing simple chat
      await currentSessionManager.sendMessage(message);
    } else if (chatMode === 'mcp') {
      // Use MCP session manager
      await currentSessionManager.sendMessage(message);
    }

    setMessage('');
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const currentModeOption = chatModeOptions.find(option => option.value === chatMode);
  const messageCount = currentSessionState.messages.length;
  const hasMessages = messageCount > 0;

  return (
    <div className={styles.container}>
      {/* Header with mode selector */}
      <div className={styles.header}>
        <div className={styles.modeSelector}>
          <Select
            value={currentModeOption}
            options={chatModeOptions}
            onChange={handleModeChange}
            placeholder="Select chat mode"
          />
        </div>
        <div className={styles.sessionInfo}>
          <Badge color="blue" text={chatMode.toUpperCase()} />
          <span>•</span>
          <span>{messageCount} messages</span>
          {hasMessages && (
            <>
              <span>•</span>
              <Button
                variant="secondary"
                size="sm"
                onClick={() => {
                  currentSessionManager.clearMessages();
                  console.log('Session cleared manually');
                }}
              >
                Clear Session
              </Button>
            </>
          )}
          <span>•</span>
          <Button
            variant="secondary"
            size="sm"
            onClick={() => setShowMCPSettings(true)}
          >
            ⚙️ MCP Settings
          </Button>
        </div>
      </div>

      {/* Mode change alert */}
      {showModeChangeAlert && (
        <Alert title="Switching Chat Mode" severity="info" className={styles.modeAlert}>
          Session will be restarted to avoid mixing different conversation types...
        </Alert>
      )}

      {/* Mode-specific info */}
      {chatMode === 'mcp' && (
        <Alert title="MCP Chat Mode" severity="info" className={styles.modeAlert}>
          🔧 <strong>Enhanced AI Agent</strong> with access to tools, context, and multi-step reasoning.
          {mcpServers.length > 0 ? (
            <span> Using {mcpServers.filter(s => s.enabled).length} of {mcpServers.length} configured servers.</span>
          ) : (
            <span> <strong>No MCP servers configured.</strong> Click &quot;MCP Settings&quot; to add servers.</span>
          )}
        </Alert>
      )}

      {chatMode === 'simple' && hasMessages === false && (
        <Alert title="Simple Chat Mode" severity="info" className={styles.modeAlert}>
          💬 <strong>Direct OpenAI API</strong> for basic conversations without additional tools or context.
        </Alert>
      )}

      {/* Messages */}
      <div className={styles.messagesContainer}>
        {currentSessionState.messages.map((msg: any) => (
          <div
            key={msg.id}
            className={`${styles.message} ${
              msg.role === 'user' ? styles.userMessage : styles.assistantMessage
            }`}
          >
            <pre className={styles.messageContent}>{msg.content}</pre>
            <div className={styles.timestamp}>
              {msg.timestamp.toLocaleTimeString()}
            </div>
          </div>
        ))}
        
        {currentSessionState.loading && (
          <div className={styles.loadingIndicator}>
            🤖 AI is thinking...
          </div>
        )}
        
        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <div className={styles.inputContainer}>
        <Input
          className={styles.messageInput}
          value={message}
          onChange={(e) => setMessage(e.currentTarget.value)}
          onKeyPress={handleKeyPress}
          placeholder={
            chatMode === 'simple' 
              ? "Ask me anything..." 
              : "Ask me anything (with tools and context)..."
          }
          disabled={currentSessionState.loading}
        />
        <Button
          className={styles.sendButton}
          onClick={handleSendMessage}
          disabled={!message.trim() || currentSessionState.loading}
          variant="primary"
        >
          {currentSessionState.loading ? 'Sending...' : 'Send'}
        </Button>
      </div>

      {/* MCP Settings Modal */}
      <MCPSettingsModal
        isOpen={showMCPSettings}
        onClose={() => setShowMCPSettings(false)}
        servers={mcpServers}
        onServersChange={setMcpServers}
      />
    </div>
  );
};
