import React, { useState, useRef, useEffect } from 'react';
import { css } from '@emotion/css';
import { useTheme2, Button, Input, Alert, Badge, Modal } from '@grafana/ui';
import { GrafanaTheme2 } from '@grafana/data';
import { MCPSessionManager } from '../services/MCPSessionManager';
import { MCPSettingsModal, MCPServer } from './MCPSettingsModal';
import { MarkdownRenderer } from './MarkdownRenderer';
import { PromptsModal } from './PromptsModal';

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
    max-width: 85%;
  `,
  userMessage: css`
    margin-left: auto;
    background: ${theme.colors.primary.main};
    color: ${theme.colors.primary.contrastText};
    padding: ${theme.spacing(1)} ${theme.spacing(2)};
    border-radius: ${theme.shape.radius.default};
    word-wrap: break-word;
  `,
  assistantMessage: css`
    background: ${theme.colors.background.canvas};
    border: 1px solid ${theme.colors.border.weak};
    padding: ${theme.spacing(1)} ${theme.spacing(2)};
    border-radius: ${theme.shape.radius.default};
    word-wrap: break-word;
  `,
  messageContent: css`
    line-height: 1.6;
    margin: 0;
    
    p {
      margin: ${theme.spacing(1)} 0;
      
      &:first-child {
        margin-top: 0;
      }
      
      &:last-child {
        margin-bottom: 0;
      }
    }
    
    ul, ol {
      margin: ${theme.spacing(1)} 0;
      padding-left: ${theme.spacing(3)};
      
      li {
        margin-bottom: ${theme.spacing(0.5)};
      }
    }
    
    code {
      background: ${theme.colors.background.secondary};
      padding: 2px 6px;
      border-radius: 3px;
      font-family: ${theme.typography.fontFamilyMonospace};
      font-size: 0.9em;
    }
    
    pre {
      background: ${theme.colors.background.secondary};
      padding: ${theme.spacing(1)};
      border-radius: ${theme.shape.radius.default};
      overflow-x: auto;
      margin: ${theme.spacing(1)} 0;
      
      code {
        background: none;
        padding: 0;
      }
    }
    
    blockquote {
      border-left: 4px solid ${theme.colors.border.medium};
      margin: ${theme.spacing(1)} 0;
      padding: ${theme.spacing(1)} ${theme.spacing(2)};
      color: ${theme.colors.text.secondary};
      background: ${theme.colors.background.secondary};
      border-radius: ${theme.shape.radius.default};
    }
    
    h1, h2, h3, h4, h5, h6 {
      margin: ${theme.spacing(1.5)} 0 ${theme.spacing(0.5)} 0;
      font-weight: ${theme.typography.fontWeightBold};
    }
    
    h1 {
      font-size: 1.5em;
    }
    
    h2 {
      font-size: 1.3em;
    }
    
    h3 {
      font-size: 1.1em;
    }
    
    strong {
      font-weight: ${theme.typography.fontWeightBold};
    }
    
    em {
      font-style: italic;
    }
    
    a {
      color: ${theme.colors.primary.main};
      text-decoration: none;
      
      &:hover {
        text-decoration: underline;
      }
    }
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
    display: flex;
    align-items: center;
    justify-content: center;
    gap: ${theme.spacing(1)};
  `,
  loadingDots: css`
    display: flex;
    gap: 4px;
    
    span {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: ${theme.colors.primary.main};
      animation: bounce 1.4s infinite ease-in-out both;
      
      &:nth-child(1) {
        animation-delay: -0.32s;
      }
      
      &:nth-child(2) {
        animation-delay: -0.16s;
      }
    }
    
    @keyframes bounce {
      0%, 80%, 100% {
        transform: scale(0);
        opacity: 0.5;
      }
      40% {
        transform: scale(1);
        opacity: 1;
      }
    }
  `,
  mcpServersButton: css`
    margin: ${theme.spacing(2)};
    margin-bottom: 0;
    padding: ${theme.spacing(1)} ${theme.spacing(2)};
    background: ${theme.colors.background.secondary};
    border: 1px solid ${theme.colors.border.weak};
    border-radius: ${theme.shape.radius.default};
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: space-between;
    transition: all 0.2s;
    
    &:hover {
      background: ${theme.colors.emphasize(theme.colors.background.secondary, 0.03)};
      border-color: ${theme.colors.border.medium};
    }
  `,
  mcpServersButtonContent: css`
    display: flex;
    align-items: center;
    gap: ${theme.spacing(1)};
    flex: 1;
  `,
  mcpServersIcon: css`
    font-size: 16px;
    transition: transform 0.2s;
    
    &.expanded {
      transform: rotate(90deg);
    }
  `,
  mcpServersList: css`
    margin: 0 ${theme.spacing(2)} ${theme.spacing(2)};
    padding: ${theme.spacing(1.5)};
    background: ${theme.colors.background.secondary};
    border: 1px solid ${theme.colors.border.weak};
    border-top: none;
    border-bottom-left-radius: ${theme.shape.radius.default};
    border-bottom-right-radius: ${theme.shape.radius.default};
  `,
  mcpServerItem: css`
    display: flex;
    align-items: center;
    gap: ${theme.spacing(1)};
    padding: ${theme.spacing(0.5)} 0;
    font-size: ${theme.typography.bodySmall.fontSize};
    
    &:not(:last-child) {
      border-bottom: 1px solid ${theme.colors.border.weak};
      padding-bottom: ${theme.spacing(1)};
      margin-bottom: ${theme.spacing(0.5)};
    }
  `,
  mcpServerName: css`
    color: ${theme.colors.text.primary};
    font-weight: ${theme.typography.fontWeightMedium};
  `,
  mcpServerCommand: css`
    color: ${theme.colors.text.secondary};
    font-family: ${theme.typography.fontFamilyMonospace};
    font-size: 0.85em;
    margin-left: auto;
  `,
  mcpServersModalContent: css`
    padding: ${theme.spacing(2)};
  `,
  mcpServersModalHeader: css`
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: ${theme.spacing(2)};
    padding-bottom: ${theme.spacing(1)};
    border-bottom: 1px solid ${theme.colors.border.weak};
  `,
  mcpServersModalTitle: css`
    display: flex;
    align-items: center;
    gap: ${theme.spacing(1)};
    font-size: ${theme.typography.h4.fontSize};
    font-weight: ${theme.typography.fontWeightBold};
  `,
  mcpServersModalList: css`
    display: flex;
    flex-direction: column;
    gap: ${theme.spacing(1)};
  `,
  mcpServerModalItem: css`
    display: flex;
    align-items: center;
    gap: ${theme.spacing(2)};
    padding: ${theme.spacing(1.5)};
    background: ${theme.colors.background.secondary};
    border: 1px solid ${theme.colors.border.weak};
    border-radius: ${theme.shape.radius.default};
    
    &:hover {
      background: ${theme.colors.emphasize(theme.colors.background.secondary, 0.03)};
    }
  `,
  mcpServerModalInfo: css`
    flex: 1;
    display: flex;
    flex-direction: column;
    gap: ${theme.spacing(0.5)};
  `,
  mcpServerModalName: css`
    font-weight: ${theme.typography.fontWeightMedium};
    color: ${theme.colors.text.primary};
  `,
  mcpServerModalCommand: css`
    font-family: ${theme.typography.fontFamilyMonospace};
    font-size: ${theme.typography.bodySmall.fontSize};
    color: ${theme.colors.text.secondary};
    word-break: break-all;
    overflow-wrap: break-word;
    line-height: 1.4;
    max-width: 100%;
    cursor: pointer;
    padding: ${theme.spacing(0.5)};
    border-radius: ${theme.shape.radius.default};
    transition: all 0.2s;
    
    &:hover {
      background: ${theme.colors.background.canvas};
      color: ${theme.colors.text.primary};
    }
  `,
  mcpServerModalCommandExpanded: css`
    background: ${theme.colors.background.canvas};
    border: 1px solid ${theme.colors.border.weak};
    padding: ${theme.spacing(1)};
  `,
  mcpServerModalBadges: css`
    display: flex;
    gap: ${theme.spacing(1)};
  `,
});

export const DualChatInterface: React.FC = () => {
  const theme = useTheme2();
  const styles = getStyles(theme);
  const [message, setMessage] = useState('');
  const [showMCPSettings, setShowMCPSettings] = useState(false);
  const [showPromptsModal, setShowPromptsModal] = useState(false);
  const [mcpServers, setMcpServers] = useState<MCPServer[]>([]);
  const [mcpSessionManager] = useState(() => new MCPSessionManager());
  const [mcpSessionState, setMcpSessionState] = useState(() => mcpSessionManager.getState());
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const [loadingServers, setLoadingServers] = useState(true);
  const [showMCPServersList, setShowMCPServersList] = useState(false);
  const [expandedModalServers, setExpandedModalServers] = useState<Set<string>>(new Set());
  
  // Tool approval state (streaming is always enabled)
  const [requireToolApproval, setRequireToolApproval] = useState(true);
  
  // Use MCP session manager for unified chat
  const sessionManager = mcpSessionManager;
  const sessionState = mcpSessionState;

  // Helper function to format command display
  const formatCommand = (command: string, args: string[]) => {
    const fullCommand = `${command} ${args.join(' ')}`;
    // If command is too long, show command + first arg + "..."
    if (fullCommand.length > 80) {
      const firstArg = args.length > 0 ? args[0] : '';
      if (firstArg.length > 50) {
        // If first arg is very long (like a path), show just the filename
        const parts = firstArg.split('/');
        const filename = parts[parts.length - 1];
        return `${command} .../${filename}${args.length > 1 ? ' +' + (args.length - 1) + ' args' : ''}`;
      }
      return `${command} ${firstArg}${args.length > 1 ? ' +' + (args.length - 1) + ' args' : ''}`;
    }
    return fullCommand;
  };

  const toggleModalServerExpanded = (serverId: string) => {
    setExpandedModalServers(prev => {
      const newSet = new Set(prev);
      if (newSet.has(serverId)) {
        newSet.delete(serverId);
      } else {
        newSet.add(serverId);
      }
      return newSet;
    });
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Subscribe to MCP session updates and fetch default servers
  useEffect(() => {
    const unsubscribe = mcpSessionManager.subscribe(setMcpSessionState);
    
    const initializeMCP = async () => {
      await mcpSessionManager.connect();
      
      // Fetch default MCP servers from backend
      try {
        const backendServers = await mcpSessionManager.fetchMCPServers();
        
        // Convert backend server format to frontend MCPServer format
        const convertedServers: MCPServer[] = backendServers.map(server => ({
          id: server.id,
          name: server.name,
          command: server.command || '',
          args: server.args || [],
          env: {},
          enabled: server.enabled,
          is_default: server.is_default || false,  // Mark as default from backend
        }));
        
        setMcpServers(convertedServers);
      } catch (error) {
        console.error('Failed to load default MCP servers:', error);
      } finally {
        setLoadingServers(false);
      }
    };
    
    initializeMCP();
    
    return () => {
      unsubscribe();
    };
  }, [mcpSessionManager]);

  useEffect(() => {
    scrollToBottom();
  }, [sessionState.messages, sessionState.loading]);

  // Also scroll when messages container is updated (for streaming tokens)
  useEffect(() => {
    const messagesContainer = document.querySelector(`.${styles.messagesContainer}`);
    if (messagesContainer) {
      const observer = new MutationObserver(() => {
        scrollToBottom();
      });
      
      observer.observe(messagesContainer, {
        childList: true,
        subtree: true,
        characterData: true,
      });
      
      return () => observer.disconnect();
    }
    
    // Return empty cleanup function if no container found
    return () => {};
  }, [styles.messagesContainer]);

  // Update session manager when MCP servers change
  useEffect(() => {
    if (mcpServers.length > 0) {
      // Convert MCPServer to MCPServerInfo format
      const serverInfos = mcpServers.map(server => ({
        id: server.id,
        name: server.name,
        command: server.command,
        args: server.args,
        transport: 'stdio' as const,
        enabled: server.enabled,
        is_default: server.is_default,
      }));
      
      mcpSessionManager.setMCPServers(serverInfos);
    }
  }, [mcpServers, mcpSessionManager]);

  const handleSendMessage = async () => {
    if (!message.trim() || sessionState.loading) {
      return;
    }

    // Always use streaming mode
    await sessionManager.streamMessage(message, requireToolApproval);
    setMessage('');
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleExecutePrompt = async (promptName: string, args: Record<string, any>, serverName: string) => {
    try {
      const { getBackendSrv } = await import('@grafana/runtime');
      const { lastValueFrom } = await import('rxjs');
      
      // Call the backend to execute the prompt
      const response = await lastValueFrom(
        getBackendSrv().fetch({
          url: '/api/plugin-proxy/wisevision-wiseos-app/agent_backend/mcp/prompts/execute',
          method: 'POST',
          data: {
            prompt_name: promptName,
            arguments: args,
            server_name: serverName,
          }
        })
      );

      if (!response.ok) {
        throw new Error('Failed to execute prompt');
      }

      const data = response.data as any;

      if (data.ok) {
        // The prompt execution returns messages from the MCP server
        // These messages contain system instructions and the user request
        if (data.messages && data.messages.length > 0) {
          // Add each message to the session state manually
          // This builds up the conversation context before sending to AI
          const promptMessages: any[] = [];
          
          for (const msg of data.messages) {
            const chatMessage = {
              role: msg.role === 'ai' ? 'assistant' : msg.role,
              content: msg.content,
              timestamp: new Date(),
              id: `prompt-${Date.now()}-${Math.random()}`,
            };
            promptMessages.push(chatMessage);
          }
          
          // Now we need to send these messages to the AI
          // We'll add them to history and send the last human message as the current request
          
          // Build the history from current messages + prompt messages (except the last one)
          const systemMessages = promptMessages.slice(0, -1); // All but last message
          const userRequest = promptMessages[promptMessages.length - 1]; // Last message should be the user request
          
          // Add the system messages to session history (context from the prompt)
          for (const msg of systemMessages) {
            sessionManager.addMessage({
              role: msg.role === 'ai' ? 'assistant' : msg.role,
              content: msg.content,
              timestamp: new Date(),
              id: `system-${Date.now()}-${Math.random()}`,
            });
          }
          
          // Always use streaming mode
          // Stream the AI response (streamMessage will add the user message automatically)
          // No callback needed - approval is handled inline in chat
          // DON'T await - let it stream in background so modal can close immediately
          sessionManager.streamMessage(userRequest.content, requireToolApproval);
          // Modal will close immediately while streaming continues
        } else {
          // Fallback: if no messages, send a description
          const argsStr = Object.keys(args).length > 0 
            ? ` with arguments: ${JSON.stringify(args, null, 2)}`
            : '';
          await sessionManager.sendMessage(`Execute prompt: ${promptName} from ${serverName}${argsStr}`);
        }
      } else {
        throw new Error(data.error || 'Failed to execute prompt');
      }
    } catch (error) {
      console.error('Error executing prompt:', error);
      // Add error message to chat instead of throwing
      sessionManager.addMessage({
        role: 'assistant',
        content: `❌ Error executing prompt: ${error instanceof Error ? error.message : 'Unknown error'}`,
        timestamp: new Date(),
        id: `error-${Date.now()}`,
      });
      // Don't re-throw so modal can close
    }
  };

  const messageCount = sessionState.messages.length;
  const hasMessages = messageCount > 0;

  return (
    <div className={styles.container}>
      {/* Header */}
      <div className={styles.header}>
        <div>
          <h3 style={{ margin: 0, marginBottom: '4px' }}>AI Agent Chat</h3>
          <p style={{ margin: 0, fontSize: '12px', color: theme.colors.text.secondary }}>
            Enhanced AI with tools and context
          </p>
        </div>
        <div className={styles.sessionInfo}>
          <Badge color="blue" text={`${messageCount} messages`} />
          {hasMessages && (
            <>
              <Button
                variant="secondary"
                size="sm"
                onClick={() => {
                  sessionManager.clearMessages();
                  console.log('Session cleared manually');
                }}
              >
                Clear Session
              </Button>
            </>
          )}
          {!loadingServers && mcpServers.length > 0 && (
            <Button
              variant="secondary"
              size="sm"
              onClick={() => setShowMCPServersList(true)}
            >
              {mcpServers.filter(s => s.enabled).length} MCP Server{mcpServers.filter(s => s.enabled).length !== 1 ? 's' : ''}
            </Button>
          )}
          <Button
            variant="secondary"
            size="sm"
            icon="list-ul"
            onClick={() => setShowPromptsModal(true)}
            title="View available MCP prompts"
          >
            Prompts
          </Button>
          <div 
            onClick={() => setRequireToolApproval(!requireToolApproval)}
            style={{ cursor: 'pointer' }}
            title="Click to toggle tool approval requirement"
          >
            <Badge 
              color={requireToolApproval ? "blue" : "purple"} 
              text={requireToolApproval ? "Approval required" : "Auto-execute"}
            />
          </div>
          <Button
            variant="secondary"
            size="sm"
            onClick={() => setShowMCPSettings(true)}
          >
            MCP Settings
          </Button>
        </div>
      </div>

      {/* MCP Status Info */}
      {loadingServers && hasMessages === false && (
        <Alert title="Loading MCP Servers..." severity="info" className={styles.modeAlert}>
          Fetching MCP server configuration from backend...
        </Alert>
      )}
      
      {!loadingServers && mcpServers.length === 0 && hasMessages === false && (
        <Alert title="AI Agent Ready" severity="info" className={styles.modeAlert}>
          Start chatting with the AI agent. You can add MCP servers in settings to extend capabilities with tools and context.
        </Alert>
      )}

      {/* Messages */}
      <div className={styles.messagesContainer}>
        {sessionState.messages.map((msg: any) => (
          <div
            key={msg.id}
            className={`${styles.message} ${
              msg.role === 'user' ? styles.userMessage : styles.assistantMessage
            }`}
          >
            <div className={styles.messageContent}>
              {/* Tool Approval UI - Inline in chat */}
              {msg.toolApproval ? (
                <div>
                  <div style={{ 
                    padding: '12px', 
                    background: theme.colors.warning.transparent,
                    border: `2px solid ${theme.colors.warning.border}`,
                    borderRadius: '8px',
                    marginBottom: '8px'
                  }}>
                    <div style={{ fontWeight: 'bold', marginBottom: '8px' }}>
                      ⚠️ Tool Approval Required
                    </div>
                    {msg.toolApproval.toolCalls.map((tool: any, idx: number) => (
                      <div key={idx} style={{
                        background: theme.colors.background.secondary,
                        padding: '8px',
                        borderRadius: '4px',
                        marginBottom: '8px'
                      }}>
                        <div style={{ fontWeight: 500, marginBottom: '4px' }}>
                          🔧 {tool.name}
                        </div>
                        {Object.keys(tool.args).length > 0 && (
                          <pre style={{
                            fontSize: '11px',
                            background: theme.colors.background.canvas,
                            padding: '8px',
                            borderRadius: '4px',
                            overflow: 'auto',
                            maxHeight: '200px'
                          }}>
                            {JSON.stringify(tool.args, null, 2)}
                          </pre>
                        )}
                      </div>
                    ))}
                    
                    {/* Approval buttons or status */}
                    {msg.toolApproval.status === 'pending' ? (
                      <div style={{ display: 'flex', gap: '8px', marginTop: '8px' }}>
                        <Button
                          variant="destructive"
                          size="sm"
                          onClick={() => {
                            sessionManager.handleToolApproval(msg.toolApproval.approvalId, false);
                          }}
                        >
                          ✗ Reject
                        </Button>
                        <Button
                          variant="primary"
                          size="sm"
                          onClick={() => {
                            sessionManager.handleToolApproval(msg.toolApproval.approvalId, true);
                          }}
                        >
                          ✓ Approve
                        </Button>
                      </div>
                    ) : msg.toolApproval.status === 'approved' ? (
                      <div style={{ color: theme.colors.success.text, fontWeight: 500 }}>
                        Approved
                      </div>
                    ) : (
                      <div style={{ color: theme.colors.error.text, fontWeight: 500 }}>
                        Rejected
                      </div>
                    )}
                  </div>
                </div>
              ) : msg.role === 'assistant' ? (
                <MarkdownRenderer content={msg.content} />
              ) : (
                <>{msg.content}</>
              )}
            </div>
            <div className={styles.timestamp}>
              {msg.timestamp.toLocaleTimeString()}
            </div>
          </div>
        ))}
        
        {sessionState.loading && (
          <div className={styles.loadingIndicator}>
            <div className={styles.loadingDots}>
              <span></span>
              <span></span>
              <span></span>
            </div>
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
          placeholder="Ask me anything..."
          disabled={sessionState.loading}
        />
        <Button
          className={styles.sendButton}
          onClick={handleSendMessage}
          disabled={!message.trim() || sessionState.loading}
          variant="primary"
        >
          {sessionState.loading ? 'Sending...' : 'Send'}
        </Button>
      </div>

      {/* MCP Settings Modal */}
      <MCPSettingsModal
        isOpen={showMCPSettings}
        onClose={() => setShowMCPSettings(false)}
        servers={mcpServers}
        onServersChange={setMcpServers}
      />

      {/* Prompts Modal */}
      <PromptsModal
        isOpen={showPromptsModal}
        onClose={() => setShowPromptsModal(false)}
        onExecutePrompt={handleExecutePrompt}
      />

      {/* MCP Servers List Modal */}
      <Modal
        title=""
        isOpen={showMCPServersList}
        onDismiss={() => setShowMCPServersList(false)}
      >
        <div className={styles.mcpServersModalContent}>
          <div className={styles.mcpServersModalHeader}>
            <div className={styles.mcpServersModalTitle}>
              <span>Active MCP Servers</span>
            </div>
            <Badge 
              color="green" 
              text={`${mcpServers.filter(s => s.enabled).length} active`} 
            />
          </div>
          
          <div className={styles.mcpServersModalList}>
            {mcpServers.filter(s => s.enabled).length === 0 ? (
              <Alert severity="info" title="No active servers">
                No MCP servers are currently active. Add servers in MCP Settings to extend the AI agent&apos;s capabilities.
              </Alert>
            ) : (
              mcpServers.filter(s => s.enabled).map((server) => (
                <div key={server.id} className={styles.mcpServerModalItem}>
                  <div className={styles.mcpServerModalInfo}>
                    <div className={styles.mcpServerModalName}>
                      {server.name}
                    </div>
                    <div 
                      className={`${styles.mcpServerModalCommand} ${expandedModalServers.has(server.id) ? styles.mcpServerModalCommandExpanded : ''}`}
                      onClick={() => toggleModalServerExpanded(server.id)}
                      title="Click to show full command"
                    >
                      {expandedModalServers.has(server.id)
                        ? `${server.command} ${server.args.join(' ')}`
                        : formatCommand(server.command, server.args)
                      }
                    </div>
                  </div>
                  <div className={styles.mcpServerModalBadges}>
                    {server.is_default && (
                      <Badge color="blue" text="Default" />
                    )}
                    <Badge color="green" text="Active" />
                  </div>
                </div>
              ))
            )}
          </div>
          
          {mcpServers.filter(s => !s.enabled).length > 0 && (
            <div style={{ marginTop: '16px', paddingTop: '16px', borderTop: '1px solid rgba(204, 204, 220, 0.2)' }}>
              <div style={{ marginBottom: '8px', fontSize: '14px', fontWeight: 500, color: 'rgba(204, 204, 220, 0.7)' }}>
                Disabled Servers ({mcpServers.filter(s => !s.enabled).length})
              </div>
              <div className={styles.mcpServersModalList}>
                {mcpServers.filter(s => !s.enabled).map((server) => (
                  <div key={server.id} className={styles.mcpServerModalItem} style={{ opacity: 0.6 }}>
                    <div className={styles.mcpServerModalInfo}>
                      <div className={styles.mcpServerModalName}>
                        {server.name}
                      </div>
                      <div 
                        className={`${styles.mcpServerModalCommand} ${expandedModalServers.has(server.id) ? styles.mcpServerModalCommandExpanded : ''}`}
                        onClick={() => toggleModalServerExpanded(server.id)}
                        title="Click to show full command"
                      >
                        {expandedModalServers.has(server.id)
                          ? `${server.command} ${server.args.join(' ')}`
                          : formatCommand(server.command, server.args)
                        }
                      </div>
                    </div>
                    <div className={styles.mcpServerModalBadges}>
                      {server.is_default && (
                        <Badge color="blue" text="Default" />
                      )}
                      <Badge color="orange" text="Disabled" />
                    </div>
                  </div>
                ))}
              </div>
            </div>
          )}
        </div>
      </Modal>
    </div>
  );
};
