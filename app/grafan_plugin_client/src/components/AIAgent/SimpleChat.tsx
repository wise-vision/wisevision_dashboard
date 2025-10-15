import React, { useState } from 'react';
import { css } from '@emotion/css';
import { useTheme2, Button } from '@grafana/ui';
import { GrafanaTheme2 } from '@grafana/data';
import { SimpleChatInterface } from './SimpleChatInterface';
import { SimpleSessionManager } from '../services/SimpleSessionManager';
import { SimpleSessionProvider } from './SimpleSessionProvider';
import { ConnectionDebug } from './ConnectionDebug';

const getStyles = (theme: GrafanaTheme2) => ({
  container: css`
    display: flex;
    flex-direction: column;
    height: calc(100vh - 120px);
    padding: ${theme.spacing(2)};
    background: ${theme.colors.background.primary};
  `,
  header: css`
    padding: ${theme.spacing(2)};
    background: ${theme.colors.background.secondary};
    border: 1px solid ${theme.colors.border.weak};
    border-radius: ${theme.shape.radius.default};
    margin-bottom: ${theme.spacing(2)};
  `,
  title: css`
    margin: 0;
    font-size: ${theme.typography.h3.fontSize};
    font-weight: ${theme.typography.h3.fontWeight};
    color: ${theme.colors.text.primary};
    display: flex;
    align-items: center;
    gap: ${theme.spacing(1)};
  `,
  subtitle: css`
    margin: ${theme.spacing(0.5)} 0 0 0;
    font-size: ${theme.typography.body.fontSize};
    color: ${theme.colors.text.secondary};
  `,
  icon: css`
    width: 32px;
    height: 32px;
    color: ${theme.colors.primary.main};
  `,
  chatContainer: css`
    flex: 1;
    background: ${theme.colors.background.secondary};
    border: 1px solid ${theme.colors.border.weak};
    border-radius: ${theme.shape.radius.default};
    overflow: hidden;
    display: flex;
    flex-direction: column;
  `,
  toggleContainer: css`
    display: flex;
    gap: ${theme.spacing(1)};
    margin-bottom: ${theme.spacing(2)};
  `,
  toggleButton: css`
    flex: 1;
  `,
});

interface SimpleChatProps {
  showMCPConfig?: boolean;
  onToggleMode?: () => void;
}

export function SimpleChat({ showMCPConfig = false, onToggleMode }: SimpleChatProps) {
  const theme = useTheme2();
  const styles = getStyles(theme);
  const [sessionManager] = useState(() => new SimpleSessionManager());
  const [showDebug, setShowDebug] = useState(false);

  return (
    <SimpleSessionProvider sessionManager={sessionManager}>
      <div className={styles.container}>
        {/* Header */}
        <div className={styles.header}>
          <h2 className={styles.title}>
            <svg
              className={styles.icon}
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M8 12h.01M12 12h.01M16 12h.01M21 12c0 4.418-4.03 8-9 8a9.863 9.863 0 01-4.255-.949L3 20l1.395-3.72C3.512 15.042 3 13.574 3 12c0-4.418 4.03-8 9-8s9 3.582 9 8z"
              />
            </svg>
            Simple AI Chat
          </h2>
          <p className={styles.subtitle}>
            Chat directly with your AI agent - no MCP configuration required
          </p>
          <div style={{ marginTop: '8px' }}>
            <Button
              variant="secondary"
              size="sm"
              onClick={() => setShowDebug(!showDebug)}
            >
              {showDebug ? 'Hide' : 'Show'} Connection Debug
            </Button>
          </div>
        </div>

        {/* Mode Toggle */}
        {onToggleMode && (
          <div className={styles.toggleContainer}>
            <Button
              variant={!showMCPConfig ? "primary" : "secondary"}
              className={styles.toggleButton}
              onClick={onToggleMode}
            >
              🤖 Simple Chat
            </Button>
            <Button
              variant={showMCPConfig ? "primary" : "secondary"}
              className={styles.toggleButton}
              onClick={onToggleMode}
            >
              ⚙️ Advanced (MCP)
            </Button>
          </div>
        )}

        {/* Connection Debug */}
        {showDebug && <ConnectionDebug />}

        {/* Chat Interface */}
        <div className={styles.chatContainer}>
          <SimpleChatInterface />
        </div>
      </div>
    </SimpleSessionProvider>
  );
}
