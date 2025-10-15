import React, { useState, useEffect, useMemo } from 'react';
import { css } from '@emotion/css';
import { useTheme2, Button, Alert } from '@grafana/ui';
import { GrafanaTheme2 } from '@grafana/data';

const getStyles = (theme: GrafanaTheme2) => ({
  container: css`
    padding: ${theme.spacing(2)};
    background: ${theme.colors.background.secondary};
    border: 1px solid ${theme.colors.border.weak};
    border-radius: ${theme.shape.radius.default};
    margin-bottom: ${theme.spacing(2)};
  `,
  testRow: css`
    display: flex;
    justify-content: between;
    align-items: center;
    padding: ${theme.spacing(1)} 0;
    border-bottom: 1px solid ${theme.colors.border.weak};
    
    &:last-child {
      border-bottom: none;
    }
  `,
  testName: css`
    flex: 1;
    font-weight: ${theme.typography.fontWeightMedium};
  `,
  testStatus: css`
    margin-left: ${theme.spacing(1)};
  `,
  testButton: css`
    margin-left: ${theme.spacing(1)};
  `,
});

interface ConnectionTest {
  name: string;
  url: string;
  status: 'pending' | 'success' | 'error';
  message?: string;
}

export function ConnectionDebug() {
  const theme = useTheme2();
  const styles = getStyles(theme);
  const [tests, setTests] = useState<ConnectionTest[]>([]);
  const [isRunning, setIsRunning] = useState(false);

  const possibleUrls = useMemo(() => {
    const hostname = window.location.hostname;
    return [
      `http://localhost:8089`,
      `http://127.0.0.1:8089`,
      `http://${hostname}:8089`,
    ];
  }, []);

  useEffect(() => {
    setTests(
      possibleUrls.map(url => ({
        name: `Backend at ${url}`,
        url,
        status: 'pending' as const,
      }))
    );
  }, [possibleUrls]);

  const runTests = async () => {
    setIsRunning(true);
    
    const updatedTests = await Promise.all(
      tests.map(async (test) => {
        try {
          const response = await fetch(`${test.url}/health`, {
            method: 'GET',
            headers: { 'Accept': 'application/json' },
          });
          
          if (response.ok) {
            const data = await response.json();
            let statusMessage = data.message || 'Connected successfully';
            if (data.openai_api_key_set === false) {
              statusMessage += ' ⚠️ (OpenAI API key not set)';
            } else if (data.ready_for_chat) {
              statusMessage += ' ✅ (Ready for chat)';
            }
            return {
              ...test,
              status: 'success' as const,
              message: statusMessage,
            };
          } else {
            return {
              ...test,
              status: 'error' as const,
              message: `HTTP ${response.status}: ${response.statusText}`,
            };
          }
        } catch (error) {
          return {
            ...test,
            status: 'error' as const,
            message: error instanceof Error ? error.message : 'Connection failed',
          };
        }
      })
    );
    
    setTests(updatedTests);
    setIsRunning(false);
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'success': return '✅';
      case 'error': return '❌';
      default: return '⏳';
    }
  };

  return (
    <div className={styles.container}>
      <div style={{ display: 'flex', justifyContent: 'between', alignItems: 'center', marginBottom: '16px' }}>
        <h4 style={{ margin: 0 }}>Connection Debug</h4>
        <Button
          variant="secondary"
          size="sm"
          onClick={runTests}
          disabled={isRunning}
        >
          {isRunning ? 'Testing...' : 'Test Connections'}
        </Button>
      </div>

      {tests.map((test, index) => (
        <div key={index} className={styles.testRow}>
          <div className={styles.testName}>{test.name}</div>
          <div className={styles.testStatus}>
            {getStatusIcon(test.status)} {test.status}
            {test.message && (
              <div style={{ fontSize: '12px', color: '#666', marginTop: '4px' }}>
                {test.message}
              </div>
            )}
          </div>
        </div>
      ))}

      <Alert severity="info" title="Troubleshooting">
        <ul style={{ margin: 0, paddingLeft: '20px' }}>
          <li>Make sure the AI Agent backend is running: <code>npm run ai-agent</code></li>
          <li><strong>Set OpenAI API key:</strong> <code>export OPENAI_API_KEY=&quot;your-key-here&quot;</code></li>
          <li>Backend should be accessible on port 8089</li>
          <li>Check if firewall is blocking the connection</li>
          <li>Frontend is running on: <code>{window.location.origin}</code></li>
        </ul>
      </Alert>
    </div>
  );
}
