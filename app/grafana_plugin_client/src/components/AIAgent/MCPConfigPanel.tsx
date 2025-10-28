/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import React, { useState } from 'react';
import { css } from '@emotion/css';
import { useTheme2, Button, Input, Field, Select, Alert, Card } from '@grafana/ui';
import { GrafanaTheme2, SelectableValue } from '@grafana/data';
import { MCPServerConfig } from './types';
import { useSession } from './SessionProvider';
import { ExampleConfigs } from './ExampleConfigs';

const getStyles = (theme: GrafanaTheme2) => ({
  container: css`
    flex: 1;
    overflow-y: auto;
    padding: ${theme.spacing(2)};
  `,
  serverCard: css`
    margin-bottom: ${theme.spacing(2)};
  `,
  serverHeader: css`
    display: flex;
    justify-content: between;
    align-items: center;
    margin-bottom: ${theme.spacing(1)};
  `,
  serverName: css`
    font-weight: ${theme.typography.fontWeightMedium};
    color: ${theme.colors.text.primary};
  `,
  serverType: css`
    font-size: ${theme.typography.bodySmall.fontSize};
    color: ${theme.colors.text.secondary};
    background: ${theme.colors.background.canvas};
    padding: ${theme.spacing(0.5)} ${theme.spacing(1)};
    border-radius: ${theme.shape.radius.default};
    margin-left: auto;
  `,
  serverDetails: css`
    font-size: ${theme.typography.bodySmall.fontSize};
    color: ${theme.colors.text.secondary};
    margin-top: ${theme.spacing(0.5)};
  `,
  form: css`
    display: flex;
    flex-direction: column;
    gap: ${theme.spacing(2)};
    margin-top: ${theme.spacing(2)};
  `,
  formRow: css`
    display: flex;
    gap: ${theme.spacing(1)};
  `,
  transportButtons: css`
    display: flex;
    gap: ${theme.spacing(1)};
  `,
  transportButton: css`
    flex: 1;
  `,
  addButton: css`
    margin-top: ${theme.spacing(2)};
  `,
  statsContainer: css`
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: ${theme.spacing(1)};
    margin-bottom: ${theme.spacing(2)};
  `,
  statCard: css`
    text-align: center;
    padding: ${theme.spacing(1)};
  `,
  statNumber: css`
    font-size: ${theme.typography.h3.fontSize};
    font-weight: ${theme.typography.fontWeightBold};
    color: ${theme.colors.primary.main};
  `,
  statLabel: css`
    font-size: ${theme.typography.bodySmall.fontSize};
    color: ${theme.colors.text.secondary};
  `,
  noServers: css`
    text-align: center;
    padding: ${theme.spacing(4)};
    color: ${theme.colors.text.secondary};
  `,
});

const transportOptions: Array<SelectableValue<'stdio' | 'sse'>> = [
  { label: 'Standard I/O', value: 'stdio' },
  { label: 'Server-Sent Events', value: 'sse' },
];

export function MCPConfigPanel() {
  const theme = useTheme2();
  const styles = getStyles(theme);
  const { sessionManager, sessionState } = useSession();

  const [showForm, setShowForm] = useState(false);
  const [formData, setFormData] = useState({
    name: '',
    transport: 'stdio' as 'stdio' | 'sse',
    command: '',
    args: '',
    url: '',
  });
  const [isCreatingSession, setIsCreatingSession] = useState(false);
  const [error, setError] = useState('');
  const [showExamples, setShowExamples] = useState(false);

  const servers = Object.entries(sessionState.mcpConfig).map(([name, config]) => ({
    name,
    ...config,
  })) as MCPServerConfig[];

  const handleAddServer = () => {
    if (!formData.name.trim()) {return;}

    const newConfig = { ...sessionState.mcpConfig };
    
    if (formData.transport === 'stdio') {
      // Better argument parsing - handle quoted strings and complex Docker args
      const parseArgs = (argsString: string): string[] => {
        if (!argsString.trim()) {return [];}
        
        // Simple but effective parsing for most Docker commands
        // Handles basic cases like: run -i --rm image:tag
        const args: string[] = [];
        const parts = argsString.trim().split(/\s+/);
        
        for (let i = 0; i < parts.length; i++) {
          const part = parts[i];
          if (part) {
            args.push(part);
          }
        }
        
        return args;
      };

      newConfig[formData.name] = {
        transport: 'stdio',
        command: formData.command,
        args: parseArgs(formData.args),
      };
    } else {
      newConfig[formData.name] = {
        transport: 'sse',
        url: formData.url,
      };
    }

    // Reset form
    setFormData({
      name: '',
      transport: 'stdio',
      command: '',
      args: '',
      url: '',
    });
    setShowForm(false);

    // Debug logging
    console.log('📝 Adding server:', formData.name);
    console.log('🔧 Config:', newConfig[formData.name]);
    console.log('📋 Full config:', newConfig);

    // Always create/restart session with the new config
    // This ensures the manually added server is immediately available
    console.log('🚀 About to create session with newConfig...');
    handleCreateSession(newConfig);
  };

  const handleRemoveServer = (serverName: string) => {
    const newConfig = { ...sessionState.mcpConfig };
    delete newConfig[serverName];

    if (sessionState.id) {
      handleCreateSession(newConfig);
    }
  };

  const handleCreateSession = async (config = sessionState.mcpConfig) => {
    try {
      setIsCreatingSession(true);
      setError('');
      console.log('🚀 Creating session with config:', JSON.stringify(config, null, 2));
      
      const sessionId = await sessionManager.createSession(config);
      console.log('✅ Session created successfully:', sessionId);
      
      // Try to fetch tools to verify the servers are working
      try {
        const tools = await sessionManager.getAvailableTools();
        console.log(`🔧 Tools loaded: ${tools.tools?.length || 0} tools available`);
        if (tools.tools?.length > 0) {
          console.log('First few tools:', tools.tools.slice(0, 3).map((t: any) => t.name));
        }
      } catch (toolError) {
        console.warn('⚠️ Could not fetch tools:', toolError);
      }
      
    } catch (err) {
      console.error('❌ Session creation failed:', err);
      const errorMessage = err instanceof Error ? err.message : 'Failed to create session';
      setError(`Session creation failed: ${errorMessage}`);
    } finally {
      setIsCreatingSession(false);
    }
  };

  const handleSelectExample = (exampleConfig: Record<string, Omit<MCPServerConfig, 'name'>>) => {
    console.log('🎯 Example config selected:', JSON.stringify(exampleConfig, null, 2));
    handleCreateSession(exampleConfig);
    setShowExamples(false);
  };

  const stdioServers = servers.filter(s => s.transport === 'stdio').length;
  const sseServers = servers.filter(s => s.transport === 'sse').length;

  return (
    <div className={styles.container}>
      {/* Statistics */}
      <div className={styles.statsContainer}>
        <Card className={styles.statCard}>
          <div className={styles.statNumber}>{stdioServers}</div>
          <div className={styles.statLabel}>stdio</div>
        </Card>
        <Card className={styles.statCard}>
          <div className={styles.statNumber}>{sseServers}</div>
          <div className={styles.statLabel}>sse</div>
        </Card>
      </div>

      {error && <Alert severity="error" title="Error">{error}</Alert>}

      {/* Example Configurations */}
      {!showExamples && servers.length === 0 && (
        <div style={{ textAlign: 'center', marginBottom: '16px' }}>
          <Button variant="secondary" onClick={() => setShowExamples(true)}>
            View Example Configurations
          </Button>
        </div>
      )}

      {showExamples && (
        <div style={{ marginBottom: '16px' }}>
          <div style={{ display: 'flex', justifyContent: 'between', alignItems: 'center', marginBottom: '8px' }}>
            <h3 style={{ margin: 0 }}>Example Configurations</h3>
            <Button size="sm" variant="secondary" onClick={() => setShowExamples(false)}>
              Hide
            </Button>
          </div>
          <ExampleConfigs onSelectConfig={handleSelectExample} />
        </div>
      )}

      {/* Server List */}
      {servers.length === 0 ? (
        <div className={styles.noServers}>
          No MCP servers configured.
          <br />
          Add a server to get started.
        </div>
      ) : (
        servers.map((server) => (
          <Card key={server.name} className={styles.serverCard}>
            <div className={styles.serverHeader}>
              <div className={styles.serverName}>{server.name}</div>
              <div className={styles.serverType}>{server.transport}</div>
              <Button
                variant="secondary"
                size="sm"
                icon="trash-alt"
                onClick={() => handleRemoveServer(server.name)}
              />
            </div>
            <div className={styles.serverDetails}>
              {server.transport === 'stdio' 
                ? `${server.command} ${server.args?.join(' ') || ''}`
                : server.url
              }
            </div>
          </Card>
        ))
      )}

      {/* Add Server Form */}
      {showForm && (
        <Card className={styles.form}>
          <Field label="Server Name">
            <Input
              value={formData.name}
              onChange={(e) => setFormData({ ...formData, name: e.currentTarget.value })}
              placeholder="e.g., math-server"
            />
          </Field>

          <Field label="Transport">
            <Select
              value={formData.transport}
              options={transportOptions}
              onChange={(option) => 
                setFormData({ ...formData, transport: option.value! })
              }
            />
          </Field>

          {formData.transport === 'stdio' ? (
            <>
              <Field label="Command" description="The executable command (e.g., 'python', 'node', 'docker')">
                <Input
                  value={formData.command}
                  onChange={(e) => setFormData({ ...formData, command: e.currentTarget.value })}
                  placeholder="docker"
                />
              </Field>
              <Field 
                label="Arguments" 
                description={
                  formData.command === 'docker' 
                    ? "Docker args: run -i --rm your-image:tag"
                    : "Command arguments separated by spaces"
                }
              >
                <Input
                  value={formData.args}
                  onChange={(e) => setFormData({ ...formData, args: e.currentTarget.value })}
                  placeholder={
                    formData.command === 'docker' 
                      ? "run -i --rm wisevision/mcp_server_ros_2:humble"
                      : "server.py --port 8080"
                  }
                />
              </Field>
            </>
          ) : (
            <Field label="URL">
              <Input
                value={formData.url}
                onChange={(e) => setFormData({ ...formData, url: e.currentTarget.value })}
                placeholder="http://localhost:8000/events"
              />
            </Field>
          )}

          <div className={styles.formRow}>
            <Button variant="secondary" onClick={() => setShowForm(false)}>
              Cancel
            </Button>
            <Button variant="primary" onClick={handleAddServer} disabled={!formData.name.trim()}>
              Add Server
            </Button>
          </div>
        </Card>
      )}

      {/* Action Buttons */}
      {!showForm && (
        <Button 
          variant="secondary" 
          icon="plus"
          onClick={() => setShowForm(true)}
          className={styles.addButton}
          fullWidth
        >
          Add Server
        </Button>
      )}

      {servers.length > 0 && (
        <Button
          variant="primary"
          onClick={() => handleCreateSession()}
          className={styles.addButton}
          fullWidth
          disabled={sessionState.isConnected || isCreatingSession}
        >
          {sessionState.isConnected ? 'Connected' : isCreatingSession ? 'Starting...' : 'Start Session'}
        </Button>
      )}
    </div>
  );
}
