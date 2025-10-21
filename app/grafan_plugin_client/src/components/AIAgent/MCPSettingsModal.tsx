import React, { useState } from 'react';
import { css } from '@emotion/css';
import { useTheme2, Modal, Button, Input, Field, Alert, IconButton, ConfirmModal, Badge } from '@grafana/ui';
import { GrafanaTheme2 } from '@grafana/data';

export interface MCPServer {
  id: string;
  name: string;
  command: string;
  args: string[];
  env?: Record<string, string>;
  enabled: boolean;
  is_default?: boolean;  // Indicates if this is a backend default server
}

interface MCPSettingsModalProps {
  isOpen: boolean;
  onClose: () => void;
  servers: MCPServer[];
  onServersChange: (servers: MCPServer[]) => void;
}

const getStyles = (theme: GrafanaTheme2) => ({
  modalContent: css`
    width: 800px;
    max-width: 90vw;
    max-height: 80vh;
    overflow-y: auto;
  `,
  header: css`
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: ${theme.spacing(3)};
  `,
  serverList: css`
    margin-bottom: ${theme.spacing(3)};
  `,
  serverItem: css`
    border: 1px solid ${theme.colors.border.weak};
    border-radius: ${theme.shape.radius.default};
    padding: ${theme.spacing(2)};
    margin-bottom: ${theme.spacing(2)};
    background: ${theme.colors.background.secondary};
  `,
  serverHeader: css`
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: ${theme.spacing(1)};
  `,
  serverName: css`
    font-weight: ${theme.typography.fontWeightMedium};
    color: ${theme.colors.text.primary};
  `,
  serverCommand: css`
    font-family: ${theme.typography.fontFamilyMonospace};
    font-size: ${theme.typography.bodySmall.fontSize};
    color: ${theme.colors.text.secondary};
    background: ${theme.colors.background.canvas};
    padding: ${theme.spacing(0.5)} ${theme.spacing(1)};
    border-radius: ${theme.shape.radius.default};
    margin-top: ${theme.spacing(1)};
    word-break: break-all;
    overflow-wrap: break-word;
    line-height: 1.4;
    cursor: pointer;
    transition: all 0.2s;
    
    &:hover {
      background: ${theme.colors.emphasize(theme.colors.background.canvas, 0.03)};
      color: ${theme.colors.text.primary};
    }
  `,
  serverCommandExpanded: css`
    background: ${theme.colors.background.secondary};
    border: 1px solid ${theme.colors.border.weak};
  `,
  serverActions: css`
    display: flex;
    gap: ${theme.spacing(1)};
    align-items: center;
  `,
  addServerForm: css`
    border: 1px solid ${theme.colors.border.medium};
    border-radius: ${theme.shape.radius.default};
    padding: ${theme.spacing(3)};
    background: ${theme.colors.background.canvas};
  `,
  formRow: css`
    display: flex;
    gap: ${theme.spacing(2)};
    margin-bottom: ${theme.spacing(2)};
  `,
  formField: css`
    flex: 1;
  `,
  argsInput: css`
    font-family: ${theme.typography.fontFamilyMonospace};
  `,
  noServers: css`
    text-align: center;
    padding: ${theme.spacing(4)};
    color: ${theme.colors.text.secondary};
    font-style: italic;
  `,
});

export const MCPSettingsModal: React.FC<MCPSettingsModalProps> = ({
  isOpen,
  onClose,
  servers,
  onServersChange,
}) => {
  const theme = useTheme2();
  const styles = getStyles(theme);
  
  const [showAddForm, setShowAddForm] = useState(false);
  const [newServer, setNewServer] = useState<Partial<MCPServer>>({
    name: '',
    command: '',
    args: [],
    env: {},
    enabled: true,
  });
  const [argsString, setArgsString] = useState('');
  const [deleteConfirm, setDeleteConfirm] = useState<string | null>(null);
  const [expandedServers, setExpandedServers] = useState<Set<string>>(new Set());

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

  const toggleServerExpanded = (serverId: string) => {
    setExpandedServers(prev => {
      const newSet = new Set(prev);
      if (newSet.has(serverId)) {
        newSet.delete(serverId);
      } else {
        newSet.add(serverId);
      }
      return newSet;
    });
  };

  const handleAddServer = () => {
    if (!newServer.name || !newServer.command) {
      return;
    }

    const server: MCPServer = {
      id: Date.now().toString(),
      name: newServer.name,
      command: newServer.command,
      args: argsString ? argsString.split(' ').filter(arg => arg.trim()) : [],
      env: newServer.env || {},
      enabled: true,
    };

    onServersChange([...servers, server]);
    
    // Reset form
    setNewServer({
      name: '',
      command: '',
      args: [],
      env: {},
      enabled: true,
    });
    setArgsString('');
    setShowAddForm(false);
  };

  const handleToggleServer = (serverId: string) => {
    const updatedServers = servers.map(server =>
      server.id === serverId ? { ...server, enabled: !server.enabled } : server
    );
    onServersChange(updatedServers);
  };

  const handleDeleteServer = (serverId: string) => {
    const updatedServers = servers.filter(server => server.id !== serverId);
    onServersChange(updatedServers);
    setDeleteConfirm(null);
  };

  if (!isOpen) {
    return null;
  }

  return (
    <>
      <Modal
        title="MCP Server Configuration"
        isOpen={isOpen}
        onDismiss={onClose}
        className={styles.modalContent}
      >
        <div>
          <div className={styles.header}>
            <div>
              <h3>Manage MCP Servers</h3>
              <p style={{ margin: 0, color: theme.colors.text.secondary, fontSize: theme.typography.bodySmall.fontSize }}>
                Configure Model Context Protocol servers to extend AI capabilities with tools and context.
              </p>
            </div>
            <Button
              variant="primary"
              onClick={() => setShowAddForm(!showAddForm)}
            >
              {showAddForm ? 'Cancel' : 'Add Server'}
            </Button>
          </div>

          {/* Server List */}
          <div className={styles.serverList}>
            {servers.length === 0 ? (
              <div className={styles.noServers}>
                <p>No MCP servers configured yet.</p>
                <p>Add your first server to enable enhanced AI capabilities.</p>
              </div>
            ) : (
              servers.map((server) => (
                <div key={server.id} className={styles.serverItem}>
                  <div className={styles.serverHeader}>
                    <div>
                      <div className={styles.serverName}>
                        {server.name}
                        {server.is_default && (
                          <Badge 
                            text="Default" 
                            color="blue" 
                            style={{ marginLeft: theme.spacing(1) }}
                          />
                        )}
                        {!server.enabled && (
                          <span style={{ color: theme.colors.text.secondary, fontWeight: 'normal', marginLeft: theme.spacing(1) }}>
                            (disabled)
                          </span>
                        )}
                      </div>
                      <div 
                        className={`${styles.serverCommand} ${expandedServers.has(server.id) ? styles.serverCommandExpanded : ''}`}
                        onClick={() => toggleServerExpanded(server.id)}
                        title="Click to show full command"
                      >
                        {expandedServers.has(server.id) 
                          ? `${server.command} ${server.args.join(' ')}`
                          : formatCommand(server.command, server.args)
                        }
                      </div>
                    </div>
                    <div className={styles.serverActions}>
                      <Button
                        variant={server.enabled ? 'secondary' : 'primary'}
                        size="sm"
                        onClick={() => handleToggleServer(server.id)}
                        disabled={server.is_default}
                        tooltip={server.is_default ? "Default servers cannot be disabled" : ""}
                      >
                        {server.enabled ? 'Disable' : 'Enable'}
                      </Button>
                      <IconButton
                        name="trash-alt"
                        onClick={() => setDeleteConfirm(server.id)}
                        tooltip={server.is_default ? "Default servers cannot be deleted" : "Delete server"}
                        disabled={server.is_default}
                      />
                    </div>
                  </div>
                </div>
              ))
            )}
          </div>

          {/* Add Server Form */}
          {showAddForm && (
            <div className={styles.addServerForm}>
              <h4>Add New MCP Server</h4>

              <div className={styles.formRow}>
                <Field label="Server Name" className={styles.formField}>
                  <Input
                    value={newServer.name || ''}
                    onChange={(e) => setNewServer({ ...newServer, name: e.currentTarget.value })}
                    placeholder="e.g., Custom MCP Server"
                  />
                </Field>
                <Field label="Command" className={styles.formField}>
                  <Input
                    value={newServer.command || ''}
                    onChange={(e) => setNewServer({ ...newServer, command: e.currentTarget.value })}
                    placeholder="e.g., mcp-server-filesystem"
                  />
                </Field>
              </div>

              <Field label="Arguments" description="Space-separated command line arguments">
                <Input
                  className={styles.argsInput}
                  value={argsString}
                  onChange={(e) => setArgsString(e.currentTarget.value)}
                  placeholder="e.g., /path1 /path2 --option value"
                />
              </Field>

              <div style={{ display: 'flex', gap: theme.spacing(2), marginTop: theme.spacing(3) }}>
                <Button
                  variant="primary"
                  onClick={handleAddServer}
                  disabled={!newServer.name || !newServer.command}
                >
                  Add Server
                </Button>
                <Button
                  variant="secondary"
                  onClick={() => setShowAddForm(false)}
                >
                  Cancel
                </Button>
              </div>
            </div>
          )}

          <Alert title="MCP Server Information" severity="info" style={{ marginTop: theme.spacing(3) }}>
            <p>MCP servers provide AI agents with access to external tools and context:</p>
            <ul>
              <li><strong>Filesystem MCP:</strong> File operations (read, write, search)</li>
              <li><strong>ROS2 MCP:</strong> Robot system integration and control</li>
              <li><strong>Custom MCP:</strong> Your own server implementing MCP protocol</li>
            </ul>
            <p>Servers must be running and accessible for the AI agent to use them.</p>
          </Alert>
        </div>
      </Modal>

      {/* Delete Confirmation */}
      {deleteConfirm && (
        <ConfirmModal
          isOpen={true}
          title="Delete MCP Server"
          body={`Are you sure you want to delete "${servers.find(s => s.id === deleteConfirm)?.name}"?`}
          confirmText="Delete"
          onConfirm={() => handleDeleteServer(deleteConfirm)}
          onDismiss={() => setDeleteConfirm(null)}
        />
      )}
    </>
  );
};
