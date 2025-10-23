import React, { useState, useEffect } from 'react';
import { css } from '@emotion/css';
import { useTheme2, Modal, Button, Input, Field, Alert, Badge, Spinner } from '@grafana/ui';
import { GrafanaTheme2 } from '@grafana/data';

interface PromptArgument {
  name: string;
  description: string;
  required: boolean;
}

interface MCPPrompt {
  name: string;
  description: string;
  server: string;  // Added server field
  arguments: PromptArgument[];
}

interface PromptsModalProps {
  isOpen: boolean;
  onClose: () => void;
  onExecutePrompt: (promptName: string, args: Record<string, any>, serverName: string) => Promise<void>;
}

const getStyles = (theme: GrafanaTheme2) => ({
  modalContent: css`
    width: 600px;
    max-width: 75vw;
    max-height: 85vh;
    overflow-y: auto;
    margin: 0 auto;
  `,
  header: css`
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: ${theme.spacing(2)};
    padding-bottom: ${theme.spacing(1)};
    border-bottom: 1px solid ${theme.colors.border.weak};
  `,
  title: css`
    display: flex;
    align-items: center;
    gap: ${theme.spacing(1)};
    font-size: ${theme.typography.h4.fontSize};
    font-weight: ${theme.typography.fontWeightBold};
  `,
  promptsList: css`
    display: flex;
    flex-direction: column;
    gap: ${theme.spacing(1)};
    margin-bottom: ${theme.spacing(2)};
  `,
  promptItem: css`
    padding: ${theme.spacing(1.5)};
    background: ${theme.colors.background.secondary};
    border: 1px solid ${theme.colors.border.weak};
    border-radius: ${theme.shape.radius.default};
    cursor: pointer;
    transition: all 0.2s;
    
    &:hover {
      background: ${theme.colors.emphasize(theme.colors.background.secondary, 0.03)};
      border-color: ${theme.colors.border.medium};
    }
    
    &.selected {
      border-color: ${theme.colors.primary.main};
      background: ${theme.colors.emphasize(theme.colors.background.secondary, 0.05)};
    }
  `,
  promptHeader: css`
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: ${theme.spacing(0.5)};
  `,
  promptName: css`
    font-weight: ${theme.typography.fontWeightMedium};
    color: ${theme.colors.text.primary};
    font-size: ${theme.typography.body.fontSize};
  `,
  promptDescription: css`
    color: ${theme.colors.text.secondary};
    font-size: ${theme.typography.bodySmall.fontSize};
    margin-bottom: ${theme.spacing(0.5)};
  `,
  argumentsInfo: css`
    display: flex;
    gap: ${theme.spacing(1)};
    flex-wrap: wrap;
    margin-top: ${theme.spacing(0.5)};
  `,
  argumentForm: css`
    margin-top: ${theme.spacing(2)};
    padding: ${theme.spacing(2)};
    background: ${theme.colors.background.canvas};
    border: 1px solid ${theme.colors.border.weak};
    border-radius: ${theme.shape.radius.default};
  `,
  argumentFormTitle: css`
    font-weight: ${theme.typography.fontWeightMedium};
    margin-bottom: ${theme.spacing(2)};
    color: ${theme.colors.text.primary};
  `,
  formRow: css`
    margin-bottom: ${theme.spacing(2)};
  `,
  actions: css`
    display: flex;
    gap: ${theme.spacing(1)};
    justify-content: flex-end;
    margin-top: ${theme.spacing(2)};
    padding-top: ${theme.spacing(2)};
    border-top: 1px solid ${theme.colors.border.weak};
  `,
  noPrompts: css`
    text-align: center;
    padding: ${theme.spacing(4)};
    color: ${theme.colors.text.secondary};
  `,
  loadingContainer: css`
    display: flex;
    align-items: center;
    justify-content: center;
    padding: ${theme.spacing(4)};
    gap: ${theme.spacing(2)};
  `,
});

export const PromptsModal: React.FC<PromptsModalProps> = ({
  isOpen,
  onClose,
  onExecutePrompt,
}) => {
  const theme = useTheme2();
  const styles = getStyles(theme);

  const [prompts, setPrompts] = useState<MCPPrompt[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [selectedPrompt, setSelectedPrompt] = useState<MCPPrompt | null>(null);
  const [argumentValues, setArgumentValues] = useState<Record<string, string>>({});
  const [executing, setExecuting] = useState(false);

  // Fetch prompts when modal opens
  useEffect(() => {
    if (isOpen) {
      fetchPrompts();
    } else {
      // Reset state when modal closes
      setSelectedPrompt(null);
      setArgumentValues({});
      setError(null);
    }
  }, [isOpen]);

  const fetchPrompts = async () => {
    setLoading(true);
    setError(null);
    
    try {
      const { getBackendSrv } = await import('@grafana/runtime');
      const { lastValueFrom } = await import('rxjs');
      
      const response = await lastValueFrom(
        getBackendSrv().fetch({
          url: '/api/plugin-proxy/wisevision-wiseos-app/agent_backend/mcp/prompts/list',
          method: 'GET',
        })
      );
      
      if (!response.ok) {
        throw new Error('Failed to fetch prompts');
      }
      
      const data = response.data as any;
      
      if (data.ok) {
        setPrompts(data.prompts || []);
        
        // Show informative message if prompts feature is not supported yet
        if (data.message && data.prompts.length === 0) {
          setError(data.message);
        }
      } else {
        setError(data.error || 'Failed to fetch prompts');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch prompts');
    } finally {
      setLoading(false);
    }
  };

  const handleSelectPrompt = (prompt: MCPPrompt) => {
    setSelectedPrompt(prompt);
    // Initialize argument values
    const initialValues: Record<string, string> = {};
    prompt.arguments.forEach(arg => {
      initialValues[arg.name] = '';
    });
    setArgumentValues(initialValues);
  };

  const handleArgumentChange = (argName: string, value: string) => {
    setArgumentValues(prev => ({
      ...prev,
      [argName]: value,
    }));
  };

  const handleExecute = async () => {
    if (!selectedPrompt) {
      return;
    }

    // Validate required arguments
    const missingArgs = selectedPrompt.arguments
      .filter(arg => arg.required && !argumentValues[arg.name]?.trim())
      .map(arg => arg.name);

    if (missingArgs.length > 0) {
      setError(`Missing required arguments: ${missingArgs.join(', ')}`);
      return;
    }

    setExecuting(true);
    setError(null);

    try {
      // Convert argument values to appropriate types
      const args: Record<string, any> = {};
      Object.entries(argumentValues).forEach(([key, value]) => {
        if (value.trim()) {
          args[key] = value;
        }
      });

      console.log('Executing prompt:', selectedPrompt.name, 'with args:', args);
      
      // Pass server name along with prompt name and args
      await onExecutePrompt(selectedPrompt.name, args, selectedPrompt.server);
      
      console.log('Prompt executed successfully, closing modal...');
    } catch (err) {
      console.error('Error executing prompt:', err);
      setError(err instanceof Error ? err.message : 'Failed to execute prompt');
      // Don't return - still close the modal
    } finally {
      setExecuting(false);
      console.log('Calling onClose()...');
      // Always close the modal after execution attempt
      // Use setTimeout to ensure state updates are processed
      setTimeout(() => {
        onClose();
      }, 100);
    }
  };

  const handleBack = () => {
    setSelectedPrompt(null);
    setArgumentValues({});
    setError(null);
  };

  return (
    <Modal
      title=""
      isOpen={isOpen}
      onDismiss={onClose}
    >
      <div className={styles.modalContent}>
        <div className={styles.header}>
          <div className={styles.title}>
            <span>📋 Available Prompts</span>
          </div>
          {prompts.length > 0 && (
            <Badge color="blue" text={`${prompts.length} prompts`} />
          )}
        </div>

        {error && (
          <Alert severity="error" title="Error" onRemove={() => setError(null)}>
            {error}
          </Alert>
        )}

        {loading ? (
          <div className={styles.loadingContainer}>
            <Spinner />
            <span>Loading prompts...</span>
          </div>
        ) : selectedPrompt ? (
          // Show argument form for selected prompt
          <div>
            <Button
              variant="secondary"
              size="sm"
              icon="arrow-left"
              onClick={handleBack}
              style={{ marginBottom: '16px' }}
            >
              Back to prompts
            </Button>

            <div className={styles.argumentForm}>
              <div className={styles.argumentFormTitle}>
                Execute: {selectedPrompt.name}
              </div>

              {selectedPrompt.description && (
                <p style={{ marginBottom: '16px', color: theme.colors.text.secondary }}>
                  {selectedPrompt.description}
                </p>
              )}

              {selectedPrompt.arguments.length === 0 ? (
                <Alert severity="info" title="No arguments required">
                  This prompt doesn't require any arguments. Click Execute to run it.
                </Alert>
              ) : (
                selectedPrompt.arguments.map((arg) => (
                  <div key={arg.name} className={styles.formRow}>
                    <Field
                      label={arg.name}
                      description={arg.description}
                      required={arg.required}
                    >
                      <Input
                        value={argumentValues[arg.name] || ''}
                        onChange={(e) => handleArgumentChange(arg.name, e.currentTarget.value)}
                        placeholder={`Enter ${arg.name}${arg.required ? ' (required)' : ' (optional)'}`}
                      />
                    </Field>
                  </div>
                ))
              )}

              <div className={styles.actions}>
                <Button variant="secondary" onClick={handleBack}>
                  Cancel
                </Button>
                <Button
                  variant="primary"
                  onClick={handleExecute}
                  disabled={executing}
                >
                  {executing ? 'Executing...' : 'Execute Prompt'}
                </Button>
              </div>
            </div>
          </div>
        ) : (
          // Show list of available prompts
          <>
            {prompts.length === 0 ? (
              <div className={styles.noPrompts}>
                {error ? (
                  <>
                    <p style={{ fontSize: theme.typography.h5.fontSize, marginBottom: theme.spacing(2) }}>
                      ℹ️ Prompts Not Available
                    </p>
                    <p style={{ color: theme.colors.text.secondary }}>
                      {error}
                    </p>
                    {error.includes('not yet supported') && (
                      <p style={{ fontSize: theme.typography.bodySmall.fontSize, marginTop: theme.spacing(2), color: theme.colors.text.secondary }}>
                        The prompts feature will be available when your MCP servers and adapter library support it.
                        In the meantime, you can use the AI chat to interact with MCP tools.
                      </p>
                    )}
                  </>
                ) : (
                  <>
                    <p>No prompts available</p>
                    <p style={{ fontSize: theme.typography.bodySmall.fontSize, marginTop: theme.spacing(1) }}>
                      Make sure your MCP servers are configured and running.
                    </p>
                  </>
                )}
              </div>
            ) : (
              <div className={styles.promptsList}>
                {prompts.map((prompt) => (
                  <div
                    key={prompt.name}
                    className={styles.promptItem}
                    onClick={() => handleSelectPrompt(prompt)}
                  >
                    <div className={styles.promptHeader}>
                      <div className={styles.promptName}>{prompt.name}</div>
                      <div style={{ display: 'flex', gap: '8px' }}>
                        <Badge color="purple" text={prompt.server} />
                        {prompt.arguments.length > 0 && (
                          <Badge
                            color="orange"
                            text={`${prompt.arguments.length} args`}
                          />
                        )}
                      </div>
                    </div>
                    {prompt.description && (
                      <div className={styles.promptDescription}>
                        {prompt.description}
                      </div>
                    )}
                    {prompt.arguments.length > 0 && (
                      <div className={styles.argumentsInfo}>
                        {prompt.arguments.map((arg) => (
                          <Badge
                            key={arg.name}
                            color={arg.required ? 'red' : 'blue'}
                            text={arg.name}
                            title={arg.description}
                          />
                        ))}
                      </div>
                    )}
                  </div>
                ))}
              </div>
            )}
          </>
        )}
      </div>
    </Modal>
  );
};
