import React from 'react';
import { css } from '@emotion/css';
import { useTheme2, Modal, Button } from '@grafana/ui';
import { GrafanaTheme2 } from '@grafana/data';

const getStyles = (theme: GrafanaTheme2) => ({
  toolCallContainer: css`
    margin: ${theme.spacing(2)} 0;
  `,
  toolCall: css`
    background: ${theme.colors.background.secondary};
    border: 1px solid ${theme.colors.border.medium};
    border-radius: ${theme.shape.radius.default};
    padding: ${theme.spacing(2)};
    margin-bottom: ${theme.spacing(1)};
  `,
  toolName: css`
    font-weight: ${theme.typography.fontWeightMedium};
    color: ${theme.colors.primary.text};
    margin-bottom: ${theme.spacing(1)};
  `,
  codeBlock: css`
    background: ${theme.colors.background.canvas};
    border: 1px solid ${theme.colors.border.weak};
    border-radius: ${theme.shape.radius.default};
    padding: ${theme.spacing(1.5)};
    font-family: ${theme.typography.fontFamilyMonospace};
    font-size: ${theme.typography.bodySmall.fontSize};
    overflow-x: auto;
    max-height: 300px;
    overflow-y: auto;
  `,
  buttonGroup: css`
    display: flex;
    gap: ${theme.spacing(2)};
    justify-content: flex-end;
    margin-top: ${theme.spacing(3)};
  `,
  warningText: css`
    color: ${theme.colors.warning.text};
    margin-bottom: ${theme.spacing(2)};
    padding: ${theme.spacing(1.5)};
    background: ${theme.colors.warning.transparent};
    border-radius: ${theme.shape.radius.default};
    border-left: 3px solid ${theme.colors.warning.border};
  `,
});

interface ToolApprovalModalProps {
  isOpen: boolean;
  toolCalls: Array<{
    name: string;
    args: Record<string, any>;
    id?: string;
  }>;
  onApprove: () => void;
  onReject: () => void;
}

export const ToolApprovalModal: React.FC<ToolApprovalModalProps> = ({
  isOpen,
  toolCalls,
  onApprove,
  onReject,
}) => {
  const theme = useTheme2();
  const styles = getStyles(theme);

  return (
    <Modal
      title="🔧 Tool Call Approval Required"
      isOpen={isOpen}
      onDismiss={onReject}
    >
      <div>
        <div className={styles.warningText}>
          ⚠️ The AI agent wants to execute the following tool{toolCalls.length > 1 ? 's' : ''}. 
          Please review and approve or reject.
        </div>

        <div className={styles.toolCallContainer}>
          {toolCalls.map((tool, index) => (
            <div key={tool.id || index} className={styles.toolCall}>
              <div className={styles.toolName}>
                🔧 {tool.name}
              </div>
              {Object.keys(tool.args).length > 0 && (
                <div>
                  <strong>Arguments:</strong>
                  <pre className={styles.codeBlock}>
                    {JSON.stringify(tool.args, null, 2)}
                  </pre>
                </div>
              )}
            </div>
          ))}
        </div>

        <div className={styles.buttonGroup}>
          <Button variant="secondary" onClick={onReject}>
            ✗ Reject
          </Button>
          <Button variant="primary" onClick={onApprove}>
            ✓ Approve
          </Button>
        </div>
      </div>
    </Modal>
  );
};
