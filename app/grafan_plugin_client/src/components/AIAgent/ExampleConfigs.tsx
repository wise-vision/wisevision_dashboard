import React from 'react';
import { css } from '@emotion/css';
import { useTheme2, Button, Card } from '@grafana/ui';
import { GrafanaTheme2 } from '@grafana/data';
import { MCPServerConfig } from './types';

const getStyles = (theme: GrafanaTheme2) => ({
  container: css`
    display: flex;
    flex-direction: column;
    gap: ${theme.spacing(2)};
  `,
  exampleCard: css`
    cursor: pointer;
    transition: all 0.2s ease;
    
    &:hover {
      border-color: ${theme.colors.primary.main};
      box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
    }
  `,
  exampleHeader: css`
    display: flex;
    justify-content: between;
    align-items: flex-start;
    margin-bottom: ${theme.spacing(1)};
  `,
  exampleTitle: css`
    font-weight: ${theme.typography.fontWeightMedium};
    color: ${theme.colors.text.primary};
    margin: 0;
  `,
  exampleDescription: css`
    font-size: ${theme.typography.bodySmall.fontSize};
    color: ${theme.colors.text.secondary};
    margin: ${theme.spacing(0.5)} 0;
  `,
  configPreview: css`
    background: ${theme.colors.background.canvas};
    border: 1px solid ${theme.colors.border.weak};
    border-radius: ${theme.shape.radius.default};
    padding: ${theme.spacing(1)};
    font-family: ${theme.typography.fontFamilyMonospace};
    font-size: ${theme.typography.bodySmall.fontSize};
    color: ${theme.colors.text.secondary};
    overflow-x: auto;
    white-space: pre;
  `,
  useButton: css`
    margin-left: auto;
    flex-shrink: 0;
  `,
});

interface ExampleConfig {
  name: string;
  description: string;
  config: Record<string, Omit<MCPServerConfig, 'name'>>;
  icon: string;
}

const EXAMPLE_CONFIGS: ExampleConfig[] = [
  {
    name: 'Math Server',
    description: 'A simple Python server that can perform mathematical operations',
    icon: '🧮',
    config: {
      math: {
        transport: 'stdio',
        command: 'python',
        args: ['agent/servers/math_server.py'],
      },
    },
  },
  {
    name: 'ROS2 Server (Docker)',
    description: 'Docker-based ROS2 MCP server with robotics tools',
    icon: '🤖',
    config: {
      ros2: {
        transport: 'stdio',
        command: 'docker',
        args: ['run', '-i', '--rm', 'wisevision/mcp_server_ros_2:humble'],
      },
    },
  },
  {
    name: 'Web Search',
    description: 'Connect to a search service via Server-Sent Events',
    icon: '🔍', 
    config: {
      search: {
        transport: 'sse',
        url: 'http://localhost:8000/search/events',
      },
    },
  },
  {
    name: 'Math + ROS2 Setup',
    description: 'Combine math operations with robotics functionality',
    icon: '🔬',
    config: {
      math: {
        transport: 'stdio',
        command: 'python',
        args: ['agent/servers/math_server.py'],
      },
      ros2: {
        transport: 'stdio',
        command: 'docker',
        args: ['run', '-i', '--rm', 'wisevision/mcp_server_ros_2:humble'],
      },
    },
  },
  {
    name: 'Full Stack Setup',
    description: 'Multiple services for comprehensive functionality',
    icon: '⚡',
    config: {
      math: {
        transport: 'stdio',
        command: 'python',
        args: ['agent/servers/math_server.py'],
      },
      ros2: {
        transport: 'stdio',
        command: 'docker',
        args: ['run', '-i', '--rm', 'wisevision/mcp_server_ros_2:humble'],
      },
      search: {
        transport: 'sse', 
        url: 'http://localhost:8000/search/events',
      },
      files: {
        transport: 'stdio',
        command: 'node',
        args: ['scripts/file_server.js'],
      },
    },
  },
];

interface ExampleConfigsProps {
  onSelectConfig: (config: Record<string, Omit<MCPServerConfig, 'name'>>) => void;
}

export function ExampleConfigs({ onSelectConfig }: ExampleConfigsProps) {
  const theme = useTheme2();
  const styles = getStyles(theme);

  return (
    <div className={styles.container}>
      {EXAMPLE_CONFIGS.map((example) => (
        <Card key={example.name} className={styles.exampleCard}>
          <div className={styles.exampleHeader}>
            <div style={{ flex: 1 }}>
              <h4 className={styles.exampleTitle}>
                {example.icon} {example.name}
              </h4>
              <p className={styles.exampleDescription}>{example.description}</p>
            </div>
            <Button
              variant="primary" 
              size="sm"
              className={styles.useButton}
              onClick={() => onSelectConfig(example.config)}
            >
              Use This
            </Button>
          </div>
          <div className={styles.configPreview}>
            {JSON.stringify(example.config, null, 2)}
          </div>
        </Card>
      ))}
    </div>
  );
}
