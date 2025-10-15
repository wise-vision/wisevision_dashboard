import React from 'react';
import { PluginPage } from '@grafana/runtime';
import { AIAgentInterface } from '../components/AIAgent/AIAgentInterface';
import { testIds } from '../components/testIds';

export function PageAIAgent() {
  return (
    <PluginPage>
      <div data-testid={testIds.pageAIAgent.container}>
        <AIAgentInterface />
      </div>
    </PluginPage>
  );
}
