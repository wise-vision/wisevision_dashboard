import React from 'react';
import { PluginPage } from '@grafana/runtime';
import { AIAgentInterface } from '../components/AIAgent/AIAgentInterface';
import { testIds } from '../components/testIds';
import logoImage from '../img/logo_name.png';

export function PageAIAgent() {
  return (
    <PluginPage>
      <div data-testid={testIds.pageAIAgent.container}>
        <div style={{ marginBottom: '0px', marginLeft: '0px' }}>
          <img
            src={logoImage}
            alt="Logo"
            style={{
              height: '60px',
              width: 'auto',
              display: 'block',
              marginBottom: '16px'
            }}
          />
          <div style={{ marginLeft: '20px' }}>
            <h2 style={{ marginBottom: '8px', marginTop: 0 }}>WiseOS AI Agent</h2>
            <p style={{ margin: 0, color: '#666', marginBottom: '16px' }}>
              Chat with the WiseOS AI Agent.
            </p>
          </div>
        </div>
        <AIAgentInterface />
      </div>
    </PluginPage>
  );
}
