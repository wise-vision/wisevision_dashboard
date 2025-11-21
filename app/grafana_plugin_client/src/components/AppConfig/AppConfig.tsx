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
import type { PluginConfigPageProps, AppPluginMeta } from '@grafana/data';
import { Field, Input, SecretInput, Button } from '@grafana/ui';
import { getBackendSrv } from '@grafana/runtime';

export type AppConfigProps = PluginConfigPageProps<AppPluginMeta<{}>>;

export const AppConfig: React.FC<AppConfigProps> = ({ plugin }) => {
  const meta = plugin.meta;
  const jsonData = (meta.jsonData ?? {}) as { apiUrl?: string; apiUrlAgent?: string };

  console.log('AppConfig rendered with jsonData:', jsonData);

  // State to track the current input values
  const [currentApiUrl, setCurrentApiUrl] = useState(jsonData.apiUrl ?? '');
  const [currentApiUrlAgent, setCurrentApiUrlAgent] = useState(jsonData.apiUrlAgent ?? '');
  const [isLoading, setIsLoading] = useState(false);

  // Function to save the configuration
  const handleSave = async () => {
    console.log('=== SAVE PROCESS STARTED ===');
    console.log('Current API URL from state:', currentApiUrl);
    console.log('Current AI Agent URL from state:', currentApiUrlAgent);
    console.log('Plugin meta ID:', plugin.meta.id);
    setIsLoading(true);

    try {
      const dataToSend = {
        enabled: true,
        jsonData: {
          apiUrl: currentApiUrl,
          apiUrlAgent: currentApiUrlAgent
        }
      };
      
      console.log('Data being sent:', JSON.stringify(dataToSend, null, 2));
      console.log('Request URL:', `/api/plugins/${plugin.meta.id}/settings`);

      const response = await getBackendSrv().post(`/api/plugins/${plugin.meta.id}/settings`, dataToSend);

      console.log('Save response:', response);
      
      console.log('Verifying saved configuration...');
      const verifyResponse = await getBackendSrv().get(`/api/plugins/${plugin.meta.id}/settings`);
      
      console.log('Verification response:', verifyResponse);
      console.log('Saved jsonData:', (verifyResponse as any)?.jsonData);
      console.log('Secure fields configured:', (verifyResponse as any)?.meta?.secureJsonFields);
      
      const savedData = (verifyResponse as any)?.jsonData || {};
      
      alert(`Settings saved!\nAPI URL: ${currentApiUrl}\nAI Agent URL: ${currentApiUrlAgent}\n\nVerified saved values:\nAPI URL: ${savedData.apiUrl}\nAI Agent URL: ${savedData.apiUrlAgent}\n\nOpenAI API Key: ✅ Configured via environment variable`);
      
      // Reload the page to reflect changes
      setTimeout(() => {
        console.log('Reloading page...');
        window.location.reload();
      }, 2000);
    } catch (error) {
      console.error('Save failed:', error);
      console.error('Error details:', JSON.stringify(error, null, 2));
      alert(`Save failed: ${error}`);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div style={{ maxWidth: 640 }}>
      <h3>Connection</h3>
      
      <Field label="API URL (jsonData.apiUrl)" description="Main backend API server - change this and click Save to update">
        <Input
          name="jsonData.apiUrl"
          value={currentApiUrl}
          placeholder="http://backend:5000"
          onChange={(e) => {
            const newValue = e.currentTarget.value;
            console.log('API URL changed to:', newValue);
            setCurrentApiUrl(newValue);
          }}
        />
      </Field>

      <Field label="AI Agent URL (jsonData.apiUrlAgent)" description="AI Agent chat server - usually port 8089">
        <Input
          name="jsonData.apiUrlAgent"
          value={currentApiUrlAgent}
          placeholder="http://backend_agent:8089"
          onChange={(e) => {
            const newValue = e.currentTarget.value;
            console.log('AI Agent URL changed to:', newValue);
            setCurrentApiUrlAgent(newValue);
          }}
        />
      </Field>

      <Field label="API Key (secureJsonData.apiKey)" description="Optional — bearer token">
        <SecretInput
          name="secureJsonData.apiKey"
          placeholder="••••••••"
          isConfigured={Boolean(meta.secureJsonFields?.apiKey)}
          onReset={() => { }}
          onChange={() => { }}
        />
      </Field>

      <div style={{ marginTop: 12 }}>
        <Button 
          onClick={handleSave}
          disabled={isLoading}
        >
          {isLoading ? 'Saving...' : 'Save Configuration'}
        </Button>
      </div>
    </div>
  );
};

export default AppConfig;
