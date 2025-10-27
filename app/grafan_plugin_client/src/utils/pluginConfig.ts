/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */


import { getBackendSrv } from '@grafana/runtime';
import { lastValueFrom } from 'rxjs';

export interface PluginSettings {
  jsonData: {
    apiUrl?: string;
    apiUrlAgent?: string;
  };
  secureJsonData?: {
    apiKey?: string;
    openaiApiKey?: string;
  };
  secureJsonFields?: {
    apiKey?: boolean;
    openaiApiKey?: boolean;
  };
}

let cachedSettings: PluginSettings | null = null;

/**
 * Get the plugin configuration/settings
 */
export async function getPluginSettings(): Promise<PluginSettings> {
  if (cachedSettings) {
    return cachedSettings;
  }

  try {
    // Get plugin settings using the backend service
    const settings = await lastValueFrom(
      getBackendSrv().fetch<PluginSettings>({
        url: '/api/plugins/wisevision-wiseos-app/settings',
        method: 'GET',
      })
    );

    cachedSettings = settings.data;
    return cachedSettings;
  } catch (error) {
    console.warn('Failed to load plugin settings:', error);
    // Return default settings if loading fails
    return {
      jsonData: {
        apiUrl: 'http://localhost:5000',
        apiUrlAgent: 'http://localhost:8089'

      }
    };
  }
}

/**
 * Get the configured API URL or fall back to auto-detection
 */
export async function getConfiguredApiUrl(): Promise<string> {
  try {
    const settings = await getPluginSettings();
    
    if (settings.jsonData?.apiUrl) {
      return settings.jsonData.apiUrl;
    }
  } catch (error) {
    console.warn('Failed to get configured API URL, falling back to auto-detection:', error);
  }

  // Fallback to auto-detection if no configured URL
  const hostname = window.location.hostname;
  if (hostname === 'localhost' || hostname === '127.0.0.1') {
    return 'http://localhost:5000';
  } else {
    return `http://${hostname}:5000`;
  }
}

/**
 * Get the configured AI Agent URL or fall back to auto-detection
 */
export async function getConfiguredAgentUrl(): Promise<string> {
  try {
    const settings = await getPluginSettings();
    
    if (settings.jsonData?.apiUrlAgent) {
      return settings.jsonData.apiUrlAgent;
    }
  } catch (error) {
    console.warn('Failed to get configured AI Agent URL, falling back to auto-detection:', error);
  }

  // Fallback to auto-detection if no configured URL
  const hostname = window.location.hostname;
  if (hostname === 'localhost' || hostname === '127.0.0.1') {
    return 'http://localhost:8089';
  } else {
    return `http://${hostname}:8089`;
  }
}

/**
 * Clear the cached settings (useful for testing or when settings change)
 */
export function clearPluginSettingsCache(): void {
  cachedSettings = null;
}