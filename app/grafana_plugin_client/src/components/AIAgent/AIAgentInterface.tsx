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
import { DualChatInterface } from './DualChatInterface';
import { SimpleSessionProvider } from './SimpleSessionProvider';
import { SimpleSessionManager } from '../services/SimpleSessionManager';

export function AIAgentInterface() {
  const [simpleSessionManager] = useState(() => new SimpleSessionManager());

  // Always show DualChatInterface with both Simple and MCP modes + settings
  return (
    <SimpleSessionProvider sessionManager={simpleSessionManager}>
      <DualChatInterface />
    </SimpleSessionProvider>
  );
}
