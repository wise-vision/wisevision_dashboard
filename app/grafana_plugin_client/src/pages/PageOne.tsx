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
import StorageSettingsModal from '../components/App/StorageSettingsModal';
import logoImage from '../img/logo_name.png';

export const PageOne: React.FC = () => {
  const [open, setOpen] = useState(true);
  return (
    <div>
      <div style={{ marginBottom: '24px', marginLeft: '40px' }}>
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
          <h2 style={{ marginBottom: '8px', marginTop: 0 }}>WiseOS storage settings</h2>
          <p style={{ margin: 0, color: '#666', marginBottom: '16px' }}>
            Manage storage settings.
          </p>
        </div>
      </div>
      <StorageSettingsModal isOpen={open} onClose={() => setOpen(false)} />
    </div>
  );
};

export default PageOne;
