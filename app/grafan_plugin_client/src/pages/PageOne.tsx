import React, { useState } from 'react';
// import { Button } from '@grafana/ui';
// import { locationService } from '@grafana/runtime';
import StorageSettingsModal from '../components/App/StorageSettingsModal';
// import { prefixRoute } from '../utils/utils.routing';
// import { ROUTES } from '../constants';
import logoImage from '../img/logo_name.png';

export const PageOne: React.FC = () => {
  const [open, setOpen] = useState(true); // możesz ustawić false i otwierać przyciskiem
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
          <h2 style={{ marginBottom: '8px', marginTop: 0 }}>WiseOS Application</h2>
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
