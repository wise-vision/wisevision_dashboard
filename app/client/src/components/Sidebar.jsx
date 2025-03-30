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
import PropTypes from 'prop-types';
import Button from './ui/Button';
import '../styles/Sidebar.css';

// Import icons or use inline SVG if icons aren't available
const DashboardIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M3.33334 10.8333H9.16668V3.33333H3.33334V10.8333ZM3.33334 16.6667H9.16668V12.5H3.33334V16.6667ZM10.8333 16.6667H16.6667V9.16667H10.8333V16.6667ZM10.8333 3.33333V7.5H16.6667V3.33333H10.8333Z" fill="currentColor"/>
  </svg>
);

const ChartIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M17.5 16.6667V7.5H12.5V16.6667H10V3.33333H5V16.6667H2.5V18.3333H17.5V16.6667Z" fill="currentColor"/>
  </svg>
);

const ActionsIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M15.8333 2.5H4.16667C3.25 2.5 2.5 3.25 2.5 4.16667V15.8333C2.5 16.75 3.25 17.5 4.16667 17.5H15.8333C16.75 17.5 17.5 16.75 17.5 15.8333V4.16667C17.5 3.25 16.75 2.5 15.8333 2.5ZM8.33333 14.1667H5.83333V7.5H8.33333V14.1667ZM12.5 14.1667H10V5.83333H12.5V14.1667ZM16.6667 14.1667H14.1667V10H16.6667V14.1667Z" fill="currentColor"/>
  </svg>
);

const GpsIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M10 1.66667C6.05 1.66667 2.5 5.05 2.5 9.58333C2.5 12.6833 5.1 16.3 10 18.3333C14.9 16.3 17.5 12.6833 17.5 9.58333C17.5 5.05 13.95 1.66667 10 1.66667ZM10 11.6667C8.625 11.6667 7.5 10.5417 7.5 9.16667C7.5 7.79167 8.625 6.66667 10 6.66667C11.375 6.66667 12.5 7.79167 12.5 9.16667C12.5 10.5417 11.375 11.6667 10 11.6667Z" fill="currentColor"/>
  </svg>
);

const ReportIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M15.8333 2.5H4.16667C3.25 2.5 2.5 3.25 2.5 4.16667V15.8333C2.5 16.75 3.25 17.5 4.16667 17.5H15.8333C16.75 17.5 17.5 16.75 17.5 15.8333V4.16667C17.5 3.25 16.75 2.5 15.8333 2.5ZM6.66667 14.1667H5V7.5H6.66667V14.1667ZM10.8333 14.1667H9.16667V5.83333H10.8333V14.1667ZM15 14.1667H13.3333V10H15V14.1667Z" fill="currentColor"/>
  </svg>
);

const SettingsIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M16.7833 10.8833C16.8333 10.6 16.8667 10.3083 16.8667 10C16.8667 9.70001 16.8333 9.40001 16.7833 9.11668L18.4333 7.83334C18.5833 7.71668 18.625 7.50001 18.5333 7.33334L16.8667 4.51668C16.7667 4.35001 16.5583 4.29168 16.3917 4.35001L14.45 5.15001C14 4.81668 13.5167 4.53334 13 4.31668L12.675 2.26668C12.65 2.08334 12.4917 1.95001 12.3 1.95001H9.0C8.80834 1.95001 8.65834 2.08334 8.63334 2.26668L8.30834 4.31668C7.79167 4.53334 7.30834 4.82501 6.86667 5.15001L4.92501 4.35001C4.75834 4.29168 4.55001 4.35001 4.45001 4.51668L2.78334 7.33334C2.68334 7.50001 2.71667 7.71668 2.87501 7.83334L4.52501 9.11668C4.47501 9.40001 4.44167 9.70834 4.44167 10C4.44167 10.2917 4.47501 10.6 4.52501 10.8833L2.87501 12.1667C2.72501 12.2833 2.68334 12.5 2.77501 12.6667L4.44167 15.4833C4.54167 15.65 4.75001 15.7083 4.91667 15.65L6.85834 14.85C7.30834 15.1833 7.79167 15.4667 8.30834 15.6833L8.63334 17.7333C8.65834 17.9167 8.80834 18.05 9.0 18.05H12.3C12.4917 18.05 12.6417 17.9167 12.6667 17.7333L12.9917 15.6833C13.5083 15.4667 13.9917 15.175 14.4333 14.85L16.375 15.65C16.5417 15.7083 16.75 15.65 16.85 15.4833L18.5167 12.6667C18.6167 12.5 18.575 12.2833 18.425 12.1667L16.7833 10.8833ZM10.65 13C9.00001 13 7.65001 11.65 7.65001 10C7.65001 8.35001 9.00001 7.00001 10.65 7.00001C12.3 7.00001 13.65 8.35001 13.65 10C13.65 11.65 12.3 13 10.65 13Z" fill="currentColor"/>
  </svg>
);

// Replace the SVG LogoIcon with an image component that handles errors
const LogoImage = () => {
  // Try different casing and file formats as fallbacks
  const [logoSrc, setLogoSrc] = useState(`${process.env.PUBLIC_URL}/wiseVisionLogoIco.ico`);
  const [hasError, setHasError] = useState(false);

  const handleError = () => {
    if (logoSrc.includes('wiseVisionLogoIco.ico')) {
      // First fallback: try lowercase version
      setLogoSrc(`${process.env.PUBLIC_URL}/wisevisionlogoico.ico`);
    } else if (logoSrc.includes('wisevisionlogoico.ico')) {
      // Second fallback: try PNG version
      setLogoSrc(`${process.env.PUBLIC_URL}/wisevisionLogoColor.png`);
    } else if (logoSrc.includes('wisevisionLogoColor.png')) {
      // Third fallback: try lowercase PNG
      setLogoSrc(`${process.env.PUBLIC_URL}/wisevisionlogocolor.png`);
    } else {
      // If all attempts fail, show error state
      setHasError(true);
    }
  };

  if (hasError) {
    // Fallback to a simple colored div with text if image loading fails
    return <div className="logo--fallback">WV</div>;
  }

  return (
    <img 
      src={logoSrc}
      alt="WiseVision Logo"
      className="logo--image"
      onError={handleError}
    />
  );
};

const AddChartIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M15.8333 2.5H4.16667C3.24167 2.5 2.5 3.25 2.5 4.16667V15.8333C2.5 16.75 3.24167 17.5 4.16667 17.5H15.8333C16.75 17.5 17.5 16.75 17.5 15.8333V4.16667C17.5 3.25 16.75 2.5 15.8333 2.5ZM13.3333 10.8333H10.8333V13.3333C10.8333 13.75 10.5 14.1667 10 14.1667C9.5 14.1667 9.16667 13.75 9.16667 13.3333V10.8333H6.66667C6.25 10.8333 5.83333 10.5 5.83333 10C5.83333 9.5 6.25 9.16667 6.66667 9.16667H9.16667V6.66667C9.16667 6.25 9.5 5.83333 10 5.83333C10.5 5.83333 10.8333 6.25 10.8333 6.66667V9.16667H13.3333C13.75 9.16667 14.1667 9.5 14.1667 10C14.1667 10.5 13.75 10.8333 13.3333 10.8333Z" fill="currentColor"/>
  </svg>
);

const RemoveChartIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M15.8333 2.5H4.16667C3.24167 2.5 2.5 3.25 2.5 4.16667V15.8333C2.5 16.75 3.24167 17.5 4.16667 17.5H15.8333C16.75 17.5 17.5 16.75 17.5 15.8333V4.16667C17.5 3.25 16.75 2.5 15.8333 2.5ZM13.3333 10.8333H6.66667C6.25 10.8333 5.83333 10.5 5.83333 10C5.83333 9.5 6.25 9.16667 6.66667 9.16667H13.3333C13.75 9.16667 14.1667 9.5 14.1667 10C14.1667 10.5 13.75 10.8333 13.3333 10.8333Z" fill="currentColor"/>
  </svg>
);

const ManageActionsIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M15.8333 9.16667V2.5H9.16667V9.16667H15.8333ZM15.8333 17.5V10.8333H9.16667V17.5H15.8333ZM7.5 17.5V10.8333H0.833336V17.5H7.5ZM7.5 9.16667V2.5H0.833336V9.16667H7.5Z" fill="currentColor"/>
  </svg>
);

const CloseIcon = () => (
  <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M18 6L6 18M6 6L18 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
  </svg>
);

const Sidebar = ({ 
  setIsModalOpen, 
  setIsDeleteModalOpen, 
  openActionsModal,
  isDarkMode = false,
  isExpanded = false,
  toggleExpanded
}) => {
  const [isCollapsed, setIsCollapsed] = useState(false);
  const [activeItem, setActiveItem] = useState('dashboard');
  const isMobile = window.innerWidth <= 768;

  const toggleCollapse = () => {
    if (isMobile) {
      // In mobile view, use the external toggle function
      if (toggleExpanded) {
        toggleExpanded();
      }
    } else {
      // On desktop, use the internal collapse state
      setIsCollapsed(!isCollapsed);
    }
  };

  const handleCreateChart = () => {
    setIsModalOpen(true);
  };

  const handleDeleteChart = () => {
    setIsDeleteModalOpen(true);
  };

  const handleMenuItemClick = (itemId) => {
    setActiveItem(itemId);
    
    // Add functionality for each menu item
    switch(itemId) {
      case 'dashboard':
        // Default view - nothing special needed
        break;
      case 'chart':
        setIsModalOpen(true);
        break;
      case 'actions':
        openActionsModal();
        break;
      case 'gps':
        // Show GPS specific view or modal
        alert('GPS Tracking feature will be available soon!');
        break;
      case 'report':
        // Open report modal or navigate to reports page
        alert('Reports feature will be available soon!');
        break;
      case 'settings':
        // Open settings modal or navigate to settings page
        alert('Settings feature will be available soon!');
        break;
      default:
        break;
    }
  };

  const menuItems = [
    { id: 'dashboard', label: 'Dashboard', icon: <DashboardIcon /> },
    { id: 'chart', label: 'Charts', icon: <ChartIcon /> },
    { id: 'actions', label: 'Actions', icon: <ActionsIcon /> },
    { id: 'gps', label: 'GPS Tracking', icon: <GpsIcon /> },
    { id: 'report', label: 'Reports', icon: <ReportIcon /> },
    { id: 'settings', label: 'Settings', icon: <SettingsIcon /> },
  ];

  return (
    <div className={`sidebar 
      ${isCollapsed ? 'sidebar-collapsed' : ''} 
      ${isDarkMode ? 'sidebar-dark' : ''} 
      ${isExpanded ? 'sidebar-expanded' : ''}`}
    >
      <div className="sidebar-header">
        <div className="logo-container">
          {/* Only show logo on mobile, not the text */}
          {!isCollapsed && window.innerWidth > 768 && <span className="logo--text">WiseVision Dashboard</span>}
          <LogoImage />
        </div>
        
        {/* Add close button for mobile */}
        {isMobile && isExpanded && (
          <button 
            className="sidebar-close-btn" 
            onClick={toggleCollapse}
            aria-label="Close sidebar"
          >
            <CloseIcon />
          </button>
        )}
      </div>

      <div className="sidebar-menu">
        {menuItems.map((item) => (
          <button
            key={item.id}
            className={`menu--item ${activeItem === item.id ? 'active' : ''}`}
            onClick={() => handleMenuItemClick(item.id)}
          >
            <span className="menu--item-icon">{item.icon}</span>
            <span className="menu--item-label">{item.label}</span>
          </button>
        ))}
      </div>

      {/* Hide sidebar actions since we're using the content header dropdown now */}
      {false && <div className="sidebar-actions">
        <Button
          variant="primary"
          size={isCollapsed && !isMobile ? 'small' : 'medium'}
          onClick={handleCreateChart}
          fullWidth
          className="sidebar-action-btn"
          icon={isCollapsed && !isMobile ? <AddChartIcon /> : null}
        >
          {isCollapsed && !isMobile ? '' : 'Add Chart'}
        </Button>
        
        <Button
          variant="outline"
          size={isCollapsed && !isMobile ? 'small' : 'medium'}
          onClick={handleDeleteChart}
          fullWidth
          className="sidebar-action-btn mt-sm"
          icon={isCollapsed && !isMobile ? <RemoveChartIcon /> : null}
        >
          {isCollapsed && !isMobile ? '' : 'Remove Chart'}
        </Button>

        <Button
          variant="secondary"
          size={isCollapsed && !isMobile ? 'small' : 'medium'}
          onClick={openActionsModal}
          fullWidth
          className="sidebar-action-btn mt-sm"
          icon={isCollapsed && !isMobile ? <ManageActionsIcon /> : null}
        >
          {isCollapsed && !isMobile ? '' : 'Manage Actions'}
        </Button>
      </div>}
    </div>
  );
};

Sidebar.propTypes = {
  setIsModalOpen: PropTypes.func.isRequired,
  setIsDeleteModalOpen: PropTypes.func.isRequired,
  openActionsModal: PropTypes.func.isRequired,
  isDarkMode: PropTypes.bool,
  isExpanded: PropTypes.bool,
  toggleExpanded: PropTypes.func
};

export default Sidebar;
