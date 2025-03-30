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
import DashboardGrid from './DashboardGrid';
import Button from './ui/Button';
import Card from './ui/Card';
import '../styles/Content.css';

// Import modals
import ChartModal from './modals/ChartModal';
import DeleteChartModal from './modals/DeleteChartModal';
import ActionsModal from './modals/ActionsModal';
import CreateReportModal from './modals/CreateReportModal';

// Add these icons for the action buttons
const AddIcon = () => (
  <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M8 1.33334V14.6667M1.33334 8H14.6667" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
  </svg>
);

const TrashIcon = () => (
  <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M2 4H3.33333H14" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M5.33334 4V2.66667C5.33334 2.31305 5.47381 1.97391 5.7239 1.72386C5.97399 1.47381 6.31305 1.33334 6.66667 1.33334H9.33334C9.68696 1.33334 10.026 1.47381 10.2761 1.72386C10.5262 1.97391 10.6667 2.31305 10.6667 2.66667V4M12.6667 4V13.3333C12.6667 13.687 12.5262 14.0261 12.2761 14.2761C12.026 14.5262 11.687 14.6667 11.3333 14.6667H4.66667C4.31305 14.6667 3.97391 14.5262 3.72386 14.2761C3.47381 14.0261 3.33334 13.687 3.33334 13.3333V4H12.6667Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
  </svg>
);

const CogIcon = () => (
  <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M8.00001 10C9.10458 10 10 9.10457 10 8C10 6.89543 9.10458 6 8.00001 6C6.89544 6 6.00001 6.89543 6.00001 8C6.00001 9.10457 6.89544 10 8.00001 10Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M12.9333 10C12.8067 10.3215 12.8267 10.6793 12.9878 10.9878C13.1489 11.2962 13.4367 11.5223 13.78 11.6133L13.7933 11.6167C14.0781 11.6935 14.328 11.8543 14.5073 12.0765C14.6866 12.2986 14.7864 12.5711 14.7933 12.8533V13.1467C14.7864 13.4289 14.6866 13.7014 14.5073 13.9235C14.328 14.1456 14.0781 14.3065 13.7933 14.3833C13.4503 14.4743 13.1628 14.7 13.0013 15.0079C12.8399 15.3158 12.8191 15.673 12.9433 15.9933C13.0498 16.2791 13.0505 16.5957 12.9452 16.8818C12.8399 17.1678 12.6345 17.4059 12.3667 17.5533L12.08 17.7133C11.8141 17.8612 11.5043 17.917 11.201 17.8712C10.8977 17.8254 10.6214 17.6805 10.4133 17.46C10.1892 17.2241 9.88335 17.0876 9.56001 17.0783C9.23668 17.069 8.92351 17.1874 8.68668 17.4067C8.47854 17.6272 8.20221 17.7721 7.8989 17.8179C7.5956 17.8637 7.28578 17.8079 7.02001 17.66L6.74001 17.5067C6.4722 17.3593 6.26684 17.1212 6.16155 16.8351C6.05626 16.5491 6.05693 16.2325 6.16335 15.9467C6.28755 15.6264 6.26679 15.2693 6.10531 14.9613C5.94383 14.6534 5.65635 14.4278 5.31335 14.3367C5.02849 14.2599 4.77864 14.099 4.59935 13.8768C4.42006 13.6547 4.32029 13.3822 4.31335 13.1V12.8533C4.32029 12.5711 4.42006 12.2986 4.59935 12.0765C4.77864 11.8543 5.02849 11.6935 5.31335 11.6167L5.32668 11.6133C5.66998 11.5223 5.95784 11.2962 6.1189 10.9878C6.27997 10.6793 6.30002 10.3215 6.17335 10C6.06679 9.71335 6.07337 9.40322 6.19205 9.12142C6.31072 8.83962 6.53497 8.60566 6.82001 8.46667L6.92668 8.41333C7.21347 8.28095 7.54386 8.25218 7.85123 8.33408C8.15859 8.41599 8.42184 8.60319 8.59335 8.86L8.64001 8.93333C8.78374 9.14709 9.00729 9.30911 9.26944 9.38879C9.5316 9.46847 9.81553 9.45977 10.0713 9.36423C10.3271 9.26869 10.5389 9.09258 10.6673 8.86423C10.7957 8.63588 10.832 8.37065 10.7667 8.12L10.74 8C10.656 7.69306 10.6799 7.36457 10.8074 7.07238C10.935 6.78019 11.1581 6.54391 11.44 6.4L11.6067 6.32C11.8925 6.17202 12.2244 6.1248 12.5427 6.18579C12.8611 6.24677 13.1451 6.41185 13.3467 6.65333L13.4133 6.73333C13.5571 6.94708 13.7806 7.10911 14.0428 7.18879C14.3049 7.26847 14.5889 7.25977 14.8447 7.16423C15.1004 7.06869 15.3122 6.89258 15.4406 6.66423C15.569 6.43588 15.6053 6.17065 15.54 5.92L15.5267 5.86C15.4407 5.56029 15.4667 5.2398 15.5998 4.95595C15.733 4.67209 15.9656 4.44473 16.2533 4.30667H16.34C16.6247 4.15869 16.9582 4.11147 17.2766 4.17245C17.5949 4.23344 17.8789 4.39851 18.0807 4.64H18.1067C18.3487 4.92517 18.6754 5.12865 19.0394 5.22316C19.4034 5.31767 19.7879 5.29812 20.14 5.16667L20.3267 5.09333C20.5872 4.98843 20.8149 4.81168 20.9874 4.58395C21.16 4.35623 21.2708 4.08624 21.3093 3.8C21.4213 3.14845 21.5 2.49286 21.54 1.83333" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
  </svg>
);

const ReportIcon = () => (
  <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M8.66667 1.33333H4C3.64638 1.33333 3.30724 1.47381 3.05719 1.72386C2.80714 1.97391 2.66667 2.31305 2.66667 2.66667V13.3333C2.66667 13.687 2.80714 14.0261 3.05719 14.2761C3.30724 14.5262 3.64638 14.6667 4 14.6667H12C12.3536 14.6667 12.6928 14.5262 12.9428 14.2761C13.1929 14.0261 13.3333 13.687 13.3333 13.3333V6M8.66667 1.33333L13.3333 6M8.66667 1.33333V6H13.3333" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
  </svg>
);

const Content = ({
  isModalOpen,
  setIsModalOpen,
  isDeleteModalOpen,
  setIsDeleteModalOpen,
  isActionsModalOpen,
  setIsActionsModalOpen,
  charts,
  addChart,
  deleteChartByName,
  layoutConfig,
  updateLayoutConfig,
  isDarkMode
}) => {
  const [isReportModalOpen, setIsReportModalOpen] = useState(false);
  const [dashboardView, setDashboardView] = useState('grid');
  const [showActionMenu, setShowActionMenu] = useState(false);
  
  // Toggle the action menu
  const toggleActionMenu = () => {
    setShowActionMenu(!showActionMenu);
  };
  
  // Close the action menu
  const closeActionMenu = () => {
    setShowActionMenu(false);
  };
  
  return (
    <div className={`content-container ${isDarkMode ? 'content-container-dark' : ''}`}>
      <div className="content-header">
        <div className="content-title-area">
          <h2 className="content-title">Dashboard</h2>
          
          <div className="content-actions-menu-wrapper">
            <Button 
              variant="outline"
              size="small"
              className="action-menu-toggle"
              onClick={toggleActionMenu}
            >
              Actions <span className={`arrow ${showActionMenu ? 'up' : 'down'}`}>▾</span>
            </Button>
            
            {/* Add backdrop for mobile */}
            {showActionMenu && window.innerWidth <= 768 && (
              <div className="action-menu-backdrop" onClick={closeActionMenu}></div>
            )}
            
            {showActionMenu && (
              <div className="actions-dropdown">
                <button 
                  className="action-item" 
                  onClick={() => {
                    setIsModalOpen(true);
                    setShowActionMenu(false);
                  }}
                >
                  <AddIcon /> Add Chart
                </button>
                <button 
                  className="action-item" 
                  onClick={() => {
                    setIsDeleteModalOpen(true);
                    setShowActionMenu(false);
                  }}
                >
                  <TrashIcon /> Remove Chart
                </button>
                <button 
                  className="action-item" 
                  onClick={() => {
                    setIsActionsModalOpen(true);
                    setShowActionMenu(false);
                  }}
                >
                  <CogIcon /> Manage Actions
                </button>
                <button 
                  className="action-item" 
                  onClick={() => {
                    setIsReportModalOpen(true);
                    setShowActionMenu(false);
                  }}
                >
                  <ReportIcon /> Generate Report
                </button>
              </div>
            )}
          </div>
        </div>
        
        <div className="content-view-controls">
          <div className="view-toggle">
            <button 
              className={`view-toggle-btn ${dashboardView === 'grid' ? 'active' : ''}`}
              onClick={() => setDashboardView('grid')}
            >
              <span className="view-toggle-icon">▤</span> Grid
            </button>
            <button 
              className={`view-toggle-btn ${dashboardView === 'list' ? 'active' : ''}`}
              onClick={() => setDashboardView('list')}
            >
              <span className="view-toggle-icon">≡</span> List
            </button>
          </div>
          
          {/* Mobile action button that appears on small screens */}
          <Button 
            variant="primary" 
            size="small"
            className="mobile-action-btn"
            onClick={toggleActionMenu}
          >
            Actions
          </Button>
        </div>
      </div>
      
      {/* Dashboard Content */}
      <div className="dashboard-content-area">
        {dashboardView === 'grid' ? (
          <DashboardGrid
            charts={charts}
            layoutConfig={layoutConfig}
            updateLayoutConfig={updateLayoutConfig}
            isDarkMode={isDarkMode}
          />
        ) : (
          <div className="dashboard-list-view">
            {charts.length > 0 ? (
              <div className="charts-list">
                {charts.map((chart, index) => (
                  <Card 
                    key={index}
                    title={chart.label || `Chart ${index + 1}`}
                    className="list-card"
                    elevation="sm"
                  >
                    <div className="list-card-content">
                      <p>Type: {chart.type}</p>
                      {chart.topic && <p>Topic: {chart.topic}</p>}
                    </div>
                  </Card>
                ))}
              </div>
            ) : (
              <div className="empty-state">
                <h3>No charts to display</h3>
                <p>Add charts using the Actions to get started.</p>
                <Button 
                  variant="primary" 
                  onClick={() => setIsModalOpen(true)}
                  className="mt-md"
                >
                  Add Chart
                </Button>
              </div>
            )}
          </div>
        )}
      </div>
      
      {/* Modals */}
      <ChartModal 
        isOpen={isModalOpen}
        onClose={() => setIsModalOpen(false)}
        addChart={addChart}
        isDarkMode={isDarkMode}
      />
      
      <DeleteChartModal 
        isOpen={isDeleteModalOpen}
        onClose={() => setIsDeleteModalOpen(false)}
        charts={charts}
        deleteChartByName={deleteChartByName}
        isDarkMode={isDarkMode}
      />
      
      <ActionsModal 
        isOpen={isActionsModalOpen}
        onClose={() => setIsActionsModalOpen(false)}
        isDarkMode={isDarkMode}
      />
      
      <CreateReportModal 
        isOpen={isReportModalOpen}
        onClose={() => setIsReportModalOpen(false)}
        isDarkMode={isDarkMode}
      />
    </div>
  );
};

Content.propTypes = {
  isModalOpen: PropTypes.bool.isRequired,
  setIsModalOpen: PropTypes.func.isRequired,
  isDeleteModalOpen: PropTypes.bool.isRequired,
  setIsDeleteModalOpen: PropTypes.func.isRequired,
  isActionsModalOpen: PropTypes.bool.isRequired,
  setIsActionsModalOpen: PropTypes.func.isRequired,
  charts: PropTypes.array.isRequired,
  addChart: PropTypes.func.isRequired,
  deleteChartByName: PropTypes.func.isRequired,
  layoutConfig: PropTypes.object,
  updateLayoutConfig: PropTypes.func.isRequired,
  isDarkMode: PropTypes.bool
};

export default Content;
