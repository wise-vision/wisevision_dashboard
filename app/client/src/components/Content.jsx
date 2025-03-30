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
import Modal from './ui/Modal';
import Button from './ui/Button';
import Card from './ui/Card';
import '../styles/Content.css';

// Import modals
import ChartModal from './modals/ChartModal';
import DeleteChartModal from './modals/DeleteChartModal';
import ActionsModal from './modals/ActionsModal';
import CreateReportModal from './modals/CreateReportModal';

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
  
  return (
    <div className={`content-container ${isDarkMode ? 'content-container-dark' : ''}`}>
      <div className="content-header">
        <h2 className="content-title">Dashboard</h2>
        
        <div className="content-actions">
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
          
          <Button 
            variant="primary" 
            size="small"
            onClick={() => setIsReportModalOpen(true)}
          >
            Generate Report
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
                <p>Add charts using the sidebar to get started.</p>
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
