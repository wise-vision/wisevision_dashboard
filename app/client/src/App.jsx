/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import React, { useState, useEffect } from 'react';
import Sidebar from './components/Sidebar';
import Content from './components/Content';
import HeaderAlerts from './components/HeaderAlerts';
import { LayoutManager } from './utils/LayoutManager';
import './App.css';

const App = () => {
    const [isModalOpen, setIsModalOpen] = useState(false);
    const [isDeleteModalOpen, setIsDeleteModalOpen] = useState(false);
    const [isActionsModalOpen, setIsActionsModalOpen] = useState(false);
    const [charts, setCharts] = useState([]);
    const [layoutConfig, setLayoutConfig] = useState(null);
    const [isDarkMode, setIsDarkMode] = useState(false);
    const [sidebarExpanded, setSidebarExpanded] = useState(false);

    // Load charts and layout from localStorage on component mount
    useEffect(() => {
        // Load charts
        const savedCharts = LayoutManager.loadCharts();
        if (savedCharts && savedCharts.length > 0) {
            setCharts(savedCharts);
        } else {
            // Fall back to legacy storage method
            const storedCharts = localStorage.getItem('charts');
            if (storedCharts) {
                setCharts(JSON.parse(storedCharts));
            }
        }

        // Load layout configuration
        const savedLayout = LayoutManager.loadLayout();
        if (savedLayout) {
            setLayoutConfig(savedLayout.layout);
        }

        // Check for dark mode preference
        const darkModePref = localStorage.getItem('darkMode') === 'true';
        setIsDarkMode(darkModePref);
        if (darkModePref) {
            document.body.classList.add('dark-mode');
        }

        // Handler for saving layout before page unload
        const handleBeforeUnload = () => {
            LayoutManager.saveLayout(layoutConfig, charts);
        };

        // Save layout on page unload
        window.addEventListener('beforeunload', handleBeforeUnload);

        return () => {
            window.removeEventListener('beforeunload', handleBeforeUnload);
        };
    }, []);

    // Save layout whenever it changes
    useEffect(() => {
        if (layoutConfig && charts.length > 0) {
            LayoutManager.saveLayout(layoutConfig, charts);
        }
    }, [layoutConfig, charts]);

    // Function to update charts and localStorage
    const updateCharts = (newCharts) => {
        setCharts(newCharts);
        // Keep legacy storage for backward compatibility
        localStorage.setItem('charts', JSON.stringify(newCharts));
    };

    const addChart = (newChart) => {
        const updatedCharts = [...charts, newChart];
        updateCharts(updatedCharts);
    };

    const deleteChartByName = (chartName) => {
        const updatedCharts = charts.filter((chart) => chart.label !== chartName);
        updateCharts(updatedCharts);
    };

    const toggleDarkMode = () => {
        const newDarkMode = !isDarkMode;
        setIsDarkMode(newDarkMode);
        localStorage.setItem('darkMode', newDarkMode);
        
        if (newDarkMode) {
            document.body.classList.add('dark-mode');
        } else {
            document.body.classList.remove('dark-mode');
        }
    };

    // Toggle sidebar function for both mobile and desktop
    const toggleSidebar = () => {
        // For mobile, show/hide the sidebar
        if (window.innerWidth <= 768) {
            setSidebarExpanded(!sidebarExpanded);
        } else {
            // For desktop, collapse/expand the sidebar
            const sidebar = document.querySelector('.sidebar');
            if (sidebar) {
                sidebar.classList.toggle('sidebar-collapsed');
            }
        }
    };

    const updateLayoutConfig = (newLayout) => {
        setLayoutConfig(newLayout);
    };

    return (
        <div className={`dashboard ${isDarkMode ? 'dark-mode' : ''}`}>
            <div className="dashboard-wrapper">
                <HeaderAlerts 
                    isDarkMode={isDarkMode} 
                    toggleDarkMode={toggleDarkMode}
                    toggleSidebar={toggleSidebar} 
                />
                <div className="dashboard-content-wrapper">
                    <Sidebar
                        setIsModalOpen={setIsModalOpen}
                        setIsDeleteModalOpen={setIsDeleteModalOpen}
                        openActionsModal={() => setIsActionsModalOpen(true)}
                        isDarkMode={isDarkMode}
                        isExpanded={sidebarExpanded}
                        toggleExpanded={toggleSidebar}
                    />
                    <div className="dashboard-content">
                        <Content
                            isModalOpen={isModalOpen}
                            setIsModalOpen={setIsModalOpen}
                            isDeleteModalOpen={isDeleteModalOpen}
                            setIsDeleteModalOpen={setIsDeleteModalOpen}
                            isActionsModalOpen={isActionsModalOpen}
                            setIsActionsModalOpen={setIsActionsModalOpen}
                            charts={charts}
                            addChart={addChart}
                            deleteChartByName={deleteChartByName}
                            layoutConfig={layoutConfig}
                            updateLayoutConfig={updateLayoutConfig}
                            isDarkMode={isDarkMode}
                        />
                    </div>
                </div>
            </div>
        </div>
    );
};

export default App;
