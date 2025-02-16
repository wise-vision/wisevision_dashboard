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
import './App.css';

const App = () => {
    const [isModalOpen, setIsModalOpen] = useState(false);
    const [isDeleteModalOpen, setIsDeleteModalOpen] = useState(false);
    const [isActionsModalOpen, setIsActionsModalOpen] = useState(false);
    const [charts, setCharts] = useState([]);

    // Load charts from localStorage on component mount
    useEffect(() => {
        const storedCharts = localStorage.getItem('charts');
        if (storedCharts) {
            setCharts(JSON.parse(storedCharts));
        }
    }, []);

    // Function to update charts and localStoragef
    const updateCharts = (newCharts) => {
        setCharts(newCharts);
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

    return (
        <div className="dashboard">
            <HeaderAlerts />
            <Sidebar
                setIsModalOpen={setIsModalOpen}
                setIsDeleteModalOpen={setIsDeleteModalOpen}
                openActionsModal={() => setIsActionsModalOpen(true)}
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
                />
            </div>
        </div>
    );
};

export default App;
