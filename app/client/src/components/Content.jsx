/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import React from 'react';
import ChartModal from './ChartModal';
import DeleteChartModal from './DeleteChartModal';
import ActionModal from './ActionModal';
import StyledLineChart from './StyledLineChart';
import StyledPieChart from './StyledPieChart';
import ContentHeader from './ContentHeader';
import '../styles/Content.css';
import Gps from './Gps';

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
                 }) => {
    return (
        <div className="content">
            <ContentHeader />

            <div className="dashboard-grid">
                {charts.map((chart) => (
                    <div
                        key={chart.id}
                        className={
                            chart.type === 'pie'
                                ? 'pie-chart-container'
                                : chart.type === 'line'
                                    ? 'line-chart-container'
                                    : chart.type === 'gps'
                                        ? 'gps-chart-container'
                                        : ''
                        }
                    >
                        <h3>{chart.label}</h3>

                        {chart.type === 'line' && <StyledLineChart data={chart} />}

                        {chart.type === 'pie' && <StyledPieChart data={chart} />}

                        {chart.type === 'gps' && <Gps chartData={chart} />}
                    </div>
                ))}
            </div>

            {isModalOpen && (
                <ChartModal
                    onClose={() => setIsModalOpen(false)}
                    onAddChart={addChart}
                />
            )}

            {isDeleteModalOpen && (
                <DeleteChartModal
                    onClose={() => setIsDeleteModalOpen(false)}
                    onDeleteChart={deleteChartByName}
                    charts={charts}
                />
            )}

            {isActionsModalOpen && (
                <ActionModal
                    onClose={() => setIsActionsModalOpen(false)}
                />
            )}
        </div>
    );
};

export default Content;
