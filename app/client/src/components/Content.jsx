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
