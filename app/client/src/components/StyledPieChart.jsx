import React, { useState } from 'react';
import { Pie } from 'react-chartjs-2';
import 'chart.js/auto';

const StyledPieChart = ({ data }) => {
    // Suppress unused state warning for displayedValue
    const [displayedValue, setDisplayedValue] = useState('Hover to see value');

    const total = data.values.reduce((acc, val) => acc + val, 0);
    const contentAValue = data.values[0] ? ((data.values[0] / total) * 100).toFixed(2) : 0;
    const contentBValue = data.values[1] ? ((data.values[1] / total) * 100).toFixed(2) : 0;

    const chartData = {
        labels: data.labels || [],
        datasets: [
            {
                data: data.values || [],
                backgroundColor: ['#4BCDF0', '#20428B'],
                hoverBackgroundColor: ['#4BCDF0', '#20428B'],
                borderWidth: 0,
            },
        ],
    };

    const chartOptions = {
        plugins: {
            legend: { display: false }
        },
        responsive: true,
        maintainAspectRatio: false,
        onHover: (event, chartElement) => {
            if (chartElement.length) {
                const index = chartElement[0].index;
                const value = chartData.datasets[0].data[index];
                setDisplayedValue(`${value}%`);
            } else {
                setDisplayedValue('Hover to see value');
            }
        },
    };

    return (
        <div style={{ width: '100%', height: '90%' }}>
            <div style={{ width: '100%', height: '60%' }}>
                <Pie data={chartData} options={chartOptions} />
            </div>
            <div style={{ display: 'flex', justifyContent: 'space-around', padding: '20px', backgroundColor: '#fff', borderRadius: '10px', boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)', marginTop: '20px' }}>
                <div style={{ textAlign: 'center' }}>
                    <span style={{ display: 'inline-block', width: '10px', height: '10px', backgroundColor: '#20428B', borderRadius: '50%', marginRight: '10px' }}></span>
                    <span style={{ fontSize: '13px', color: '#666' }}>Content A</span>
                    <h3 style={{ fontSize: '20px', margin: '0', color: '#000' }}>{contentAValue}%</h3>
                </div>
                <div style={{ textAlign: 'center', borderLeft: '1px solid #ddd', paddingLeft: '20px' }}>
                    <span style={{ display: 'inline-block', width: '10px', height: '10px', backgroundColor: '#4BCDF0', borderRadius: '50%', marginRight: '10px' }}></span>
                    <span style={{ fontSize: '13px', color: '#666' }}>Content B</span>
                    <h3 style={{ fontSize: '20px', margin: '0', color: '#000' }}>{contentBValue}%</h3>
                </div>
            </div>
        </div>
    );
};

export default StyledPieChart;
