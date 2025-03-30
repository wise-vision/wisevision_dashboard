/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import React, { useState, useEffect, useCallback } from 'react';
import PropTypes from 'prop-types';
import { Line } from 'react-chartjs-2';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
} from 'chart.js';
import './ChartStyles.css';

// Register ChartJS components
ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

const LineChart = ({ topic, label, isDarkMode = false }) => {
  const [chartData, setChartData] = useState({
    labels: [],
    datasets: []
  });
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [lastUpdate, setLastUpdate] = useState('');

  // Define fetchData with useCallback to prevent infinite loop
  const fetchData = useCallback(async () => {
    if (!topic) {
      setError('No topic specified');
      setLoading(false);
      return;
    }
    
    try {
      const response = await fetch(
        `${process.env.REACT_APP_API_BASE_URL || ''}/api/topic_echo_data_base_any_last_week/${encodeURIComponent(topic)}?type=${encodeURIComponent('std_msgs/msg/Float32')}`
      );
      
      if (!response.ok) {
        throw new Error(`Error fetching data: ${response.statusText}`);
      }
      
      const data = await response.json();
      
      // Process response data for chart
      const timestamps = data.timestamps?.map(ts => 
        `${ts.month}/${ts.day} ${ts.hour}:${ts.minute}`
      ) || [];
      
      const values = data.messages?.map(msg => 
        typeof msg.data === 'number' ? msg.data : 0
      ) || [];
      
      // Set chart data
      setChartData({
        labels: timestamps,
        datasets: [
          {
            label: label || 'Sensor Data',
            data: values,
            borderColor: '#1E88E5',
            backgroundColor: 'rgba(66, 165, 245, 0.1)',
            borderWidth: 2,
            pointBackgroundColor: '#1565C0',
            pointBorderColor: '#fff',
            pointHoverBackgroundColor: '#fff',
            pointHoverBorderColor: '#1E88E5',
            fill: true,
            tension: 0.3
          }
        ]
      });
      
      setLastUpdate(new Date().toLocaleTimeString());
      setError(null);
      setLoading(false);
    } catch (err) {
      console.error('Error fetching chart data:', err);
      setError(`Failed to load data: ${err.message}`);
      setLoading(false);
    }
  }, [topic]); // Topic as dependency

  useEffect(() => {
    fetchData();
    // Poll for new data every 5 seconds
    const intervalId = setInterval(fetchData, 5000);
    
    return () => clearInterval(intervalId);
  }, [fetchData]); // Added fetchData as dependency

  // Chart options
  const options = {
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: {
        position: 'top',
        labels: {
          color: isDarkMode ? '#E0E0E0' : '#1A1F36'
        }
      },
      title: {
        display: false
      },
      tooltip: {
        backgroundColor: isDarkMode ? '#2D3748' : '#fff',
        titleColor: isDarkMode ? '#E0E0E0' : '#1A1F36',
        bodyColor: isDarkMode ? '#E0E0E0' : '#1A1F36',
        borderColor: isDarkMode ? '#4A5568' : '#E0E0E0',
        borderWidth: 1,
        padding: 10,
        boxPadding: 5,
        usePointStyle: true,
      }
    },
    scales: {
      x: {
        grid: {
          color: isDarkMode ? 'rgba(255, 255, 255, 0.1)' : 'rgba(0, 0, 0, 0.1)'
        },
        ticks: {
          color: isDarkMode ? '#A0AEC0' : '#546E7A',
          maxRotation: 45,
          minRotation: 45
        }
      },
      y: {
        grid: {
          color: isDarkMode ? 'rgba(255, 255, 255, 0.1)' : 'rgba(0, 0, 0, 0.1)'
        },
        ticks: {
          color: isDarkMode ? '#A0AEC0' : '#546E7A'
        }
      }
    }
  };

  return (
    <div className={`line-chart-container ${isDarkMode ? 'dark' : ''}`}>
      {loading ? (
        <div className="chart-loading">
          <div className="spinner"></div>
          <p>Loading data...</p>
        </div>
      ) : error ? (
        <div className="chart-error">
          <p>{error}</p>
          <button onClick={fetchData} className="retry-button">
            Retry
          </button>
        </div>
      ) : (
        <>
          <div className="chart-wrapper">
            <Line data={chartData} options={options} />
          </div>
          <div className="chart-footer">
            <span className="chart-topic">{topic}</span>
            <span className="chart-updated">Updated: {lastUpdate}</span>
          </div>
        </>
      )}
    </div>
  );
};

LineChart.propTypes = {
  topic: PropTypes.string.isRequired,
  label: PropTypes.string,
  isDarkMode: PropTypes.bool
};

export default LineChart;
