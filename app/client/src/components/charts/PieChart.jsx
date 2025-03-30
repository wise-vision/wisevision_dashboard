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
import PropTypes from 'prop-types';
import { Pie } from 'react-chartjs-2';
import {
  Chart as ChartJS,
  ArcElement,
  Tooltip,
  Legend
} from 'chart.js';
import './ChartStyles.css';

// Register ChartJS components
ChartJS.register(
  ArcElement,
  Tooltip,
  Legend
);

// Chart color palette
const colorPalette = [
  '#42A5F5', // Primary blue
  '#4BCDF0', // Secondary blue
  '#20428B', // Dark blue
  '#64B5F6', // Light blue
  '#1565C0', // Deep blue
  '#90CAF9', // Very light blue
  '#0D47A1', // Navy blue
  '#BBDEFB'  // Pastel blue
];

const PieChart = ({ data = [], label, isDarkMode = false }) => {
  // Format data for Chart.js
  const chartData = {
    labels: data.map(item => item.name),
    datasets: [
      {
        data: data.map(item => item.value),
        backgroundColor: colorPalette,
        borderColor: isDarkMode ? '#242C43' : '#FFFFFF',
        borderWidth: 2,
        hoverOffset: 10,
      },
    ],
  };

  // Chart options
  const options = {
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: {
        position: 'bottom',
        labels: {
          padding: 20,
          boxWidth: 12,
          color: isDarkMode ? '#E0E0E0' : '#1A1F36',
        },
      },
      tooltip: {
        backgroundColor: isDarkMode ? '#2D3748' : '#FFFFFF',
        titleColor: isDarkMode ? '#E0E0E0' : '#1A1F36',
        bodyColor: isDarkMode ? '#E0E0E0' : '#1A1F36',
        borderColor: isDarkMode ? '#4A5568' : '#E0E0E0',
        borderWidth: 1,
        displayColors: true,
        padding: 12,
        boxPadding: 5
      },
      title: {
        display: true,
        text: label || 'Distribution',
        color: isDarkMode ? '#E0E0E0' : '#1A1F36',
        padding: {
          top: 10,
          bottom: 20
        },
        font: {
          size: 14,
          weight: 'normal'
        }
      }
    },
    cutout: '30%',
    layout: {
      padding: 10
    },
    animation: {
      animateRotate: true,
      animateScale: true
    }
  };

  return (
    <div className={`pie-chart-container ${isDarkMode ? 'dark' : ''}`}>
      <div className="chart-wrapper">
        <Pie data={chartData} options={options} />
      </div>
    </div>
  );
};

PieChart.propTypes = {
  data: PropTypes.arrayOf(
    PropTypes.shape({
      name: PropTypes.string.isRequired,
      value: PropTypes.number.isRequired
    })
  ),
  label: PropTypes.string,
  isDarkMode: PropTypes.bool
};

export default PieChart;
