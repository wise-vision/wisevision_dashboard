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
import PropTypes from 'prop-types';
import Modal from '../ui/Modal';
import Button from '../ui/Button';
import { v4 as uuidv4 } from 'uuid';
import './ModalStyles.css';

const ChartModal = ({ isOpen, onClose, addChart, isDarkMode = false }) => {
  const [chartType, setChartType] = useState('line');
  const [chartTitle, setChartTitle] = useState('');
  const [topic, setTopic] = useState('');
  const [availableTopics, setAvailableTopics] = useState([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  useEffect(() => {
    if (isOpen) {
      fetchTopics();
    }
  }, [isOpen]);

  const fetchTopics = async () => {
    setLoading(true);
    setError(null);
    try {
      const response = await fetch(`${process.env.REACT_APP_API_BASE_URL || ''}/api/topics`);
      if (!response.ok) {
        throw new Error(`Error fetching topics: ${response.statusText}`);
      }
      const data = await response.json();
      setAvailableTopics(data);
    } catch (err) {
      console.error('Error fetching topics:', err);
      setError('Failed to load topics. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (!chartTitle.trim()) {
      setError('Chart title is required');
      return;
    }

    if (chartType !== 'pie' && !topic) {
      setError('Please select a topic');
      return;
    }

    const newChart = {
      id: uuidv4(),
      type: chartType,
      label: chartTitle.trim(),
      topic: topic,
      data: chartType === 'pie' ? [
        { name: 'Sample A', value: 400 },
        { name: 'Sample B', value: 300 },
        { name: 'Sample C', value: 300 },
        { name: 'Sample D', value: 200 }
      ] : null
    };

    addChart(newChart);
    resetForm();
    onClose();
  };

  const resetForm = () => {
    setChartType('line');
    setChartTitle('');
    setTopic('');
    setError(null);
  };

  return (
    <Modal
      isOpen={isOpen}
      onClose={onClose}
      title="Add New Chart"
      size="md"
      animation="slide-up"
      contentClassName={isDarkMode ? 'dark-mode' : ''}
    >
      <form onSubmit={handleSubmit} className="modal-form">
        {error && <div className="error-message">{error}</div>}
        
        <div className="form-group">
          <label htmlFor="chart-title">Chart Title</label>
          <input
            id="chart-title"
            type="text"
            value={chartTitle}
            onChange={(e) => setChartTitle(e.target.value)}
            placeholder="Enter chart title"
            className={isDarkMode ? 'dark-input' : ''}
          />
        </div>
        
        <div className="form-group">
          <label htmlFor="chart-type">Chart Type</label>
          <select
            id="chart-type"
            value={chartType}
            onChange={(e) => setChartType(e.target.value)}
            className={isDarkMode ? 'dark-input' : ''}
          >
            <option value="line">Line Chart</option>
            <option value="pie">Pie Chart</option>
            <option value="gps">GPS Chart</option>
          </select>
        </div>
        
        {chartType !== 'pie' && (
          <div className="form-group">
            <label htmlFor="topic">ROS2 Topic</label>
            {loading ? (
              <div className="loading-spinner">Loading topics...</div>
            ) : (
              <select
                id="topic"
                value={topic}
                onChange={(e) => setTopic(e.target.value)}
                className={isDarkMode ? 'dark-input' : ''}
              >
                <option value="">Select a topic</option>
                {availableTopics.map((topic, index) => (
                  <option key={index} value={topic.name}>
                    {topic.name}
                  </option>
                ))}
              </select>
            )}
            <small className="form-help">
              Select the ROS2 topic to visualize in this chart
            </small>
          </div>
        )}
        
        <div className="modal-actions">
          <Button 
            variant="outline" 
            onClick={() => {
              resetForm();
              onClose();
            }}
          >
            Cancel
          </Button>
          <Button 
            variant="primary" 
            type="submit"
            disabled={loading}
          >
            Add Chart
          </Button>
        </div>
      </form>
    </Modal>
  );
};

ChartModal.propTypes = {
  isOpen: PropTypes.bool.isRequired,
  onClose: PropTypes.func.isRequired,
  addChart: PropTypes.func.isRequired,
  isDarkMode: PropTypes.bool
};

export default ChartModal;
