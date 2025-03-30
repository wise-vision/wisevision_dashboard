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
import { jsPDF } from 'jspdf';
import './ModalStyles.css';

const CreateReportModal = ({ isOpen, onClose, isDarkMode = false }) => {
  const [topics, setTopics] = useState([]);
  const [selectedTopic, setSelectedTopic] = useState('');
  const [reportName, setReportName] = useState('');
  const [messageType, setMessageType] = useState('');
  const [timeRange, setTimeRange] = useState('all');
  const [customStartDate, setCustomStartDate] = useState('');
  const [customEndDate, setCustomEndDate] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');

  useEffect(() => {
    if (isOpen) {
      fetchTopics();
      // Set default dates for custom range (last 7 days)
      const today = new Date();
      const lastWeek = new Date();
      lastWeek.setDate(today.getDate() - 7);
      
      setCustomEndDate(formatDateForInput(today));
      setCustomStartDate(formatDateForInput(lastWeek));
    }
  }, [isOpen]);

  const formatDateForInput = (date) => {
    return date.toISOString().split('T')[0];
  };

  const fetchTopics = async () => {
    setLoading(true);
    try {
      const response = await fetch(`${process.env.REACT_APP_API_BASE_URL || ''}/api/topics`);
      if (!response.ok) {
        throw new Error(`Error fetching topics: ${response.statusText}`);
      }
      const data = await response.json();
      setTopics(data);
    } catch (error) {
      console.error('Error fetching topics:', error);
      setError('Failed to load topics. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const fetchMessageType = async (topicName) => {
    if (!topicName) return;
    
    try {
      const response = await fetch(`${process.env.REACT_APP_API_BASE_URL || ''}/api/message_type/${encodeURIComponent(topicName)}`);
      if (!response.ok) {
        throw new Error('Failed to fetch message type');
      }
      const data = await response.json();
      setMessageType(data.message_type);
    } catch (error) {
      console.error('Error fetching message type:', error);
      setMessageType('');
    }
  };

  const handleTopicChange = (e) => {
    const topic = e.target.value;
    setSelectedTopic(topic);
    if (topic) {
      fetchMessageType(topic);
    } else {
      setMessageType('');
    }
  };

  const getTimeParameters = () => {
    switch (timeRange) {
      case 'today':
        const today = new Date();
        today.setHours(0, 0, 0, 0);
        return {
          start: today.toISOString(),
          end: new Date().toISOString()
        };
      case 'yesterday':
        const yesterday = new Date();
        yesterday.setDate(yesterday.getDate() - 1);
        yesterday.setHours(0, 0, 0, 0);
        const yesterdayEnd = new Date();
        yesterdayEnd.setDate(yesterdayEnd.getDate() - 1);
        yesterdayEnd.setHours(23, 59, 59, 999);
        return {
          start: yesterday.toISOString(),
          end: yesterdayEnd.toISOString()
        };
      case 'lastWeek':
        const lastWeek = new Date();
        lastWeek.setDate(lastWeek.getDate() - 7);
        return {
          start: lastWeek.toISOString(),
          end: new Date().toISOString()
        };
      case 'custom':
        if (!customStartDate || !customEndDate) {
          throw new Error('Custom date range requires both start and end dates');
        }
        const startDate = new Date(`${customStartDate}T00:00:00`);
        const endDate = new Date(`${customEndDate}T23:59:59`);
        return {
          start: startDate.toISOString(),
          end: endDate.toISOString()
        };
      default:
        // All time - no time filters
        return {};
    }
  };

  const handleGenerateReport = async () => {
    if (!selectedTopic) {
      setError('Please select a topic');
      return;
    }

    if (!reportName) {
      setError('Please enter a report name');
      return;
    }

    if (!messageType) {
      setError('No message type available for this topic');
      return;
    }

    setLoading(true);
    setError('');
    setSuccess('');
    
    try {
      const params = new URLSearchParams();
      params.append('type', messageType);
      
      // Add time parameters if applicable
      try {
        const timeParams = getTimeParameters();
        if (timeParams.start) {
          params.append('time_start', timeParams.start);
        }
        if (timeParams.end) {
          params.append('time_end', timeParams.end);
        }
      } catch (err) {
        setError(err.message);
        setLoading(false);
        return;
      }

      // Fetch data for report
      const response = await fetch(
        `${process.env.REACT_APP_API_BASE_URL || ''}/api/topic_echo_data_base_any/${encodeURIComponent(selectedTopic)}?${params.toString()}`
      );

      if (!response.ok) {
        throw new Error(`Error fetching data: ${response.statusText}`);
      }

      const data = await response.json();

      if (!data.messages || data.messages.length === 0) {
        setError('No data available for the selected topic and time range');
        setLoading(false);
        return;
      }

      // Generate PDF report
      await generatePDF(data);
      setSuccess('Report generated successfully!');
      
      // Reset form after short delay
      setTimeout(() => {
        resetForm();
        onClose();
      }, 2000);
    } catch (error) {
      console.error('Error generating report:', error);
      setError(error.message || 'Error generating report');
    } finally {
      setLoading(false);
    }
  };

  const generatePDF = async (data) => {
    // Create a new PDF document
    const doc = new jsPDF();
    const pageWidth = doc.internal.pageSize.getWidth();
    const margin = 10;
    
    // Add report title
    doc.setFontSize(18);
    doc.text(reportName, margin, 20);
    
    // Add report metadata
    doc.setFontSize(11);
    doc.text(`Generated: ${new Date().toLocaleString()}`, margin, 30);
    doc.text(`Topic: ${selectedTopic}`, margin, 35);
    doc.text(`Message Type: ${messageType}`, margin, 40);
    doc.text(`Data Points: ${data.messages.length}`, margin, 45);
    
    // Add time range info
    let yOffset = 55;
    if (timeRange !== 'all') {
      const timeParams = getTimeParameters();
      const startStr = new Date(timeParams.start).toLocaleString();
      const endStr = new Date(timeParams.end).toLocaleString();
      doc.text(`Time Range: ${startStr} to ${endStr}`, margin, yOffset);
      yOffset += 10;
    }
    
    // Add horizontal line
    doc.line(margin, yOffset, pageWidth - margin, yOffset);
    yOffset += 10;
    
    // Add message data
    doc.setFontSize(10);
    let messageCount = 0;
    
    const addDataPage = (pageData) => {
      doc.text("Message Data:", margin, yOffset);
      yOffset += 10;
      
      // Convert data to string representation
      const dataString = JSON.stringify(pageData, null, 2);
      const lines = dataString.split('\n');
      
      for (const line of lines) {
        if (yOffset > doc.internal.pageSize.getHeight() - margin) {
          doc.addPage();
          yOffset = 20;
        }
        
        doc.text(line, margin, yOffset);
        yOffset += 5;
      }
    };
    
    // Handle different kinds of data based on what's available
    if (data.messages && data.messages.length > 0) {
      // Process at most 10 messages to keep the PDF manageable
      const samplesToShow = Math.min(data.messages.length, 10);
      addDataPage(data.messages.slice(0, samplesToShow));
    }
    
    // Save the PDF
    doc.save(`${reportName.replace(/\s+/g, '_')}.pdf`);
  };

  const resetForm = () => {
    setSelectedTopic('');
    setReportName('');
    setMessageType('');
    setTimeRange('all');
    setError('');
    setSuccess('');
  };

  return (
    <Modal
      isOpen={isOpen}
      onClose={onClose}
      title="Generate Report"
      size="md"
      animation="slide-up"
      contentClassName={isDarkMode ? 'dark-mode' : ''}
    >
      <div className="modal-form">
        {error && <div className="error-message">{error}</div>}
        {success && <div className="success-message">{success}</div>}
        
        <div className="form-group">
          <label htmlFor="report-name">Report Name</label>
          <input
            id="report-name"
            type="text"
            value={reportName}
            onChange={(e) => setReportName(e.target.value)}
            placeholder="Enter a name for your report"
            className={isDarkMode ? 'dark-input' : ''}
            required
          />
        </div>
        
        <div className="form-group">
          <label htmlFor="topic-select">Select Topic</label>
          {loading ? (
            <div className="loading-spinner">Loading topics...</div>
          ) : (
            <select
              id="topic-select"
              value={selectedTopic}
              onChange={handleTopicChange}
              className={isDarkMode ? 'dark-input' : ''}
              required
            >
              <option value="">Select a topic</option>
              {topics.map((topic, index) => (
                <option key={index} value={topic.name}>
                  {topic.name}
                </option>
              ))}
            </select>
          )}
        </div>
        
        {selectedTopic && (
          <div className="form-group">
            <label>Message Type</label>
            <div className="text-field">
              {messageType || 'Loading...'}
            </div>
          </div>
        )}
        
        <div className="form-group">
          <label htmlFor="time-range">Time Range</label>
          <select
            id="time-range"
            value={timeRange}
            onChange={(e) => setTimeRange(e.target.value)}
            className={isDarkMode ? 'dark-input' : ''}
          >
            <option value="all">All Time</option>
            <option value="today">Today</option>
            <option value="yesterday">Yesterday</option>
            <option value="lastWeek">Last 7 Days</option>
            <option value="custom">Custom Range</option>
          </select>
        </div>
        
        {timeRange === 'custom' && (
          <div className="form-row">
            <div className="form-group form-group-half">
              <label htmlFor="start-date">Start Date</label>
              <input
                id="start-date"
                type="date"
                value={customStartDate}
                onChange={(e) => setCustomStartDate(e.target.value)}
                className={isDarkMode ? 'dark-input' : ''}
                required
              />
            </div>
            <div className="form-group form-group-half">
              <label htmlFor="end-date">End Date</label>
              <input
                id="end-date"
                type="date"
                value={customEndDate}
                onChange={(e) => setCustomEndDate(e.target.value)}
                className={isDarkMode ? 'dark-input' : ''}
                required
              />
            </div>
          </div>
        )}
        
        <div className="modal-actions">
          <Button 
            variant="outline" 
            onClick={() => {
              resetForm();
              onClose();
            }}
            disabled={loading}
          >
            Cancel
          </Button>
          <Button 
            variant="primary" 
            onClick={handleGenerateReport}
            loading={loading}
          >
            Generate Report
          </Button>
        </div>
      </div>
    </Modal>
  );
};

CreateReportModal.propTypes = {
  isOpen: PropTypes.bool.isRequired,
  onClose: PropTypes.func.isRequired,
  isDarkMode: PropTypes.bool
};

export default CreateReportModal;
