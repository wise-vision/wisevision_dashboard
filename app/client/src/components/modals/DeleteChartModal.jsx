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
import Modal from '../ui/Modal';
import Button from '../ui/Button';
import './ModalStyles.css';

const DeleteChartModal = ({ isOpen, onClose, charts, deleteChartByName, isDarkMode = false }) => {
  const [selectedChart, setSelectedChart] = useState('');
  const [error, setError] = useState(null);

  const handleDelete = () => {
    if (!selectedChart) {
      setError('Please select a chart to delete');
      return;
    }

    deleteChartByName(selectedChart);
    resetForm();
    onClose();
  };

  const resetForm = () => {
    setSelectedChart('');
    setError(null);
  };

  return (
    <Modal
      isOpen={isOpen}
      onClose={onClose}
      title="Delete Chart"
      size="sm"
      animation="scale"
      contentClassName={isDarkMode ? 'dark-mode' : ''}
    >
      <div className="modal-form">
        {error && <div className="error-message">{error}</div>}
        
        {charts.length === 0 ? (
          <div className="empty-state-message">
            <p>No charts available to delete.</p>
          </div>
        ) : (
          <>
            <div className="form-group">
              <label htmlFor="chart-select">Select Chart</label>
              <select
                id="chart-select"
                value={selectedChart}
                onChange={(e) => setSelectedChart(e.target.value)}
                className={isDarkMode ? 'dark-input' : ''}
              >
                <option value="">Select a chart</option>
                {charts.map((chart, index) => (
                  <option key={index} value={chart.label}>
                    {chart.label}
                  </option>
                ))}
              </select>
            </div>
            
            <div className="warning-message">
              Warning: This action cannot be undone.
            </div>
            
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
                variant="danger" 
                onClick={handleDelete}
              >
                Delete Chart
              </Button>
            </div>
          </>
        )}
      </div>
    </Modal>
  );
};

DeleteChartModal.propTypes = {
  isOpen: PropTypes.bool.isRequired,
  onClose: PropTypes.func.isRequired,
  charts: PropTypes.array.isRequired,
  deleteChartByName: PropTypes.func.isRequired,
  isDarkMode: PropTypes.bool
};

export default DeleteChartModal;
