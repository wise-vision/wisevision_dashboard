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
import Card from '../ui/Card';
import './ModalStyles.css';

const ActionsModal = ({ isOpen, onClose, isDarkMode = false }) => {
  const [tabIndex, setTabIndex] = useState(0);
  const [actions, setActions] = useState([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedAction, setSelectedAction] = useState(null);

  // Form states for creating a new action
  const [actionType, setActionType] = useState('simple');
  const [actionName, setActionName] = useState('');
  const [listenTopic, setListenTopic] = useState('');
  const [messageTrigger, setMessageTrigger] = useState('');
  const [publishTopic, setPublishTopic] = useState('');
  const [publishMessage, setPublishMessage] = useState('');
  const [logicExpression, setLogicExpression] = useState('');
  const [availableTopics, setAvailableTopics] = useState([]);

  // Convert fetchActions to useCallback
  const fetchActions = React.useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const response = await fetch(`${process.env.REACT_APP_API_BASE_URL || ''}/api/available_topics`);
      if (!response.ok) {
        throw new Error(`Error fetching actions: ${response.statusText}`);
      }
      const data = await response.json();
      
      // Transform the data into a consistent format
      const formattedActions = data.available_topics_with_parameters_and_time?.map(action => ({
        id: action.action_and_publisher_name,
        name: action.action_and_publisher_name,
        type: 'simple',
        listenTopic: action.listen_topic,
        trigger: action.trigger_text || 'Any message',
        active: true,
        createdAt: formatDateTime(action.time_of_creation),
        lastTriggered: formatDateTime(action.time_of_last_trigger)
      })) || [];
      
      setActions(formattedActions);
    } catch (err) {
      console.error('Error fetching actions:', err);
      setError('Failed to load automatic actions. Please try again.');
    } finally {
      setLoading(false);
    }
  }, []); // No dependencies for now

  const fetchTopics = React.useCallback(async () => {
    try {
      const response = await fetch(`${process.env.REACT_APP_API_BASE_URL || ''}/api/topics`);
      if (!response.ok) {
        throw new Error(`Error fetching topics: ${response.statusText}`);
      }
      const data = await response.json();
      setAvailableTopics(data);
    } catch (err) {
      console.error('Error fetching topics:', err);
    }
  }, []);

  useEffect(() => {
    if (isOpen) {
      fetchActions();
      fetchTopics();
    }
  }, [isOpen, fetchActions, fetchTopics]); // Added missing dependencies

  const formatDateTime = (dateTimeObj) => {
    if (!dateTimeObj) return 'Never';
    
    try {
      const { year, month, day, hour, minute, second } = dateTimeObj;
      return `${year}-${String(month).padStart(2, '0')}-${String(day).padStart(2, '0')} ${String(hour).padStart(2, '0')}:${String(minute).padStart(2, '0')}:${String(second).padStart(2, '0')}`;
    } catch (e) {
      return 'Invalid date';
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError(null);
    
    if (!actionName || !listenTopic || !publishTopic) {
      setError('Please fill out all required fields');
      return;
    }
    
    setLoading(true);
    try {
      let response;
      
      // Simple action creation
      if (actionType === 'simple') {
        const payload = {
          listen_topic: listenTopic,
          listen_message_type: "std_msgs/msg/String",
          trigger_text: messageTrigger || "Any message",
          action_and_publisher_name: actionName,
          pub_msg_type: "std_msgs/msg/String",
          publication_method: 0
        };
        
        response = await fetch(`${process.env.REACT_APP_API_BASE_URL || ''}/api/create_automatic_action`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(payload),
        });
      } else {
        // Combined action creation
        const listenTopics = logicExpression.match(/\b[a-zA-Z0-9_/]+\b/g).filter(word => 
          word !== 'and' && word !== 'or'
        );
        
        const payload = {
          listen_topics: listenTopics,
          logic_expression: logicExpression,
          action_and_publisher_name: actionName,
          trigger_text: messageTrigger || "Message matches condition",
          publication_method: 0
        };
        
        response = await fetch(`${process.env.REACT_APP_API_BASE_URL || ''}/api/create_combined_automatic_action`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(payload),
        });
      }
      
      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || 'Failed to create action');
      }
      
      // Refresh action list and reset form
      fetchActions();
      resetForm();
      setTabIndex(0); // Switch to actions list tab
    } catch (err) {
      console.error('Error creating action:', err);
      setError(`Failed to create action: ${err.message}`);
    } finally {
      setLoading(false);
    }
  };

  const handleDeleteAction = async (actionToDelete) => {
    setLoading(true);
    setError(null);
    
    try {
      const endpoint = actionToDelete.type === 'simple' 
        ? '/api/delete_automatic_action'
        : '/api/delete_combined_automatic_action';
      
      const payload = actionToDelete.type === 'simple'
        ? { listen_topic_to_delete: actionToDelete.listenTopic }
        : { name_of_combined_topics_publisher: actionToDelete.name };
      
      const response = await fetch(`${process.env.REACT_APP_API_BASE_URL || ''}${endpoint}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });
      
      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || 'Failed to delete action');
      }
      
      // Refresh actions list
      fetchActions();
      setSelectedAction(null);
    } catch (err) {
      console.error('Error deleting action:', err);
      setError(`Failed to delete action: ${err.message}`);
    } finally {
      setLoading(false);
    }
  };

  const resetForm = () => {
    setActionType('simple');
    setActionName('');
    setListenTopic('');
    setMessageTrigger('');
    setPublishTopic('');
    setPublishMessage('');
    setLogicExpression('');
  };

  return (
    <Modal
      isOpen={isOpen}
      onClose={onClose}
      title="Automatic Actions"
      size="lg"
      contentClassName={isDarkMode ? 'dark-mode' : ''}
    >
      <div className="tabs-navigation">
        <button 
          className={`tab-item ${tabIndex === 0 ? 'active' : ''}`} 
          onClick={() => setTabIndex(0)}
        >
          Actions List
        </button>
        <button 
          className={`tab-item ${tabIndex === 1 ? 'active' : ''}`} 
          onClick={() => setTabIndex(1)}
        >
          Create New Action
        </button>
        {selectedAction && (
          <button 
            className={`tab-item ${tabIndex === 2 ? 'active' : ''}`} 
            onClick={() => setTabIndex(2)}
          >
            Action Details
          </button>
        )}
      </div>
      
      {error && <div className="error-message">{error}</div>}
      
      {tabIndex === 0 && (
        <div className="actions-list">
          {loading && <div className="loading-spinner">Loading actions...</div>}
          
          {actions.length > 0 ? (
            actions.map(action => (
              <Card
                key={action.id}
                title={action.name}
                subtitle={`Type: ${action.type === 'simple' ? 'Simple' : 'Combined'}`}
                className="action-card mb-md"
                elevation="sm"
              >
                <div className="action-card-content">
                  <div className="action-details">
                    <p><strong>Listen Topic:</strong> {action.listenTopic}</p>
                    <p><strong>Trigger:</strong> {action.trigger}</p>
                    <p><strong>Created:</strong> {action.createdAt}</p>
                    <p><strong>Last Triggered:</strong> {action.lastTriggered}</p>
                  </div>
                  <div className="action-controls mt-md">
                    <Button
                      variant="outline"
                      size="small"
                      onClick={() => {
                        setSelectedAction(action);
                        setTabIndex(2);
                      }}
                      className="mr-md"
                    >
                      View Details
                    </Button>
                    <Button
                      variant="danger"
                      size="small"
                      onClick={() => handleDeleteAction(action)}
                    >
                      Delete
                    </Button>
                  </div>
                </div>
              </Card>
            ))
          ) : !loading && (
            <div className="empty-state-message">
              <p>No automatic actions defined yet.</p>
              <Button 
                variant="primary"
                onClick={() => setTabIndex(1)}
                className="mt-md"
              >
                Create New Action
              </Button>
            </div>
          )}
        </div>
      )}
      
      {tabIndex === 1 && (
        <form onSubmit={handleSubmit} className="modal-form">
          <div className="form-group">
            <label htmlFor="action-type">Action Type</label>
            <select
              id="action-type"
              value={actionType}
              onChange={(e) => setActionType(e.target.value)}
              className={isDarkMode ? 'dark-input' : ''}
            >
              <option value="simple">Simple Action</option>
              <option value="combined">Combined Action</option>
            </select>
            <small className="form-help">
              {actionType === 'simple' 
                ? 'Trigger an action when a message is received on a single topic' 
                : 'Create a complex condition based on multiple topics'}
            </small>
          </div>
          
          <div className="form-group">
            <label htmlFor="action-name">Action Name</label>
            <input
              id="action-name"
              type="text"
              value={actionName}
              onChange={(e) => setActionName(e.target.value)}
              placeholder="Enter a unique name for this action"
              className={isDarkMode ? 'dark-input' : ''}
              required
            />
          </div>
          
          {actionType === 'simple' ? (
            <>
              <div className="form-group">
                <label htmlFor="listen-topic">Listen Topic</label>
                <select
                  id="listen-topic"
                  value={listenTopic}
                  onChange={(e) => setListenTopic(e.target.value)}
                  className={isDarkMode ? 'dark-input' : ''}
                  required
                >
                  <option value="">Select a topic</option>
                  {availableTopics.map((topic, index) => (
                    <option key={index} value={topic.name}>
                      {topic.name}
                    </option>
                  ))}
                </select>
              </div>
              
              <div className="form-group">
                <label htmlFor="message-trigger">Message Trigger (optional)</label>
                <input
                  id="message-trigger"
                  type="text"
                  value={messageTrigger}
                  onChange={(e) => setMessageTrigger(e.target.value)}
                  placeholder="Trigger on specific message content (leave empty for any message)"
                  className={isDarkMode ? 'dark-input' : ''}
                />
              </div>
              
              <div className="form-group">
                <label htmlFor="publish-topic">Publish Topic</label>
                <input
                  id="publish-topic"
                  type="text"
                  value={publishTopic}
                  onChange={(e) => setPublishTopic(e.target.value)}
                  placeholder="Topic to publish to when triggered"
                  className={isDarkMode ? 'dark-input' : ''}
                  required
                />
              </div>
              
              <div className="form-group">
                <label htmlFor="publish-message">Publish Message (optional)</label>
                <textarea
                  id="publish-message"
                  value={publishMessage}
                  onChange={(e) => setPublishMessage(e.target.value)}
                  placeholder="Message to publish (leave empty for default message)"
                  className={isDarkMode ? 'dark-input' : ''}
                  rows={3}
                />
              </div>
            </>
          ) : (
            <>
              <div className="form-group">
                <label htmlFor="logic-expression">Logic Expression</label>
                <textarea
                  id="logic-expression"
                  value={logicExpression}
                  onChange={(e) => setLogicExpression(e.target.value)}
                  placeholder="Example: (topic1 and topic2) or topic3"
                  className={isDarkMode ? 'dark-input' : ''}
                  rows={3}
                  required
                />
                <small className="form-help">
                  Combine topic names using 'and' and 'or' operators. Use parentheses to group expressions.
                </small>
              </div>
              
              <div className="form-group">
                <label htmlFor="message-trigger">Trigger Message (optional)</label>
                <input
                  id="message-trigger"
                  type="text"
                  value={messageTrigger}
                  onChange={(e) => setMessageTrigger(e.target.value)}
                  placeholder="Custom trigger message"
                  className={isDarkMode ? 'dark-input' : ''}
                />
              </div>
              
              <div className="form-group">
                <label htmlFor="publish-topic">Publish Topic</label>
                <input
                  id="publish-topic"
                  type="text"
                  value={publishTopic}
                  onChange={(e) => setPublishTopic(e.target.value)}
                  placeholder="Topic to publish to when triggered"
                  className={isDarkMode ? 'dark-input' : ''}
                  required
                />
              </div>
            </>
          )}
          
          <div className="modal-actions">
            <Button 
              variant="outline" 
              onClick={() => {
                resetForm();
                setTabIndex(0);
              }}
            >
              Cancel
            </Button>
            <Button 
              variant="primary" 
              type="submit"
              loading={loading}
            >
              Create Action
            </Button>
          </div>
        </form>
      )}
      
      {tabIndex === 2 && selectedAction && (
        <div className="action-details-view">
          <Card
            title={selectedAction.name}
            subtitle={`Type: ${selectedAction.type === 'simple' ? 'Simple' : 'Combined'}`}
            elevation="md"
          >
            <div className="action-details-content">
              <div className="detail-item">
                <span className="detail-label">Listen Topic:</span>
                <span className="detail-value">{selectedAction.listenTopic}</span>
              </div>
              <div className="detail-item">
                <span className="detail-label">Trigger:</span>
                <span className="detail-value">{selectedAction.trigger}</span>
              </div>
              <div className="detail-item">
                <span className="detail-label">Status:</span>
                <span className={`detail-value status-${selectedAction.active ? 'active' : 'inactive'}`}>
                  {selectedAction.active ? 'Active' : 'Inactive'}
                </span>
              </div>
              <div className="detail-item">
                <span className="detail-label">Created:</span>
                <span className="detail-value">{selectedAction.createdAt}</span>
              </div>
              <div className="detail-item">
                <span className="detail-label">Last Triggered:</span>
                <span className="detail-value">{selectedAction.lastTriggered}</span>
              </div>
            </div>
            
            <div className="modal-actions mt-md">
              <Button 
                variant="outline" 
                onClick={() => setTabIndex(0)}
              >
                Back to List
              </Button>
              <Button 
                variant="danger" 
                onClick={() => handleDeleteAction(selectedAction)}
                loading={loading}
              >
                Delete Action
              </Button>
            </div>
          </Card>
        </div>
      )}
    </Modal>
  );
};

ActionsModal.propTypes = {
  isOpen: PropTypes.bool.isRequired,
  onClose: PropTypes.func.isRequired,
  isDarkMode: PropTypes.bool
};

export default ActionsModal;
