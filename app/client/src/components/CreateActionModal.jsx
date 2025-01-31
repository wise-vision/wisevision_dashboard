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
import '../styles/CreateActionModal.css';

const CreateActionModal = ({ isOpen, onClose, onActionCreated }) => {
    const [isClosing, setIsClosing] = useState(false);
    const [actionType, setActionType] = useState(null); // 'action' or 'combined'

    // State for single action
    const [actionData, setActionData] = useState({
        actionAndPublisherName: '',
        listenTopic: '',
        listenMessageType: '',
        value: '',
        triggerVal: '',
        triggerType: 'LessThan',
        pubMessageType: '',
        triggerText: '',
        dataValidityMs: '',
        publicationMethod: 0 // Dodane pole
    });

    // State for combined action
    const [numActions, setNumActions] = useState(2);
    const [selectedActions, setSelectedActions] = useState([]);
    const [availableActions, setAvailableActions] = useState([]);
    const [combinedActionData, setCombinedActionData] = useState({
        actionAndPublisherName: '',
        logicExpression: '',
        triggerText: '',
        publicationMethod: 0
    });

    const [topics, setTopics] = useState([]);
    const [messageStructure, setMessageStructure] = useState({});
    const [fields, setFields] = useState([]);
    const [message, setMessage] = useState('');
    const [isLoadingFields, setIsLoadingFields] = useState(false);

    useEffect(() => {
        if (isOpen) {
            const fetchTopics = async () => {
                try {
                    const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/topics`);
                    const data = await response.json();
                    setTopics(data);
                } catch (error) {
                    console.error('Error fetching topics:', error);
                }
            };

            fetchTopics();
        }
    }, [isOpen]);

    useEffect(() => {
        if (actionData.listenMessageType) {
            const fetchMessageStructure = async () => {
                setIsLoadingFields(true);
                try {
                    const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/message_structure/${actionData.listenMessageType}`);
                    const data = await response.json();
                    setMessageStructure(data);
                } catch (error) {
                    console.error('Error fetching message structure:', error);
                    setMessageStructure({});
                    setFields([]);
                } finally {
                    setIsLoadingFields(false);
                }
            };

            fetchMessageStructure();
        } else {
            // Reset message structure and fields if listenMessageType is empty
            setMessageStructure({});
            setFields([]);
        }
    }, [actionData.listenMessageType]);

    useEffect(() => {
        if (messageStructure && Object.keys(messageStructure).length > 0) {
            const availableFields = extractFields(messageStructure);
            setFields(availableFields);
        } else {
            setFields([]);
        }
    }, [messageStructure]);

    useEffect(() => {
        if (actionType === 'combined' && isOpen) {
            const fetchAvailableActions = async () => {
                try {
                    const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/available_topics`);
                    const data = await response.json();
                    setAvailableActions(data.available_topics_with_parameters_and_time);
                } catch (error) {
                    console.error('Error fetching available actions:', error);
                }
            };

            fetchAvailableActions();
        }
    }, [actionType, isOpen]);

    useEffect(() => {
        if (!isOpen) {
            // Reset all states when modal is closed
            setActionType(null);
            setActionData({
                actionAndPublisherName: '',
                listenTopic: '',
                listenMessageType: '',
                value: '',
                triggerVal: '',
                triggerType: 'LessThan',
                pubMessageType: '',
                triggerText: '',
                dataValidityMs: '',
                publicationMethod: 0
            });
            setCombinedActionData({
                actionAndPublisherName: '',
                logicExpression: '',
                triggerText: '',
                publicationMethod: 0
            });
            setSelectedActions([]);
            setFields([]);
            setMessageStructure({});
            setMessage('');
        }
    }, [isOpen]);

    const extractFields = (structure, parent = '') => {
        let fields = [];
        for (let key in structure) {
            const value = structure[key];
            const fullPath = parent ? `${parent}.${key}` : key;

            if (Array.isArray(value)) {
                if (value.length > 0 && typeof value[0] === 'object') {
                    fields = fields.concat(extractFields(value[0], fullPath + '[]'));
                } else {
                    fields.push(fullPath + '[]');
                }
            } else if (typeof value === 'object' && value !== null) {
                fields = fields.concat(extractFields(value, fullPath));
            } else {
                fields.push(fullPath);
            }
        }
        return fields;
    };

    const handleChange = (e) => {
        const { name, value } = e.target;
        setActionData((prev) => ({ ...prev, [name]: value }));
    };

    const handleCombinedChange = (e) => {
        const { name, value } = e.target;
        setCombinedActionData((prev) => ({ ...prev, [name]: value }));
    };

    const handleSelectedActionChange = (e, index) => {
        const value = e.target.value;
        setSelectedActions((prev) => {
            const newSelectedActions = [...prev];
            newSelectedActions[index] = value;
            return newSelectedActions;
        });
    };

    const handleSubmit = async (e) => {
        e.preventDefault();

        const dataValidityMsAsNumber = parseInt(actionData.dataValidityMs, 10);
        const publicationMethodAsNumber = parseInt(actionData.publicationMethod, 10);

        // Walidacja publicationMethod
        if (isNaN(publicationMethodAsNumber) || publicationMethodAsNumber < 0 || publicationMethodAsNumber > 6) {
            setMessage('Publication Method musi być liczbą w zakresie od 0 do 6.');
            return;
        }

        try {
            const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/create_automatic_action`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    listen_topic: actionData.listenTopic,
                    listen_message_type: actionData.listenMessageType,
                    value: actionData.value,
                    trigger_val: actionData.triggerVal,
                    trigger_type: actionData.triggerType,
                    action_and_publisher_name: actionData.actionAndPublisherName,
                    pub_message_type: actionData.pubMessageType,
                    trigger_text: actionData.triggerText,
                    data_validity_ms: dataValidityMsAsNumber,
                    publication_method: publicationMethodAsNumber // Dodane pole
                })
            });

            const result = await response.json();

            if (result.success) {
                setMessage(`Action created successfully. Response: ${JSON.stringify(result)}`);
                if (onActionCreated) {
                    onActionCreated(actionData);
                }

                setTimeout(() => {
                    handleCancel();
                }, 5000);
            } else {
                setMessage(`Failed to create action: ${JSON.stringify(result)}`);
            }
        } catch (error) {
            console.error('Error creating action:', error);
            setMessage('An error occurred while creating the action.');
        }
    };

    const handleCombinedSubmit = async (e) => {
        e.preventDefault();

        const publicationMethodAsNumber = parseInt(combinedActionData.publicationMethod, 10);

        // Walidacja publicationMethod
        if (isNaN(publicationMethodAsNumber) || publicationMethodAsNumber < 0 || publicationMethodAsNumber > 6) {
            setMessage('Publication Method musi być liczbą w zakresie od 0 do 6.');
            return;
        }

        const data = {
            listen_topics: selectedActions,
            logic_expression: combinedActionData.logicExpression,
            action_and_publisher_name: combinedActionData.actionAndPublisherName,
            trigger_text: combinedActionData.triggerText,
            publication_method: publicationMethodAsNumber // Dodane pole
        };

        try {
            //api
            const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/create_combined_automatic_action`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(data)
            });

            const result = await response.json();
            console.log("API Response:", result);

            if (result.success) {
                setMessage(`Combined action created successfully. Response: ${JSON.stringify(result)}`);
                if (onActionCreated) {
                    onActionCreated(combinedActionData);
                }

                setTimeout(() => {
                    handleCancel();
                }, 5000);
            } else {
                setMessage(`Failed to create combined action. Response: ${JSON.stringify(result)}`); // Display error message without closing
            }
        } catch (error) {
            console.error('Error creating combined action:', error);
            setMessage('An error occurred while creating the combined action.');
        }
    };

    const handleCancel = () => {
        setIsClosing(true);
        setTimeout(() => {
            setIsClosing(false);
            onClose();
        }, 600);
    };

    if (!isOpen && !isClosing) return null;

    if (!actionType) {
        return (
            <div className={`create-action-modal ${isClosing ? 'closing' : ''}`}>
                <div className={`modal-content ${isClosing ? 'closing' : ''}`}>
                    <button type="button" onClick={handleCancel} className="close-icon">&times;</button>
                    <h2>Select Action Type</h2>
                    <div className="modal-actions">
                        <button onClick={() => setActionType('action')} className="select-button">Action</button>
                        <button onClick={() => setActionType('combined')} className="select-button">Combined Action</button>
                    </div>
                </div>
            </div>
        );
    }

    if (actionType === 'combined') {
        return (
            <div className={`create-action-modal ${isClosing ? 'closing' : ''}`}>
                <div className={`modal-content ${isClosing ? 'closing' : ''}`}>
                    <h2>Create Combined Action</h2>
                    {message && <div className="message">{message}</div>}
                    <form onSubmit={handleCombinedSubmit} className="new-action-form">
                        <div className="form-group">
                            <label>Number of Actions to Combine:</label>
                            <select value={numActions} onChange={(e) => setNumActions(parseInt(e.target.value, 10))}>
                                <option value={2}>2</option>
                                <option value={3}>3</option>
                                <option value={4}>4</option>
                                <option value={5}>5</option>
                                <option value={6}>6</option>
                            </select>
                        </div>
                        {[...Array(numActions)].map((_, index) => (
                            <div className="form-group" key={index}>
                                <label>Select Action {index + 1}:</label>
                                <select
                                    value={selectedActions[index] || ''}
                                    onChange={(e) => handleSelectedActionChange(e, index)}
                                    required
                                >
                                    <option value="">Select an Action</option>
                                    {availableActions.map((action, idx) => (
                                        <option key={idx} value={action.action_and_publisher_name}>
                                            {action.action_and_publisher_name}
                                        </option>
                                    ))}
                                </select>
                            </div>
                        ))}
                        <div className="form-group">
                            <label>Logic Expression:</label>
                            <input
                                type="text"
                                name="logicExpression"
                                value={combinedActionData.logicExpression}
                                onChange={handleCombinedChange}
                                required
                            />
                            <div className="info-with-button">
                                <small>Use the action names selected above in your logic expression (e.g., "Action1 or Action2")</small>
                            </div>
                        </div>
                        <div className="form-group">
                            <label>Action and Publisher Name:</label>
                            <input
                                type="text"
                                name="actionAndPublisherName"
                                value={combinedActionData.actionAndPublisherName}
                                onChange={handleCombinedChange}
                                required
                            />
                        </div>
                        <div className="form-group">
                            <label>Trigger Text:</label>
                            <input
                                type="text"
                                name="triggerText"
                                value={combinedActionData.triggerText}
                                onChange={handleCombinedChange}
                                required
                            />
                        </div>
                        <div className="form-group">
                            <label>Publication Method (0-6):</label>
                            <input
                                type="number"
                                name="publicationMethod"
                                value={combinedActionData.publicationMethod}
                                onChange={handleCombinedChange}
                                min="0"
                                max="6"
                                required
                            />
                        </div>
                        <div className="modal-actions">
                            <button type="submit" className="add-button">Create</button>
                            <button type="button" onClick={handleCancel} className="close-button">Cancel</button>
                            <button type="button" onClick={() => setActionType(null)} className="back-button">Back</button>
                        </div>
                    </form>
                </div>
            </div>
        );
    }

    // Existing action creation form
    return (
        <div className={`create-action-modal ${isClosing ? 'closing' : ''}`}>
            <div className={`modal-content ${isClosing ? 'closing' : ''}`}>
                <h2>Create Action</h2>
                {message && <div className="message">{message}</div>}
                <form onSubmit={handleSubmit} className="new-action-form">
                    <div className="form-group">
                        <label>Action and Publisher Name:</label>
                        <input
                            type="text"
                            name="actionAndPublisherName"
                            value={actionData.actionAndPublisherName}
                            onChange={handleChange}
                            required
                        />
                    </div>
                    <div className="form-group">
                        <label>Listen Topic:</label>
                        <select name="listenTopic" value={actionData.listenTopic} onChange={handleChange} required>
                            <option value="">Select a Topic</option>
                            {topics.map((topic) => (
                                <option key={topic.name} value={topic.name}>
                                    {topic.name}
                                </option>
                            ))}
                        </select>
                    </div>
                    <div className="form-group">
                        <label>Listen Message Type:</label>
                        <input
                            type="text"
                            name="listenMessageType"
                            value={actionData.listenMessageType}
                            onChange={handleChange}
                            placeholder="e.g., std_msgs/String"
                            required
                        />
                    </div>
                    <div className="form-group">
                        <label>Field to Read:</label>
                        {isLoadingFields ? (
                            <p>Loading fields...</p>
                        ) : (
                            <select name="value" value={actionData.value} onChange={handleChange} required>
                                <option value="">Select a Field</option>
                                {fields.map((field, index) => (
                                    <option key={index} value={field}>
                                        {field}
                                    </option>
                                ))}
                            </select>
                        )}
                    </div>
                    <div className="form-group">
                        <label>Trigger Value:</label>
                        <input type="text" name="triggerVal" value={actionData.triggerVal} onChange={handleChange} required />
                    </div>
                    <div className="form-group">
                        <label>Trigger Type:</label>
                        <select name="triggerType" value={actionData.triggerType} onChange={handleChange}>
                            <option value="LessThan">Less Than</option>
                            <option value="GreaterThan">Greater Than</option>
                            <option value="EqualTo">Equal To</option>
                        </select>
                    </div>
                    <div className="form-group">
                        <label>Publish Message Type:</label>
                        <input type="text" name="pubMessageType" value={actionData.pubMessageType} onChange={handleChange} required />
                    </div>
                    <div className="form-group">
                        <label>Publish Message Value (Trigger Text):</label>
                        <input type="text" name="triggerText" value={actionData.triggerText} onChange={handleChange} required />
                    </div>
                    <div className="form-group">
                        <label>Data Validity (ms):</label>
                        <input type="number" name="dataValidityMs" value={actionData.dataValidityMs} onChange={handleChange} required />
                    </div>
                    <div className="form-group">
                        <label>Publication Method (0-6):</label>
                        <input
                            type="number"
                            name="publicationMethod"
                            value={actionData.publicationMethod}
                            onChange={handleChange}
                            min="0"
                            max="6"
                            required
                        />
                    </div>
                    <div className="modal-actions">
                        <button type="submit" className="add-button">Create</button>
                        <button type="button" onClick={handleCancel} className="close-button">Cancel</button>
                        <button type="button" onClick={() => setActionType(null)} className="back-button">Back</button>
                    </div>
                </form>
            </div>
        </div>
    );
};

export default CreateActionModal;
