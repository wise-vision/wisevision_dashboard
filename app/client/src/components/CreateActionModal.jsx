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
import { CiFilter } from "react-icons/ci";
import '../styles/CreateActionModal.css';
import FilterTopicsModal from './FilterTopicsModal';

const CreateActionModal = ({ isOpen, onClose, onActionCreated }) => {
    const [isClosing, setIsClosing] = useState(false);
    const [actionType, setActionType] = useState(null); // 'action' or 'combined'
    const [selectedTopic, setSelectedTopic] = useState({
        name: '',
        type: '',
    });
    // State variables to track manual input mode
    const [messageStructure, setMessageStructure] = useState({});
    const [nestedPaths, setNestedPaths] = useState([]);
    const [selectedPath, setSelectedPath] = useState('');
    const [selectedPathManualInput, setSelectedPathManualInput] = useState(false);

    // Filter stuff
    const [isFilterModalOpen, setIsFilterModalOpen] = useState(false);
    const [selectedFilters, setSelectedFilters] = useState({
        name: "",
        messageTypes: [],
        namespaces: []
    });

    const handleFiltersApply = (filters) => {
        setSelectedFilters(filters);
    };

    const [publicationOptions, setPublicationOptions] = useState({
        email: false,
        push: false,
        webPush: false
    });


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
        publicationMethod: 0
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
    const [message, setMessage] = useState('');

    useEffect(() => {
        if (isOpen) {
            const fetchTopics = async () => {
                try {
                    const queryParams = new URLSearchParams();
                    if (selectedFilters.name) {
                        queryParams.append("name_contains", selectedFilters.name);
                    }
                    if (selectedFilters.messageTypes.length > 0) {
                        selectedFilters.messageTypes.forEach(type => queryParams.append("message_types", type));
                    }
                    if (selectedFilters.namespaces.length > 0) {
                        selectedFilters.namespaces.forEach(ns => queryParams.append("message_namespaces", ns));
                    }
                    const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/topics?${queryParams.toString()}`);
                    const data = await response.json();
                    setTopics(data);
                    if (data[0]) {
                        setSelectedTopic({
                            name: data[0].name,
                            type: data[0].type,
                        });
                    }
                } catch (error) {
                    console.error('Error fetching topics:', error);
                }
            };

            fetchTopics();
        }
    }, [isOpen, selectedFilters]);

    useEffect(() => {
        if (!actionData.listenTopic) {
            setMessageStructure({});
            setNestedPaths([]); // Reset nested paths, when topic is not selected
            setSelectedPath("");
            return;
        }

        const fetchMessageStructure = async () => {
            try {
                const encodedType = encodeURIComponent(selectedTopic.type);
                const response = await fetch(
                    `${process.env.REACT_APP_API_BASE_URL}/api/message_structure/${encodedType}`
                );
                const data = await response.json();
                setMessageStructure(data);
            } catch (error) {
                console.error('Error fetching message structure:', error);
                setMessageStructure({});
            }
        };

        if (selectedTopic.type) {
            fetchMessageStructure();
        }
    }, [actionData.listenTopic, selectedTopic]);

    const extractFields = useCallback((structure, parent = '') => {
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
    }, []);

    useEffect(() => {
        if (messageStructure && Object.keys(messageStructure).length > 0) {
            const availableFields = extractFields(messageStructure);
            setNestedPaths(availableFields);
            if (availableFields.length > 0) {
                setSelectedPath(availableFields[0]);
            }
        } else {
            setNestedPaths([]);
            setSelectedPath("");
        }
    }, [messageStructure, extractFields]);

    useEffect(() => {
        if (!isOpen) {
            setPublicationOptions({
                email: false,
                push: false,
                webPush: false
            });
        }
    }, [isOpen]);

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
            setMessage('');

            setSelectedFilters({
                name: "",
                messageTypes: [],
                namespaces: []
            });
        }
    }, [isOpen]);

    const handlePublicationChange = (method) => {
        setPublicationOptions((prev) => {
            const newOptions = { ...prev, [method]: !prev[method] };

            let newValue = 0;
            if (newOptions.email && !newOptions.push && !newOptions.webPush) newValue = 0;
            if (!newOptions.email && newOptions.push && !newOptions.webPush) newValue = 1;
            if (!newOptions.email && !newOptions.push && newOptions.webPush) newValue = 2;
            if (newOptions.email && newOptions.push && !newOptions.webPush) newValue = 3;
            if (newOptions.email && !newOptions.push && newOptions.webPush) newValue = 4;
            if (!newOptions.email && newOptions.push && newOptions.webPush) newValue = 5;
            if (newOptions.email && newOptions.push && newOptions.webPush) newValue = 6;

            setActionData((prev) => ({
                ...prev,
                publicationMethod: newValue
            }));

            return newOptions;
        });
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

        // Validation for publicationMethod
        if (isNaN(publicationMethodAsNumber) || publicationMethodAsNumber < 0 || publicationMethodAsNumber > 6) {
            setMessage('Publication Method must be a number between 0 and 6.');
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
                    publication_method: publicationMethodAsNumber
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

        // Validation for publicationMethod
        if (isNaN(publicationMethodAsNumber) || publicationMethodAsNumber < 0 || publicationMethodAsNumber > 6) {
            setMessage('Publication Method must be a number between 0 and 6.');
            return;
        }

        const data = {
            listen_topics: selectedActions,
            logic_expression: combinedActionData.logicExpression,
            action_and_publisher_name: combinedActionData.actionAndPublisherName,
            trigger_text: combinedActionData.triggerText,
            publication_method: publicationMethodAsNumber
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
                            <label>Publication Method:</label>
                            <div className="checkbox-group">
                                <label>
                                    <input
                                        type="checkbox"
                                        checked={publicationOptions.email}
                                        onChange={() => handlePublicationChange("email")}
                                    />
                                    Email
                                </label>
                                <label>
                                    <input
                                        type="checkbox"
                                        checked={publicationOptions.push}
                                        onChange={() => handlePublicationChange("push")}
                                    />
                                    App Push Notification
                                </label>
                                <label>
                                    <input
                                        type="checkbox"
                                        checked={publicationOptions.webPush}
                                        onChange={() => handlePublicationChange("webPush")}
                                    />
                                    Web Push Notification
                                </label>
                            </div>
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
                        <div className="select-with-icon">
                            <select
                                name="listenTopic"
                                value={actionData.listenTopic}
                                onChange={(e) => {
                                    const topic = topics.find((t) => t.name === e.target.value);
                                    setActionData((prev) => ({ ...prev, listenTopic: topic.name }));
                                    setSelectedTopic({
                                        name: topic.name,
                                        type: topic.type,
                                    });
                                }}
                                required
                            >
                                <option value="" disabled>
                                    Select a Topic
                                </option>
                                {topics.map((topic) => (
                                    <option key={topic.name} value={topic.name}>
                                        {topic.name}
                                    </option>
                                ))}
                            </select>

                            {/* Filter button */}
                            <button type="button" className="icon" onClick={() => setIsFilterModalOpen(true)}>
                                <CiFilter className="filter-icon" />
                            </button>
                        </div>
                    </div>
                    <div className="form-group">
                        <label htmlFor="nestedMessage">Field to Read</label>
                        <div className="select-with-icon">
                            {selectedPathManualInput ? (
                                <input
                                    id="nestedMessageInput"
                                    type="text"
                                    value={selectedPath}
                                    onChange={(e) => setSelectedPath(e.target.value)}
                                />
                            ) : (
                                <select
                                    id="nestedMessage"
                                    value={selectedPath}
                                    onChange={(e) => setSelectedPath(e.target.value)}
                                    required
                                >
                                    <option value="" disabled>
                                        Select a Field
                                    </option>
                                    {nestedPaths.map((path) => (
                                        <option key={path} value={path}>
                                            {path}
                                        </option>
                                    ))}
                                </select>
                            )}
                            <button
                                type="button"
                                className="icon"
                                onClick={() => setSelectedPathManualInput(!selectedPathManualInput)}
                            >
                                &#9998;
                            </button>
                        </div>
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
                        <label>Publication Method:</label>
                        <div className="checkbox-group">
                            <label>
                                <input
                                    type="checkbox"
                                    checked={publicationOptions.email}
                                    onChange={() => handlePublicationChange("email")}
                                />
                                Email
                            </label>
                            <label>
                                <input
                                    type="checkbox"
                                    checked={publicationOptions.push}
                                    onChange={() => handlePublicationChange("push")}
                                />
                                App Push Notification
                            </label>
                            <label>
                                <input
                                    type="checkbox"
                                    checked={publicationOptions.webPush}
                                    onChange={() => handlePublicationChange("webPush")}
                                />
                                Web Push Notification
                            </label>
                        </div>
                    </div>
                    <div className="modal-actions">
                        <button type="submit" className="add-button">Create</button>
                        <button type="button" onClick={handleCancel} className="close-button">Cancel</button>
                        <button type="button" onClick={() => setActionType(null)} className="back-button">Back</button>
                    </div>
                </form>
            </div>
            <FilterTopicsModal
                isOpen={isFilterModalOpen}
                onClose={() => setIsFilterModalOpen(false)}
                onApplyFilters={handleFiltersApply}
            />
        </div>
    );
};

export default CreateActionModal;
