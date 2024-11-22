import React, { useState, useEffect } from 'react';
import '../styles/ActionModal.css';


import loadingGif from '../assets/images/loadingGif.gif';

const ActionModal = ({ onClose }) => {
    const [actions, setActions] = useState([]);
    const [selectedAction, setSelectedAction] = useState(null);
    const [isEditing, setIsEditing] = useState(false);
    const [editForm, setEditForm] = useState({});
    const [loading, setLoading] = useState(true);

    const fetchActions = async () => {
        setLoading(true); // Start loading
        try {
            const [responseNormal, responseCombined] = await Promise.all([
                fetch('http://localhost:5000/api/available_topics'),
                fetch('http://localhost:5000/api/available_topics_combined')
            ]);

            const dataNormal = await responseNormal.json();
            const dataCombined = await responseCombined.json();

            let actionsData = [];

            // Process normal actions
            if (dataNormal.available_topics_with_parameters_and_time) {
                const normalActions = dataNormal.available_topics_with_parameters_and_time.map((action, index) => {
                    return {
                        id: `normal-${index}`,
                        active: true,
                        selected: false,
                        actionType: 'normal', // Set action type
                        ...action,
                    };
                });
                actionsData = actionsData.concat(normalActions);
            } else {
                alert('Failed to fetch normal actions.');
            }

            // Process combined actions
            if (dataCombined.available_combined_topics_with_parameters_and_time) {
                const combinedActions = dataCombined.available_combined_topics_with_parameters_and_time.map((action, index) => {
                    return {
                        id: `combined-${index}`,
                        active: true,
                        selected: false,
                        actionType: 'combined',
                        ...action,
                    };
                });
                actionsData = actionsData.concat(combinedActions);
            } else {
                alert('Failed to fetch combined actions.');
            }

            setActions(actionsData);
            setLoading(false); // Data has loaded
        } catch (error) {
            console.error('Error fetching actions:', error);
            alert('An error occurred while fetching actions.');
            setLoading(false);
        }
    };

    useEffect(() => {
        fetchActions();
    }, []);

    const toggleActive = (id, currentStatus) => {
        setActions(actions.map(action =>
            action.id === id ? { ...action, active: !currentStatus } : action
        ));
    };

    const toggleSelect = (id) => {
        setActions(
            actions.map(action =>
                action.id === id ? { ...action, selected: !action.selected } : action
            )
        );
    };

    const handleTriggerSelected = async () => {
        const selectedActions = actions.filter(action => action.selected);
        try {
            const response = await fetch('http://localhost:5000/api/trigger_actions', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ action_names: selectedActions.map(action => action.action_and_publisher_name) })
            });
            const data = await response.json();
            if (data.success) {
                alert('Selected actions have been triggered.');
            } else {
                alert('Failed to trigger selected actions.');
            }
        } catch (error) {
            console.error('Error triggering actions:', error);
            alert('An error occurred while triggering actions.');
        }
    };

    const handleActionNameClick = (action) => {
        setSelectedAction(action);
        if (action.actionType === 'normal') {
            setEditForm({
                new_action_and_publisher_name: action.action_and_publisher_name,
                listen_topic: action.listen_topic,
                listen_message_type: action.listen_message_type,
                value: action.value,
                trigger_val: action.trigger_val,
                trigger_type: action.trigger_type,
                pub_message_type: action.pub_message_type,
                trigger_text: action.trigger_text,
                data_validity_ms: action.data_validity_ms,
                publication_method: action.publication_method,
            });
        } else if (action.actionType === 'combined') {
            setEditForm({
                new_action_and_publisher_name: action.action_and_publisher_name,
                listen_topics: action.listen_topics,
                logic_expression: action.logic_expression,
                trigger_text: action.trigger_text,
                publication_method: action.publication_method,
            });
        }
        setIsEditing(false);
    };

    const closeActionDetailsModal = () => {
        setSelectedAction(null);
    };

    const handleEditToggle = () => {
        setIsEditing(!isEditing);
    };

    const handleEditChange = (e) => {
        const { name, value } = e.target;
        if (name === 'listen_topics') {
            setEditForm({
                ...editForm,
                [name]: value.split(',').map(topic => topic.trim()),
            });
        } else {
            setEditForm({
                ...editForm,
                [name]: value,
            });
        }
    };

    const handleEditSubmit = async (e) => {
        e.preventDefault();
        try {
            let requestBody = {};
            if (selectedAction.actionType === 'normal') {
                requestBody = {
                    action_and_publisher_name_to_change: selectedAction.action_and_publisher_name,
                    ...editForm,
                };
            } else if (selectedAction.actionType === 'combined') {
                requestBody = {
                    action_and_publisher_name_to_change: selectedAction.action_and_publisher_name,
                    new_action_and_publisher_name: editForm.new_action_and_publisher_name,
                    listen_topics: editForm.listen_topics,
                    logic_expression: editForm.logic_expression,
                    trigger_text: editForm.trigger_text,
                    publication_method: editForm.publication_method,
                };
            }
            const response = await fetch('http://localhost:5000/api/change_automatic_action', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(requestBody)
            });
            const data = await response.json();
            if (data.success) {
                alert('Action updated successfully.');
                fetchActions();
                setIsEditing(false);
                setSelectedAction(null);
            } else {
                alert('Failed to update the action.');
            }
        } catch (error) {
            console.error('Error updating action:', error);
            alert('An error occurred while updating the action.');
        }
    };

    return (
        <div className="modal">
            <div className="modal-content">
                <h2>Actions</h2>
                {loading ? (
                    <div className="loading-container">
                        <img src={loadingGif} alt="Loading..." className="loading-gif" />
                    </div>
                ) : (
                    <div className="actions-table-container">
                        <table className="actions-table">
                            <thead>
                            <tr>
                                <th>Select</th>
                                <th>Name</th>
                                <th>Active</th>
                                <th>Action Type</th> {/* Changed column header */}
                            </tr>
                            </thead>
                            <tbody>
                            {actions.map((action) => (
                                <tr key={action.id}>
                                    <td>
                                        <input
                                            type="checkbox"
                                            checked={action.selected || false}
                                            onChange={() => toggleSelect(action.id)}
                                        />
                                    </td>
                                    <td>
                                        <button
                                            className="action-name-button"
                                            onClick={() => handleActionNameClick(action)}
                                        >
                                            {action.action_and_publisher_name}
                                        </button>
                                    </td>
                                    <td>
                                        <label className="switch">
                                            <input
                                                type="checkbox"
                                                checked={action.active}
                                                onChange={() => toggleActive(action.id, action.active)}
                                            />
                                            <span className="slider round"></span>
                                        </label>
                                    </td>
                                    <td>{action.actionType}</td> {/* Display action type */}
                                </tr>
                            ))}
                            </tbody>
                        </table>
                    </div>
                )}

                <div className="modal-actions">
                    <p className="trigger-instructions">Manually trigger selected actions</p>
                    <button onClick={handleTriggerSelected} className="trigger-button">Trigger Now</button>
                    <button onClick={onClose} className="close-button">Close</button>
                </div>
            </div>

            {selectedAction && (
                <div className="modal">
                    <div className="modal-content action-details-modal">
                        <h3>Action Details</h3>
                        {!isEditing ? (
                            <div>
                                {selectedAction.actionType === 'normal' ? (
                                    <ul className="action-details-list">
                                        <li><strong>Name:</strong> {selectedAction.action_and_publisher_name}</li>
                                        <li><strong>Listen Topic:</strong> {selectedAction.listen_topic}</li>
                                        <li><strong>Listen Message Type:</strong> {selectedAction.listen_message_type}</li>
                                        <li><strong>Value:</strong> {selectedAction.value}</li>
                                        <li><strong>Trigger Value:</strong> {selectedAction.trigger_val}</li>
                                        <li><strong>Trigger Type:</strong> {selectedAction.trigger_type}</li>
                                        <li><strong>Publish Message Type:</strong> {selectedAction.pub_message_type}</li>
                                        <li><strong>Trigger Text:</strong> {selectedAction.trigger_text}</li>
                                        <li><strong>Data Validity (ms):</strong> {selectedAction.data_validity_ms}</li>
                                        <li><strong>Publication Method:</strong> {selectedAction.publication_method}</li>
                                    </ul>
                                ) : (
                                    <ul className="action-details-list">
                                        <li><strong>Name:</strong> {selectedAction.action_and_publisher_name}</li>
                                        <li><strong>Listen Topics:</strong> {selectedAction.listen_topics.join(', ')}</li>
                                        <li><strong>Logic Expression:</strong> {selectedAction.logic_expression}</li>
                                        <li><strong>Trigger Text:</strong> {selectedAction.trigger_text}</li>
                                        <li><strong>Publication Method:</strong> {selectedAction.publication_method}</li>
                                    </ul>
                                )}
                                <div className="action-details-footer">
                                    <button onClick={closeActionDetailsModal} className="close-details-button">Close</button>
                                    <button onClick={handleEditToggle} className="edit-button">Edit</button>
                                </div>
                            </div>
                        ) : (
                            <form onSubmit={handleEditSubmit} className="edit-form">
                                <label>
                                    New Name:
                                    <input
                                        type="text"
                                        name="new_action_and_publisher_name"
                                        value={editForm.new_action_and_publisher_name}
                                        onChange={handleEditChange}
                                    />
                                </label>
                                {selectedAction.actionType === 'normal' ? (
                                    <>
                                        <label>
                                            Listen Topic:
                                            <input
                                                type="text"
                                                name="listen_topic"
                                                value={editForm.listen_topic}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                        <label>
                                            Listen Message Type:
                                            <input
                                                type="text"
                                                name="listen_message_type"
                                                value={editForm.listen_message_type}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                        <label>
                                            Value:
                                            <input
                                                type="text"
                                                name="value"
                                                value={editForm.value}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                        <label>
                                            Trigger Value:
                                            <input
                                                type="text"
                                                name="trigger_val"
                                                value={editForm.trigger_val}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                        <label>
                                            Trigger Type:
                                            <input
                                                type="text"
                                                name="trigger_type"
                                                value={editForm.trigger_type}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                        <label>
                                            Publish Message Type:
                                            <input
                                                type="text"
                                                name="pub_message_type"
                                                value={editForm.pub_message_type}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                        <label>
                                            Trigger Text:
                                            <input
                                                type="text"
                                                name="trigger_text"
                                                value={editForm.trigger_text}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                        <label>
                                            Data Validity (ms):
                                            <input
                                                type="number"
                                                name="data_validity_ms"
                                                value={editForm.data_validity_ms}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                        <label>
                                            Publication Method:
                                            <input
                                                type="number"
                                                name="publication_method"
                                                value={editForm.publication_method}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                    </>
                                ) : (
                                    <>
                                        <label>
                                            Listen Topics (comma-separated):
                                            <input
                                                type="text"
                                                name="listen_topics"
                                                value={editForm.listen_topics.join(', ')}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                        <label>
                                            Logic Expression:
                                            <input
                                                type="text"
                                                name="logic_expression"
                                                value={editForm.logic_expression}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                        <label>
                                            Trigger Text:
                                            <input
                                                type="text"
                                                name="trigger_text"
                                                value={editForm.trigger_text}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                        <label>
                                            Publication Method:
                                            <input
                                                type="number"
                                                name="publication_method"
                                                value={editForm.publication_method}
                                                onChange={handleEditChange}
                                            />
                                        </label>
                                    </>
                                )}
                                <div className="form-buttons">
                                    <button type="submit" className="save-button">Save</button>
                                    <button onClick={handleEditToggle} className="cancel-button">Cancel</button>
                                </div>
                            </form>
                        )}
                    </div>
                </div>
            )}
        </div>
    );
};

export default ActionModal;
