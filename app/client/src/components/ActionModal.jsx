import React, { useState, useEffect, useRef } from 'react';
import '../styles/ActionModal.css';

// Make sure that this GIF is animated
import loadingGif from '../assets/images/loadingGif.gif';

const ActionModal = ({ onClose }) => {
    const [actions, setActions] = useState([]);
    const [selectedAction, setSelectedAction] = useState(null);
    const [isEditing, setIsEditing] = useState(false);
    const [editForm, setEditForm] = useState({});
    const [loading, setLoading] = useState(true);

    const intervalRef = useRef(null);

    const fetchActions = async () => {
        try {
            const [responseNormal, responseCombined] = await Promise.allSettled([
                fetch('http://localhost:5000/api/available_topics'),
                fetch('http://localhost:5000/api/available_topics_combined')
            ]);

            let actionsData = [];

            // Processing normal actions
            if (responseNormal.status === 'fulfilled') {
                const dataNormal = await responseNormal.value.json();
                if (dataNormal.available_topics_with_parameters_and_time) {
                    const normalActions = dataNormal.available_topics_with_parameters_and_time.map((action, index) => ({
                        id: `normal-${index}`,
                        active: true,
                        selected: false,
                        actionType: 'normal',
                        ...action,
                    }));
                    actionsData = actionsData.concat(normalActions);
                }
            } else {
                console.error('Failed to fetch normal actions:', responseNormal.reason);
            }

            // Processing combined actions
            if (responseCombined.status === 'fulfilled') {
                const dataCombined = await responseCombined.value.json();
                if (dataCombined.available_combined_topics_with_parameters_and_time) {
                    const combinedActions = dataCombined.available_combined_topics_with_parameters_and_time.map((action, index) => ({
                        id: `combined-${index}`,
                        active: true,
                        selected: false,
                        actionType: 'combined',
                        ...action,
                    }));
                    actionsData = actionsData.concat(combinedActions);
                }
            } else {
                console.error('Failed to fetch combined actions:', responseCombined.reason);
            }

            if (actionsData.length > 0) {
                setActions(actionsData);
                setLoading(false);
                // If actions are loaded, stop the interval
                if (intervalRef.current) {
                    clearInterval(intervalRef.current);
                    intervalRef.current = null;
                }
            }
            // If no actions, continue loading and let the interval retry
        } catch (error) {
            console.error('Error fetching actions:', error);
            // Continue loading to keep the loading GIF visible
        }
    };

    useEffect(() => {
        // Start fetching actions when modal is mounted
        fetchActions();

        // Set an interval to retry fetching actions every 30 seconds
        intervalRef.current = setInterval(() => {
            fetchActions();
        }, 30000);

        // Cleanup: clear the interval when unmounting
        return () => {
            if (intervalRef.current) {
                clearInterval(intervalRef.current);
            }
        };
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
        if (selectedActions.length === 0) {
            return;
        }
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
                alert('Selected actions have been triggered successfully.');
            } else {
                console.error('Failed to trigger selected actions.');
            }
        } catch (error) {
            console.error('Error triggering actions:', error);
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
        setIsEditing(false);
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

        const publicationMethodAsNumber = parseInt(editForm.publication_method, 10);
        if (isNaN(publicationMethodAsNumber) || publicationMethodAsNumber < 0 || publicationMethodAsNumber > 6) {
            alert('Publication Method must be a number between 0 and 6.');
            return;
        }

        try {
            let requestBody = {};
            if (selectedAction.actionType === 'normal') {
                requestBody = {
                    action_and_publisher_name_to_change: selectedAction.action_and_publisher_name,
                    new_action_and_publisher_name: editForm.new_action_and_publisher_name,
                    listen_topic: editForm.listen_topic,
                    listen_message_type: editForm.listen_message_type,
                    value: editForm.value,
                    trigger_val: editForm.trigger_val,
                    trigger_type: editForm.trigger_type,
                    pub_message_type: editForm.pub_message_type,
                    trigger_text: editForm.trigger_text,
                    data_validity_ms: parseInt(editForm.data_validity_ms, 10),
                    publication_method: publicationMethodAsNumber,
                };
            } else if (selectedAction.actionType === 'combined') {
                requestBody = {
                    action_and_publisher_name_to_change: selectedAction.action_and_publisher_name,
                    new_action_and_publisher_name: editForm.new_action_and_publisher_name,
                    listen_topics: editForm.listen_topics,
                    logic_expression: editForm.logic_expression,
                    trigger_text: editForm.trigger_text,
                    publication_method: publicationMethodAsNumber,
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
                alert('Action was successfully updated.');
                fetchActions();
                setIsEditing(false);
                setSelectedAction(null);
            } else {
                console.error('Failed to update the action.');
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
                ) : actions.length > 0 ? (
                    <div className="actions-table-container">
                        <table className="actions-table">
                            <thead>
                            <tr>
                                <th>Select</th>
                                <th>Name</th>
                                <th>Active</th>
                                <th>Action Type</th>
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
                                    <td>{action.actionType}</td>
                                </tr>
                            ))}
                            </tbody>
                        </table>
                    </div>
                ) : (
                    // If loading is false but actions are empty, continue showing loading GIF
                    <div className="loading-container">
                        <img src={loadingGif} alt="Loading..." className="loading-gif" />
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
                                        required
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
                                                required
                                            />
                                        </label>
                                        <label>
                                            Listen Message Type:
                                            <input
                                                type="text"
                                                name="listen_message_type"
                                                value={editForm.listen_message_type}
                                                onChange={handleEditChange}
                                                required
                                            />
                                        </label>
                                        <label>
                                            Value:
                                            <input
                                                type="text"
                                                name="value"
                                                value={editForm.value}
                                                onChange={handleEditChange}
                                                required
                                            />
                                        </label>
                                        <label>
                                            Trigger Value:
                                            <input
                                                type="text"
                                                name="trigger_val"
                                                value={editForm.trigger_val}
                                                onChange={handleEditChange}
                                                required
                                            />
                                        </label>
                                        <label>
                                            Trigger Type:
                                            <input
                                                type="text"
                                                name="trigger_type"
                                                value={editForm.trigger_type}
                                                onChange={handleEditChange}
                                                required
                                            />
                                        </label>
                                        <label>
                                            Publish Message Type:
                                            <input
                                                type="text"
                                                name="pub_message_type"
                                                value={editForm.pub_message_type}
                                                onChange={handleEditChange}
                                                required
                                            />
                                        </label>
                                        <label>
                                            Trigger Text:
                                            <input
                                                type="text"
                                                name="trigger_text"
                                                value={editForm.trigger_text}
                                                onChange={handleEditChange}
                                                required
                                            />
                                        </label>
                                        <label>
                                            Data Validity (ms):
                                            <input
                                                type="number"
                                                name="data_validity_ms"
                                                value={editForm.data_validity_ms}
                                                onChange={handleEditChange}
                                                required
                                                min="0"
                                            />
                                        </label>
                                        <label>
                                            Publication Method (0-6):
                                            <input
                                                type="number"
                                                name="publication_method"
                                                value={editForm.publication_method}
                                                onChange={handleEditChange}
                                                required
                                                min="0"
                                                max="6"
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
                                                required
                                            />
                                        </label>
                                        <label>
                                            Logic Expression:
                                            <input
                                                type="text"
                                                name="logic_expression"
                                                value={editForm.logic_expression}
                                                onChange={handleEditChange}
                                                required
                                            />
                                        </label>
                                        <label>
                                            Trigger Text:
                                            <input
                                                type="text"
                                                name="trigger_text"
                                                value={editForm.trigger_text}
                                                onChange={handleEditChange}
                                                required
                                            />
                                        </label>
                                        <label>
                                            Publication Method (0-6):
                                            <input
                                                type="number"
                                                name="publication_method"
                                                value={editForm.publication_method}
                                                onChange={handleEditChange}
                                                required
                                                min="0"
                                                max="6"
                                            />
                                        </label>
                                    </>
                                )}
                                <div className="form-buttons">
                                    <button type="submit" className="save-button">Save</button>
                                    <button type="button" onClick={handleEditToggle} className="cancel-button">Cancel</button>
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
