import React, { useState, useEffect } from 'react';
import '../styles/DeleteActionModal.css';

const DeleteActionModal = ({ isOpen, onClose }) => {
    const [actions, setActions] = useState([]);
    const [selectedAction, setSelectedAction] = useState(null);
    const [message, setMessage] = useState('');

    useEffect(() => {
        if (isOpen) {
            fetchActions();
        }
    }, [isOpen]);

    const fetchActions = async () => {
        try {
            const [responseNormal, responseCombined] = await Promise.all([
                fetch(`${process.env.REACT_APP_API_BASE_URL}/api/available_topics`),
                fetch(`${process.env.REACT_APP_API_BASE_URL}/api/available_topics_combined`)
            ]);

            const dataNormal = await responseNormal.json();
            const dataCombined = await responseCombined.json();

            let actionsData = [];

            // Process normal actions
            if (dataNormal.available_topics_with_parameters_and_time) {
                const normalActions = dataNormal.available_topics_with_parameters_and_time.map((action) => {
                    return {
                        action_and_publisher_name: action.action_and_publisher_name,
                        actionType: 'normal',
                        displayName: action.action_and_publisher_name,
                    };
                });
                actionsData = actionsData.concat(normalActions);
            } else {
                alert('Failed to fetch normal actions.');
            }

            // Process combined actions
            if (dataCombined.available_combined_topics_with_parameters_and_time) {
                const combinedActions = dataCombined.available_combined_topics_with_parameters_and_time.map((action) => {
                    return {
                        action_and_publisher_name: action.action_and_publisher_name,
                        actionType: 'combined',
                        displayName: action.action_and_publisher_name,
                    };
                });
                actionsData = actionsData.concat(combinedActions);
            } else {
                alert('Failed to fetch combined actions.');
            }

            setActions(actionsData);
        } catch (error) {
            console.error('Error fetching actions:', error);
            alert('An error occurred while fetching actions.');
        }
    };

    const handleDelete = async () => {
        if (!selectedAction) {
            alert('Please select an action to delete.');
            return;
        }

        try {
            let response;
            if (selectedAction.actionType === 'normal') {
                response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/delete_automatic_action`, {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        listen_topic_to_delete: selectedAction.action_and_publisher_name
                    })
                });
            } else if (selectedAction.actionType === 'combined') {
                response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/delete_combined_automatic_action`, {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        name_of_combined_topics_publisher: selectedAction.action_and_publisher_name
                    })
                });
            }

            const data = await response.json();
            if (data.success) {
                setMessage(`Action "${selectedAction.action_and_publisher_name}" deleted successfully.`);
                setActions(actions.filter(action => action.action_and_publisher_name !== selectedAction.action_and_publisher_name));
                setSelectedAction(null);
            } else {
                setMessage(`Failed to delete action "${selectedAction.action_and_publisher_name}".`);
            }
        } catch (error) {
            console.error('Error deleting action:', error);
            alert('An error occurred while deleting the action.');
        }
    };

    const handleCancel = () => {
        setSelectedAction(null);
        setMessage('');
        onClose();
    };

    return (
        isOpen && (
            <div className="modal">
                <div className="modal-content delete-action-modal">
                    <h2>Delete Action</h2>
                    <div className="action-selection">
                        <label>Select an action to delete:</label>
                        <select
                            value={selectedAction ? selectedAction.action_and_publisher_name : ''}
                            onChange={(e) => {
                                const actionName = e.target.value;
                                const action = actions.find(a => a.action_and_publisher_name === actionName);
                                setSelectedAction(action);
                                setMessage('');
                            }}
                        >
                            <option value="" disabled>Select an action</option>
                            {actions.map((action) => (
                                <option key={action.action_and_publisher_name} value={action.action_and_publisher_name}>
                                    {action.displayName} ({action.actionType})
                                </option>
                            ))}
                        </select>
                    </div>
                    <div className="modal-actions">
                        <button onClick={handleDelete} className="delete-button">Delete</button>
                        <button onClick={handleCancel} className="cancel-button">Cancel</button>
                    </div>
                    {message && <p className="message">{message}</p>}
                </div>
            </div>
        )
    );
};

export default DeleteActionModal;
