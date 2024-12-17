import React, { useState, useEffect } from 'react';
import '../styles/ChartModal.css';
import penIcon from '../assets/images/pen.png';

const unitsSI = ['°C', 'kW', 'V', 'A', 'W', 'Hz', 'Pa'];

const ChartModal = ({ onClose, onAddChart }) => {
    const [chartType, setChartType] = useState('');
    const [chartLabel, setChartLabel] = useState('');
    const [errorMessage, setErrorMessage] = useState('');
    const [topics, setTopics] = useState([]);
    const [selectedTopic, setSelectedTopic] = useState({
        name: '',
        type: '',
    });
    const [selectedUnit, setSelectedUnit] = useState('');
    const [nestedPaths, setNestedPaths] = useState([]);
    const [selectedPath, setSelectedPath] = useState('');
    const [step, setStep] = useState(1);

    const [selectedTopicManualInput, setSelectedTopicManualInput] = useState(false);
    const [selectedUnitManualInput, setSelectedUnitManualInput] = useState(false);
    const [selectedPathManualInput, setSelectedPathManualInput] = useState(false);
    const [isLoadingFields, setIsLoadingFields] = useState(false);

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

    useEffect(() => {
        const fetchTopics = async () => {
            try {
                const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/topics`);
                const result = await response.json();
                setTopics(result);
                if (result[0] && chartType !== 'gps') {
                    setSelectedTopic({
                        name: result[0].name,
                        type: result[0].type,
                    });
                }
            } catch (error) {
                console.error('Error fetching topics:', error);
            }
        };
        fetchTopics();
    }, [chartType]);

    useEffect(() => {
        if (step === 2 && selectedTopic.type && chartType !== 'gps') {
            const fetchMessageStructure = async () => {
                setIsLoadingFields(true);
                try {
                    const encodedType = encodeURIComponent(selectedTopic.type);
                    const response = await fetch(
                        `${process.env.REACT_APP_API_BASE_URL}/api/message_structure/${encodedType}`
                    );
                    const result = await response.json();
                    const paths = extractFields(result);
                    setNestedPaths(paths);
                    if (paths[0]) {
                        setSelectedPath(paths[0]);
                    }
                } catch (error) {
                    console.error('Error fetching message structure:', error);
                    setNestedPaths([]);
                } finally {
                    setIsLoadingFields(false);
                }
            };
            fetchMessageStructure();
        } else if (chartType === 'gps' || step !== 2) {
            setNestedPaths([]);
        }
    }, [step, selectedTopic, chartType]);

    const handleChartTypeSelection = (type) => {
        setChartType(type);
        setStep(2);
        setErrorMessage('');
        setChartLabel('');
        setSelectedUnit('');
        setSelectedTopic({ name: '', type: '' });
        setSelectedPath('');
    };

    const handleCreate = () => {
        if (chartType === 'gps') {
            if (!chartLabel.trim()) {
                setErrorMessage('Please provide a map name.');
                return;
            }

            const newChart = {
                id: Date.now(),
                type: chartType,
                label: chartLabel,
            };

            onAddChart(newChart);
            onClose();
        } else {
            if (!chartLabel.trim()) {
                setErrorMessage('Please provide a chart name.');
                return;
            }

            if (!selectedUnit) {
                setErrorMessage('Please select a unit.');
                return;
            }

            if (!selectedTopic.name || !selectedTopic.type || !selectedPath) {
                setErrorMessage('Please select a topic and path.');
                return;
            }

            const newChart = {
                id: Date.now(),
                type: chartType,
                label: chartLabel,
                selectedTopic: selectedTopic,
                selectedPath: selectedPath,
                unit: selectedUnit,
            };

            onAddChart(newChart);
            onClose();
        }
    };

    const handleBack = () => {
        if (step === 2) {
            setStep(1);
            setChartType('');
            setChartLabel('');
            setSelectedUnit('');
            setSelectedTopic({ name: '', type: '' });
            setSelectedPath('');
            setErrorMessage('');
        }
    };

    return (
        <div className="modal">
            <div className="modal-content">
                <h2>
                    {step === 1
                        ? 'Create a New Chart'
                        : chartType === 'gps'
                            ? 'Add Map'
                            : 'Create a New Chart'}
                </h2>

                {step === 1 && (
                    <div className="step step-1">
                        <div className="form-group">
                            <label></label>
                            <div className="chart-type-options">
                                <button
                                    type="button"
                                    className={`chart-type-button ${chartType === 'line' ? 'selected' : ''}`}
                                    onClick={() => handleChartTypeSelection('line')}
                                >
                                    Line Chart
                                </button>
                                <button
                                    type="button"
                                    className={`chart-type-button ${chartType === 'gps' ? 'selected' : ''}`}
                                    onClick={() => handleChartTypeSelection('gps')}
                                >
                                    GPS Map
                                </button>
                            </div>
                        </div>

                        {errorMessage && <p className="error-message">{errorMessage}</p>}

                        <div className="modal-actions">
                            <button onClick={onClose} className="cancel-button">
                                Cancel
                            </button>
                            <button
                                onClick={() => {
                                    if (chartType) {
                                        setStep(2);
                                        setErrorMessage('');
                                    } else {
                                        setErrorMessage('Please select a chart type.');
                                    }
                                }}
                                className="next-button"
                            >
                                Next
                            </button>
                        </div>
                    </div>
                )}

                {step === 2 && (
                    <>
                        <div className="form-group">
                            <label htmlFor="chartLabel">
                                {chartType === 'gps' ? 'Map Name' : 'Chart Name'}
                            </label>
                            <input
                                id="chartLabel"
                                type="text"
                                value={chartLabel}
                                onChange={(e) => setChartLabel(e.target.value)}
                            />
                        </div>

                        {chartType !== 'gps' && (
                            <>
                                <div className="form-group">
                                    <label htmlFor="topic">Select Topic</label>
                                    <div className="select-with-icon">
                                        {selectedTopicManualInput ? (
                                            <input
                                                id="topicInput"
                                                type="text"
                                                value={selectedTopic.name}
                                                onChange={(e) =>
                                                    setSelectedTopic({
                                                        ...selectedTopic,
                                                        name: e.target.value,
                                                    })
                                                }
                                            />
                                        ) : (
                                            <select
                                                id="topic"
                                                value={selectedTopic.name}
                                                onChange={(e) => {
                                                    const topic = topics.find((t) => t.name === e.target.value);
                                                    setSelectedTopic({
                                                        name: topic ? topic.name : '',
                                                        type: topic ? topic.type : '',
                                                    });
                                                }}
                                            >
                                                <option value="" disabled>
                                                    Select topic
                                                </option>
                                                {topics.map((topic) => (
                                                    <option key={topic.name} value={topic.name}>
                                                        {topic.name}
                                                    </option>
                                                ))}
                                            </select>
                                        )}
                                        <button
                                            type="button"
                                            className="icon"
                                            onClick={() =>
                                                setSelectedTopicManualInput(!selectedTopicManualInput)
                                            }
                                        >
                                            &#9998;
                                        </button>
                                    </div>
                                </div>

                                <div className="form-group">
                                    <label htmlFor="chartUnit">Unit</label>
                                    <div className="select-with-icon">
                                        {selectedUnitManualInput ? (
                                            <input
                                                id="chartUnitInput"
                                                type="text"
                                                value={selectedUnit}
                                                onChange={(e) => setSelectedUnit(e.target.value)}
                                            />
                                        ) : (
                                            <select
                                                id="chartUnit"
                                                value={selectedUnit}
                                                onChange={(e) => setSelectedUnit(e.target.value)}
                                            >
                                                <option value="" disabled>
                                                    Select unit
                                                </option>
                                                {unitsSI.map((unit) => (
                                                    <option key={unit} value={unit}>
                                                        {unit}
                                                    </option>
                                                ))}
                                            </select>
                                        )}
                                        <button
                                            type="button"
                                            className="icon"
                                            onClick={() =>
                                                setSelectedUnitManualInput(!selectedUnitManualInput)
                                            }
                                        >
                                            &#9998;
                                        </button>
                                    </div>
                                </div>

                                <div className="form-group">
                                    <label htmlFor="nestedMessage">Select Nested Message Path</label>
                                    {isLoadingFields ? (
                                        <p>Loading fields...</p>
                                    ) : (
                                        <div className="select-with-icon">
                                            {selectedPathManualInput ? (
                                                <input
                                                    id="nestedMessageInput"
                                                    type="text"
                                                    value={selectedPath}
                                                    onChange={(e) => setSelectedPath(e.target.value)}
                                                    placeholder="Enter path manually"
                                                />
                                            ) : (
                                                <select
                                                    id="nestedMessage"
                                                    value={selectedPath}
                                                    onChange={(e) => setSelectedPath(e.target.value)}
                                                >
                                                    <option value="" disabled>
                                                        Select path
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
                                                onClick={() =>
                                                    setSelectedPathManualInput(!selectedPathManualInput)
                                                }
                                            >
                                                &#9998;
                                            </button>
                                        </div>
                                    )}
                                    <small>You can manually edit the path to include array indices, e.g. <code>someArray[0].someField</code>.</small>
                                </div>
                            </>
                        )}

                        {errorMessage && <p className="error-message">{errorMessage}</p>}

                        <div className="modal-actions">
                            <button onClick={handleCreate} className="create-button">
                                {chartType === 'gps' ? 'Add Map' : 'Create Chart'}
                            </button>
                            <button onClick={handleBack} className="back-button">
                                Back
                            </button>
                            <button onClick={onClose} className="cancel-button">
                                Cancel
                            </button>
                        </div>
                    </>
                )}
            </div>
        </div>
    );
};

export default ChartModal;