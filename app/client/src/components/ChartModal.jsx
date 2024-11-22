import React, { useState, useEffect } from 'react';
import '../styles/ChartModal.css';
import penIcon from '../assets/images/pen.png';

const unitsSI = ['°C', 'kW', 'V', 'A', 'W', 'Hz', 'Pa'];

const ChartModal = ({ onClose, onAddChart }) => {
    const [chartType, setChartType] = useState('line');
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

    // State variables to track manual input mode
    const [selectedTopicManualInput, setSelectedTopicManualInput] = useState(false);
    const [selectedUnitManualInput, setSelectedUnitManualInput] = useState(false);
    const [selectedPathManualInput, setSelectedPathManualInput] = useState(false);

    // GPS device state variables
    const [deviceName, setDeviceName] = useState('');
    const [deviceEUI, setDeviceEUI] = useState('');
    const [latitude, setLatitude] = useState('');
    const [longitude, setLongitude] = useState('');
    const [altitude, setAltitude] = useState('');

    // Fetch topics from the API
    useEffect(() => {
        const fetchTopics = async () => {
            try {
                const response = await fetch('http://localhost:5000/api/topics');
                const result = await response.json();
                setTopics(result);
                if (result[0]) {
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
    }, []);

    // Fetch message structure and available numeric paths
    useEffect(() => {
        if (step === 2 && selectedTopic.type && chartType !== 'gps') {
            const fetchMessageStructure = async () => {
                try {
                    const encodedType = encodeURIComponent(selectedTopic.type);
                    const response = await fetch(
                        `http://localhost:5000/api/message_structure/${encodedType}`
                    );
                    const result = await response.json();

                    const paths = [];
                    const traverse = (obj, currentPath = '') => {
                        for (let key in obj) {
                            const value = obj[key];
                            const path = currentPath ? `${currentPath}.${key}` : key;

                            if (typeof value === 'object' && !Array.isArray(value)) {
                                traverse(value, path);
                            } else if (
                                typeof value === 'string' &&
                                [
                                    'int8',
                                    'int16',
                                    'int32',
                                    'int64',
                                    'uint8',
                                    'uint16',
                                    'uint32',
                                    'uint64',
                                    'float',
                                    'double',
                                ].includes(value)
                            ) {
                                paths.push(path); // Collect numeric paths
                            }
                        }
                    };
                    traverse(result);
                    setNestedPaths(paths);
                    if (paths[0]) {
                        setSelectedPath(paths[0]);
                    }
                } catch (error) {
                    console.error('Error fetching message structure:', error);
                }
            };
            fetchMessageStructure();
        }
    }, [step, selectedTopic, chartType]);

    const handleNext = () => {
        if (!chartLabel.trim()) {
            setErrorMessage('Please provide a chart name.');
            return;
        }

        if (chartType !== 'gps' && !selectedUnit) {
            setErrorMessage('Please select a unit.');
            return;
        }

        setErrorMessage('');
        setStep(2); // Proceed to next step
    };

    const handleCreate = () => {
        if (chartType === 'gps') {
            // Validate GPS fields
            if (
                !deviceName.trim() ||
                !deviceEUI.trim() ||
                latitude === '' || // Checking for empty string to allow zero values
                longitude === '' ||
                altitude === ''
            ) {
                setErrorMessage('Please fill in all the GPS device information.');
                return;
            }


            const data = {
                device_name: deviceName,
                device_eui: {
                    data: deviceEUI.split(',').map((num) => parseInt(num.trim(), 10)),
                },
                nav_value: {
                    latitude: parseFloat(latitude),
                    longitude: parseFloat(longitude),
                    altitude: parseFloat(altitude),
                },
                is_moving: false,
            };

            // Send POST request
            fetch('http://localhost:5000/api/add_gps_device', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'Accept': 'application/json',
                },
                body: JSON.stringify(data),
            })
                .then((response) => {
                    if (!response.ok) {
                        console.error('Response status:', response.status);
                        console.error('Response text:', response.statusText);
                        throw new Error('Failed to add GPS device');
                    }
                    return response.json();
                })
                .then((result) => {
                    console.log('GPS device added:', result);

                    const newChart = {
                        id: Date.now(),
                        type: chartType,
                        label: chartLabel,
                        deviceData: data,
                    };

                    onAddChart(newChart);
                    onClose();
                })
                .catch((error) => {
                    console.error('Error adding GPS device:', error);
                    setErrorMessage('Failed to add GPS device. Please try again.');
                });
        } else {
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

    return (
        <div className="modal">
            <div className="modal-content">
                <h2>Create a New Chart</h2>
                {step === 1 && (
                    <>
                        <div className="form-group">
                            <label htmlFor="chartType">Chart Type</label>
                            <select
                                id="chartType"
                                value={chartType}
                                onChange={(e) => setChartType(e.target.value)}
                            >
                                <option value="line">Line Chart</option>
                                <option value="gps">GPS</option>
                            </select>
                        </div>

                        <div className="form-group">
                            <label htmlFor="chartLabel">Chart Name</label>
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
                                                    const topic = topics.find(
                                                        (t) => t.name === e.target.value
                                                    );
                                                    setSelectedTopic({
                                                        name: topic.name,
                                                        type: topic.type,
                                                    });
                                                }}
                                            >
                                                {topics.map((topic) => (
                                                    <option key={topic.name} value={topic.name}>
                                                        {topic.name}
                                                    </option>
                                                ))}
                                            </select>
                                        )}
                                        <img
                                            src={penIcon}
                                            className="icon"
                                            alt="Edit"
                                            onClick={() =>
                                                setSelectedTopicManualInput(
                                                    !selectedTopicManualInput
                                                )
                                            }
                                        />
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
                                        <img
                                            src={penIcon}
                                            className="icon"
                                            alt="Edit"
                                            onClick={() =>
                                                setSelectedUnitManualInput(
                                                    !selectedUnitManualInput
                                                )
                                            }
                                        />
                                    </div>
                                </div>
                            </>
                        )}

                        {errorMessage && <p style={{ color: 'red' }}>{errorMessage}</p>}

                        <div className="modal-actions">
                            <button onClick={handleNext}>Next</button>
                            <button onClick={onClose}>Cancel</button>
                        </div>
                    </>
                )}

                {step === 2 && chartType !== 'gps' && (
                    <>
                        <div className="form-group">
                            <label htmlFor="nestedMessage">
                                Select Nested Message Path
                            </label>
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
                                    >
                                        {nestedPaths.map((path) => (
                                            <option key={path} value={path}>
                                                {path}
                                            </option>
                                        ))}
                                    </select>
                                )}
                                <img
                                    src={penIcon}
                                    className="icon"
                                    alt="Edit"
                                    onClick={() =>
                                        setSelectedPathManualInput(!selectedPathManualInput)
                                    }
                                />
                            </div>
                        </div>

                        {errorMessage && <p style={{ color: 'red' }}>{errorMessage}</p>}

                        <div className="modal-actions">
                            <button onClick={handleCreate}>Create Chart</button>
                            <button
                                className="back-button"
                                onClick={() => setStep(1)}
                            >
                                Back
                            </button>
                            <button onClick={onClose}>Cancel</button>
                        </div>
                    </>
                )}

                {step === 2 && chartType === 'gps' && (
                    <>
                        <div className="form-group">
                            <label htmlFor="deviceName">Device Name</label>
                            <input
                                id="deviceName"
                                type="text"
                                value={deviceName}
                                onChange={(e) => setDeviceName(e.target.value)}
                            />
                        </div>
                        <div className="form-group">
                            <label htmlFor="deviceEUI">
                                Device EUI (comma-separated numbers)
                            </label>
                            <input
                                id="deviceEUI"
                                type="text"
                                value={deviceEUI}
                                onChange={(e) => setDeviceEUI(e.target.value)}
                            />
                        </div>
                        <div className="form-group">
                            <label htmlFor="latitude">Latitude</label>
                            <input
                                id="latitude"
                                type="number"
                                step="any"
                                value={latitude}
                                onChange={(e) => setLatitude(e.target.value)}
                            />
                        </div>
                        <div className="form-group">
                            <label htmlFor="longitude">Longitude</label>
                            <input
                                id="longitude"
                                type="number"
                                step="any"
                                value={longitude}
                                onChange={(e) => setLongitude(e.target.value)}
                            />
                        </div>
                        <div className="form-group">
                            <label htmlFor="altitude">Altitude</label>
                            <input
                                id="altitude"
                                type="number"
                                step="any"
                                value={altitude}
                                onChange={(e) => setAltitude(e.target.value)}
                            />
                        </div>
                        {errorMessage && <p style={{ color: 'red' }}>{errorMessage}</p>}

                        <div className="modal-actions">
                            <button onClick={handleCreate}>Create GPS Map</button>
                            <button
                                className="back-button"
                                onClick={() => setStep(1)}
                            >
                                Back
                            </button>
                            <button onClick={onClose}>Cancel</button>
                        </div>
                    </>
                )}
            </div>
        </div>
    );
};

export default ChartModal;
