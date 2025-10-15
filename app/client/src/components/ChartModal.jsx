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
import { CiFilter } from "react-icons/ci";
import { InfluxDB } from '@influxdata/influxdb-client-browser';
import FilterTopicsModal from './FilterTopicsModal';
import '../styles/ChartModal.css';

const unitsSI = ['°C', 'kW', 'V', 'A', 'W', 'Hz', 'Pa'];

const ChartModal = ({ onClose, onAddChart }) => {
    const [chartType, setChartType] = useState('');
    const [chartLabel, setChartLabel] = useState('');
    const [errorMessage, setErrorMessage] = useState('');
    
    // InfluxDB related state
    const [buckets, setBuckets] = useState([]);
    const [selectedBucket, setSelectedBucket] = useState('');
    const [topics, setTopics] = useState([]);
    const [selectedTopic, setSelectedTopic] = useState('');
    const [fields, setFields] = useState([]);
    const [selectedField, setSelectedField] = useState('');
    
    const [selectedUnit, setSelectedUnit] = useState('');
    const [step, setStep] = useState(1);
    const [loading, setLoading] = useState(false);

    // State variables to track filter modal
    const [isFilterModalOpen, setIsFilterModalOpen] = useState(false);
    const [selectedFilters, setSelectedFilters] = useState({
        name: "",
        messageTypes: [],
        namespaces: []
    });

    const handleFiltersApply = (filters) => {
        setSelectedFilters(filters);
    };

    // State variables to track manual input mode
    const [selectedBucketManualInput, setSelectedBucketManualInput] = useState(false);
    const [selectedTopicManualInput] = useState(false);
    const [selectedUnitManualInput, setSelectedUnitManualInput] = useState(false);
    const [selectedFieldManualInput, setSelectedFieldManualInput] = useState(false);

    // Initialize InfluxDB client
    const influxDB = new InfluxDB({
        url: process.env.REACT_APP_INFLUXDB_URL,
        token: process.env.REACT_APP_INFLUXDB_TOKEN,
    });

    // Fetch buckets with wisevision_influxdb_ros2 label using Flask API
    const fetchBuckets = async () => {
        try {
            setLoading(true);
            const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/get_influx_buckets`);
            const result = await response.json();
            
            if (result.success && result.influx_buckets) {
                // Convert bucket names to the expected format
                const bucketsList = result.influx_buckets.map(bucketName => ({
                    name: bucketName,
                    id: bucketName
                }));
                
                setBuckets(bucketsList);
                
                if (bucketsList.length > 0) {
                    setSelectedBucket(bucketsList[0].name);
                } else {
                    setErrorMessage('No buckets found with wisevision_influxdb_ros2 label.');
                }
            } else {
                console.error('Failed to fetch buckets from Flask API:', result.error_message);
                setErrorMessage(result.error_message || 'Failed to fetch buckets from server.');
                
                // Fallback to manual input
                setBuckets([]);
            }
        } catch (error) {
            console.error('Error fetching buckets from Flask API:', error);
            setErrorMessage('Cannot connect to Flask server. Please check your server configuration.');
            
            // Fallback to manual input
            setBuckets([]);
        } finally {
            setLoading(false);
        }
    };

    // Fetch topics (measurements) from selected bucket
    const fetchTopics = async (bucketName) => {
        if (!bucketName) return;
        
        try {
            setLoading(true);
            const queryApi = influxDB.getQueryApi(process.env.REACT_APP_INFLUXDB_USERNAME);
            
            const fluxQuery = `
                import "influxdata/influxdb/schema"
                schema.measurements(bucket: "${bucketName}")
            `;
            
            const topicsList = [];
            await queryApi.queryRows(fluxQuery, {
                next(row, tableMeta) {
                    const o = tableMeta.toObject(row);
                    if (o._value && !topicsList.includes(o._value)) {
                        topicsList.push(o._value);
                    }
                },
                error(error) {
                    console.error('Error in flux query:', error);
                    setErrorMessage('Failed to fetch topics from InfluxDB');
                },
                complete() {
                    setTopics(topicsList);
                    if (topicsList.length > 0 && chartType !== 'gps') {
                        setSelectedTopic(topicsList[0]);
                    }
                }
            });
        } catch (error) {
            console.error('Error fetching topics:', error);
            setErrorMessage('Failed to fetch topics from InfluxDB');
        } finally {
            setLoading(false);
        }
    };

    // Fetch fields from selected topic
    const fetchFields = async (bucketName, topicName) => {
        if (!bucketName || !topicName) return;
        
        try {
            setLoading(true);
            const queryApi = influxDB.getQueryApi(process.env.REACT_APP_INFLUXDB_USERNAME);
            
            const fluxQuery = `
                import "influxdata/influxdb/schema"
                schema.fieldKeys(
                    bucket: "${bucketName}",
                    predicate: (r) => r._measurement == "${topicName}",
                    start: -7d
                )
            `;
            
            const fieldsList = [];
            const excludedFields = ['payload_b64', 'record_id', 'status', 'stop_record_id'];
            
            await queryApi.queryRows(fluxQuery, {
                next(row, tableMeta) {
                    const o = tableMeta.toObject(row);
                    if (o._value && !fieldsList.includes(o._value) && !excludedFields.includes(o._value)) {
                        fieldsList.push(o._value);
                    }
                },
                error(error) {
                    console.error('Error in flux query:', error);
                    setErrorMessage('Failed to fetch fields from InfluxDB');
                },
                complete() {
                    setFields(fieldsList);
                    if (fieldsList.length > 0) {
                        setSelectedField(fieldsList[0]);
                    }
                }
            });
        } catch (error) {
            console.error('Error fetching fields:', error);
            setErrorMessage('Failed to fetch fields from InfluxDB');
        } finally {
            setLoading(false);
        }
    };

    // Fetch buckets on component mount
    useEffect(() => {
        if (chartType && chartType !== 'gps') {
            fetchBuckets();
        }
    }, [chartType]);

    // Fetch topics when bucket changes
    useEffect(() => {
        if (selectedBucket) {
            fetchTopics(selectedBucket);
        }
    }, [selectedBucket]);

    // Fetch fields when topic changes
    useEffect(() => {
        if (selectedBucket && selectedTopic) {
            fetchFields(selectedBucket, selectedTopic);
        }
    }, [selectedBucket, selectedTopic]);



    const handleChartTypeSelection = (type) => {
        setChartType(type);
        setStep(2);
        setErrorMessage('');
        // Reset fields when chart type changes
        setChartLabel('');
        setSelectedUnit('');
        setSelectedBucket('');
        setSelectedTopic('');
        setSelectedField('');
        setBuckets([]);
        setTopics([]);
        setFields([]);
    };

    const handleCreate = () => {
        if (chartType === 'gps') {
            // Validate map name
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
            // Validate fields for other chart types
            if (!chartLabel.trim()) {
                setErrorMessage('Please provide a chart name.');
                return;
            }

            if (!selectedUnit) {
                setErrorMessage('Please select a unit.');
                return;
            }

            if (!selectedBucket || !selectedTopic || !selectedField) {
                setErrorMessage('Please select a bucket, topic and field.');
                return;
            }

            const newChart = {
                id: Date.now(),
                type: chartType,
                label: chartLabel,
                selectedBucket: selectedBucket,
                selectedTopic: selectedTopic,
                selectedField: selectedField,
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
            setSelectedBucket('');
            setSelectedTopic('');
            setSelectedField('');
            setBuckets([]);
            setTopics([]);
            setFields([]);
            setErrorMessage('');
        }
    };

    return (
        <div className="modal">
            <div className="modal-content">
                {/* Dynamic Modal Title */}
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
                            {/* Dynamic Label for Chart Name / Map Name */}
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
                                    <label htmlFor="bucket">Select Bucket</label>
                                    <div className="select-with-icon">
                                        {selectedBucketManualInput ? (
                                            <input
                                                id="bucketInput"
                                                type="text"
                                                value={selectedBucket}
                                                onChange={(e) => setSelectedBucket(e.target.value)}
                                                placeholder="Enter bucket name"
                                            />
                                        ) : (
                                            <select
                                                id="bucket"
                                                value={selectedBucket}
                                                onChange={(e) => setSelectedBucket(e.target.value)}
                                                disabled={loading}
                                            >
                                                <option value="" disabled>
                                                    {loading ? 'Loading buckets...' : 'Select bucket'}
                                                </option>
                                                {buckets.map((bucket) => (
                                                    <option key={bucket.id} value={bucket.name}>
                                                        {bucket.name}
                                                    </option>
                                                ))}
                                            </select>
                                        )}
                                        <button
                                            type="button"
                                            className="icon"
                                            onClick={() => setSelectedBucketManualInput(!selectedBucketManualInput)}
                                        >
                                            &#9998;
                                        </button>
                                    </div>
                                </div>

                                <div className="form-group">
                                    <label htmlFor="topic">Select Topic</label>
                                    <div className="select-with-icon">
                                        {selectedTopicManualInput ? (
                                            <input
                                                id="topicInput"
                                                type="text"
                                                value={selectedTopic}
                                                onChange={(e) => setSelectedTopic(e.target.value)}
                                            />
                                        ) : (
                                            <div className='select-with-icon'>
                                                <select
                                                    id="topic"
                                                    value={selectedTopic}
                                                    onChange={(e) => setSelectedTopic(e.target.value)}
                                                    disabled={loading || !selectedBucket}
                                                >
                                                    <option value="" disabled>
                                                        {loading ? 'Loading topics...' : !selectedBucket ? 'Select bucket first' : 'Select topic'}
                                                    </option>
                                                    {topics.map((topic) => (
                                                        <option key={topic} value={topic}>
                                                            {topic.replace('_', '/')}
                                                        </option>
                                                    ))}
                                                </select>
                                                {/* Filter button */}
                                                <button type="button" className="icon" onClick={() => setIsFilterModalOpen(true)}>
                                                    <CiFilter className="filter-icon" />
                                                </button>
                                            </div>
                                        )}
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
                                    <label htmlFor="field">Select Field</label>
                                    <div className="select-with-icon">
                                        {selectedFieldManualInput ? (
                                            <input
                                                id="fieldInput"
                                                type="text"
                                                value={selectedField}
                                                onChange={(e) => setSelectedField(e.target.value)}
                                            />
                                        ) : (
                                            <select
                                                id="field"
                                                value={selectedField}
                                                onChange={(e) => setSelectedField(e.target.value)}
                                                disabled={loading || !selectedTopic}
                                            >
                                                <option value="" disabled>
                                                    {loading ? 'Loading fields...' : !selectedTopic ? 'Select topic first' : 'Select field'}
                                                </option>
                                                {fields.map((field) => (
                                                    <option key={field} value={field}>
                                                        {field}
                                                    </option>
                                                ))}
                                            </select>
                                        )}
                                        <button
                                            type="button"
                                            className="icon"
                                            onClick={() =>
                                                setSelectedFieldManualInput(!selectedFieldManualInput)
                                            }
                                        >
                                            &#9998;
                                        </button>
                                    </div>
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
            <FilterTopicsModal
                isOpen={isFilterModalOpen}
                onClose={() => setIsFilterModalOpen(false)}
                onApplyFilters={handleFiltersApply}
            />
        </div>
    );
};

export default ChartModal;
