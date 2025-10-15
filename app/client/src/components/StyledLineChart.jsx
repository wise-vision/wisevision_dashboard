/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// File: src/components/StyledLineChart.jsx

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { Line } from 'react-chartjs-2';
import DatePicker from 'react-datepicker';
import { InfluxDB } from '@influxdata/influxdb-client-browser';
import { FaEllipsisV } from 'react-icons/fa';
import 'chart.js/auto';
import 'react-datepicker/dist/react-datepicker.css';
import '../styles/StyledLineChart.css';
import '../styles/ModalAnimations.css';

const kLineColor = 'rgba(36,198,221,0.96)';
const kBorderWidth = 4;

const StyledLineChart = ({ data: { label, selectedBucket, selectedTopic, selectedField, unit } }) => {
    const [data, setData] = useState([]);
    const [filteredData, setFilteredData] = useState([]);
    const [displayedValue, setDisplayedValue] = useState(`0 ${unit}`);
    const [aggregationPeriod, setAggregationPeriod] = useState(localStorage.getItem('aggregationPeriod') || 'week');
    const [isLive, setIsLive] = useState(JSON.parse(localStorage.getItem('isLive')) || false);
    const [liveDataInterval, setLiveDataInterval] = useState(null);
    const [loading, setLoading] = useState(true);
    const [startDate, setStartDate] = useState(localStorage.getItem('startDate') ? new Date(localStorage.getItem('startDate')) : null);
    const [endDate, setEndDate] = useState(localStorage.getItem('endDate') ? new Date(localStorage.getItem('endDate')) : null);
    const [showSettingsModal, setShowSettingsModal] = useState(false);
    const [viewportStartIndex, setViewportStartIndex] = useState(0);
    const [isDragging, setIsDragging] = useState(false);
    const [dragStartX, setDragStartX] = useState(0);
    const [kMaxDataPoints, setKMaxDataPoints] = useState(parseInt(localStorage.getItem('kMaxDataPoints')) || 12);
    const [refreshInterval, setRefreshInterval] = useState(parseInt(localStorage.getItem('refreshInterval')) || 5000);
    const [isDayAvg, setIsDayAvg] = useState(false);

    const hasFetchedData = useRef(false);

    // Initialize InfluxDB client
    const influxDB = new InfluxDB({
        url: process.env.REACT_APP_INFLUXDB_URL,
        token: process.env.REACT_APP_INFLUXDB_TOKEN,
    });



    const fetchData = useCallback(async () => {
        if (hasFetchedData.current) return;

        try {
            setLoading(true);
            const queryApi = influxDB.getQueryApi(process.env.REACT_APP_INFLUXDB_USERNAME);
            
            // Query last week of data from InfluxDB
            const fluxQuery = `
                from(bucket: "${selectedBucket}")
                |> range(start: -7d)
                |> filter(fn: (r) => r._measurement == "${selectedTopic}")
                |> filter(fn: (r) => r._field == "${selectedField}")
                |> sort(columns: ["_time"], desc: false)
            `;

            const simplifiedData = [];
            
            await queryApi.queryRows(fluxQuery, {
                next(row, tableMeta) {
                    const o = tableMeta.toObject(row);
                    if (o._value !== undefined && o._time) {
                        const date = new Date(o._time);
                        const value = parseFloat(o._value);
                        
                        if (!isNaN(value)) {
                            simplifiedData.push({ value, date });
                        }
                    }
                },
                error(error) {
                    console.error('Error in InfluxDB query:', error);
                    setLoading(false);
                },
                complete() {
                    // Remove duplicates and sort by date
                    const uniqueData = simplifiedData.filter((item, index, self) =>
                        index === self.findIndex((t) => t.date.getTime() === item.date.getTime())
                    );

                    uniqueData.sort((a, b) => a.date - b.date);

                    setData(uniqueData);
                    setFilteredData(uniqueData);
                    if (viewportStartIndex === 0) {
                        setViewportStartIndex(Math.max(uniqueData.length - kMaxDataPoints, 0));
                    }
                    setLoading(false);
                    hasFetchedData.current = true;
                }
            });

        } catch (error) {
            console.error('Error fetching data from InfluxDB:', error);
            setLoading(false);
        }
    }, [selectedBucket, selectedTopic, selectedField, kMaxDataPoints, viewportStartIndex, influxDB]);

    useEffect(() => {
        fetchData();
    }, [fetchData]);

    useEffect(() => {
        if (startDate && endDate) {
            const filtered = data.filter(item => item.date >= startDate && item.date <= endDate);
            setFilteredData(filtered);
        } else {
            setFilteredData(data);
        }
    }, [startDate, endDate, data]);

    const startLiveUpdates = () => {
        setIsLive(true);
        if (liveDataInterval) {
            clearInterval(liveDataInterval);
        }
        const intervalId = setInterval(async () => {
            try {
                console.log('Fetching live data from InfluxDB...');
                const queryApi = influxDB.getQueryApi(process.env.REACT_APP_INFLUXDB_USERNAME);
                
                // Query last 1 minute of data to get the most recent value
                const fluxQuery = `
                    from(bucket: "${selectedBucket}")
                    |> range(start: -1m)
                    |> filter(fn: (r) => r._measurement == "${selectedTopic}")
                    |> filter(fn: (r) => r._field == "${selectedField}")
                    |> sort(columns: ["_time"], desc: true)
                    |> limit(n: 1)
                `;

                let latestValue = null;
                let latestTime = null;

                await queryApi.queryRows(fluxQuery, {
                    next(row, tableMeta) {
                        const o = tableMeta.toObject(row);
                        if (o._value !== undefined && o._time) {
                            latestValue = parseFloat(o._value);
                            latestTime = new Date(o._time);
                        }
                    },
                    error(error) {
                        console.error('Error in live InfluxDB query:', error);
                    },
                    complete() {
                        if (latestValue !== null && latestTime) {
                            console.log(`Live data - Value: ${latestValue}, Time: ${latestTime}`);
                            
                            const newDataPoint = { value: latestValue, date: latestTime };

                            setData(prevData => {
                                // Check if this is actually a new data point
                                const lastPoint = prevData[prevData.length - 1];
                                if (!lastPoint || lastPoint.date.getTime() !== latestTime.getTime()) {
                                    const updatedData = [...prevData, newDataPoint];
                                    setFilteredData(updatedData);
                                    
                                    // Auto-scroll to latest data when live updates are active
                                    setViewportStartIndex(Math.max(updatedData.length - kMaxDataPoints, 0));
                                    
                                    return updatedData;
                                }
                                return prevData;
                            });
                            setDisplayedValue(`${latestValue} ${unit}`);
                        } else {
                            console.warn('No live data available');
                            setDisplayedValue(`No data ${unit}`);
                        }
                    }
                });

            } catch (error) {
                console.error('Error fetching live data from InfluxDB:', error);
            }
        }, refreshInterval);
        setLiveDataInterval(intervalId);
        console.log(`Started live updates with interval: ${refreshInterval} ms`);
    };

    const stopLiveUpdates = () => {
        setIsLive(false);
        if (liveDataInterval) {
            clearInterval(liveDataInterval);
            setLiveDataInterval(null);
        }
    };

    useEffect(() => {
        return () => {
            if (liveDataInterval) {
                clearInterval(liveDataInterval);
            }
        };
    }, [liveDataInterval]);

    const handleMouseDown = (e) => {
        setIsDragging(true);
        setDragStartX(e.clientX);
    };

    const handleMouseMove = (e) => {
        if (isDragging) {
            const deltaX = e.clientX - dragStartX;
            const threshold = 10;
            if (Math.abs(deltaX) > threshold) {
                const direction = deltaX > 0 ? -1 : 1;
                setViewportStartIndex(prevIndex => {
                    let newIndex = prevIndex + direction;

                    if (newIndex < 0) newIndex = 0;
                    if (newIndex > filteredData.length - kMaxDataPoints) {
                        newIndex = filteredData.length - kMaxDataPoints;
                    }

                    return newIndex;
                });
                setDragStartX(e.clientX);
            }
        }
    };

    const handleMouseUp = () => {
        setIsDragging(false);
    };

    const resetDates = () => {
        setStartDate(null);
        setEndDate(null);
    };

    const updateSettings = (e) => {
        e.preventDefault();
        const newMaxDataPoints = parseInt(e.target.maxDataPoints.value, 10);
        const newRefreshInterval = parseInt(e.target.refreshInterval.value, 10) * 1000;

        if (!isNaN(newMaxDataPoints) && newMaxDataPoints > 0) {
            setKMaxDataPoints(newMaxDataPoints);
        }

        if (!isNaN(newRefreshInterval) && newRefreshInterval >= 1000) {
            setRefreshInterval(newRefreshInterval);
        }

        setShowSettingsModal(false);
    };

    const getViewportData = () => {
        if (isDayAvg) {
            return calculateDayAverages();
        }
        return filteredData.slice(viewportStartIndex, Math.min(viewportStartIndex + kMaxDataPoints, filteredData.length)).slice(-100);
    };

    const calculateDayAverages = () => {
        const dayAverages = [];
        const dayMap = {};

        filteredData.forEach((item) => {
            const day = item.date.toDateString();
            if (!dayMap[day]) {
                dayMap[day] = { total: 0, count: 0 };
            }
            dayMap[day].total += item.value;
            dayMap[day].count += 1;
        });

        for (const day in dayMap) {
            const avg = dayMap[day].total / dayMap[day].count;
            dayAverages.push({ date: new Date(day), value: avg });
        }

        dayAverages.sort((a, b) => a.date - b.date);
        return dayAverages;
    };

    const getChartData = () => {
        const viewportData = getViewportData();
        return {
            labels: viewportData.map(item => item.date.toLocaleString()),
            datasets: [
                {
                    label: isDayAvg ? `${label} (Day Avg)` : label,
                    data: viewportData.map(item => item.value),
                    fill: false,
                    borderColor: kLineColor,
                    borderWidth: kBorderWidth,
                    tension: 0.4,
                    pointRadius: 6,
                    pointBackgroundColor: kLineColor,
                    pointBorderColor: '#fff',
                    pointBorderWidth: 3,
                    pointHoverRadius: 10,
                },
            ],
        };
    };

    const chartOptions = {
        scales: {
            x: {
                ticks: { font: { size: 14 }, color: '#b0b0b0' },
                grid: { display: false },
            },
            y: { display: false, grid: { display: false } },
        },
        plugins: {
            legend: { display: false },
            tooltip: {
                enabled: false,
                mode: 'nearest',
                intersect: true,
                external: (context) => {
                    const tooltipModel = context.tooltip;
                    if (tooltipModel.opacity === 0) {
                        if (filteredData.length > 0) {
                            const lastPoint = filteredData[filteredData.length - 1];
                            setDisplayedValue(`${lastPoint.value} ${unit}`);
                        }
                        return;
                    }
                    const tooltipData = tooltipModel.dataPoints[0];
                    const value = tooltipData.raw;
                    setDisplayedValue(`${value} ${unit}`);
                },
            },
        },
        maintainAspectRatio: false,
        responsive: true,
    };

    useEffect(() => {
        localStorage.setItem('aggregationPeriod', aggregationPeriod);
        localStorage.setItem('isLive', isLive);
        localStorage.setItem('startDate', startDate ? startDate.toISOString() : '');
        localStorage.setItem('endDate', endDate ? endDate.toISOString() : '');
        localStorage.setItem('kMaxDataPoints', kMaxDataPoints);
        localStorage.setItem('refreshInterval', refreshInterval);
    }, [aggregationPeriod, isLive, startDate, endDate, kMaxDataPoints, refreshInterval]);

    if (loading) {
        return <p>Loading data...</p>;
    }

    if (filteredData.length === 0) {
        return <p>No data to display.</p>;
    }

    return (
        <div
            className="chart-container"
            style={{ width: '100%', height: '78%' }}
            onMouseDown={handleMouseDown}
            onMouseMove={handleMouseMove}
            onMouseUp={handleMouseUp}
            onMouseLeave={handleMouseUp}
        >
            <div className="value-display" style={{ marginBottom: '5px' }}>
                <h2 style={{ fontSize: '34px', fontWeight: 'bold', marginTop: '5px' }}>
                    {displayedValue}
                </h2>
            </div>
            <div className="aggregation-buttons" style={{ display: 'flex', alignItems: 'center', gap: '10px', marginBottom: '15px' }}>
                <button onClick={() => setAggregationPeriod('all')} className={aggregationPeriod === 'all' ? 'active' : ''}>All</button>
                <button
                    onClick={() => setViewportStartIndex(Math.max(filteredData.length - kMaxDataPoints, 0))}
                    style={{
                        padding: '10px',
                        backgroundColor: '#5c81b8',
                        color: 'white',
                        border: 'none',
                        cursor: 'pointer',
                    }}
                >
                    Go to Latest
                </button>
                <button
                    onClick={() => setIsDayAvg(prev => !prev)}
                    style={{
                        padding: '10px',
                        backgroundColor: isDayAvg ? '#d9534f' : '#5cb85c',
                        color: 'white',
                        border: 'none',
                        cursor: 'pointer',
                    }}
                >
                    {isDayAvg ? 'Show Raw Data' : 'Day Avg'}
                </button>
                <button
                    onClick={isLive ? stopLiveUpdates : startLiveUpdates}
                    style={{
                        padding: '10px',
                        backgroundColor: isLive ? '#d9534f' : '#5cb85c',
                        color: 'white',
                        border: 'none',
                        cursor: 'pointer',
                    }}
                >
                    {isLive ? 'Stop Live' : 'Start Live'}
                </button>
                <FaEllipsisV
                    size={24}
                    style={{ cursor: 'pointer', marginLeft: '10px' }}
                    onClick={() => setShowSettingsModal(!showSettingsModal)}
                />
            </div>
            {showSettingsModal && (
                <>
                    <div className="overlay fade-in" style={{
                        position: 'fixed',
                        top: 0,
                        left: 0,
                        right: 0,
                        bottom: 0,
                        backgroundColor: 'rgba(0, 0, 0, 0.5)',
                        zIndex: 999
                    }} onClick={() => setShowSettingsModal(false)} />
                    <div className="settings-modal fade-in" style={{
                        position: 'fixed',
                        top: '50%',
                        left: '50%',
                        transform: 'translate(-50%, -50%)',
                        backgroundColor: 'white',
                        padding: '20px',
                        zIndex: 1000,
                        borderRadius: '8px',
                        boxShadow: '0 4px 8px rgba(0, 0, 0, 0.3)'
                    }}>
                        <form onSubmit={updateSettings}>
                            <div style={{ marginBottom: '15px' }}>
                                <label>Max Number of Points: </label>
                                <input type="number" name="maxDataPoints" min="1" defaultValue={kMaxDataPoints} />
                            </div>
                            <div style={{ marginBottom: '15px' }}>
                                <label>Refresh Interval (seconds): </label>
                                <input type="number" name="refreshInterval" min="1" defaultValue={refreshInterval / 1000} />
                            </div>
                            <div style={{ marginBottom: '15px' }}>
                                <label>Select Date Range:</label>
                                <div className="date-pickers">
                                    <DatePicker
                                        selected={startDate}
                                        onChange={(date) => setStartDate(date)}
                                        selectsStart
                                        startDate={startDate}
                                        endDate={endDate}
                                        dateFormat="dd-MM-yyyy hh:mm aa"
                                        showTimeSelect
                                        timeFormat="HH:mm"
                                        timeIntervals={15}
                                        placeholderText="Start Date"
                                    />
                                    <DatePicker
                                        selected={endDate}
                                        onChange={(date) => setEndDate(date)}
                                        selectsEnd
                                        startDate={startDate}
                                        endDate={endDate}
                                        minDate={startDate}
                                        dateFormat="dd-MM-yyyy hh:mm aa"
                                        showTimeSelect
                                        timeFormat="HH:mm"
                                        timeIntervals={15}
                                        placeholderText="End Date"
                                    />
                                </div>
                            </div>
                            <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                                <button type="button" onClick={resetDates} style={{ padding: '10px', backgroundColor: '#5c81b8', color: 'white', border: 'none' }}>
                                    Reset Dates
                                </button>
                                <button type="button" onClick={() => setShowSettingsModal(false)} style={{ padding: '10px', backgroundColor: '#5c81b8', color: 'white', border: 'none' }}>
                                    Cancel
                                </button>
                                <button type="submit" style={{ padding: '10px', backgroundColor: '#5c81b8', color: 'white', border: 'none' }}>
                                    Save Settings
                                </button>
                            </div>
                        </form>
                    </div>
                </>
            )}
            <Line data={getChartData()} options={chartOptions} />
        </div>
    );

};

export default StyledLineChart;
