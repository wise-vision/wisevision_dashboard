// File: src/components/StyledLineChart.jsx

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { Line } from 'react-chartjs-2';
import DatePicker from 'react-datepicker';
import axios from 'axios';
import { FaEllipsisV } from 'react-icons/fa';
import 'chart.js/auto';
import 'react-datepicker/dist/react-datepicker.css';
import '../styles/StyledLineChart.css';
import '../styles/ModalAnimations.css';

const kLineColor = 'rgba(36,198,221,0.96)';
const kBorderWidth = 4;

const StyledLineChart = ({ data: { label, selectedTopic, unit, selectedPath } }) => {
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

    const traverseForNumericValues = (obj, path) => {
        if (!obj || typeof obj !== 'object') {
            console.error('Invalid object provided:', obj);
            return undefined;
        }

        const parts = path.split('.');
        let currentObj = obj;

        for (let part of parts) {
            if (currentObj[part] !== undefined) {
                currentObj = currentObj[part];
            } else {
                console.error(`Path "${part}" not found in`, currentObj);
                return undefined;
            }
        }

        if (typeof currentObj === 'number') {
            return currentObj;
        } else {
            console.error(`Value at path "${path}" is not a number:`, currentObj);
            return undefined;
        }
    };

    const fetchData = useCallback(async () => {
        if (hasFetchedData.current) return;

        const controller = new AbortController();
        const signal = controller.signal;

        try {
            const encodedTopic = encodeURIComponent(selectedTopic.name);
            const encodedType = encodeURIComponent(selectedTopic.type);
            const response = await axios.get(
                `${process.env.REACT_APP_API_BASE_URL}/api/topic_echo_data_base_any_last_week/${encodedTopic}?type=${encodedType}`,
                { signal }
            );

            const { messages, timestamps } = response.data;

            const simplifiedData = messages.map((message, index) => {

                const value = traverseForNumericValues(message, selectedPath);
                const timestamp = timestamps[index];
                const date = timestamp
                    ? new Date(
                        timestamp.year,
                        timestamp.month - 1,
                        timestamp.day,
                        timestamp.hour,
                        timestamp.minute,
                        timestamp.second
                    )
                    : null;

                return { value, date };
            }).filter(item => item.value !== undefined && item.date !== null);

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
        } catch (error) {
            if (axios.isCancel(error)) {
                console.log('Request canceled', error.message);
            } else {
                console.error('Error fetching data', error);
            }
            setLoading(false);
        }

        return () => controller.abort();
    }, [selectedTopic, selectedPath, kMaxDataPoints, viewportStartIndex]);

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
                console.log('Fetching live data...');
                const encodedTopic = encodeURIComponent(selectedTopic.name);
                const encodedType = encodeURIComponent(selectedTopic.type);
                const response = await axios.get(
                    `${process.env.REACT_APP_API_BASE_URL}/api/topic_echo/${encodedTopic}?type=${encodedType}`
                );

                console.log('Live data response:', response.data);

                let { message } = response.data;


                console.log('Message received:', message);

                // If message is empty, stop processing
                if (!message || Object.keys(message).length === 0) {
                    console.warn('Received empty message:', message);
                    setDisplayedValue(`No data ${unit}`);
                    return;
                }

                const value = traverseForNumericValues(message, selectedPath);
                console.log(`Extracted value: ${value}`);

                if (value !== undefined) {
                    const date = new Date();
                    const newDataPoint = { value, date };

                    setData(prevData => {
                        const updatedData = [...prevData, newDataPoint];
                        console.log('Updated data:', updatedData);
                        setFilteredData(updatedData);
                        return updatedData;
                    });
                    setDisplayedValue(`${value} ${unit}`);
                } else {
                    console.warn(`Path "${selectedPath}" not found in message:`, message);
                    setDisplayedValue(`No data ${unit}`);
                }
            } catch (error) {
                console.error('Error fetching live data', error);
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
