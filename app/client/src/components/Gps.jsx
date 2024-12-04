import React, { useEffect, useRef, useState } from "react";
import L from "leaflet";
import 'leaflet/dist/leaflet.css';
import '../styles/Gps.css';


import markerIcon2x from 'leaflet/dist/images/marker-icon-2x.png';
import markerIcon from 'leaflet/dist/images/marker-icon.png';
import markerShadow from 'leaflet/dist/images/marker-shadow.png';


import spinnerIcon from '../assets/images/loadingGif.gif';


delete L.Icon.Default.prototype._getIconUrl;

L.Icon.Default.mergeOptions({
    iconRetinaUrl: markerIcon2x,
    iconUrl: markerIcon,
    shadowUrl: markerShadow,
});

export default function Gps({ chartData }) {
    const mapRef = useRef();
    const markersRef = useRef([]);
    const [showAddModal, setShowAddModal] = useState(false);
    const [showDeleteModal, setShowDeleteModal] = useState(false);
    const [showEditModal, setShowEditModal] = useState(false);
    const [deviceName, setDeviceName] = useState('');
    const [deviceEUI, setDeviceEUI] = useState('');
    const [latitude, setLatitude] = useState('');
    const [longitude, setLongitude] = useState('');
    const [altitude, setAltitude] = useState('');
    const [errorMessage, setErrorMessage] = useState('');
    const [devicesData, setDevicesData] = useState([]); // Store devices data
    const [selectedDevice, setSelectedDevice] = useState(null);
    const [deleteMessage, setDeleteMessage] = useState(''); // Delete success/error message
    const [editMessage, setEditMessage] = useState(''); // Edit success/error message
    const [loadingDevices, setLoadingDevices] = useState(false); // Loading devices for modal

    const fetchDataAndDisplayMarkers = async () => {
        try {
            const response = await fetch(
                `${process.env.REACT_APP_API_BASE_URL}/api/topic_echo_gps_devices`
            );
            const data = await response.json();


            markersRef.current.forEach(marker => {
                mapRef.current.removeLayer(marker);
            });
            markersRef.current = [];

            if (data.message?.devices_data) {
                setDevicesData(data.message.devices_data);

                data.message.devices_data.forEach((device) => {
                    const { latitude, longitude } = device.nav_value;
                    const { device_name } = device;

                    if (latitude && longitude) {
                        const marker = L.marker([latitude, longitude])
                            .addTo(mapRef.current)
                            .bindTooltip(`${device_name}`, {
                                permanent: false,
                                direction: 'top',
                            });

                        markersRef.current.push(marker);
                    }
                });
            }
        } catch (error) {
            console.error("Error fetching or displaying data:", error);
        }
    };

    useEffect(() => {
        if (!mapRef.current) {
            mapRef.current = L.map(`map-${chartData.id}`, {
                center: [52.2297, 21.0122],
                zoom: 5,
                maxBounds: [
                    [-90, -180],
                    [90, 180],
                ],
                worldCopyJump: true,
            });

            L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
                maxZoom: 19,
                attribution:
                    '&copy; <a href="https://www.openstreetmap.org/copyright">' +
                    'OpenStreetMap</a> contributors',
            }).addTo(mapRef.current);

            setTimeout(() => {
                mapRef.current.invalidateSize();
            }, 100);
        }

        // Initial data fetch
        fetchDataAndDisplayMarkers();

        const interval = setInterval(() => {
            fetchDataAndDisplayMarkers();
        }, 60000);

        // Cleanup on unmount
        return () => {
            clearInterval(interval);
            // Remove markers
            markersRef.current.forEach(marker => {
                mapRef.current.removeLayer(marker);
            });
            markersRef.current = [];
        };
    }, [chartData.id]);

    useEffect(() => {
        const handleResize = () => {
            if (mapRef.current) {
                mapRef.current.invalidateSize();
            }
        };

        window.addEventListener('resize', handleResize);
        return () => window.removeEventListener('resize', handleResize);
    }, []);

    const handleCreate = () => {
        if (
            !deviceName.trim() ||
            !deviceEUI.trim() ||
            latitude === '' ||
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

        fetch(`${process.env.REACT_APP_API_BASE_URL}/api/add_gps_device`, {
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
                setShowAddModal(false);
                setDeviceName('');
                setDeviceEUI('');
                setLatitude('');
                setLongitude('');
                setAltitude('');
                setErrorMessage('');
                // Refresh markers
                fetchDataAndDisplayMarkers();
            })
            .catch((error) => {
                console.error('Error adding GPS device:', error);
                setErrorMessage('Failed to add GPS device. Please try again.');
            });
    };

    const handleDelete = () => {
        if (!selectedDevice) {
            setErrorMessage('Please select a device to delete.');
            return;
        }

        setDeleteMessage('');
        setErrorMessage('');

        const data = {
            device_eui: {
                data: selectedDevice.device_eui.data,
            },
        };

        fetch(`${process.env.REACT_APP_API_BASE_URL}/api/delete_gps_device`, {
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
                    throw new Error('Failed to delete GPS device');
                }
                return response.json();
            })
            .then((result) => {
                console.log('GPS device deleted:', result);
                setDeleteMessage('Device deleted successfully.');
                setSelectedDevice(null);
                // Refresh markers
                fetchDataAndDisplayMarkers();
            })
            .catch((error) => {
                console.error('Error deleting GPS device:', error);
                setDeleteMessage('Failed to delete GPS device. Please try again.');
            });
    };

    const handleEdit = () => {
        if (!selectedDevice) {
            setErrorMessage('Please select a device to edit.');
            return;
        }

        if (
            !deviceName.trim() ||
            latitude === '' ||
            longitude === '' ||
            altitude === ''
        ) {
            setErrorMessage('Please fill in all the fields.');
            return;
        }

        setEditMessage('');
        setErrorMessage('');

        const data = {
            device_name: deviceName,
            device_eui: {
                data: selectedDevice.device_eui.data,
            },
            nav_value: {
                latitude: parseFloat(latitude),
                longitude: parseFloat(longitude),
                altitude: parseFloat(altitude),
            },
            is_moving: false,
        };

        fetch(`${process.env.REACT_APP_API_BASE_URL}/api/modify_gps_device`, {
            method: 'POST', // Assuming POST request
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
                    throw new Error('Failed to edit GPS device');
                }
                return response.json();
            })
            .then((result) => {
                console.log('GPS device edited:', result);
                setEditMessage('Device updated successfully.');
                setSelectedDevice(null);
                setDeviceName('');
                setDeviceEUI('');
                setLatitude('');
                setLongitude('');
                setAltitude('');
                // Refresh markers
                fetchDataAndDisplayMarkers();
            })
            .catch((error) => {
                console.error('Error editing GPS device:', error);
                setEditMessage('Failed to edit GPS device. Please try again.');
            });
    };

    const openEditModal = () => {
        setErrorMessage('');
        setEditMessage('');
        setShowEditModal(true);
        setSelectedDevice(null);
        // Clear input fields
        setDeviceName('');
        setDeviceEUI('');
        setLatitude('');
        setLongitude('');
        setAltitude('');
    };

    const handleDeviceSelection = (e) => {
        const selectedEUI = e.target.value;
        const device = devicesData.find(
            (d) => d.device_eui.data.join(',') === selectedEUI
        );
        setSelectedDevice(device);

        if (device) {
            setDeviceName(device.device_name);
            setLatitude(device.nav_value.latitude);
            setLongitude(device.nav_value.longitude);
            setAltitude(device.nav_value.altitude);
        } else {
            // Clear fields if no device is selected
            setDeviceName('');
            setLatitude('');
            setLongitude('');
            setAltitude('');
        }
    };

    const openDeleteModal = () => {
        setErrorMessage('');
        setDeleteMessage('');
        setSelectedDevice(null);
        setShowDeleteModal(true);
        setLoadingDevices(true);

        // Fetch devices data
        fetch(`${process.env.REACT_APP_API_BASE_URL}/api/topic_echo_gps_devices`)
            .then((response) => response.json())
            .then((data) => {
                if (data.message?.devices_data) {
                    setDevicesData(data.message.devices_data);
                }
                setLoadingDevices(false); // Devices loaded
            })
            .catch((error) => {
                console.error('Error fetching devices:', error);
                setLoadingDevices(false);
                setErrorMessage('Failed to load devices.');
            });
    };

    const closeDeleteModal = () => {
        setShowDeleteModal(false);
        setDeleteMessage('');
        setErrorMessage('');
        setSelectedDevice(null);
    };

    return (
        <div className="gps-container">
            <div className="button-group right">
                <button onClick={() => setShowAddModal(true)}>Add</button>
                <button onClick={openEditModal}>Edit</button>
                <button onClick={openDeleteModal}>Delete</button>
            </div>
            <div id={`map-${chartData.id}`} className="map">
                {/* Map content */}
            </div>

            {/* Add Device Modal */}
            {showAddModal && (
                <div className="modal">
                    <div className="modal-content">
                        <h2>Add a New GPS Device</h2>
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
                        {errorMessage && (
                            <p style={{ color: 'red' }}>{errorMessage}</p>
                        )}
                        <div className="modal-actions">
                            <button className="delete-button" onClick={handleCreate}>Create Device</button>
                            <button className="cancel-button" onClick={() => setShowAddModal(false)}>Cancel</button>
                        </div>
                    </div>
                </div>
            )}

            {/* Edit Device Modal */}
            {showEditModal && (
                <div className="modal">
                    <div className="modal-content">
                        <h2>Edit GPS Device</h2>
                        <div className="form-group">
                            <label htmlFor="selectDeviceEdit">Select Device to Edit</label>
                            <select
                                id="selectDeviceEdit"
                                value={selectedDevice ? selectedDevice.device_eui.data.join(',') : ''}
                                onChange={handleDeviceSelection}
                            >
                                <option value="" disabled>Select a device</option>
                                {devicesData.map((device) => (
                                    <option
                                        key={device.device_eui.data.join(',')}
                                        value={device.device_eui.data.join(',')}
                                    >
                                        {device.device_name}
                                    </option>
                                ))}
                            </select>
                        </div>
                        {selectedDevice && (
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
                                    <label>Device EUI</label>
                                    <input
                                        type="text"
                                        value={selectedDevice.device_eui.data.join(',')}
                                        disabled
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
                            </>
                        )}
                        {editMessage && (
                            <p style={{ color: 'green' }}>{editMessage}</p>
                        )}
                        {errorMessage && (
                            <p style={{ color: 'red' }}>{errorMessage}</p>
                        )}
                        <div className="modal-actions">
                            <button className="delete-button" onClick={handleEdit}>Save Changes</button>
                            <button className="cancel-button" onClick={() => {
                                setShowEditModal(false);
                                setEditMessage('');
                                setErrorMessage('');
                                setSelectedDevice(null);
                            }}>Cancel</button>
                        </div>
                    </div>
                </div>
            )}

            {/* Delete Device Modal */}
            {showDeleteModal && (
                <div className="modal">
                    <div className="modal-content">
                        <h2>Delete GPS Device</h2>
                        {loadingDevices ? (
                            <div className="loading-spinner-modal">
                                <img src={spinnerIcon} alt="Loading..." />
                            </div>
                        ) : (
                            <div className="form-group">
                                <label htmlFor="selectDeviceDelete">Select Device to Delete</label>
                                <select
                                    id="selectDeviceDelete"
                                    value={selectedDevice ? selectedDevice.device_eui.data.join(',') : ''}
                                    onChange={(e) => {
                                        const selectedEUI = e.target.value;
                                        const device = devicesData.find(
                                            (d) => d.device_eui.data.join(',') === selectedEUI
                                        );
                                        setSelectedDevice(device);
                                    }}
                                >
                                    <option value="" disabled>Select a device</option>
                                    {devicesData.map((device) => (
                                        <option
                                            key={device.device_eui.data.join(',')}
                                            value={device.device_eui.data.join(',')}
                                        >
                                            {device.device_name}
                                        </option>
                                    ))}
                                </select>
                            </div>
                        )}
                        {deleteMessage && (
                            <p style={{ color: 'green' }}>{deleteMessage}</p>
                        )}
                        {errorMessage && (
                            <p style={{ color: 'red' }}>{errorMessage}</p>
                        )}
                        <div className="modal-actions">
                            <button className="delete-button" onClick={handleDelete}>Delete Device</button>
                            <button className="cancel-button" onClick={closeDeleteModal}>Cancel</button>
                        </div>
                    </div>
                </div>
            )}
        </div>
    );
}