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
import PropTypes from 'prop-types';
import { MapContainer, TileLayer, Marker, Popup } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';
import './ChartStyles.css';

// Fix for Leaflet marker icons
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: 'https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon-2x.png',
  iconUrl: 'https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon.png',
  shadowUrl: 'https://unpkg.com/leaflet@1.7.1/dist/images/marker-shadow.png',
});

// Custom marker icon for devices
const deviceIcon = new L.Icon({
  iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-blue.png',
  shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
  iconSize: [25, 41],
  iconAnchor: [12, 41],
  popupAnchor: [1, -34],
  shadowSize: [41, 41]
});

const GpsChart = ({ isDarkMode = false }) => {
  const [devices, setDevices] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [lastUpdate, setLastUpdate] = useState('');
  
  // Default center location (update with your default location)
  const [mapCenter, setMapCenter] = useState([50.049683, 19.944544]); // Krakow as default
  const [mapZoom, setMapZoom] = useState(13);

  useEffect(() => {
    fetchGpsData();
    // Poll for new GPS data every 5 seconds
    const intervalId = setInterval(fetchGpsData, 5000);
    
    return () => clearInterval(intervalId);
  }, []);

  const fetchGpsData = async () => {
    try {
      const response = await fetch(`${process.env.REACT_APP_API_BASE_URL || ''}/api/topic_echo_gps_devices`);
      
      if (!response.ok) {
        throw new Error(`Error fetching GPS data: ${response.statusText}`);
      }
      
      const data = await response.json();
      
      if (data && data.message && data.message.devices) {
        const formattedDevices = data.message.devices.map(device => ({
          id: device.device_eui ? bufferToString(device.device_eui.data) : 'unknown',
          name: device.device_name || 'Unnamed Device',
          position: {
            lat: device.nav_value.latitude,
            lng: device.nav_value.longitude,
            alt: device.nav_value.altitude,
          },
          isMoving: device.is_moving
        }));
        
        setDevices(formattedDevices);
        
        // If we have devices, center the map on the first device
        if (formattedDevices.length > 0) {
          const firstDevice = formattedDevices[0];
          setMapCenter([firstDevice.position.lat, firstDevice.position.lng]);
        }
      }
      
      setLastUpdate(new Date().toLocaleTimeString());
      setError(null);
    } catch (err) {
      console.error('Error fetching GPS data:', err);
      setError(`Failed to load GPS data: ${err.message}`);
    } finally {
      setLoading(false);
    }
  };
  
  // Helper function to convert buffer array to hex string
  const bufferToString = (buffer) => {
    if (!buffer || !Array.isArray(buffer)) return 'invalid';
    return buffer.map(b => b.toString(16).padStart(2, '0')).join('');
  };

  // Map style for dark mode
  const mapStyles = {
    dark: 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',
    light: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
  };

  const mapAttribution = '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors';

  return (
    <div className={`gps-chart-container ${isDarkMode ? 'dark' : ''}`}>
      {loading && !devices.length ? (
        <div className="chart-loading">
          <div className="spinner"></div>
          <p>Loading GPS data...</p>
        </div>
      ) : error && !devices.length ? (
        <div className="chart-error">
          <p>{error}</p>
          <button onClick={fetchGpsData} className="retry-button">
            Retry
          </button>
        </div>
      ) : (
        <>
          <div className="gps-chart-wrapper">
            <MapContainer 
              center={mapCenter} 
              zoom={mapZoom} 
              className="gps-map"
            >
              <TileLayer
                url={isDarkMode ? mapStyles.dark : mapStyles.light}
                attribution={mapAttribution}
              />
              
              {devices.map(device => (
                <Marker 
                  key={device.id} 
                  position={[device.position.lat, device.position.lng]}
                  icon={deviceIcon}
                >
                  <Popup className={isDarkMode ? 'dark-popup' : ''}>
                    <div>
                      <h3>{device.name}</h3>
                      <p>ID: {device.id}</p>
                      <p>Latitude: {device.position.lat.toFixed(6)}</p>
                      <p>Longitude: {device.position.lng.toFixed(6)}</p>
                      <p>Altitude: {device.position.alt.toFixed(2)} m</p>
                      <p>Status: {device.isMoving ? 'Moving' : 'Stationary'}</p>
                    </div>
                  </Popup>
                </Marker>
              ))}
            </MapContainer>
          </div>
          
          <div className="chart-footer">
            <div className="device-counter">
              <span>{devices.length} Device{devices.length !== 1 ? 's' : ''}</span>
            </div>
            <span className="chart-updated">Updated: {lastUpdate}</span>
          </div>
        </>
      )}
    </div>
  );
};

GpsChart.propTypes = {
  isDarkMode: PropTypes.bool
};

export default GpsChart;
