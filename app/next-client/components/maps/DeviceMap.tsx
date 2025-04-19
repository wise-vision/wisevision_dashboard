'use client';

import { useEffect, useState } from 'react';
import { MapContainer, TileLayer, Marker, Popup, Circle } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import { GpsDevice } from '../../types/ros2-types';

interface DeviceMapProps {
  devices: GpsDevice[];
  height?: string;
  width?: string;
  center?: [number, number];
  zoom?: number;
  showTrails?: boolean;
}

const DeviceMap: React.FC<DeviceMapProps> = ({
  devices,
  height = '600px',
  width = '100%',
  center = [51.505, -0.09], // Default to London if no devices
  zoom = 13,
  showTrails = false,
}) => {
  const [mapCenter, setMapCenter] = useState<[number, number]>(center);
  const [mapZoom, setMapZoom] = useState<number>(zoom);

  // Fix for Leaflet's default icon paths
  useEffect(() => {
    // This is needed because Leaflet's default marker icons reference assets
    // that might not be available in the build directory
    delete (L.Icon.Default.prototype as any)._getIconUrl;
    L.Icon.Default.mergeOptions({
      iconRetinaUrl: '/assets/marker-icon-2x.png',
      iconUrl: '/assets/marker-icon.png',
      shadowUrl: '/assets/marker-shadow.png',
    });
  }, []);

  // Set map center based on devices if available
  useEffect(() => {
    if (devices.length > 0) {
      // Use the first device as center
      const firstDevice = devices[0];
      setMapCenter([firstDevice.nav_value.latitude, firstDevice.nav_value.longitude]);
    }
  }, [devices]);

  // Create custom icons for different device types
  const robotIcon = new L.Icon({
    iconUrl: '/assets/robot-marker.png',
    iconSize: [32, 32],
    iconAnchor: [16, 32],
    popupAnchor: [0, -32]
  });

  const lorawanIcon = new L.Icon({
    iconUrl: '/assets/lorawan-marker.png',
    iconSize: [32, 32],
    iconAnchor: [16, 32],
    popupAnchor: [0, -32]
  });

  const droneIcon = new L.Icon({
    iconUrl: '/assets/drone-marker.png',
    iconSize: [32, 32],
    iconAnchor: [16, 32],
    popupAnchor: [0, -32]
  });

  // Get appropriate icon based on device name
  const getDeviceIcon = (deviceName: string) => {
    if (deviceName.toLowerCase().includes('robot')) {
      return robotIcon;
    } else if (deviceName.toLowerCase().includes('lorawan')) {
      return lorawanIcon;
    } else if (deviceName.toLowerCase().includes('drone')) {
      return droneIcon;
    } else {
      return new L.Icon.Default();
    }
  };

  // Get appropriate color based on device status (moving/not moving)
  const getDeviceColor = (isMoving: boolean) => {
    return isMoving ? '#4ade80' : '#3b82f6';
  };

  return (
    <div style={{ height, width }}>
      <MapContainer
        center={mapCenter}
        zoom={mapZoom}
        style={{ height: '100%', width: '100%', borderRadius: '0.5rem' }}
      >
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />

        {devices.map((device) => (
          <div key={device.device_eui.data.join('-')}>
            <Marker
              position={[device.nav_value.latitude, device.nav_value.longitude]}
              icon={getDeviceIcon(device.device_name)}
            >
              <Popup>
                <div className="text-sm">
                  <p className="font-bold text-gray-900">{device.device_name}</p>
                  <p className="text-gray-700">
                    Lat: {device.nav_value.latitude.toFixed(6)}, 
                    Lon: {device.nav_value.longitude.toFixed(6)}
                  </p>
                  <p className="text-gray-700">
                    Altitude: {device.nav_value.altitude.toFixed(2)}m
                  </p>
                  <p className="text-gray-700">
                    Status: {device.is_moving ? 'Moving' : 'Stationary'}
                  </p>
                </div>
              </Popup>
            </Marker>
            
            {/* Add a circle to indicate the area around the device */}
            <Circle 
              center={[device.nav_value.latitude, device.nav_value.longitude]}
              radius={100} // 100 meter radius
              pathOptions={{
                color: getDeviceColor(device.is_moving),
                fillColor: getDeviceColor(device.is_moving),
                fillOpacity: 0.2
              }}
            />
          </div>
        ))}
      </MapContainer>
    </div>
  );
};

export default DeviceMap;