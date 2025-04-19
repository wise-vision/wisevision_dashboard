'use client';

import { useState, useEffect } from 'react';
import { ServerIcon, PlusCircleIcon, ArrowPathIcon } from '@heroicons/react/24/outline';
import { useDevices } from '../../../hooks/useDevices';

type DeviceType = 'all' | 'robot' | 'lorawan' | 'drone' | 'other';
type DeviceStatus = 'all' | 'online' | 'offline' | 'warning';

interface DeviceFilterProps {
  deviceType: DeviceType;
  setDeviceType: (type: DeviceType) => void;
  deviceStatus: DeviceStatus;
  setDeviceStatus: (status: DeviceStatus) => void;
}

const DeviceFilters = ({ deviceType, setDeviceType, deviceStatus, setDeviceStatus }: DeviceFilterProps) => {
  return (
    <div className="flex flex-col md:flex-row gap-4 mb-6">
      <div className="flex flex-col md:flex-row gap-2 md:items-center">
        <label htmlFor="device-type" className="text-sm font-medium text-gray-700 dark:text-gray-300">
          Device Type:
        </label>
        <div className="flex flex-wrap gap-2">
          {(['all', 'robot', 'lorawan', 'drone', 'other'] as DeviceType[]).map((type) => (
            <button
              key={type}
              onClick={() => setDeviceType(type)}
              className={`px-3 py-1 text-sm rounded-full transition-colors ${
                deviceType === type
                  ? 'bg-primary-500 text-white'
                  : 'bg-gray-100 dark:bg-gray-800 text-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-700'
              }`}
            >
              {type.charAt(0).toUpperCase() + type.slice(1)}
            </button>
          ))}
        </div>
      </div>
      
      <div className="flex flex-col md:flex-row gap-2 md:items-center md:ml-6">
        <label htmlFor="device-status" className="text-sm font-medium text-gray-700 dark:text-gray-300">
          Status:
        </label>
        <div className="flex flex-wrap gap-2">
          {(['all', 'online', 'offline', 'warning'] as DeviceStatus[]).map((status) => (
            <button
              key={status}
              onClick={() => setDeviceStatus(status)}
              className={`px-3 py-1 text-sm rounded-full transition-colors ${
                deviceStatus === status
                  ? 'bg-primary-500 text-white'
                  : 'bg-gray-100 dark:bg-gray-800 text-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-700'
              }`}
            >
              {status.charAt(0).toUpperCase() + status.slice(1)}
            </button>
          ))}
        </div>
      </div>
    </div>
  );
};

export default function DevicesPage() {
  const {
    devices,
    isLoading,
    error,
    refresh,
    stats,
    autoRefresh,
    setAutoRefresh,
    refreshInterval,
    setRefreshInterval
  } = useDevices();

  const [deviceType, setDeviceType] = useState<DeviceType>('all');
  const [deviceStatus, setDeviceStatus] = useState<DeviceStatus>('all');
  const [searchQuery, setSearchQuery] = useState('');
  const [showAddDeviceModal, setShowAddDeviceModal] = useState(false);
  const [selectedDeviceId, setSelectedDeviceId] = useState<string | null>(null);
  
  // Filter devices based on criteria
  const filteredDevices = devices.filter(device => {
    // Type filter
    if (deviceType !== 'all') {
      const type = deviceType.toLowerCase();
      if (!device.device_name.toLowerCase().includes(type)) {
        return false;
      }
    }
    
    // Status filter
    if (deviceStatus !== 'all') {
      if (deviceStatus === 'online' && !device.is_moving) return false;
      if (deviceStatus === 'offline' && device.is_moving) return false;
    }
    
    // Search query
    if (searchQuery) {
      const query = searchQuery.toLowerCase();
      return device.device_name.toLowerCase().includes(query) ||
             device.device_eui.data.join(':').toLowerCase().includes(query);
    }
    
    return true;
  });

  // Handle refresh interval change
  const handleRefreshIntervalChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    setRefreshInterval(parseInt(event.target.value, 10));
  };
  
  // Get device status class
  const getStatusClass = (isMoving: boolean) => {
    if (isMoving) return 'bg-green-100 text-green-800 dark:bg-green-900/30 dark:text-green-300';
    return 'bg-blue-100 text-blue-800 dark:bg-blue-900/30 dark:text-blue-300';
  };
  
  // Get device status text
  const getStatusText = (isMoving: boolean) => {
    return isMoving ? 'Active' : 'Idle';
  };
  
  // Get device type based on name
  const getDeviceType = (deviceName: string) => {
    if (deviceName.toLowerCase().includes('robot')) return 'Robot';
    if (deviceName.toLowerCase().includes('lorawan')) return 'LoRaWAN';
    if (deviceName.toLowerCase().includes('drone')) return 'Drone';
    return 'Other';
  };

  return (
    <div className="py-6">
      <div className="flex flex-col md:flex-row justify-between items-start mb-6">
        <div>
          <h1 className="text-2xl font-bold text-gray-900 dark:text-white flex items-center">
            <ServerIcon className="h-6 w-6 mr-2 text-primary-500" />
            Device Management
          </h1>
          <p className="text-gray-500 dark:text-gray-400 mt-1">
            Manage and monitor all your ROS2 systems and LoRaWAN devices
          </p>
        </div>
        
        <div className="mt-4 md:mt-0 flex flex-wrap gap-4">
          <div className="relative">
            <input
              type="text"
              placeholder="Search devices..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              className="input-field pl-10 pr-4 py-2"
            />
            <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none">
              <svg className="h-5 w-5 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" />
              </svg>
            </div>
          </div>
          
          <div className="flex items-center">
            <label htmlFor="refreshInterval" className="mr-2 text-sm text-gray-700 dark:text-gray-300">
              Refresh:
            </label>
            <select
              id="refreshInterval"
              value={refreshInterval}
              onChange={handleRefreshIntervalChange}
              className="input-field text-sm py-1 px-2 w-24"
            >
              <option value="5">5s</option>
              <option value="10">10s</option>
              <option value="30">30s</option>
              <option value="60">60s</option>
            </select>
          </div>
          
          <div className="flex items-center">
            <label className="inline-flex items-center cursor-pointer">
              <input 
                type="checkbox" 
                checked={autoRefresh}
                onChange={() => setAutoRefresh(!autoRefresh)}
                className="sr-only peer"
              />
              <div className="relative w-11 h-6 bg-gray-200 peer-focus:outline-none peer-focus:ring-4 peer-focus:ring-primary-300 dark:peer-focus:ring-primary-800 rounded-full peer dark:bg-gray-700 peer-checked:after:translate-x-full rtl:peer-checked:after:-translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-[2px] after:start-[2px] after:bg-white after:border-gray-300 after:border after:rounded-full after:h-5 after:w-5 after:transition-all dark:border-gray-600 peer-checked:bg-primary-500"></div>
              <span className="ms-3 text-sm font-medium text-gray-700 dark:text-gray-300">Auto-refresh</span>
            </label>
          </div>
          
          <button
            onClick={refresh}
            disabled={isLoading}
            className="btn-secondary flex items-center"
          >
            <ArrowPathIcon className={`h-4 w-4 mr-1 ${isLoading ? 'animate-spin' : ''}`} />
            {isLoading ? 'Refreshing...' : 'Refresh'}
          </button>
          
          <button
            onClick={() => setShowAddDeviceModal(true)}
            className="btn-primary flex items-center"
          >
            <PlusCircleIcon className="h-4 w-4 mr-1" />
            Add Device
          </button>
        </div>
      </div>
      
      {error && (
        <div className="bg-red-50 dark:bg-red-900/20 border border-red-200 dark:border-red-800 text-red-800 dark:text-red-200 px-4 py-3 rounded-md mb-6">
          {error}
        </div>
      )}
      
      {/* Device Stats */}
      <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4 mb-6">
        <div className="card">
          <h3 className="text-lg font-medium text-gray-700 dark:text-gray-300 mb-1">Total Devices</h3>
          <p className="text-3xl font-bold text-gray-900 dark:text-white">
            {isLoading ? '...' : stats.total}
          </p>
        </div>
        <div className="card">
          <h3 className="text-lg font-medium text-gray-700 dark:text-gray-300 mb-1">Active Devices</h3>
          <p className="text-3xl font-bold text-green-600 dark:text-green-400">
            {isLoading ? '...' : stats.active}
          </p>
        </div>
        <div className="card">
          <h3 className="text-lg font-medium text-gray-700 dark:text-gray-300 mb-1">Idle Devices</h3>
          <p className="text-3xl font-bold text-blue-600 dark:text-blue-400">
            {isLoading ? '...' : stats.stationary}
          </p>
        </div>
        <div className="card flex items-center justify-center">
          <button 
            onClick={() => setShowAddDeviceModal(true)}
            className="btn-primary w-full flex items-center justify-center"
          >
            <PlusCircleIcon className="h-5 w-5 mr-2" />
            Add New Device
          </button>
        </div>
      </div>
      
      {/* Filters */}
      <DeviceFilters 
        deviceType={deviceType}
        setDeviceType={setDeviceType}
        deviceStatus={deviceStatus}
        setDeviceStatus={setDeviceStatus}
      />
      
      {/* Devices List */}
      <div className="bg-white dark:bg-gray-800 shadow overflow-hidden rounded-lg">
        <div className="overflow-x-auto">
          <table className="min-w-full divide-y divide-gray-200 dark:divide-gray-700">
            <thead className="bg-gray-50 dark:bg-gray-900">
              <tr>
                <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                  Device Name
                </th>
                <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                  Type
                </th>
                <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                  Location
                </th>
                <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                  Status
                </th>
                <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                  Device EUI
                </th>
                <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                  Actions
                </th>
              </tr>
            </thead>
            <tbody className="bg-white dark:bg-gray-800 divide-y divide-gray-200 dark:divide-gray-700">
              {isLoading ? (
                <tr>
                  <td colSpan={6} className="px-6 py-4 text-center whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                    <div className="flex justify-center items-center">
                      <div className="animate-spin h-5 w-5 border-2 border-primary-500 border-t-transparent rounded-full inline-block mr-2"></div>
                      Loading devices...
                    </div>
                  </td>
                </tr>
              ) : filteredDevices.length === 0 ? (
                <tr>
                  <td colSpan={6} className="px-6 py-4 text-center whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                    {deviceType !== 'all' || deviceStatus !== 'all' || searchQuery ? 'No devices match the current filters' : 'No devices found'}
                  </td>
                </tr>
              ) : (
                filteredDevices.map((device) => (
                  <tr key={device.device_eui.data.join('-')} className="hover:bg-gray-50 dark:hover:bg-gray-900/30">
                    <td className="px-6 py-4 whitespace-nowrap">
                      <div className="flex items-center">
                        <div className="flex-shrink-0 h-10 w-10 flex items-center justify-center rounded-full bg-gray-100 dark:bg-gray-800">
                          <ServerIcon className="h-5 w-5 text-gray-600 dark:text-gray-300" />
                        </div>
                        <div className="ml-4">
                          <div className="text-sm font-medium text-gray-900 dark:text-white">
                            {device.device_name}
                          </div>
                        </div>
                      </div>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap">
                      <span className="text-sm text-gray-700 dark:text-gray-300">
                        {getDeviceType(device.device_name)}
                      </span>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-700 dark:text-gray-300">
                      {device.nav_value.latitude.toFixed(6)}, {device.nav_value.longitude.toFixed(6)}
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap">
                      <span className={`px-3 py-1 inline-flex text-xs leading-5 font-semibold rounded-full ${getStatusClass(device.is_moving)}`}>
                        {getStatusText(device.is_moving)}
                      </span>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-700 dark:text-gray-300">
                      <span className="font-mono">
                        {device.device_eui.data.join(':')}
                      </span>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm space-x-2">
                      <button 
                        onClick={() => setSelectedDeviceId(device.device_eui.data.join('-'))}
                        className="text-primary-500 hover:text-primary-700 transition-colors"
                      >
                        View Details
                      </button>
                    </td>
                  </tr>
                ))
              )}
            </tbody>
          </table>
        </div>
        {filteredDevices.length > 0 && (
          <div className="px-6 py-3 bg-white dark:bg-gray-800 border-t dark:border-gray-700 text-right text-sm">
            Showing {filteredDevices.length} of {devices.length} devices
          </div>
        )}
      </div>

      {/* Add Device Modal (placeholder - would be implemented with a proper modal component) */}
      {showAddDeviceModal && (
        <div className="fixed inset-0 bg-black/50 flex items-center justify-center z-50">
          <div className="bg-white dark:bg-gray-800 rounded-lg shadow-lg max-w-lg w-full p-6">
            <h3 className="text-lg font-medium text-gray-900 dark:text-white mb-4">Add New Device</h3>
            <p className="text-gray-500 dark:text-gray-400 mb-4">
              This is a placeholder for the Add Device modal. In a real implementation, this would contain a form to add a new ROS2 or LoRaWAN device.
            </p>
            <div className="flex justify-end">
              <button 
                onClick={() => setShowAddDeviceModal(false)}
                className="btn-secondary mr-2"
              >
                Cancel
              </button>
              <button className="btn-primary">
                Add Device
              </button>
            </div>
          </div>
        </div>
      )}
      
      {/* Device Details Modal (placeholder) */}
      {selectedDeviceId && (
        <div className="fixed inset-0 bg-black/50 flex items-center justify-center z-50">
          <div className="bg-white dark:bg-gray-800 rounded-lg shadow-lg max-w-2xl w-full p-6">
            <h3 className="text-lg font-medium text-gray-900 dark:text-white mb-4">Device Details</h3>
            <p className="text-gray-500 dark:text-gray-400 mb-4">
              This is a placeholder for the Device Details modal. In a real implementation, this would show detailed information about the selected device.
            </p>
            <p className="text-gray-700 dark:text-gray-300 mb-4">
              Selected Device ID: <span className="font-mono">{selectedDeviceId}</span>
            </p>
            <div className="flex justify-end">
              <button 
                onClick={() => setSelectedDeviceId(null)}
                className="btn-primary"
              >
                Close
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}