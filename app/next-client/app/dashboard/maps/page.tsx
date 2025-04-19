'use client';

import dynamic from 'next/dynamic';
import { useDevices } from '../../../hooks/useDevices';
import { MapIcon, ArrowPathIcon } from '@heroicons/react/24/outline';

// Import the DeviceMap component dynamically to prevent SSR issues with Leaflet
const DeviceMap = dynamic(
  () => import('../../../components/maps/DeviceMap'),
  { 
    ssr: false,
    loading: () => (
      <div className="h-[600px] w-full flex items-center justify-center bg-gray-100 dark:bg-gray-800 rounded-lg">
        <div className="text-center">
          <div className="animate-spin h-8 w-8 border-4 border-primary-500 border-t-transparent rounded-full mx-auto mb-2"></div>
          <p className="text-gray-600 dark:text-gray-300">Loading map...</p>
        </div>
      </div>
    )
  }
);

export default function MapsPage() {
  // Use our custom hook to manage device data
  const {
    devices,
    isLoading,
    error,
    refresh,
    autoRefresh,
    setAutoRefresh,
    refreshInterval,
    setRefreshInterval,
    stats
  } = useDevices();

  // Handle refresh interval change
  const handleRefreshIntervalChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    setRefreshInterval(parseInt(event.target.value, 10));
  };

  return (
    <div className="py-6">
      <div className="flex flex-col md:flex-row justify-between items-start mb-6">
        <div>
          <h1 className="text-2xl font-bold text-gray-900 dark:text-white flex items-center">
            <MapIcon className="h-6 w-6 mr-2 text-primary-500" />
            Device Locations
          </h1>
          <p className="text-gray-500 dark:text-gray-400 mt-1">
            Monitor the real-time locations of all your ROS2 systems and LoRaWAN devices
          </p>
        </div>
        
        <div className="mt-4 md:mt-0 flex flex-col sm:flex-row gap-4">
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
            className="btn-primary flex items-center"
            disabled={isLoading}
          >
            <ArrowPathIcon className={`h-4 w-4 mr-1 ${isLoading ? 'animate-spin' : ''}`} />
            {isLoading ? 'Refreshing...' : 'Refresh Now'}
          </button>
        </div>
      </div>
      
      {error && (
        <div className="bg-red-50 dark:bg-red-900/20 border border-red-200 dark:border-red-800 text-red-800 dark:text-red-200 px-4 py-3 rounded-md mb-6">
          {error}
        </div>
      )}
      
      {/* Device Statistics */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
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
          <h3 className="text-lg font-medium text-gray-700 dark:text-gray-300 mb-1">Stationary Devices</h3>
          <p className="text-3xl font-bold text-blue-600 dark:text-blue-400">
            {isLoading ? '...' : stats.stationary}
          </p>
        </div>
      </div>
      
      {/* Main Map */}
      <div className="card p-0 overflow-hidden">
        {devices.length === 0 && !isLoading ? (
          <div className="h-[600px] w-full flex items-center justify-center bg-gray-100 dark:bg-gray-800">
            <div className="text-center">
              <p className="text-gray-600 dark:text-gray-300">No device data available</p>
            </div>
          </div>
        ) : (
          <DeviceMap devices={devices} height="600px" />
        )}
      </div>
      
      {/* Device List */}
      <div className="mt-6">
        <h2 className="text-lg font-bold text-gray-900 dark:text-white mb-4">Device List</h2>
        <div className="bg-white dark:bg-gray-800 shadow overflow-hidden rounded-lg">
          <table className="min-w-full divide-y divide-gray-200 dark:divide-gray-700">
            <thead className="bg-gray-50 dark:bg-gray-900">
              <tr>
                <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                  Device Name
                </th>
                <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                  Location
                </th>
                <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                  Altitude
                </th>
                <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                  Status
                </th>
                <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                  Device EUI
                </th>
              </tr>
            </thead>
            <tbody className="bg-white dark:bg-gray-800 divide-y divide-gray-200 dark:divide-gray-700">
              {isLoading ? (
                <tr>
                  <td colSpan={5} className="px-6 py-4 text-center text-sm text-gray-500 dark:text-gray-400">
                    Loading...
                  </td>
                </tr>
              ) : devices.length === 0 ? (
                <tr>
                  <td colSpan={5} className="px-6 py-4 text-center text-sm text-gray-500 dark:text-gray-400">
                    No devices found
                  </td>
                </tr>
              ) : (
                devices.map((device) => (
                  <tr key={device.device_eui.data.join('-')}>
                    <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900 dark:text-white">
                      {device.device_name}
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                      {device.nav_value.latitude.toFixed(6)}, {device.nav_value.longitude.toFixed(6)}
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                      {device.nav_value.altitude.toFixed(2)}m
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm">
                      <span className={`px-2 inline-flex text-xs leading-5 font-semibold rounded-full ${
                        device.is_moving ? 'bg-green-100 text-green-800 dark:bg-green-900/30 dark:text-green-300' : 'bg-blue-100 text-blue-800 dark:bg-blue-900/30 dark:text-blue-300'
                      }`}>
                        {device.is_moving ? 'Moving' : 'Stationary'}
                      </span>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                      {device.device_eui.data.join(':')}
                    </td>
                  </tr>
                ))
              )}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );
}