'use client';

import { useState, useEffect } from 'react';
import { BellIcon, ExclamationCircleIcon, CheckCircleIcon } from '@heroicons/react/outline';

export default function AlertsPage() {
  const [alerts, setAlerts] = useState([]);
  const [loading, setLoading] = useState(true);
  const [filter, setFilter] = useState('all');

  useEffect(() => {
    // This would be an API call in a real application
    const fetchAlerts = async () => {
      setLoading(true);
      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 800));
      
      // Sample data for demonstration
      const mockAlerts = [
        {
          id: '1',
          title: 'High Temperature Alert',
          message: 'Robot-1 temperature exceeds 85°C',
          severity: 'critical',
          timestamp: new Date(Date.now() - 3600000).toISOString(),
          source: 'temperature_sensor',
          acknowledged: false
        },
        {
          id: '2',
          title: 'Battery Low',
          message: 'Robot-3 battery level below 15%',
          severity: 'warning',
          timestamp: new Date(Date.now() - 7200000).toISOString(),
          source: 'power_system',
          acknowledged: false
        },
        {
          id: '3',
          title: 'Connection Lost',
          message: 'Lost connection with LoRaWAN sensor node 5',
          severity: 'info',
          timestamp: new Date(Date.now() - 86400000).toISOString(),
          source: 'connectivity',
          acknowledged: true
        },
        {
          id: '4',
          title: 'Motion Detected',
          message: 'Unexpected motion detected in zone B during maintenance hours',
          severity: 'warning',
          timestamp: new Date(Date.now() - 43200000).toISOString(),
          source: 'motion_sensor',
          acknowledged: true
        },
        {
          id: '5',
          title: 'System Update Required',
          message: 'Security update available for Robot-2',
          severity: 'info',
          timestamp: new Date(Date.now() - 259200000).toISOString(),
          source: 'system',
          acknowledged: false
        }
      ];
      
      setAlerts(mockAlerts);
      setLoading(false);
    };

    fetchAlerts();
  }, []);

  const filteredAlerts = alerts.filter(alert => {
    if (filter === 'all') return true;
    if (filter === 'critical') return alert.severity === 'critical';
    if (filter === 'warning') return alert.severity === 'warning';
    if (filter === 'info') return alert.severity === 'info';
    if (filter === 'unacknowledged') return !alert.acknowledged;
    return true;
  });

  const formatDate = (dateString) => {
    const date = new Date(dateString);
    return date.toLocaleString();
  };

  const getSeverityColor = (severity) => {
    switch (severity) {
      case 'critical': 
        return 'bg-red-600';
      case 'warning':
        return 'bg-yellow-500';
      case 'info':
        return 'bg-blue-500';
      default:
        return 'bg-gray-500';
    }
  };

  const acknowledgeAlert = (id) => {
    setAlerts(alerts.map(alert => 
      alert.id === id ? { ...alert, acknowledged: true } : alert
    ));
  };

  return (
    <div className="py-6">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 md:px-8">
        <div className="flex flex-col md:flex-row md:items-center md:justify-between mb-8">
          <div>
            <h1 className="text-2xl font-semibold text-gray-900 dark:text-white">System Alerts</h1>
            <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
              Monitor and manage alerts from all connected devices
            </p>
          </div>
          <div className="mt-4 md:mt-0">
            <div className="inline-flex rounded-md shadow">
              <button className="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500">
                <BellIcon className="h-5 w-5 mr-2" />
                Configure Alerts
              </button>
            </div>
          </div>
        </div>
        
        {/* Filter controls */}
        <div className="bg-white dark:bg-gray-800 shadow rounded-lg p-4 mb-6">
          <div className="flex flex-wrap items-center">
            <span className="text-sm font-medium text-gray-700 dark:text-gray-300 mr-4">Filter by:</span>
            <div className="flex flex-wrap gap-2">
              <button
                onClick={() => setFilter('all')}
                className={`px-3 py-1 rounded-full text-sm ${filter === 'all' ? 
                  'bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-200' : 
                  'bg-gray-100 text-gray-800 dark:bg-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-600'}`}
              >
                All
              </button>
              <button
                onClick={() => setFilter('critical')}
                className={`px-3 py-1 rounded-full text-sm ${filter === 'critical' ? 
                  'bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-200' : 
                  'bg-gray-100 text-gray-800 dark:bg-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-600'}`}
              >
                Critical
              </button>
              <button
                onClick={() => setFilter('warning')}
                className={`px-3 py-1 rounded-full text-sm ${filter === 'warning' ? 
                  'bg-yellow-100 text-yellow-800 dark:bg-yellow-900 dark:text-yellow-200' : 
                  'bg-gray-100 text-gray-800 dark:bg-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-600'}`}
              >
                Warning
              </button>
              <button
                onClick={() => setFilter('info')}
                className={`px-3 py-1 rounded-full text-sm ${filter === 'info' ? 
                  'bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-200' : 
                  'bg-gray-100 text-gray-800 dark:bg-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-600'}`}
              >
                Info
              </button>
              <button
                onClick={() => setFilter('unacknowledged')}
                className={`px-3 py-1 rounded-full text-sm ${filter === 'unacknowledged' ? 
                  'bg-purple-100 text-purple-800 dark:bg-purple-900 dark:text-purple-200' : 
                  'bg-gray-100 text-gray-800 dark:bg-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-600'}`}
              >
                Unacknowledged
              </button>
            </div>
          </div>
        </div>
        
        {/* Alert list */}
        <div className="bg-white dark:bg-gray-800 shadow rounded-lg">
          {loading ? (
            <div className="flex justify-center items-center h-64">
              <div className="animate-spin rounded-full h-10 w-10 border-t-2 border-b-2 border-blue-500"></div>
            </div>
          ) : filteredAlerts.length > 0 ? (
            <div className="overflow-x-auto">
              <table className="min-w-full divide-y divide-gray-200 dark:divide-gray-700">
                <thead className="bg-gray-50 dark:bg-gray-700">
                  <tr>
                    <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                      Severity
                    </th>
                    <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                      Alert
                    </th>
                    <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                      Source
                    </th>
                    <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                      Time
                    </th>
                    <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                      Status
                    </th>
                    <th scope="col" className="px-6 py-3 text-right text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                      Actions
                    </th>
                  </tr>
                </thead>
                <tbody className="bg-white dark:bg-gray-800 divide-y divide-gray-200 dark:divide-gray-700">
                  {filteredAlerts.map((alert) => (
                    <tr key={alert.id} className={!alert.acknowledged ? 'bg-gray-50 dark:bg-gray-700' : ''}>
                      <td className="px-6 py-4 whitespace-nowrap">
                        <span className={`h-2.5 w-2.5 rounded-full ${getSeverityColor(alert.severity)} inline-flex`}></span>
                        <span className="ml-2 text-sm font-medium text-gray-900 dark:text-gray-200 capitalize">
                          {alert.severity}
                        </span>
                      </td>
                      <td className="px-6 py-4">
                        <div className="text-sm font-medium text-gray-900 dark:text-white">{alert.title}</div>
                        <div className="text-sm text-gray-500 dark:text-gray-400">{alert.message}</div>
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                        {alert.source.replace('_', ' ')}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                        {formatDate(alert.timestamp)}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap">
                        {alert.acknowledged ? (
                          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800 dark:bg-green-800 dark:text-green-100">
                            <CheckCircleIcon className="h-4 w-4 mr-1" />
                            Acknowledged
                          </span>
                        ) : (
                          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-red-100 text-red-800 dark:bg-red-800 dark:text-red-100">
                            <ExclamationCircleIcon className="h-4 w-4 mr-1" />
                            Unacknowledged
                          </span>
                        )}
                      </td>
                      <td className="px-6 py-4 whitespace-nowrap text-right text-sm font-medium">
                        {!alert.acknowledged && (
                          <button 
                            onClick={() => acknowledgeAlert(alert.id)}
                            className="text-blue-600 hover:text-blue-800 dark:text-blue-400 dark:hover:text-blue-300"
                          >
                            Acknowledge
                          </button>
                        )}
                        <button className="text-blue-600 hover:text-blue-800 dark:text-blue-400 dark:hover:text-blue-300 ml-4">
                          Details
                        </button>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          ) : (
            <div className="flex flex-col items-center justify-center h-64">
              <BellIcon className="h-12 w-12 text-gray-400" />
              <h3 className="mt-2 text-sm font-medium text-gray-900 dark:text-gray-200">No alerts found</h3>
              <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                {filter === 'all' ? 'No alerts currently in the system.' : 'No alerts matching the selected filter.'}
              </p>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}