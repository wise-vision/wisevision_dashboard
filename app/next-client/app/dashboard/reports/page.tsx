'use client';

import { useState } from 'react';
import { 
  DocumentReportIcon, 
  DownloadIcon, 
  PlusIcon, 
  CalendarIcon,
  ShareIcon, 
  ChartBarIcon, 
  TableIcon,
  ClockIcon
} from '@heroicons/react/outline';

export default function ReportsPage() {
  const [selectedReport, setSelectedReport] = useState<string | null>(null);
  const [selectedFormat, setSelectedFormat] = useState('pdf');
  const [dateRange, setDateRange] = useState('last30days');

  const reports = [
    { 
      id: 'device-status',
      name: 'Device Status Report', 
      description: 'Overview of all device statuses, uptime, and health metrics',
      lastGenerated: '2025-04-10T08:30:00Z',
      icon: ServerIcon,
      formats: ['pdf', 'csv', 'json']
    },
    { 
      id: 'sensor-analytics',
      name: 'Sensor Analytics', 
      description: 'Detailed analysis of sensor data trends and anomalies',
      lastGenerated: '2025-04-12T15:45:00Z',
      icon: ChartBarIcon,
      formats: ['pdf', 'xlsx', 'json']
    },
    { 
      id: 'audit-log',
      name: 'System Audit Log', 
      description: 'Comprehensive log of system events and user actions',
      lastGenerated: '2025-04-13T01:15:00Z',
      icon: ClockIcon,
      formats: ['pdf', 'txt', 'json']
    },
    { 
      id: 'location-history',
      name: 'Location History', 
      description: 'Geographical tracking data for all mobile devices',
      lastGenerated: '2025-04-11T19:20:00Z',
      icon: MapIcon,
      formats: ['pdf', 'kml', 'geojson']
    },
    { 
      id: 'performance-metrics',
      name: 'Performance Metrics', 
      description: 'Key performance indicators and system benchmarks',
      lastGenerated: '2025-04-08T12:10:00Z',
      icon: TableIcon,
      formats: ['pdf', 'xlsx', 'csv']
    }
  ];

  const formatDate = (dateString: string) => {
    const date = new Date(dateString);
    return date.toLocaleDateString('en-US', { 
      year: 'numeric', 
      month: 'short', 
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit'
    });
  };

  const handleGenerateReport = () => {
    if (!selectedReport) return;
    
    alert(`Generating ${selectedReport} report in ${selectedFormat} format for ${dateRange}...`);
    // In a real application, this would call an API endpoint to generate the report
  };

  const handleDownloadReport = (reportId: string) => {
    alert(`Downloading ${reportId} report...`);
    // In a real application, this would initiate a file download
  };

  function ServerIcon(props: React.ComponentProps<'svg'>) {
    return (
      <svg 
        xmlns="http://www.w3.org/2000/svg" 
        fill="none" 
        viewBox="0 0 24 24" 
        stroke="currentColor" 
        {...props}
      >
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 12h14M5 12a2 2 0 01-2-2V6a2 2 0 012-2h14a2 2 0 012 2v4a2 2 0 01-2 2M5 12a2 2 0 00-2 2v4a2 2 0 002 2h14a2 2 0 002-2v-4a2 2 0 00-2-2m-2-4h.01M17 16h.01" />
      </svg>
    );
  }

  function MapIcon(props: React.ComponentProps<'svg'>) {
    return (
      <svg 
        xmlns="http://www.w3.org/2000/svg" 
        fill="none" 
        viewBox="0 0 24 24" 
        stroke="currentColor" 
        {...props}
      >
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 20l-5.447-2.724A1 1 0 013 16.382V5.618a1 1 0 011.447-.894L9 7m0 13l6-3m-6 3V7m6 10l4.553 2.276A1 1 0 0021 18.382V7.618a1 1 0 00-.553-.894L15 4m0 13V4m0 0L9 7" />
      </svg>
    );
  }

  return (
    <div className="py-6">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 md:px-8">
        <div className="flex flex-col md:flex-row md:items-center md:justify-between mb-8">
          <div>
            <h1 className="text-2xl font-semibold text-gray-900 dark:text-white">Reports</h1>
            <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
              Generate and view detailed system reports
            </p>
          </div>
          <div className="mt-4 md:mt-0">
            <div className="inline-flex rounded-md shadow">
              <button
                onClick={() => setSelectedReport('custom')}
                className="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
              >
                <PlusIcon className="h-5 w-5 mr-2" />
                Create Custom Report
              </button>
            </div>
          </div>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          {/* Report Generator Panel */}
          <div className="md:col-span-1">
            <div className="bg-white dark:bg-gray-800 shadow rounded-lg p-6">
              <h2 className="text-lg font-medium text-gray-900 dark:text-white mb-4">Generate Report</h2>
              
              <div className="space-y-4">
                <div>
                  <label htmlFor="report-type" className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                    Report Type
                  </label>
                  <select
                    id="report-type"
                    value={selectedReport || ''}
                    onChange={(e) => setSelectedReport(e.target.value || null)}
                    className="w-full px-3 py-2 border border-gray-300 dark:border-gray-700 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500 bg-white dark:bg-gray-700 text-gray-900 dark:text-gray-100"
                  >
                    <option value="">Select a report</option>
                    {reports.map((report) => (
                      <option key={report.id} value={report.id}>
                        {report.name}
                      </option>
                    ))}
                    <option value="custom">Custom Report</option>
                  </select>
                </div>
                
                <div>
                  <label htmlFor="date-range" className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                    Date Range
                  </label>
                  <div className="relative">
                    <select
                      id="date-range"
                      value={dateRange}
                      onChange={(e) => setDateRange(e.target.value)}
                      className="w-full pl-10 pr-3 py-2 border border-gray-300 dark:border-gray-700 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500 bg-white dark:bg-gray-700 text-gray-900 dark:text-gray-100"
                    >
                      <option value="today">Today</option>
                      <option value="yesterday">Yesterday</option>
                      <option value="last7days">Last 7 days</option>
                      <option value="last30days">Last 30 days</option>
                      <option value="thisMonth">This month</option>
                      <option value="lastMonth">Last month</option>
                      <option value="custom">Custom range...</option>
                    </select>
                    <CalendarIcon className="absolute top-1/2 left-3 -translate-y-1/2 h-5 w-5 text-gray-400" />
                  </div>
                </div>
                
                <div>
                  <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                    Format
                  </label>
                  <div className="flex space-x-4">
                    {['pdf', 'csv', 'xlsx', 'json'].map((format) => (
                      <label key={format} className="inline-flex items-center">
                        <input
                          type="radio"
                          checked={selectedFormat === format}
                          onChange={() => setSelectedFormat(format)}
                          className="h-4 w-4 text-blue-600 border-gray-300 focus:ring-blue-500"
                        />
                        <span className="ml-2 text-sm text-gray-700 dark:text-gray-300 uppercase">
                          {format}
                        </span>
                      </label>
                    ))}
                  </div>
                </div>
                
                <button
                  onClick={handleGenerateReport}
                  disabled={!selectedReport}
                  className={`w-full flex justify-center items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white ${
                    selectedReport
                      ? 'bg-blue-600 hover:bg-blue-700 focus:ring-blue-500'
                      : 'bg-gray-400 cursor-not-allowed'
                  } focus:outline-none focus:ring-2 focus:ring-offset-2`}
                >
                  <DocumentReportIcon className="h-5 w-5 mr-2" />
                  Generate Report
                </button>
              </div>
            </div>
          </div>

          {/* Recent Reports Panel */}
          <div className="md:col-span-2">
            <div className="bg-white dark:bg-gray-800 shadow rounded-lg">
              <div className="px-4 py-5 sm:px-6 border-b dark:border-gray-700">
                <h2 className="text-lg font-medium text-gray-900 dark:text-white">Recent Reports</h2>
                <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                  Access and download previously generated reports
                </p>
              </div>
              <ul className="divide-y divide-gray-200 dark:divide-gray-700">
                {reports.map((report) => (
                  <li key={report.id} className="px-4 py-4 sm:px-6 hover:bg-gray-50 dark:hover:bg-gray-700">
                    <div className="flex items-center justify-between">
                      <div className="flex items-center">
                        <div className="flex-shrink-0 h-10 w-10 bg-blue-100 dark:bg-blue-900 rounded-md flex items-center justify-center text-blue-500">
                          <report.icon className="h-6 w-6" />
                        </div>
                        <div className="ml-4">
                          <h3 className="text-sm font-medium text-gray-900 dark:text-white">{report.name}</h3>
                          <p className="text-sm text-gray-500 dark:text-gray-400">{report.description}</p>
                          <p className="text-xs text-gray-500 dark:text-gray-400 mt-1">
                            Last generated: {formatDate(report.lastGenerated)}
                          </p>
                        </div>
                      </div>
                      <div className="flex items-center space-x-2">
                        <button
                          onClick={() => handleDownloadReport(report.id)}
                          className="inline-flex items-center p-2 border border-gray-300 dark:border-gray-600 rounded-md text-sm font-medium text-gray-700 dark:text-gray-300 bg-white dark:bg-gray-700 hover:bg-gray-50 dark:hover:bg-gray-600 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
                        >
                          <DownloadIcon className="h-5 w-5" />
                        </button>
                        <button className="inline-flex items-center p-2 border border-gray-300 dark:border-gray-600 rounded-md text-sm font-medium text-gray-700 dark:text-gray-300 bg-white dark:bg-gray-700 hover:bg-gray-50 dark:hover:bg-gray-600 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500">
                          <ShareIcon className="h-5 w-5" />
                        </button>
                      </div>
                    </div>
                  </li>
                ))}
              </ul>
              <div className="px-4 py-3 sm:px-6 border-t dark:border-gray-700 text-right">
                <button className="text-sm font-medium text-blue-600 hover:text-blue-800 dark:text-blue-400 dark:hover:text-blue-300">
                  View All Reports <span aria-hidden="true">→</span>
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}