'use client';

import { useState, useEffect } from 'react';
import { ChartBarIcon, ArrowPathIcon, ArrowDownTrayIcon } from '@heroicons/react/24/outline';
import { useTopicData } from '../../../hooks/useTopicData';
import LineChart from '../../../components/charts/LineChart';
import TopicSelector from '../../../components/charts/TopicSelector';

export default function ChartsPage() {
  const {
    topics,
    selectedTopic,
    selectTopic,
    data,
    timestamps,
    isLoading,
    isLoadingTopicList,
    error,
    refresh,
    autoRefresh,
    setAutoRefresh,
    refreshInterval,
    setRefreshInterval
  } = useTopicData();

  const [chartType, setChartType] = useState<'line' | 'bar'>('line');
  const [chartColor, setChartColor] = useState<string>('#0073ff');
  const [yAxisMin, setYAxisMin] = useState<string>('');
  const [yAxisMax, setYAxisMax] = useState<string>('');
  
  // Extract numeric values from data for charting
  const [chartData, setChartData] = useState<number[]>([]);
  const [chartLabel, setChartLabel] = useState<string>('Value');

  // Extract numeric data from possibly complex ROS2 messages
  useEffect(() => {
    if (data.length === 0) {
      setChartData([]);
      return;
    }

    try {
      // Try to handle various data structures
      if (typeof data[0] === 'number') {
        // Direct numeric array
        setChartData(data as number[]);
        setChartLabel('Value');
      } else if (typeof data[0] === 'object') {
        // For objects like sensor msgs, try to extract temperature as an example
        const firstItem = data[0];
        
        // Check for common ROS2 sensor patterns
        if ('temperature' in firstItem) {
          const values = data.map(item => item.temperature);
          setChartData(values);
          setChartLabel('Temperature');
        } else if ('tpb_value' in firstItem && 'temperature' in firstItem.tpb_value) {
          const values = data.map(item => item.tpb_value.temperature);
          setChartData(values);
          setChartLabel('Temperature');
        } else if ('data' in firstItem && Array.isArray(firstItem.data)) {
          // Handle array data fields
          const values = data.map(item => item.data[0]);
          setChartData(values);
          setChartLabel('Data[0]');
        } else {
          // Try to find any numeric values
          const keys = Object.keys(firstItem);
          const numericKey = keys.find(key => typeof firstItem[key] === 'number');
          
          if (numericKey) {
            const values = data.map(item => item[numericKey]);
            setChartData(values);
            setChartLabel(numericKey);
          } else {
            setChartData([]);
            console.warn('Could not extract numeric data from topic for charting');
          }
        }
      }
    } catch (err) {
      console.error('Error processing data for chart:', err);
      setChartData([]);
    }
  }, [data]);

  // Handle refresh interval change
  const handleRefreshIntervalChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    setRefreshInterval(parseInt(event.target.value, 10));
  };
  
  // Export current chart data as CSV
  const exportChartDataCsv = () => {
    if (chartData.length === 0 || timestamps.length === 0) return;
    
    // Create CSV content
    let csvContent = 'timestamp,value\n';
    
    chartData.forEach((value, index) => {
      const timestamp = timestamps[index] 
        ? `${timestamps[index].year}-${timestamps[index].month}-${timestamps[index].day} ${timestamps[index].hour}:${timestamps[index].minute}:${timestamps[index].second}`
        : new Date().toISOString();
      csvContent += `${timestamp},${value}\n`;
    });
    
    // Create download link
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.setAttribute('href', url);
    link.setAttribute('download', `${selectedTopic?.name.replace(/\//g, '_')}_data.csv`);
    link.style.visibility = 'hidden';
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  };

  return (
    <div className="py-6">
      <div className="flex flex-col md:flex-row justify-between items-start mb-6">
        <div>
          <h1 className="text-2xl font-bold text-gray-900 dark:text-white flex items-center">
            <ChartBarIcon className="h-6 w-6 mr-2 text-primary-500" />
            ROS2 Topic Visualizer
          </h1>
          <p className="text-gray-500 dark:text-gray-400 mt-1">
            Visualize real-time data from any ROS2 topic
          </p>
        </div>
        
        <div className="mt-4 md:mt-0 flex flex-col sm:flex-row gap-4">
          <div className="flex items-center">
            <label htmlFor="chartType" className="mr-2 text-sm text-gray-700 dark:text-gray-300">
              Type:
            </label>
            <select
              id="chartType"
              value={chartType}
              onChange={(e) => setChartType(e.target.value as 'line' | 'bar')}
              className="input-field text-sm py-1 px-2 w-24"
            >
              <option value="line">Line</option>
              <option value="bar">Bar</option>
            </select>
          </div>
          
          <div className="flex items-center">
            <label htmlFor="chartColor" className="mr-2 text-sm text-gray-700 dark:text-gray-300">
              Color:
            </label>
            <input
              type="color"
              id="chartColor"
              value={chartColor}
              onChange={(e) => setChartColor(e.target.value)}
              className="w-8 h-8 cursor-pointer rounded border border-gray-300 dark:border-gray-700"
            />
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
              <option value="1">1s</option>
              <option value="5">5s</option>
              <option value="10">10s</option>
              <option value="30">30s</option>
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
        </div>
      </div>
      
      {error && (
        <div className="bg-red-50 dark:bg-red-900/20 border border-red-200 dark:border-red-800 text-red-800 dark:text-red-200 px-4 py-3 rounded-md mb-6">
          {error}
        </div>
      )}
      
      {/* Main Content: Topic Selector and Chart */}
      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Topic Selector */}
        <div className="lg:col-span-1">
          <TopicSelector 
            topics={topics}
            selectedTopic={selectedTopic}
            onSelectTopic={selectTopic}
            isLoading={isLoadingTopicList}
          />
          
          {/* Chart Options */}
          {selectedTopic && (
            <div className="bg-white dark:bg-gray-800 rounded-lg shadow mt-6 p-4">
              <h3 className="text-lg font-medium text-gray-900 dark:text-gray-100 mb-3">Chart Options</h3>
              
              <div className="space-y-4">
                <div>
                  <label htmlFor="yAxisMin" className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                    Y-Axis Minimum
                  </label>
                  <input
                    type="number"
                    id="yAxisMin"
                    placeholder="Auto"
                    value={yAxisMin}
                    onChange={(e) => setYAxisMin(e.target.value)}
                    className="input-field"
                  />
                </div>
                
                <div>
                  <label htmlFor="yAxisMax" className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                    Y-Axis Maximum
                  </label>
                  <input
                    type="number"
                    id="yAxisMax"
                    placeholder="Auto"
                    value={yAxisMax}
                    onChange={(e) => setYAxisMax(e.target.value)}
                    className="input-field"
                  />
                </div>
                
                <div>
                  <label htmlFor="chartLabel" className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                    Chart Label
                  </label>
                  <input
                    type="text"
                    id="chartLabel"
                    value={chartLabel}
                    onChange={(e) => setChartLabel(e.target.value)}
                    className="input-field"
                  />
                </div>
                
                <div className="pt-2">
                  <button
                    onClick={exportChartDataCsv}
                    disabled={chartData.length === 0}
                    className="btn-secondary w-full flex items-center justify-center"
                  >
                    <ArrowDownTrayIcon className="h-4 w-4 mr-1" />
                    Export Data (CSV)
                  </button>
                </div>
              </div>
            </div>
          )}
        </div>
        
        {/* Chart Area */}
        <div className="lg:col-span-3">
          <div className="bg-white dark:bg-gray-800 rounded-lg shadow p-4">
            <div className="flex justify-between items-center mb-4">
              <h2 className="text-lg font-medium text-gray-900 dark:text-gray-100">
                {selectedTopic ? `Chart: ${selectedTopic.name}` : 'Select a Topic to Visualize'}
              </h2>
              
              <button
                onClick={refresh}
                disabled={isLoading || !selectedTopic}
                className="btn-secondary flex items-center text-sm"
              >
                <ArrowPathIcon className={`h-4 w-4 mr-1 ${isLoading ? 'animate-spin' : ''}`} />
                {isLoading ? 'Loading...' : 'Refresh'}
              </button>
            </div>
            
            {!selectedTopic ? (
              <div className="h-80 flex items-center justify-center border border-dashed border-gray-300 dark:border-gray-700 rounded-lg">
                <div className="text-center text-gray-500 dark:text-gray-400">
                  <ChartBarIcon className="h-12 w-12 mx-auto mb-2 opacity-30" />
                  <p>Select a topic from the list to visualize data</p>
                </div>
              </div>
            ) : chartData.length === 0 ? (
              <div className="h-80 flex items-center justify-center border border-dashed border-gray-300 dark:border-gray-700 rounded-lg">
                <div className="text-center text-gray-500 dark:text-gray-400">
                  {isLoading ? (
                    <>
                      <div className="animate-spin h-8 w-8 border-4 border-primary-500 border-t-transparent rounded-full mx-auto mb-2"></div>
                      <p>Loading data from {selectedTopic.name}...</p>
                    </>
                  ) : (
                    <>
                      <svg className="h-12 w-12 mx-auto mb-2 opacity-30" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
                      </svg>
                      <p>No numeric data available for this topic</p>
                      <p className="text-sm mt-2">Try selecting a different topic</p>
                    </>
                  )}
                </div>
              </div>
            ) : (
              <LineChart
                title=""
                data={chartData}
                timestamps={timestamps.length > 0 ? timestamps : Array(chartData.length).fill(null)}
                label={chartLabel}
                color={chartColor}
                fill={true}
                height={400}
                yAxisLabel="Value"
                xAxisLabel="Time"
                minY={yAxisMin ? parseFloat(yAxisMin) : undefined}
                maxY={yAxisMax ? parseFloat(yAxisMax) : undefined}
              />
            )}
          </div>
          
          {/* Data Preview */}
          {selectedTopic && chartData.length > 0 && (
            <div className="bg-white dark:bg-gray-800 rounded-lg shadow p-4 mt-6">
              <h3 className="text-lg font-medium text-gray-900 dark:text-gray-100 mb-3">Data Preview</h3>
              <div className="overflow-x-auto">
                <table className="min-w-full divide-y divide-gray-200 dark:divide-gray-700">
                  <thead className="bg-gray-50 dark:bg-gray-900">
                    <tr>
                      <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                        Index
                      </th>
                      <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                        Timestamp
                      </th>
                      <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                        {chartLabel}
                      </th>
                    </tr>
                  </thead>
                  <tbody className="bg-white dark:bg-gray-800 divide-y divide-gray-200 dark:divide-gray-700">
                    {chartData.slice(0, 10).map((value, index) => (
                      <tr key={index}>
                        <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-white">
                          {index}
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                          {timestamps[index] ? 
                            `${timestamps[index].hour}:${timestamps[index].minute}:${timestamps[index].second}.${timestamps[index].nanosecond.toString().slice(0, 3)}` : 
                            'N/A'
                          }
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-white">
                          {typeof value === 'number' ? value.toFixed(4) : String(value)}
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
                {chartData.length > 10 && (
                  <div className="mt-2 text-right text-sm text-gray-500 dark:text-gray-400 italic">
                    Showing 10 of {chartData.length} data points
                  </div>
                )}
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}