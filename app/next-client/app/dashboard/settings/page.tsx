'use client';

import { useState } from 'react';
import { 
  CogIcon, 
  UserIcon, 
  ShieldCheckIcon, 
  GlobeIcon, 
  BellIcon,
  DeviceMobileIcon,
  ChipIcon,
  SwitchHorizontalIcon,
  SaveIcon,
  DatabaseIcon
} from '@heroicons/react/outline';
import { useTheme } from '../../../components/theme-provider';

export default function SettingsPage() {
  const { theme, setTheme } = useTheme();
  const [activeTab, setActiveTab] = useState('general');
  const [notificationsEnabled, setNotificationsEnabled] = useState(true);
  const [autoRefreshEnabled, setAutoRefreshEnabled] = useState(true);
  const [autoRefreshInterval, setAutoRefreshInterval] = useState(30);
  const [darkModeEnabled, setDarkModeEnabled] = useState(theme === 'dark');
  const [dataRetentionPeriod, setDataRetentionPeriod] = useState(90);
  const [language, setLanguage] = useState('en');
  const [timeZone, setTimeZone] = useState('UTC');
  const [saveStatus, setSaveStatus] = useState('');
  
  const handleSave = () => {
    // In a real app, this would save to backend
    setSaveStatus('saving');
    setTimeout(() => {
      setSaveStatus('saved');
      setTimeout(() => setSaveStatus(''), 3000);
    }, 1000);
  };
  
  const handleThemeChange = (isDark: boolean) => {
    setDarkModeEnabled(isDark);
    setTheme(isDark ? 'dark' : 'light');
  };
  
  const tabs = [
    { id: 'general', name: 'General', icon: CogIcon },
    { id: 'account', name: 'Account', icon: UserIcon },
    { id: 'security', name: 'Security', icon: ShieldCheckIcon },
    { id: 'notifications', name: 'Notifications', icon: BellIcon },
    { id: 'ros2', name: 'ROS2 Config', icon: ChipIcon },
    { id: 'lorawanDevices', name: 'LoRaWAN Devices', icon: DeviceMobileIcon },
    { id: 'integration', name: 'Integrations', icon: SwitchHorizontalIcon },
    { id: 'dataManagement', name: 'Data Management', icon: DatabaseIcon },
  ];
  
  function ChipIcon(props: React.ComponentProps<'svg'>) {
    return (
      <svg 
        xmlns="http://www.w3.org/2000/svg" 
        fill="none" 
        viewBox="0 0 24 24" 
        stroke="currentColor" 
        {...props}
      >
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z" />
      </svg>
    );
  }
  
  return (
    <div className="py-6">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 md:px-8">
        <div className="flex flex-col md:flex-row md:items-center md:justify-between mb-8">
          <div>
            <h1 className="text-2xl font-semibold text-gray-900 dark:text-white">Settings</h1>
            <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
              Manage your dashboard preferences and system configuration
            </p>
          </div>
          <div className="mt-4 md:mt-0">
            <button
              onClick={handleSave}
              className={`inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 ${
                saveStatus === 'saving' 
                  ? 'bg-gray-400 cursor-wait'
                  : 'bg-blue-600 hover:bg-blue-700'
              }`}
            >
              {saveStatus === 'saving' ? (
                <>
                  <div className="animate-spin rounded-full h-4 w-4 border-b-2 border-white mr-2"></div>
                  Saving...
                </>
              ) : (
                <>
                  <SaveIcon className="h-5 w-5 mr-2" />
                  {saveStatus === 'saved' ? 'Saved!' : 'Save Settings'}
                </>
              )}
            </button>
          </div>
        </div>
        
        <div className="bg-white dark:bg-gray-800 shadow overflow-hidden sm:rounded-lg">
          <div className="border-b border-gray-200 dark:border-gray-700">
            <nav className="-mb-px flex space-x-6 overflow-x-auto py-3 px-4">
              {tabs.map((tab) => (
                <button
                  key={tab.id}
                  onClick={() => setActiveTab(tab.id)}
                  className={`
                    whitespace-nowrap pb-3 pt-1 px-1 border-b-2 text-sm font-medium
                    ${
                      activeTab === tab.id
                        ? 'border-blue-600 text-blue-600 dark:border-blue-400 dark:text-blue-400'
                        : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300 dark:text-gray-400 dark:hover:text-gray-300 dark:hover:border-gray-600'
                    }
                  `}
                >
                  <div className="flex items-center">
                    <tab.icon className="h-5 w-5 mr-2" />
                    {tab.name}
                  </div>
                </button>
              ))}
            </nav>
          </div>

          <div className="px-4 py-5 sm:p-6">
            {activeTab === 'general' && (
              <div className="space-y-6">
                <div>
                  <h3 className="text-lg font-medium leading-6 text-gray-900 dark:text-gray-100">Display Settings</h3>
                  <div className="mt-5 space-y-4">
                    <div className="flex items-center justify-between">
                      <div>
                        <label htmlFor="dark-mode-toggle" className="font-medium text-gray-700 dark:text-gray-300">Dark Mode</label>
                        <p className="text-sm text-gray-500 dark:text-gray-400">Toggle between light and dark theme</p>
                      </div>
                      <div className="relative inline-block w-12 align-middle select-none transition duration-200 ease-in">
                        <input
                          type="checkbox"
                          name="dark-mode-toggle"
                          id="dark-mode-toggle"
                          checked={darkModeEnabled}
                          onChange={(e) => handleThemeChange(e.target.checked)}
                          className="toggle-checkbox absolute block w-6 h-6 rounded-full bg-white border-4 border-gray-300 appearance-none cursor-pointer transition-transform duration-200 ease-in-out"
                        />
                        <label
                          htmlFor="dark-mode-toggle"
                          className={`toggle-label block overflow-hidden h-6 rounded-full cursor-pointer ${
                            darkModeEnabled ? 'bg-blue-600' : 'bg-gray-300'
                          }`}
                        ></label>
                      </div>
                    </div>
                    
                    <div className="flex items-center justify-between">
                      <div>
                        <label htmlFor="auto-refresh-toggle" className="font-medium text-gray-700 dark:text-gray-300">Auto-refresh Data</label>
                        <p className="text-sm text-gray-500 dark:text-gray-400">Automatically refresh dashboard data</p>
                      </div>
                      <div className="relative inline-block w-12 align-middle select-none transition duration-200 ease-in">
                        <input
                          type="checkbox"
                          name="auto-refresh-toggle"
                          id="auto-refresh-toggle"
                          checked={autoRefreshEnabled}
                          onChange={(e) => setAutoRefreshEnabled(e.target.checked)}
                          className="toggle-checkbox absolute block w-6 h-6 rounded-full bg-white border-4 border-gray-300 appearance-none cursor-pointer transition-transform duration-200 ease-in-out"
                        />
                        <label
                          htmlFor="auto-refresh-toggle"
                          className={`toggle-label block overflow-hidden h-6 rounded-full cursor-pointer ${
                            autoRefreshEnabled ? 'bg-blue-600' : 'bg-gray-300'
                          }`}
                        ></label>
                      </div>
                    </div>
                    
                    {autoRefreshEnabled && (
                      <div>
                        <label htmlFor="refresh-interval" className="block text-sm font-medium text-gray-700 dark:text-gray-300">Refresh Interval (seconds)</label>
                        <div className="mt-1 flex rounded-md shadow-sm">
                          <input
                            type="number"
                            name="refresh-interval"
                            id="refresh-interval"
                            className="focus:ring-blue-500 focus:border-blue-500 flex-1 block w-full rounded-md sm:text-sm border-gray-300 dark:border-gray-600 dark:bg-gray-700 dark:text-white"
                            value={autoRefreshInterval}
                            onChange={(e) => setAutoRefreshInterval(Number(e.target.value))}
                            min="5"
                            max="300"
                          />
                        </div>
                      </div>
                    )}
                  </div>
                </div>
                
                <div className="border-t border-gray-200 dark:border-gray-700 pt-6">
                  <h3 className="text-lg font-medium leading-6 text-gray-900 dark:text-gray-100">Regional Settings</h3>
                  <div className="mt-5 grid grid-cols-1 gap-y-6 gap-x-4 sm:grid-cols-6">
                    <div className="sm:col-span-3">
                      <label htmlFor="language" className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                        Language
                      </label>
                      <div className="mt-1">
                        <select
                          id="language"
                          name="language"
                          value={language}
                          onChange={(e) => setLanguage(e.target.value)}
                          className="shadow-sm focus:ring-blue-500 focus:border-blue-500 block w-full sm:text-sm border-gray-300 rounded-md dark:border-gray-600 dark:bg-gray-700 dark:text-white"
                        >
                          <option value="en">English</option>
                          <option value="es">Spanish</option>
                          <option value="fr">French</option>
                          <option value="de">German</option>
                          <option value="ja">Japanese</option>
                          <option value="zh">Chinese</option>
                        </select>
                      </div>
                    </div>

                    <div className="sm:col-span-3">
                      <label htmlFor="timezone" className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                        Time Zone
                      </label>
                      <div className="mt-1">
                        <select
                          id="timezone"
                          name="timezone"
                          value={timeZone}
                          onChange={(e) => setTimeZone(e.target.value)}
                          className="shadow-sm focus:ring-blue-500 focus:border-blue-500 block w-full sm:text-sm border-gray-300 rounded-md dark:border-gray-600 dark:bg-gray-700 dark:text-white"
                        >
                          <option value="UTC">UTC</option>
                          <option value="America/New_York">Eastern Time (US & Canada)</option>
                          <option value="America/Chicago">Central Time (US & Canada)</option>
                          <option value="America/Denver">Mountain Time (US & Canada)</option>
                          <option value="America/Los_Angeles">Pacific Time (US & Canada)</option>
                          <option value="Europe/London">London</option>
                          <option value="Europe/Paris">Paris</option>
                          <option value="Asia/Tokyo">Tokyo</option>
                        </select>
                      </div>
                    </div>
                  </div>
                </div>
                
                <div className="border-t border-gray-200 dark:border-gray-700 pt-6">
                  <h3 className="text-lg font-medium leading-6 text-gray-900 dark:text-gray-100">Data Settings</h3>
                  <div className="mt-5">
                    <div>
                      <label htmlFor="data-retention" className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                        Data Retention Period (days)
                      </label>
                      <div className="mt-1">
                        <input
                          type="number"
                          name="data-retention"
                          id="data-retention"
                          value={dataRetentionPeriod}
                          onChange={(e) => setDataRetentionPeriod(Number(e.target.value))}
                          className="shadow-sm focus:ring-blue-500 focus:border-blue-500 block w-full sm:text-sm border-gray-300 rounded-md dark:border-gray-600 dark:bg-gray-700 dark:text-white"
                          min="1"
                          max="365"
                        />
                      </div>
                      <p className="mt-2 text-sm text-gray-500 dark:text-gray-400">
                        Historical data older than this many days will be automatically archived
                      </p>
                    </div>
                  </div>
                </div>
              </div>
            )}
            
            {activeTab === 'notifications' && (
              <div className="space-y-6">
                <div>
                  <h3 className="text-lg font-medium leading-6 text-gray-900 dark:text-gray-100">Notification Preferences</h3>
                  <div className="mt-5 space-y-4">
                    <div className="flex items-center justify-between">
                      <div>
                        <label htmlFor="notifications-toggle" className="font-medium text-gray-700 dark:text-gray-300">Enable Notifications</label>
                        <p className="text-sm text-gray-500 dark:text-gray-400">Receive alerts about system events</p>
                      </div>
                      <div className="relative inline-block w-12 align-middle select-none transition duration-200 ease-in">
                        <input
                          type="checkbox"
                          name="notifications-toggle"
                          id="notifications-toggle"
                          checked={notificationsEnabled}
                          onChange={(e) => setNotificationsEnabled(e.target.checked)}
                          className="toggle-checkbox absolute block w-6 h-6 rounded-full bg-white border-4 border-gray-300 appearance-none cursor-pointer transition-transform duration-200 ease-in-out"
                        />
                        <label
                          htmlFor="notifications-toggle"
                          className={`toggle-label block overflow-hidden h-6 rounded-full cursor-pointer ${
                            notificationsEnabled ? 'bg-blue-600' : 'bg-gray-300'
                          }`}
                        ></label>
                      </div>
                    </div>
                    
                    {notificationsEnabled && (
                      <div className="pl-4 border-l-4 border-blue-500 space-y-4">
                        <div className="flex items-center">
                          <input
                            id="critical-alerts"
                            name="critical-alerts"
                            type="checkbox"
                            defaultChecked
                            className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                          />
                          <label htmlFor="critical-alerts" className="ml-3 text-sm font-medium text-gray-700 dark:text-gray-300">
                            Critical Alerts
                          </label>
                        </div>
                        
                        <div className="flex items-center">
                          <input
                            id="warning-alerts"
                            name="warning-alerts"
                            type="checkbox"
                            defaultChecked
                            className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                          />
                          <label htmlFor="warning-alerts" className="ml-3 text-sm font-medium text-gray-700 dark:text-gray-300">
                            Warning Alerts
                          </label>
                        </div>
                        
                        <div className="flex items-center">
                          <input
                            id="info-alerts"
                            name="info-alerts"
                            type="checkbox"
                            defaultChecked
                            className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                          />
                          <label htmlFor="info-alerts" className="ml-3 text-sm font-medium text-gray-700 dark:text-gray-300">
                            Informational Alerts
                          </label>
                        </div>
                        
                        <div className="flex items-center">
                          <input
                            id="system-updates"
                            name="system-updates"
                            type="checkbox"
                            defaultChecked
                            className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                          />
                          <label htmlFor="system-updates" className="ml-3 text-sm font-medium text-gray-700 dark:text-gray-300">
                            System Updates
                          </label>
                        </div>
                        
                        <div className="flex items-center">
                          <input
                            id="device-status-changes"
                            name="device-status-changes"
                            type="checkbox"
                            defaultChecked
                            className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                          />
                          <label htmlFor="device-status-changes" className="ml-3 text-sm font-medium text-gray-700 dark:text-gray-300">
                            Device Status Changes
                          </label>
                        </div>
                      </div>
                    )}
                  </div>
                </div>
                
                <div className="border-t border-gray-200 dark:border-gray-700 pt-6">
                  <h3 className="text-lg font-medium leading-6 text-gray-900 dark:text-gray-100">Delivery Methods</h3>
                  <div className="mt-5 space-y-4">
                    <div className="flex items-center">
                      <input
                        id="delivery-browser"
                        name="delivery-browser"
                        type="checkbox"
                        defaultChecked
                        className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                      />
                      <label htmlFor="delivery-browser" className="ml-3 text-sm font-medium text-gray-700 dark:text-gray-300">
                        Browser Notifications
                      </label>
                    </div>
                    
                    <div className="flex items-center">
                      <input
                        id="delivery-email"
                        name="delivery-email"
                        type="checkbox"
                        defaultChecked
                        className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                      />
                      <label htmlFor="delivery-email" className="ml-3 text-sm font-medium text-gray-700 dark:text-gray-300">
                        Email Notifications
                      </label>
                    </div>
                    
                    <div className="flex items-center">
                      <input
                        id="delivery-slack"
                        name="delivery-slack"
                        type="checkbox"
                        className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                      />
                      <label htmlFor="delivery-slack" className="ml-3 text-sm font-medium text-gray-700 dark:text-gray-300">
                        Slack Notifications
                      </label>
                    </div>
                  </div>
                </div>
              </div>
            )}
            
            {activeTab === 'ros2' && (
              <div className="space-y-6">
                <div>
                  <h3 className="text-lg font-medium leading-6 text-gray-900 dark:text-gray-100">ROS2 Connection Settings</h3>
                  <div className="mt-5">
                    <div className="grid grid-cols-1 gap-y-6 gap-x-4 sm:grid-cols-6">
                      <div className="sm:col-span-3">
                        <label htmlFor="ros-domain-id" className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                          ROS_DOMAIN_ID
                        </label>
                        <div className="mt-1">
                          <input
                            type="number"
                            name="ros-domain-id"
                            id="ros-domain-id"
                            defaultValue={0}
                            className="shadow-sm focus:ring-blue-500 focus:border-blue-500 block w-full sm:text-sm border-gray-300 rounded-md dark:border-gray-600 dark:bg-gray-700 dark:text-white"
                          />
                        </div>
                      </div>

                      <div className="sm:col-span-3">
                        <label htmlFor="ros-middleware" className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                          RMW Implementation
                        </label>
                        <div className="mt-1">
                          <select
                            id="ros-middleware"
                            name="ros-middleware"
                            defaultValue="cyclonedds"
                            className="shadow-sm focus:ring-blue-500 focus:border-blue-500 block w-full sm:text-sm border-gray-300 rounded-md dark:border-gray-600 dark:bg-gray-700 dark:text-white"
                          >
                            <option value="cyclonedds">Eclipse Cyclone DDS</option>
                            <option value="fastrtps">eProsima Fast DDS</option>
                            <option value="connext">RTI Connext DDS</option>
                          </select>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
                
                <div className="border-t border-gray-200 dark:border-gray-700 pt-6">
                  <h3 className="text-lg font-medium leading-6 text-gray-900 dark:text-gray-100">Topic Monitoring</h3>
                  <div className="mt-5">
                    <p className="text-sm text-gray-500 dark:text-gray-400 mb-4">
                      Configure which ROS2 topics to monitor and display in the dashboard
                    </p>
                    
                    <div className="shadow-sm border border-gray-300 dark:border-gray-600 rounded-md overflow-hidden">
                      <table className="min-w-full divide-y divide-gray-200 dark:divide-gray-700">
                        <thead className="bg-gray-50 dark:bg-gray-700">
                          <tr>
                            <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                              Topic
                            </th>
                            <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                              Type
                            </th>
                            <th scope="col" className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                              Enabled
                            </th>
                          </tr>
                        </thead>
                        <tbody className="bg-white dark:bg-gray-800 divide-y divide-gray-200 dark:divide-gray-700">
                          <tr>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-gray-300">
                              /robot1/status
                            </td>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                              std_msgs/String
                            </td>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                              <input
                                type="checkbox"
                                defaultChecked
                                className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                              />
                            </td>
                          </tr>
                          <tr>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-gray-300">
                              /robot1/temperature
                            </td>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                              std_msgs/Float32
                            </td>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                              <input
                                type="checkbox"
                                defaultChecked
                                className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                              />
                            </td>
                          </tr>
                          <tr>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-gray-300">
                              /robot1/battery_level
                            </td>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                              std_msgs/Int32
                            </td>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                              <input
                                type="checkbox"
                                defaultChecked
                                className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                              />
                            </td>
                          </tr>
                          <tr>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-gray-300">
                              /robot1/location
                            </td>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                              nav_msgs/Odometry
                            </td>
                            <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                              <input
                                type="checkbox"
                                defaultChecked
                                className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                              />
                            </td>
                          </tr>
                        </tbody>
                      </table>
                    </div>
                    
                    <div className="mt-4 flex">
                      <button className="mr-3 inline-flex items-center px-3 py-1.5 border border-transparent text-xs font-medium rounded text-blue-700 bg-blue-100 hover:bg-blue-200 dark:bg-blue-900 dark:text-blue-300 dark:hover:bg-blue-800 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500">
                        Add Topic
                      </button>
                      <button className="inline-flex items-center px-3 py-1.5 border border-transparent text-xs font-medium rounded text-gray-700 bg-gray-100 hover:bg-gray-200 dark:bg-gray-700 dark:text-gray-300 dark:hover:bg-gray-600 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-gray-500">
                        Auto-Discover Topics
                      </button>
                    </div>
                  </div>
                </div>
              </div>
            )}
            
            {/* Placeholder content for other tabs */}
            {(activeTab !== 'general' && activeTab !== 'notifications' && activeTab !== 'ros2') && (
              <div className="text-center py-10">
                <h3 className="text-lg font-medium text-gray-900 dark:text-gray-100 mb-2">
                  {tabs.find(tab => tab.id === activeTab)?.name} Settings
                </h3>
                <p className="text-gray-500 dark:text-gray-400">
                  This settings panel is under construction.
                </p>
              </div>
            )}
          </div>
        </div>
      </div>

      <style jsx>{`
        .toggle-checkbox:checked {
          transform: translateX(100%);
          border-color: #3b82f6;
        }
        .toggle-label {
          transition: background-color 0.2s ease-in-out;
        }
      `}</style>
    </div>
  );
}