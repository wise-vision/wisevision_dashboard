'use client';

import { useState, useEffect } from 'react';
import { ClockIcon, ServerIcon, DevicePhoneMobileIcon, ExclamationTriangleIcon } from '@heroicons/react/24/outline';

// Dashboard Stat Card component
interface StatCardProps {
  title: string;
  value: string | number;
  icon: React.ElementType;
  change?: string;
  trend?: 'up' | 'down' | 'neutral';
  color?: 'blue' | 'green' | 'red' | 'yellow';
}

const StatCard = ({ title, value, icon: Icon, change, trend, color = 'blue' }: StatCardProps) => {
  const colorClasses = {
    blue: 'bg-blue-50 text-blue-500 dark:bg-blue-900/20 dark:text-blue-300',
    green: 'bg-green-50 text-green-500 dark:bg-green-900/20 dark:text-green-300',
    red: 'bg-red-50 text-red-500 dark:bg-red-900/20 dark:text-red-300',
    yellow: 'bg-yellow-50 text-yellow-500 dark:bg-yellow-900/20 dark:text-yellow-300',
  };

  const trendClasses = {
    up: 'text-green-500',
    down: 'text-red-500',
    neutral: 'text-gray-500',
  };

  return (
    <div className="bg-white dark:bg-gray-800 rounded-lg shadow p-5">
      <div className="flex justify-between">
        <div>
          <h3 className="text-lg font-medium text-gray-700 dark:text-gray-300">{title}</h3>
          <p className="text-3xl font-bold text-gray-900 dark:text-white mt-2">{value}</p>
          {change && (
            <p className={`text-sm mt-2 ${trend ? trendClasses[trend] : ''}`}>
              {trend === 'up' && '↑ '}
              {trend === 'down' && '↓ '}
              {change}
            </p>
          )}
        </div>
        <div className={`rounded-full p-3 ${colorClasses[color]}`}>
          <Icon className="h-6 w-6" />
        </div>
      </div>
    </div>
  );
};

export default function DashboardPage() {
  // State for mock dashboard data
  const [stats, setStats] = useState({
    activeDevices: 0,
    totalTopics: 0,
    activeAlerts: 0,
    uptime: '0d 0h 0m'
  });

  // Fetch dashboard data on component mount
  useEffect(() => {
    // In a real app, this would be an API call to your Flask backend
    // For now, let's simulate loading data
    const timer = setTimeout(() => {
      setStats({
        activeDevices: 24,
        totalTopics: 156,
        activeAlerts: 3,
        uptime: '12d 5h 33m'
      });
    }, 500);

    return () => clearTimeout(timer);
  }, []);

  return (
    <div className="py-6">
      <div className="flex flex-col md:flex-row justify-between items-start mb-8">
        <div>
          <h1 className="text-2xl font-bold text-gray-900 dark:text-white">Dashboard</h1>
          <p className="text-gray-500 dark:text-gray-400">Overview of your ROS2 systems and LoRaWAN devices</p>
        </div>
        <div className="mt-4 md:mt-0">
          <button className="btn-primary">
            Connect New Device
          </button>
        </div>
      </div>

      {/* Stats Grid */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
        <StatCard 
          title="Active Devices" 
          value={stats.activeDevices} 
          icon={DevicePhoneMobileIcon} 
          change="+2 from yesterday" 
          trend="up" 
          color="blue"
        />
        <StatCard 
          title="Total ROS2 Topics" 
          value={stats.totalTopics} 
          icon={ServerIcon}
          change="12 new topics" 
          trend="up" 
          color="green"
        />
        <StatCard 
          title="Active Alerts" 
          value={stats.activeAlerts} 
          icon={ExclamationTriangleIcon} 
          change="1 critical" 
          trend="down" 
          color="red"
        />
        <StatCard 
          title="System Uptime" 
          value={stats.uptime} 
          icon={ClockIcon} 
          color="yellow"
        />
      </div>

      {/* Main Content Panels */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Recent Activity Panel */}
        <div className="lg:col-span-2 bg-white dark:bg-gray-800 rounded-lg shadow p-6">
          <h2 className="text-lg font-medium text-gray-900 dark:text-white mb-4">Recent Activity</h2>
          <div className="space-y-4">
            {/* Activity items would go here - for now just placeholders */}
            {[1, 2, 3].map((item) => (
              <div key={item} className="border-b dark:border-gray-700 pb-4 last:border-0 last:pb-0">
                <div className="flex items-center justify-between">
                  <div className="flex items-center">
                    <span className="w-2 h-2 bg-primary-500 rounded-full mr-3"></span>
                    <span className="text-gray-800 dark:text-gray-200">
                      New data received on <span className="font-medium">/robot1/sensor_data</span>
                    </span>
                  </div>
                  <span className="text-xs text-gray-500 dark:text-gray-400">2 min ago</span>
                </div>
                <p className="text-sm text-gray-500 dark:text-gray-400 mt-1 ml-5">
                  Temperature reading: 24.5°C, Humidity: 45%
                </p>
              </div>
            ))}
          </div>
          <button className="mt-4 text-primary-500 hover:text-primary-600 text-sm font-medium">
            View All Activity
          </button>
        </div>

        {/* Device Status Panel */}
        <div className="bg-white dark:bg-gray-800 rounded-lg shadow p-6">
          <h2 className="text-lg font-medium text-gray-900 dark:text-white mb-4">Device Status</h2>
          <div className="space-y-3">
            {/* This would be populated from actual device data */}
            <div className="flex items-center justify-between py-2">
              <div className="flex items-center">
                <div className="w-3 h-3 bg-green-500 rounded-full mr-3"></div>
                <span className="text-gray-800 dark:text-gray-200">Robot-1</span>
              </div>
              <span className="text-xs bg-green-100 text-green-800 dark:bg-green-900/30 dark:text-green-300 py-1 px-2 rounded-full">
                Online
              </span>
            </div>
            <div className="flex items-center justify-between py-2 border-t dark:border-gray-700">
              <div className="flex items-center">
                <div className="w-3 h-3 bg-green-500 rounded-full mr-3"></div>
                <span className="text-gray-800 dark:text-gray-200">LoRaWAN-Sensor-45</span>
              </div>
              <span className="text-xs bg-green-100 text-green-800 dark:bg-green-900/30 dark:text-green-300 py-1 px-2 rounded-full">
                Online
              </span>
            </div>
            <div className="flex items-center justify-between py-2 border-t dark:border-gray-700">
              <div className="flex items-center">
                <div className="w-3 h-3 bg-yellow-500 rounded-full mr-3"></div>
                <span className="text-gray-800 dark:text-gray-200">Drone-2</span>
              </div>
              <span className="text-xs bg-yellow-100 text-yellow-800 dark:bg-yellow-900/30 dark:text-yellow-300 py-1 px-2 rounded-full">
                Idle
              </span>
            </div>
            <div className="flex items-center justify-between py-2 border-t dark:border-gray-700">
              <div className="flex items-center">
                <div className="w-3 h-3 bg-red-500 rounded-full mr-3"></div>
                <span className="text-gray-800 dark:text-gray-200">Robot-3</span>
              </div>
              <span className="text-xs bg-red-100 text-red-800 dark:bg-red-900/30 dark:text-red-300 py-1 px-2 rounded-full">
                Offline
              </span>
            </div>
          </div>
          <button className="mt-4 text-primary-500 hover:text-primary-600 text-sm font-medium">
            View All Devices
          </button>
        </div>
      </div>
    </div>
  );
}