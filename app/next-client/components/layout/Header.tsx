'use client';

import { useState, useEffect } from 'react';
// Update the import path for Heroicons v2 (assuming 24px outline icons)
import { 
  BellIcon, 
  Bars3Icon, // Note: MenuIcon is often renamed to Bars3Icon in v2
  MoonIcon, 
  SunIcon,
  MagnifyingGlassIcon // Note: SearchIcon is often renamed to MagnifyingGlassIcon in v2
} from '@heroicons/react/24/outline'; 
import { useTheme } from '../theme-provider';

interface HeaderProps {
  toggleSidebar: () => void;
}

export default function Header({ toggleSidebar }: HeaderProps) {
  const { theme, setTheme } = useTheme();
  const [notifications, setNotifications] = useState<Array<{id: string, message: string, time: string}>>([]);
  const [showNotifications, setShowNotifications] = useState(false);
  const [isMounted, setIsMounted] = useState(false);

  // Ensure hooks are not conditionally rendered
  useEffect(() => {
    setIsMounted(true);
  }, []);

  // Example notifications - in a real app, these would come from an API
  useEffect(() => {
    if (isMounted) {
      setNotifications([
        { id: '1', message: 'Device temperature alert: Robot-1 exceeding threshold', time: '2 min ago' },
        { id: '2', message: 'New device connected: LoRaWAN-Sensor-45', time: '10 min ago' },
        { id: '3', message: 'System update available for dashboard', time: '25 min ago' },
      ]);
    }
  }, [isMounted]);

  const toggleTheme = () => {
    setTheme(theme === 'dark' ? 'light' : 'dark');
  };

  if (!isMounted) {
    // Don't render theme toggle button until mounted on client
    // to avoid hydration mismatch
    return null; 
  }

  return (
    <header className="bg-white dark:bg-gray-900 shadow-sm border-b dark:border-gray-700">
      <div className="px-4 py-3 flex items-center justify-between">
        <div className="flex items-center">
          <button 
            onClick={toggleSidebar} 
            className="p-2 rounded-md text-gray-500 hover:bg-gray-100 dark:hover:bg-gray-800 md:hidden"
          >
            <Bars3Icon className="h-6 w-6" />
          </button>
          
          <div className="relative ml-4">
            <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none">
              <MagnifyingGlassIcon className="h-5 w-5 text-gray-400" />
            </div>
            <input
              type="text"
              placeholder="Search..."
              className="pl-10 pr-4 py-2 border border-gray-300 dark:border-gray-700 rounded-md bg-gray-50 dark:bg-gray-800 text-gray-900 dark:text-gray-100 focus:outline-none focus:ring-2 focus:ring-primary-500"
            />
          </div>
        </div>

        <div className="flex items-center space-x-4">
          <button 
            onClick={toggleTheme} 
            className="p-2 rounded-md text-gray-500 hover:bg-gray-100 dark:hover:bg-gray-800"
          >
            {theme === 'dark' ? (
              <SunIcon className="h-5 w-5" />
            ) : (
              <MoonIcon className="h-5 w-5" />
            )}
          </button>
          
          <div className="relative">
            <button 
              onClick={() => setShowNotifications(!showNotifications)} 
              className="p-2 rounded-md text-gray-500 hover:bg-gray-100 dark:hover:bg-gray-800 relative"
            >
              <BellIcon className="h-5 w-5" />
              {notifications.length > 0 && (
                <span className="absolute top-0 right-0 block h-2 w-2 rounded-full bg-red-500 transform translate-x-1 -translate-y-1" />
              )}
            </button>
            
            {showNotifications && (
              <div className="absolute right-0 mt-2 w-80 bg-white dark:bg-gray-800 rounded-md shadow-lg py-1 z-10 border dark:border-gray-700">
                <div className="px-4 py-2 border-b dark:border-gray-700">
                  <h3 className="text-sm font-medium text-gray-900 dark:text-gray-100">Notifications</h3>
                </div>
                <div className="max-h-96 overflow-y-auto">
                  {notifications.length > 0 ? (
                    <div>
                      {notifications.map((notification) => (
                        <div 
                          key={notification.id} 
                          className="px-4 py-3 hover:bg-gray-50 dark:hover:bg-gray-700 border-b dark:border-gray-700 last:border-0"
                        >
                          <p className="text-sm text-gray-800 dark:text-gray-200">{notification.message}</p>
                          <p className="text-xs text-gray-500 mt-1">{notification.time}</p>
                        </div>
                      ))}
                    </div>
                  ) : (
                    <div className="px-4 py-3 text-sm text-gray-500">
                      No new notifications
                    </div>
                  )}
                </div>
                {notifications.length > 0 && (
                  <div className="px-4 py-2 border-t dark:border-gray-700">
                    <button className="text-sm text-primary-500 hover:text-primary-600 font-medium">
                      View all notifications
                    </button>
                  </div>
                )}
              </div>
            )}
          </div>
        </div>
      </div>
    </header>
  );
}