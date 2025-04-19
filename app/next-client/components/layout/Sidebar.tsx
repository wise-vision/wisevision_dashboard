'use client';

import React from 'react';
import Link from 'next/link';
import { usePathname } from 'next/navigation';
import { 
  HomeIcon, 
  ChartBarIcon, 
  CogIcon, 
  BellIcon, 
  MapIcon, 
  ChevronLeftIcon, 
  ChevronRightIcon,
  ServerIcon, 
  DocumentChartBarIcon as DocumentReportIcon,
  SparklesIcon 
} from '@heroicons/react/24/outline';
import { useAuth } from '../../hooks/useAuth';

interface SidebarProps {
  collapsed: boolean;
  setCollapsed: (collapsed: boolean) => void;
}

export default function Sidebar({ collapsed, setCollapsed }: SidebarProps) {
  const pathname = usePathname();
  const { user } = useAuth();
  
  const navItems = [
    { name: 'Dashboard', href: '/dashboard', icon: HomeIcon },
    { name: 'Charts', href: '/dashboard/charts', icon: ChartBarIcon },
    { name: 'Maps', href: '/dashboard/maps', icon: MapIcon },
    { name: 'Devices', href: '/dashboard/devices', icon: ServerIcon },
    { name: 'Alerts', href: '/dashboard/alerts', icon: BellIcon },
    { name: 'Reports', href: '/dashboard/reports', icon: DocumentReportIcon },
    // Only show AI command interface to admin users
    ...(user?.role === 'admin' ? [{ name: 'AI Commands', href: '/dashboard/ai', icon: SparklesIcon }] : []),
    { name: 'Settings', href: '/dashboard/settings', icon: CogIcon },
  ];

  return (
    <aside className={`fixed inset-y-0 left-0 z-40 w-64 bg-white dark:bg-gray-900 border-r dark:border-gray-800 transition-all duration-300 ease-in-out transform ${
      collapsed ? 'md:w-16 -translate-x-full md:translate-x-0' : 'translate-x-0'
    }`}>
      <div className="flex flex-col h-full">
        <div className="flex items-center justify-between p-4 border-b dark:border-gray-700">
          <Link href="/dashboard" className={`text-xl font-bold text-blue-600 dark:text-blue-400 transition-opacity duration-200 ${collapsed ? 'opacity-0 md:opacity-0 hidden md:hidden' : ''}`}>
            WiseVision
          </Link>
          <button 
            onClick={() => setCollapsed(!collapsed)} 
            className="p-2 rounded-md text-gray-500 hover:bg-gray-100 dark:hover:bg-gray-800 focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            {collapsed ? (
              <ChevronRightIcon className="h-5 w-5" />
            ) : (
              <ChevronLeftIcon className="h-5 w-5" />
            )}
          </button>
        </div>
        
        <nav className="flex-1 pt-4 pb-4 overflow-y-auto">
          <ul className="space-y-1 px-3">
            {navItems.map((item) => (
              <li key={item.name}>
                <Link 
                  href={item.href}
                  className={`flex items-center p-3 rounded-md transition-colors ${
                    pathname === item.href 
                      ? 'bg-blue-600 text-white' 
                      : 'text-gray-700 hover:bg-gray-100 dark:text-gray-300 dark:hover:bg-gray-800'
                  }`}
                >
                  <item.icon className={`h-5 w-5 ${pathname === item.href ? 'text-white' : 'text-gray-500 dark:text-gray-400'}`} />
                  <span className={`ml-3 transition-opacity duration-200 ${collapsed ? 'opacity-0 md:opacity-0 hidden md:hidden' : ''}`}>
                    {item.name}
                  </span>
                </Link>
              </li>
            ))}
          </ul>
        </nav>
        
        <div className="p-4 border-t dark:border-gray-700">
          <div className={`flex items-center ${collapsed ? 'justify-center' : 'justify-start'}`}>
            <div className="w-8 h-8 rounded-full bg-blue-600 flex items-center justify-center text-white">
              {user?.username?.charAt(0).toUpperCase() || 'U'}
            </div>
            {!collapsed && (
              <div className="ml-3">
                <p className="text-sm font-medium text-gray-700 dark:text-gray-300">
                  {user?.full_name || user?.username || 'User'}
                </p>
                <p className="text-xs text-gray-500 dark:text-gray-400">
                  {user?.email || ''}
                </p>
              </div>
            )}
          </div>
        </div>
      </div>
    </aside>
  );
}