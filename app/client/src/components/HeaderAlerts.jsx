/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import React, { useState, useEffect, useRef } from 'react';
import PropTypes from 'prop-types';
import './HeaderAlerts.css';

// Icons
const BellIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M10 18.3333C11.0083 18.3333 11.8333 17.5083 11.8333 16.5H8.16667C8.16667 17.5083 8.99167 18.3333 10 18.3333ZM15.4167 13.8333V9.16667C15.4167 6.6 13.7083 4.48333 11.25 3.93333V3.33333C11.25 2.64167 10.6917 2.08333 10 2.08333C9.30833 2.08333 8.75 2.64167 8.75 3.33333V3.93333C6.3 4.48333 4.58333 6.59167 4.58333 9.16667V13.8333L2.75 15.6667V16.5833H17.25V15.6667L15.4167 13.8333Z" fill="currentColor"/>
  </svg>
);

const SearchIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M12.9167 11.6667H12.2583L12.025 11.4417C12.8417 10.4917 13.3333 9.25833 13.3333 7.91667C13.3333 4.925 10.9083 2.5 7.91667 2.5C4.925 2.5 2.5 4.925 2.5 7.91667C2.5 10.9083 4.925 13.3333 7.91667 13.3333C9.25833 13.3333 10.4917 12.8417 11.4417 12.025L11.6667 12.2583V12.9167L15.8333 17.075L17.075 15.8333L12.9167 11.6667ZM7.91667 11.6667C5.84167 11.6667 4.16667 9.99167 4.16667 7.91667C4.16667 5.84167 5.84167 4.16667 7.91667 4.16667C9.99167 4.16667 11.6667 5.84167 11.6667 7.91667C11.6667 9.99167 9.99167 11.6667 7.91667 11.6667Z" fill="currentColor"/>
  </svg>
);

const UserIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M10 10C12.0708 10 13.75 8.32083 13.75 6.25C13.75 4.17917 12.0708 2.5 10 2.5C7.92917 2.5 6.25 4.17917 6.25 6.25C6.25 8.32083 7.92917 10 10 10ZM10 11.875C7.49583 11.875 2.5 13.1292 2.5 15.625V17.5H17.5V15.625C17.5 13.1292 12.5042 11.875 10 11.875Z" fill="currentColor"/>
  </svg>
);

const DarkModeIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M10 2.5C5.8625 2.5 2.5 5.8625 2.5 10C2.5 14.1375 5.8625 17.5 10 17.5C14.1375 17.5 17.5 14.1375 17.5 10C17.5 5.8625 14.1375 2.5 10 2.5ZM10 15.8333C6.7875 15.8333 4.16667 13.2125 4.16667 10C4.16667 6.7875 6.7875 4.16667 10 4.16667V15.8333Z" fill="currentColor"/>
  </svg>
);

// Add hamburger menu icon
const MenuIcon = () => (
  <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M3.33333 5H16.6667M3.33333 10H16.6667M3.33333 15H16.6667" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
  </svg>
);

const HeaderAlerts = ({ isDarkMode, toggleDarkMode, toggleSidebar }) => {
  const [notifications, setNotifications] = useState([]);
  const [showNotifications, setShowNotifications] = useState(false);
  const [showUserMenu, setShowUserMenu] = useState(false);
  const [searchQuery, setSearchQuery] = useState('');
  const [showSearch, setShowSearch] = useState(false);
  
  const notificationsRef = useRef(null);
  const userMenuRef = useRef(null);
  const searchRef = useRef(null);

  useEffect(() => {
    // Fetch notifications from API
    const fetchNotifications = async () => {
      try {
        // Mock data for now - in a real app, fetch from API
        const mockNotifications = [
          { id: 1, title: 'New data available', message: 'Sensor data updated for GPS-01', time: '5 min ago', read: false },
          { id: 2, title: 'System alert', message: 'Connection lost with node ROS2-NODE-3', time: '1 hour ago', read: false },
          { id: 3, title: 'Chart saved', message: 'Chart "Temperature Sensors" saved successfully', time: '3 hours ago', read: true }
        ];
        setNotifications(mockNotifications);
      } catch (error) {
        console.error('Error fetching notifications:', error);
      }
    };

    fetchNotifications();

    // Close dropdowns when clicking outside
    const handleClickOutside = (event) => {
      if (notificationsRef.current && !notificationsRef.current.contains(event.target)) {
        setShowNotifications(false);
      }
      if (userMenuRef.current && !userMenuRef.current.contains(event.target)) {
        setShowUserMenu(false);
      }
      if (searchRef.current && !searchRef.current.contains(event.target) && !event.target.closest('.search-container')) {
        setShowSearch(false);
      }
    };

    // Use both mouse and touch events
    document.addEventListener('mousedown', handleClickOutside);
    document.addEventListener('touchstart', handleClickOutside);
    
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
      document.removeEventListener('touchstart', handleClickOutside);
    };
  }, []);

  const markAsRead = (id) => {
    setNotifications(notifications.map(notification => 
      notification.id === id ? { ...notification, read: true } : notification
    ));
  };

  const markAllAsRead = () => {
    setNotifications(notifications.map(notification => ({ ...notification, read: true })));
  };

  const unreadCount = notifications.filter(notification => !notification.read).length;

  return (
    <header className={`header ${isDarkMode ? 'header-dark' : ''}`}>
      <div className="header-left">
        <button 
          className="header-icon-button mobile-menu-toggle"
          onClick={toggleSidebar}
          aria-label="Toggle menu"
        >
          <MenuIcon />
        </button>
        <h1 className="header-title">WiseVision Dashboard</h1>
      </div>
      
      <div className="header-search" ref={searchRef}>
        {showSearch ? (
          <div className="search-container">
            <input 
              type="text" 
              className="search-input"
              placeholder="Search..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              autoFocus
            />
            <button className="search-close" onClick={() => setShowSearch(false)}>×</button>
          </div>
        ) : (
          <button className="header-icon-button" onClick={() => setShowSearch(true)}>
            <SearchIcon />
          </button>
        )}
      </div>
      
      <div className="header-right">
        <div className="notification-container" ref={notificationsRef}>
          <button 
            className="header-icon-button" 
            onClick={() => setShowNotifications(!showNotifications)}
          >
            <BellIcon />
            {unreadCount > 0 && <span className="notification-badge">{unreadCount}</span>}
          </button>
          
          {showNotifications && (
            <div className="dropdown-menu notifications-dropdown">
              <div className="dropdown-header">
                <h3>Notifications</h3>
                {unreadCount > 0 && (
                  <button className="text-button" onClick={markAllAsRead}>
                    Mark all as read
                  </button>
                )}
              </div>
              
              <div className="notifications-list">
                {notifications.length > 0 ? (
                  notifications.map(notification => (
                    <div 
                      key={notification.id} 
                      className={`notification-item ${notification.read ? 'read' : 'unread'}`}
                      onClick={() => markAsRead(notification.id)}
                    >
                      <div className="notification-content">
                        <h4 className="notification-title">{notification.title}</h4>
                        <p className="notification-message">{notification.message}</p>
                        <span className="notification-time">{notification.time}</span>
                      </div>
                      {!notification.read && <div className="unread-indicator"></div>}
                    </div>
                  ))
                ) : (
                  <div className="empty-notifications">No notifications</div>
                )}
              </div>
            </div>
          )}
        </div>
        
        <button className="header-icon-button" onClick={toggleDarkMode}>
          <DarkModeIcon />
        </button>
        
        <div className="user-menu-container" ref={userMenuRef}>
          <button 
            className="header-icon-button user-button" 
            onClick={() => setShowUserMenu(!showUserMenu)}
          >
            <UserIcon />
          </button>
          
          {showUserMenu && (
            <div className="dropdown-menu user-dropdown">
              <div className="user-info">
                <h3>User</h3>
                <p>admin@example.com</p>
              </div>
              <div className="dropdown-divider"></div>
              <button className="menu-item">
                Profile Settings
              </button>
              <button className="menu-item">
                System Preferences
              </button>
              <div className="dropdown-divider"></div>
              <button className="menu-item">
                Sign Out
              </button>
            </div>
          )}
        </div>
      </div>
    </header>
  );
};

HeaderAlerts.propTypes = {
  isDarkMode: PropTypes.bool.isRequired,
  toggleDarkMode: PropTypes.func.isRequired,
  toggleSidebar: PropTypes.func.isRequired
};

export default HeaderAlerts;
