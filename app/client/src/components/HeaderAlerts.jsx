import React, { useState, useEffect, useRef, useCallback } from 'react';
import blueBellIcon from '../assets/images/blueBell.png';
import yellowBellIcon from '../assets/images/yellowBell.png';
import redBellIcon from '../assets/images/redBell.png';
import whiteBellIcon from '../assets/images/whiteBell.png';
import menuIcon from '../assets/images/menuIcon.png';
import profileIcon from '../assets/images/profile.png';
import settingsIcon from '../assets/images/settings.png';
import notificationSound from '../assets/sounds/notification.mp3'; // Import notification sound
import SettingsModal from '../components/SettingsModal';
import '../styles/HeaderAlerts.css';

const notificationLevels = {
    info: 'info',
    warning: 'warning',
    error: 'error',
};

const HeaderAlerts = () => {
    const [notifications, setNotifications] = useState([]);
    const [highlightedNotifications, setHighlightedNotifications] = useState({
        info: false,
        warning: false,
        error: false,
    });
    const [unreadNotifications, setUnreadNotifications] = useState({
        info: 0,
        warning: 0,
        error: 0,
    });
    const [activeType, setActiveType] = useState(null);
    const [showModal, setShowModal] = useState(false);
    const [showMenu, setShowMenu] = useState(false);
    const [showSettingsModal, setShowSettingsModal] = useState(false);
    const [sortCriteria, setSortCriteria] = useState('time');
    const [soundEnabled, setSoundEnabled] = useState(true); // Sound state

    const soundEnabledRef = useRef(soundEnabled);
    const timeoutRef = useRef(null);
    const isFetchingRef = useRef(false); // Flaga wskazująca, czy aktualnie trwają zapytanie

    useEffect(() => {
        soundEnabledRef.current = soundEnabled;
    }, [soundEnabled]);


    useEffect(() => {
        const storedNotifications = JSON.parse(localStorage.getItem('notifications')) || [];
        const storedUnread = JSON.parse(localStorage.getItem('unreadNotifications')) || {
            info: 0,
            warning: 0,
            error: 0,
        };
        const savedMuteState = localStorage.getItem('isMuted') === 'true';
        setSoundEnabled(!savedMuteState);
        setNotifications(storedNotifications);
        setUnreadNotifications(storedUnread);
    }, []);

    // Function to fetch data from API
    const fetchNotifications = useCallback(async () => {
        if (isFetchingRef.current) {
            return;
        }

        isFetchingRef.current = true;

        try {
            const response = await fetch(`${process.env.REACT_APP_API_BASE_URL}/api/topic_echo/notifications?type=notification_msgs/msg/Notification&number_of_msgs=1`);
            const data = await response.json();

            if (!data || !data.message || !data.message.info) {
                scheduleNextFetch();
                isFetchingRef.current = false;
                return;
            }


            const { source, severity, info } = data.message;

            const severityMap = {
                0: notificationLevels.info,
                1: notificationLevels.warning,
                2: notificationLevels.error,
            };

            const currentTime = new Date();
            const receivedAt = `${currentTime.getHours()}:${currentTime.getMinutes().toString().padStart(2, '0')}`;

            const apiNotification = {
                id: Date.now(),
                type: severityMap[severity] || notificationLevels.info,
                source: source,
                info: info,
                receivedAt: receivedAt,
            };

            // Update notifications state and localStorage
            setNotifications((prevNotifications) => {
                if (prevNotifications.some(n => n.id === apiNotification.id)) {
                    return prevNotifications;
                }
                const updatedNotifications = [...prevNotifications, apiNotification];
                localStorage.setItem('notifications', JSON.stringify(updatedNotifications));
                return updatedNotifications;
            });

            setHighlightedNotifications((prevState) => ({
                ...prevState,
                [apiNotification.type]: true,
            }));

            setUnreadNotifications((prev) => {
                const updatedUnread = {
                    ...prev,
                    [apiNotification.type]: prev[apiNotification.type] + 1,
                };
                localStorage.setItem('unreadNotifications', JSON.stringify(updatedUnread));
                return updatedUnread;
            });


            if (soundEnabledRef.current) {
                const audio = new Audio(notificationSound);
                audio.play();
            }
        } catch (error) {
            console.error('Error fetching notifications:', error);
        } finally {
            scheduleNextFetch();
            isFetchingRef.current = false;
        }
    }, []);

    const scheduleNextFetch = () => {
        timeoutRef.current = setTimeout(() => {
            fetchNotifications();
        }, 5000);
    };

    useEffect(() => {
        fetchNotifications();

        return () => {
            if (timeoutRef.current) {
                clearTimeout(timeoutRef.current);
            }
        };
    }, [fetchNotifications]);

    const handleNotificationClick = (type) => {
        setActiveType(type);
        setShowModal(true);
        setHighlightedNotifications((prev) => ({
            ...prev,
            [type]: false,
        }));
        setUnreadNotifications((prev) => ({
            ...prev,
            [type]: 0,
        }));

        // Reset unread notifications in localStorage
        const storedUnread = JSON.parse(localStorage.getItem('unreadNotifications')) || {};
        storedUnread[type] = 0;
        localStorage.setItem('unreadNotifications', JSON.stringify(storedUnread));
    };

    const handleCloseModal = () => {
        setShowModal(false);
        setActiveType(null);
    };

    const toggleMenu = () => {
        setShowMenu((prev) => !prev);
    };

    const openSettingsModal = () => {
        setShowSettingsModal(true);
        setShowMenu(false);
    };

    const closeSettingsModal = () => {
        setShowSettingsModal(false);
    };

    const getNotificationIcon = (type, isHighlighted) => {
        if (isHighlighted) {
            return whiteBellIcon;
        }
        switch (type) {
            case notificationLevels.error:
                return redBellIcon;
            case notificationLevels.warning:
                return yellowBellIcon;
            default:
                return blueBellIcon;
        }
    };

    const clearNotifications = (type) => {
        const filteredNotifications = notifications.filter(notification => notification.type !== type);
        setNotifications(filteredNotifications);
        setUnreadNotifications((prev) => ({
            ...prev,
            [type]: 0,
        }));

        // Update localStorage
        localStorage.setItem('notifications', JSON.stringify(filteredNotifications));
        const storedUnread = JSON.parse(localStorage.getItem('unreadNotifications')) || {};
        storedUnread[type] = 0;
        localStorage.setItem('unreadNotifications', JSON.stringify(storedUnread));
    };

    const handleSettingsChange = (settings) => {
        if (settings.soundEnabled !== undefined) {
            setSoundEnabled(settings.soundEnabled);
        }
    };

    return (
        <div>
            <div className="header-alerts">
                {Object.keys(notificationLevels).map((type) => (
                    <div
                        key={type}
                        className={`alert-section ${highlightedNotifications[type] ? `highlighted ${type}` : ''}`}
                        onClick={() => handleNotificationClick(type)}
                    >
                        <div className="alert-icon">
                            <img src={getNotificationIcon(type, highlightedNotifications[type])} alt={`${type} Icon`} />
                            {unreadNotifications[type] > 0 && (
                                <span className="notification-badge">{unreadNotifications[type]}</span>
                            )}
                        </div>
                        <div className="alert-text">
                            <span>{type}</span> alerts
                        </div>
                    </div>
                ))}
                <div className="menu-icon">
                    <button className="menu-button" onClick={toggleMenu}>
                        <img src={menuIcon} alt="Menu Icon" />
                    </button>
                    {showMenu && (
                        <div className="dropdown-menu">
                            <div className="menu-item">
                                <img src={profileIcon} alt="Profile Icon" />
                                <span>Profile</span>
                            </div>
                            <div className="menu-item" onClick={openSettingsModal}>
                                <img src={settingsIcon} alt="Settings Icon" />
                                <span>Settings</span>
                            </div>
                        </div>
                    )}
                </div>
            </div>

            {showModal && activeType && (
                <div className="modal" onClick={handleCloseModal}>
                    <div className="modal-content" onClick={(e) => e.stopPropagation()}>
                        <div className="modal-header">
                            <h2>{activeType.toUpperCase()} Alerts</h2>
                            <select
                                className="sort-select"
                                value={sortCriteria}
                                onChange={(e) => setSortCriteria(e.target.value)}
                            >
                                <option value="time">Sort by Time</option>
                                <option value="source">Sort by Source</option>
                                <option value="severity">Sort by Severity</option>
                            </select>
                            <button className="close-button" onClick={handleCloseModal}>×</button>
                        </div>
                        <div className="modal-actions">
                            <button className="clear-button" onClick={() => clearNotifications(activeType)}>Clear {activeType} Notifications</button>
                        </div>
                        <div className="scrollable-content">
                            <div className="notifications-list">
                                {notifications
                                    .filter((notification) => notification.type === activeType)
                                    .sort((a, b) => {
                                        if (sortCriteria === 'time') {
                                            return new Date(`1970/01/01 ${a.receivedAt}`) - new Date(`1970/01/01 ${b.receivedAt}`);
                                        } else if (sortCriteria === 'source') {
                                            return a.source.localeCompare(b.source);
                                        } else if (sortCriteria === 'severity') {
                                            return Object.values(notificationLevels).indexOf(a.type) - Object.values(notificationLevels).indexOf(b.type);
                                        }
                                        return 0;
                                    })
                                    .map((notification, index) => (
                                        <div key={notification.id} className="notification-item">
                                            <div className="notification-header">
                                                <span className="notification-number">{index + 1}.</span>
                                                <span className="notification-source">
                                                    <strong>Source:</strong> {notification.source}
                                                </span>
                                                <span className="notification-received">
                                                    <strong>Received At:</strong> {notification.receivedAt}
                                                </span>
                                            </div>
                                            <div className="notification-info">
                                                <strong>Info:</strong> {notification.info}
                                            </div>
                                        </div>
                                    ))}
                            </div>
                        </div>
                    </div>
                </div>
            )}

            {showSettingsModal && (
                <SettingsModal
                    onClose={closeSettingsModal}
                    soundEnabled={soundEnabled}
                    onSettingsChange={handleSettingsChange}
                />
            )}
        </div>
    );
};

export default HeaderAlerts;
