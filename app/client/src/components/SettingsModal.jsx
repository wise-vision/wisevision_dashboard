/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import React, { useState, useEffect } from 'react';
import volumeUpIcon from '../assets/images/volumeUp.png';
import muteIcon from '../assets/images/mute.png';
import unmuteSound from '../assets/sounds/notification.mp3';
import '../styles/SettingsModal.css';

const SettingsModal = ({ onClose, soundEnabled, onSettingsChange }) => {
    const [isMuted, setIsMuted] = useState(!soundEnabled);

    // Function to play sound
    const playSound = (sound) => {
        const audio = new Audio(sound);
        audio.play();
    };

    // Load mute state from localStorage on first render
    useEffect(() => {
        const savedMuteState = localStorage.getItem('isMuted') === 'true';
        setIsMuted(savedMuteState);
    }, []);

    // Function to toggle mute/unmute state
    const toggleMute = () => {
        const newMuteState = !isMuted;
        setIsMuted(newMuteState);
        localStorage.setItem('isMuted', newMuteState);

        if (onSettingsChange) {
            onSettingsChange({ soundEnabled: !newMuteState });
        }

        if (!newMuteState) {
            playSound(unmuteSound);
        }
    };

    return (
        <div className="settings-modal-overlay" onClick={onClose}>
            <div className="settings-modal" onClick={(e) => e.stopPropagation()}>
                <div className="settings-header">
                    <h2>Settings</h2>
                    <button className="close-button" onClick={onClose}>×</button>
                </div>
                <div className="settings-content">
                    <div className="settings-option">
                        <button className="volume-button" onClick={toggleMute}>
                            <img src={isMuted ? muteIcon : volumeUpIcon} alt="Volume Icon" style={{ width: '24px', height: '24px' }} />
                        </button>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default SettingsModal;
