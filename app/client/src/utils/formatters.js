/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/**
 * Format a timestamp object from ROS2 message to a human-readable string
 * @param {Object} timestamp - ROS2 timestamp object with year, month, day, etc.
 * @param {boolean} includeTime - Whether to include time in the output
 * @returns {string} Formatted date string
 */
export const formatTimestamp = (timestamp, includeTime = true) => {
  if (!timestamp) return 'N/A';
  
  try {
    const { year, month, day, hour, minute, second } = timestamp;
    
    const dateString = `${year}-${String(month).padStart(2, '0')}-${String(day).padStart(2, '0')}`;
    
    if (!includeTime) return dateString;
    
    return `${dateString} ${String(hour).padStart(2, '0')}:${String(minute).padStart(2, '0')}:${String(second).padStart(2, '0')}`;
  } catch (error) {
    console.error('Error formatting timestamp:', error);
    return 'Invalid date';
  }
};

/**
 * Format a number with specified precision
 * @param {number} value - Number to format
 * @param {number} precision - Number of decimal places
 * @returns {string} Formatted number
 */
export const formatNumber = (value, precision = 2) => {
  if (value === undefined || value === null) return 'N/A';
  
  if (typeof value !== 'number') {
    try {
      value = parseFloat(value);
    } catch (error) {
      return 'Invalid number';
    }
  }
  
  return value.toFixed(precision);
};

/**
 * Truncate a string if it's longer than maxLength
 * @param {string} str - String to truncate
 * @param {number} maxLength - Maximum length before truncation
 * @returns {string} Truncated string
 */
export const truncateString = (str, maxLength = 30) => {
  if (!str) return '';
  
  if (str.length <= maxLength) return str;
  
  return `${str.substring(0, maxLength)}...`;
};

/**
 * Convert bytes to a human-readable string (KB, MB, GB)
 * @param {number} bytes - Number of bytes
 * @returns {string} Human-readable size
 */
export const formatBytes = (bytes) => {
  if (bytes === 0) return '0 Bytes';
  
  const k = 1024;
  const sizes = ['Bytes', 'KB', 'MB', 'GB', 'TB'];
  const i = Math.floor(Math.log(bytes) / Math.log(k));
  
  return `${parseFloat((bytes / Math.pow(k, i)).toFixed(2))} ${sizes[i]}`;
};

/**
 * Convert a buffer array to a hex string
 * @param {Array} buffer - Array of bytes
 * @returns {string} Hex string representation
 */
export const bufferToHexString = (buffer) => {
  if (!buffer || !Array.isArray(buffer)) return 'invalid';
  
  return buffer.map(b => b.toString(16).padStart(2, '0')).join('');
};

/**
 * Generate random ID for elements
 * @returns {string} Random ID
 */
export const generateId = () => {
  return Math.random().toString(36).substring(2, 9);
};
