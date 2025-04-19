/**
 * API utilities for WiseVision Dashboard
 */

const API_BASE_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:5000';

/**
 * Fetches GPS device data from the backend
 * @returns {Promise<GpsDeviceData>} - Promise that resolves to GPS device data
 */
export async function fetchGpsDevices() {
  try {
    const response = await fetch(`${API_BASE_URL}/api/topic_echo_gps_devices`);
    
    if (!response.ok) {
      throw new Error(`Failed to fetch GPS devices: ${response.status}`);
    }
    
    const data = await response.json();
    return data.message;
  } catch (error) {
    console.error('Error fetching GPS devices:', error);
    throw error;
  }
}

/**
 * Fetches available topics from the backend
 * @returns {Promise<Array<{name: string, type: string}>>} - Promise that resolves to an array of topics
 */
export async function fetchTopics(options = {}) {
  try {
    const queryParams = new URLSearchParams(options);
    const response = await fetch(`${API_BASE_URL}/api/topics?${queryParams}`);
    
    if (!response.ok) {
      throw new Error(`Failed to fetch topics: ${response.status}`);
    }
    
    return await response.json();
  } catch (error) {
    console.error('Error fetching topics:', error);
    throw error;
  }
}

/**
 * Fetches data from a specific topic
 * @param {string} topicName - The name of the topic
 * @param {string} topicType - The type of the topic
 * @returns {Promise<any>} - Promise that resolves to topic data
 */
export async function fetchTopicData(topicName, topicType) {
  try {
    // Handle topics with slashes by encoding them
    const encodedTopicName = topicName.startsWith('/') 
      ? encodeURIComponent(topicName.substring(1)) 
      : encodeURIComponent(topicName);
      
    const response = await fetch(
      `${API_BASE_URL}/api/topic_echo/${encodedTopicName}?type=${encodeURIComponent(topicType)}`
    );
    
    if (!response.ok) {
      throw new Error(`Failed to fetch topic data: ${response.status}`);
    }
    
    return await response.json();
  } catch (error) {
    console.error(`Error fetching data for topic ${topicName}:`, error);
    throw error;
  }
}