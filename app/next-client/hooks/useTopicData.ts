import { useState, useEffect, useCallback } from 'react';
import { fetchTopics, fetchTopicData } from '../lib/api';
import { ROS2Topic, FullDateTime } from '../types/ros2-types';

interface UseTopicDataOptions {
  topicName?: string;
  topicType?: string;
  autoRefresh?: boolean;
  refreshInterval?: number;
}

interface UseTopicDataReturn {
  topics: ROS2Topic[];
  selectedTopic: ROS2Topic | null;
  selectTopic: (name: string, type: string) => void;
  data: any[];
  timestamps: FullDateTime[];
  isLoading: boolean;
  isLoadingTopicList: boolean;
  error: string | null;
  refresh: () => Promise<void>;
  autoRefresh: boolean;
  setAutoRefresh: (value: boolean) => void;
  refreshInterval: number;
  setRefreshInterval: (value: number) => void;
}

/**
 * Custom hook for fetching and managing ROS2 topic data
 */
export function useTopicData({
  topicName = '',
  topicType = '',
  autoRefresh = true,
  refreshInterval = 10
}: UseTopicDataOptions = {}): UseTopicDataReturn {
  // State for available topics list
  const [topics, setTopics] = useState<ROS2Topic[]>([]);
  const [isLoadingTopicList, setIsLoadingTopicList] = useState(true);
  
  // State for selected topic
  const [selectedTopic, setSelectedTopic] = useState<ROS2Topic | null>(
    topicName && topicType ? { name: topicName, type: topicType } : null
  );
  
  // State for topic data
  const [data, setData] = useState<any[]>([]);
  const [timestamps, setTimestamps] = useState<FullDateTime[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  
  // State for auto-refresh settings
  const [shouldAutoRefresh, setShouldAutoRefresh] = useState<boolean>(autoRefresh);
  const [dataRefreshInterval, setDataRefreshInterval] = useState<number>(refreshInterval);

  // Function to fetch list of topics
  const fetchTopicsList = useCallback(async () => {
    try {
      setIsLoadingTopicList(true);
      const result = await fetchTopics();
      setTopics(result);
    } catch (err) {
      console.error('Error fetching topics:', err);
      setError('Failed to load topics list');
    } finally {
      setIsLoadingTopicList(false);
    }
  }, []);

  // Function to select a topic
  const selectTopic = useCallback((name: string, type: string) => {
    setSelectedTopic({ name, type });
    // Reset data when changing topics
    setData([]);
    setTimestamps([]);
  }, []);

  // Function to fetch data from selected topic
  const fetchSelectedTopicData = useCallback(async () => {
    if (!selectedTopic) return;
    
    try {
      setIsLoading(true);
      setError(null);
      const result = await fetchTopicData(selectedTopic.name, selectedTopic.type);
      
      // Process the data based on the message type
      if (result && result.message) {
        if (Array.isArray(result.message)) {
          setData(result.message);
        } else if (typeof result.message === 'object' && result.timestamps) {
          // Handle structured data with timestamps
          setData([result.message]);
          setTimestamps(Array.isArray(result.timestamps) ? result.timestamps : []);
        } else {
          // Simple scalar value
          setData([result.message]);
        }
      }
    } catch (err) {
      console.error(`Error fetching data for topic ${selectedTopic.name}:`, err);
      setError(`Failed to load data for topic ${selectedTopic.name}`);
    } finally {
      setIsLoading(false);
    }
  }, [selectedTopic]);

  // Combined refresh function
  const refresh = useCallback(async () => {
    if (selectedTopic) {
      await fetchSelectedTopicData();
    } else {
      await fetchTopicsList();
    }
  }, [selectedTopic, fetchSelectedTopicData, fetchTopicsList]);

  // Fetch topics list on initial load
  useEffect(() => {
    fetchTopicsList();
  }, [fetchTopicsList]);

  // Fetch topic data when selected topic changes
  useEffect(() => {
    if (selectedTopic) {
      fetchSelectedTopicData();
    }
  }, [selectedTopic, fetchSelectedTopicData]);

  // Set up auto-refresh for topic data
  useEffect(() => {
    if (!selectedTopic || !shouldAutoRefresh) return;

    const intervalId = setInterval(() => {
      fetchSelectedTopicData();
    }, dataRefreshInterval * 1000);

    return () => clearInterval(intervalId);
  }, [selectedTopic, shouldAutoRefresh, dataRefreshInterval, fetchSelectedTopicData]);

  return {
    topics,
    selectedTopic,
    selectTopic,
    data,
    timestamps,
    isLoading,
    isLoadingTopicList,
    error,
    refresh,
    autoRefresh: shouldAutoRefresh,
    setAutoRefresh: setShouldAutoRefresh,
    refreshInterval: dataRefreshInterval,
    setRefreshInterval: setDataRefreshInterval
  };
}