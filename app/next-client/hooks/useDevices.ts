import { useState, useEffect, useCallback } from 'react';
import { fetchGpsDevices } from '../lib/api';
import { GpsDevicesData, GpsDevice } from '../types/ros2-types';

interface UseDevicesOptions {
  autoRefresh?: boolean;
  refreshInterval?: number;
}

interface UseDevicesReturn {
  devices: GpsDevice[];
  isLoading: boolean;
  error: string | null;
  refresh: () => Promise<void>;
  setAutoRefresh: (value: boolean) => void;
  setRefreshInterval: (value: number) => void;
  autoRefresh: boolean;
  refreshInterval: number;
  stats: {
    total: number;
    active: number;
    stationary: number;
  };
}

/**
 * Custom hook for managing device data from the ROS2 system
 * Handles loading, auto-refresh, and error states
 */
export function useDevices({
  autoRefresh: initialAutoRefresh = true,
  refreshInterval: initialRefreshInterval = 10,
}: UseDevicesOptions = {}): UseDevicesReturn {
  const [devicesData, setDevicesData] = useState<GpsDevicesData | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const [autoRefresh, setAutoRefresh] = useState<boolean>(initialAutoRefresh);
  const [refreshInterval, setRefreshInterval] = useState<number>(initialRefreshInterval);

  // Calculate stats
  const stats = {
    total: devicesData?.devices_data.length || 0,
    active: devicesData?.devices_data.filter(d => d.is_moving).length || 0,
    stationary: devicesData?.devices_data.filter(d => !d.is_moving).length || 0,
  };

  // Function to fetch device data
  const refresh = useCallback(async () => {
    try {
      setIsLoading(true);
      const data = await fetchGpsDevices();
      setDevicesData(data);
      setError(null);
    } catch (err) {
      setError('Failed to load device data. Please try again.');
      console.error('Error fetching device data:', err);
    } finally {
      setIsLoading(false);
    }
  }, []);

  // Set up initial data fetch and auto-refresh
  useEffect(() => {
    refresh();

    let intervalId: NodeJS.Timeout | undefined;
    
    if (autoRefresh) {
      intervalId = setInterval(refresh, refreshInterval * 1000);
    }
    
    return () => {
      if (intervalId) clearInterval(intervalId);
    };
  }, [autoRefresh, refreshInterval, refresh]);

  return {
    devices: devicesData?.devices_data || [],
    isLoading,
    error,
    refresh,
    setAutoRefresh,
    setRefreshInterval,
    autoRefresh,
    refreshInterval,
    stats,
  };
}