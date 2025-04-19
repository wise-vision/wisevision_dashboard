/**
 * Type definitions for ROS2 data structures used in the WiseVision Dashboard
 */

// GPS Device Data Types
export interface DeviceEUI {
  data: number[];
}

export interface NavValue {
  latitude: number;
  longitude: number;
  altitude: number;
  header?: {
    frame_id: string;
    stamp: {
      sec: number;
      nanosec: number;
    };
  };
  position_covariance?: number[];
  position_covariance_type?: number;
  status?: {
    status: number;
    service: number;
  };
}

export interface GpsDevice {
  device_name: string;
  device_eui: DeviceEUI;
  nav_value: NavValue;
  is_moving: boolean;
}

export interface GpsDevicesData {
  devices_data: GpsDevice[];
}

// ROS2 Message Types
export interface ROS2Topic {
  name: string;
  type: string;
}

export interface FullDateTime {
  year: number;
  month: number;
  day: number;
  hour: number;
  minute: number;
  second: number;
  nanosecond: number;
}

// Sensor Data Types
export interface TPBValue {
  value_type: number;
  temperature: number;
  pressure: number;
  binary_value: boolean;
}

export interface SensorData {
  id: number;
  tpb_value: TPBValue;
}

export interface MicroPublisherData {
  sensors_data: SensorData[];
}

// API Response Types
export interface TopicEchoResponse {
  message: any;
}

export interface GetMessagesResponse {
  int32_msgs: number[];
  micro_publisher_data: MicroPublisherData[];
  timestamps: FullDateTime[];
}

export interface TopicDataResponse<T> {
  messages: T[];
  timestamps: FullDateTime[];
}