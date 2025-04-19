/**
 * TypeScript types for AI command interface
 */

/**
 * Command data structure for sending robot commands through OpenAI
 */
export interface Command {
  command_text: string;
  device_ids?: string[];
}

/**
 * Command response structure received from the backend
 */
export interface CommandResponse {
  success: boolean;
  message: string;
  command_id: string;
  timestamp: string;
  command_type?: string;
  robot_id?: string;
  parameters?: Record<string, any>;
}

/**
 * Command history item structure for displaying past commands
 */
export interface CommandHistoryItem {
  id?: string;
  timestamp: string;
  command_text: string;
  command_type: string;
  robot_id: string;
  parameters: Record<string, any>;
  success: boolean;
  response?: string;
}

/**
 * Command history response from the backend API
 */
export interface CommandHistoryResponse {
  history: CommandHistoryItem[];
}

/**
 * Supported commands response from the backend API
 */
export interface SupportedCommandsResponse {
  commands: string[];
}