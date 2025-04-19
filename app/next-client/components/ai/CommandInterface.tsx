import { useCallback, useEffect, useState } from "react";
import { Button, Card, Input, Select, Spinner, Textarea } from "../ui";
import { Command, CommandHistoryItem } from "../../types/ai-types";

interface CommandInterfaceProps {
  className?: string;
}

/**
 * CommandInterface component for interacting with robots using natural language commands
 * 
 * This component provides a chat-like interface for sending commands to robots
 * via OpenAI's natural language processing API. It displays command history and
 * allows administrators to send new commands.
 */
export default function CommandInterface({ className = "" }: CommandInterfaceProps) {
  const [command, setCommand] = useState("");
  const [loading, setLoading] = useState(false);
  const [availableDevices, setAvailableDevices] = useState<string[]>([]);
  const [selectedDevices, setSelectedDevices] = useState<string[]>([]);
  const [commandHistory, setCommandHistory] = useState<CommandHistoryItem[]>([]);
  const [error, setError] = useState<string | null>(null);

  // Fetch available devices on component mount
  useEffect(() => {
    const fetchDevices = async () => {
      try {
        // In a real implementation, this would fetch available devices from the API
        const demoDevices = [
          "robot_1", 
          "robot_2", 
          "delivery_bot", 
          "survey_drone"
        ];
        setAvailableDevices(demoDevices);
      } catch (err) {
        console.error("Failed to fetch devices:", err);
        setError("Failed to fetch available devices");
      }
    };

    fetchDevices();
  }, []);

  // Fetch command history on component mount
  useEffect(() => {
    const fetchCommandHistory = async () => {
      try {
        const response = await fetch("/api/v1/ai/commands/history", {
          method: "GET",
          headers: {
            "Content-Type": "application/json",
            // In a real implementation, this would include authentication
            "Authorization": `Bearer ${localStorage.getItem("token")}`
          }
        });

        if (!response.ok) {
          throw new Error(`HTTP error! Status: ${response.status}`);
        }

        const data = await response.json();
        setCommandHistory(data.history || []);
      } catch (err) {
        console.error("Failed to fetch command history:", err);
        // Don't show an error for this, just log it
      }
    };

    fetchCommandHistory();
  }, []);

  // Send a command to the API
  const sendCommand = useCallback(async () => {
    if (!command.trim()) {
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await fetch("/api/v1/ai/commands", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          // In a real implementation, this would include authentication
          "Authorization": `Bearer ${localStorage.getItem("token")}`
        },
        body: JSON.stringify({
          command_text: command,
          device_ids: selectedDevices.length > 0 ? selectedDevices : undefined
        })
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || `HTTP error! Status: ${response.status}`);
      }

      const data = await response.json();
      
      // Add the command to history
      setCommandHistory(prev => [
        {
          id: data.command_id || Date.now().toString(),
          timestamp: new Date().toISOString(),
          command_text: command,
          command_type: data.command_type || "unknown",
          robot_id: data.robot_id || "unknown",
          parameters: data.parameters || {},
          success: data.success || false,
          response: data.message || "",
        },
        ...prev
      ]);

      // Clear the command input
      setCommand("");
    } catch (err: any) {
      console.error("Failed to send command:", err);
      setError(err.message || "Failed to send command");
    } finally {
      setLoading(false);
    }
  }, [command, selectedDevices]);

  // Clear command history
  const clearHistory = useCallback(async () => {
    try {
      const response = await fetch("/api/v1/ai/commands/clear-history", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          // In a real implementation, this would include authentication
          "Authorization": `Bearer ${localStorage.getItem("token")}`
        }
      });

      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`);
      }

      setCommandHistory([]);
    } catch (err) {
      console.error("Failed to clear command history:", err);
      setError("Failed to clear command history");
    }
  }, []);

  return (
    <div className={`flex flex-col space-y-4 ${className}`}>
      <Card className="p-4">
        <h2 className="text-xl font-bold mb-2">Robot Command Interface</h2>
        <p className="text-sm text-gray-500 dark:text-gray-400 mb-4">
          Send natural language commands to robots. Use this interface to control robots
          with simple English commands.
        </p>

        <div className="mb-4">
          <label className="block text-sm font-medium mb-2">Target Devices</label>
          <Select
            multiple
            value={selectedDevices}
            onChange={(e) => {
              const options = Array.from(e.target.selectedOptions, option => option.value);
              setSelectedDevices(options);
            }}
            className="w-full"
          >
            <option value="">All Available Devices</option>
            {availableDevices.map((device) => (
              <option key={device} value={device}>
                {device}
              </option>
            ))}
          </Select>
          <p className="text-xs text-gray-500 mt-1">
            Leave empty to send command to all available devices
          </p>
        </div>

        <div className="mb-4">
          <label className="block text-sm font-medium mb-2">Command</label>
          <Textarea
            value={command}
            onChange={(e) => setCommand(e.target.value)}
            placeholder="Enter your command in natural language, e.g., 'Move robot_1 to the kitchen'"
            rows={3}
            className="w-full"
          />
        </div>

        {error && (
          <div className="mb-4 p-3 bg-red-100 dark:bg-red-900 text-red-800 dark:text-red-200 rounded-md">
            {error}
          </div>
        )}

        <div className="flex justify-between">
          <Button variant="secondary" onClick={clearHistory} disabled={loading || commandHistory.length === 0}>
            Clear History
          </Button>
          <Button onClick={sendCommand} disabled={!command.trim() || loading}>
            {loading ? <Spinner size="sm" /> : "Send Command"}
          </Button>
        </div>
      </Card>

      <Card className="p-4">
        <h3 className="text-lg font-semibold mb-2">Command History</h3>
        
        {commandHistory.length === 0 ? (
          <p className="text-gray-500 dark:text-gray-400 text-center py-4">
            No command history yet
          </p>
        ) : (
          <div className="space-y-3 max-h-80 overflow-auto">
            {commandHistory.map((item) => (
              <div 
                key={item.id || item.timestamp} 
                className={`p-3 rounded-lg ${
                  item.success 
                    ? "bg-green-50 dark:bg-green-900/20 border-l-4 border-green-500" 
                    : "bg-red-50 dark:bg-red-900/20 border-l-4 border-red-500"
                }`}
              >
                <div className="flex justify-between items-start">
                  <p className="font-medium">{item.command_text}</p>
                  <span className="text-xs text-gray-500">
                    {new Date(item.timestamp).toLocaleTimeString()}
                  </span>
                </div>
                <div className="mt-1 text-sm">
                  <span className="font-medium">Robot:</span> {item.robot_id}
                </div>
                <div className="mt-1 text-sm">
                  <span className="font-medium">Command:</span> {item.command_type}
                  {Object.keys(item.parameters).length > 0 && (
                    <span className="ml-1 text-xs">
                      ({JSON.stringify(item.parameters)})
                    </span>
                  )}
                </div>
                {item.response && (
                  <div className="mt-1 text-sm">
                    <span className="font-medium">Response:</span> {item.response}
                  </div>
                )}
              </div>
            ))}
          </div>
        )}
      </Card>
    </div>
  );
}