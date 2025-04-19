'use client';

import { useState, useEffect } from 'react';
import { ROS2Topic } from '../../types/ros2-types';

interface TopicSelectorProps {
  topics: ROS2Topic[];
  selectedTopic: ROS2Topic | null;
  onSelectTopic: (name: string, type: string) => void;
  className?: string;
  isLoading?: boolean;
}

const TopicSelector: React.FC<TopicSelectorProps> = ({
  topics,
  selectedTopic,
  onSelectTopic,
  className = '',
  isLoading = false
}) => {
  const [filter, setFilter] = useState('');
  const [filteredTopics, setFilteredTopics] = useState<ROS2Topic[]>([]);
  
  // Group topics by type for better organization
  const [groupedTopics, setGroupedTopics] = useState<Record<string, ROS2Topic[]>>({});

  // Apply filter to topics when topics or filter change
  useEffect(() => {
    if (!filter) {
      setFilteredTopics(topics);
    } else {
      const lowerFilter = filter.toLowerCase();
      setFilteredTopics(
        topics.filter(
          topic => topic.name.toLowerCase().includes(lowerFilter) || 
                  topic.type.toLowerCase().includes(lowerFilter)
        )
      );
    }
  }, [topics, filter]);

  // Group topics by type
  useEffect(() => {
    const grouped: Record<string, ROS2Topic[]> = {};
    
    filteredTopics.forEach(topic => {
      if (!grouped[topic.type]) {
        grouped[topic.type] = [];
      }
      grouped[topic.type].push(topic);
    });
    
    setGroupedTopics(grouped);
  }, [filteredTopics]);

  return (
    <div className={`bg-white dark:bg-gray-800 rounded-lg shadow ${className}`}>
      <div className="p-4 border-b border-gray-200 dark:border-gray-700">
        <h3 className="text-lg font-medium text-gray-900 dark:text-gray-100">Select ROS2 Topic</h3>
        <div className="mt-2 relative">
          <input
            type="text"
            placeholder="Search topics..."
            className="input-field pl-10"
            value={filter}
            onChange={(e) => setFilter(e.target.value)}
            disabled={isLoading}
          />
          <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none">
            <svg className="h-5 w-5 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" />
            </svg>
          </div>
        </div>
      </div>
      
      <div className="max-h-80 overflow-y-auto">
        {isLoading ? (
          <div className="p-4 text-center text-gray-500 dark:text-gray-400">
            <div className="animate-spin h-5 w-5 border-2 border-primary-500 border-t-transparent rounded-full inline-block mr-2"></div>
            Loading topics...
          </div>
        ) : filteredTopics.length === 0 ? (
          <div className="p-4 text-center text-gray-500 dark:text-gray-400">
            No topics found
          </div>
        ) : (
          <div className="p-2">
            {Object.entries(groupedTopics).map(([type, typeTopics]) => (
              <div key={type} className="mb-4">
                <h4 className="text-xs font-semibold text-gray-500 dark:text-gray-400 uppercase tracking-wider px-2 mb-1">
                  {type.split('/').pop()}
                </h4>
                <div className="space-y-1">
                  {typeTopics.map(topic => (
                    <button
                      key={topic.name}
                      onClick={() => onSelectTopic(topic.name, topic.type)}
                      className={`w-full text-left px-3 py-2 rounded-md text-sm transition-colors ${
                        selectedTopic?.name === topic.name
                          ? 'bg-primary-500 text-white'
                          : 'hover:bg-gray-100 dark:hover:bg-gray-700 text-gray-700 dark:text-gray-300'
                      }`}
                    >
                      <div className="font-medium truncate">{topic.name}</div>
                      <div className="text-xs truncate opacity-70">{topic.type}</div>
                    </button>
                  ))}
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
      
      {selectedTopic && (
        <div className="p-4 border-t border-gray-200 dark:border-gray-700">
          <h4 className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">Selected Topic:</h4>
          <div className="bg-gray-50 dark:bg-gray-900 rounded p-2 text-sm">
            <div className="font-medium text-gray-900 dark:text-gray-100">{selectedTopic.name}</div>
            <div className="text-gray-500 dark:text-gray-400 text-xs">{selectedTopic.type}</div>
          </div>
        </div>
      )}
    </div>
  );
};

export default TopicSelector;