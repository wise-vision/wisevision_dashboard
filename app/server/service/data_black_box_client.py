"""
DataBlackBoxClient Service

This service provides a client interface for interacting with the wisevision_data_black_box,
which manages the storage of time-series data in InfluxDB.

It handles:
- Connecting to the data black box through its REST API or gRPC service
- Querying historical topic data
- Retrieving aggregated metrics
- Managing database creation and storage assignments
"""

import json
import logging
import os
import requests
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional, Union

# Set up logging
logger = logging.getLogger(__name__)

class DataBlackBoxClient:
    """
    Client for interacting with wisevision_data_black_box
    """
    _instance = None
    _initialized = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(DataBlackBoxClient, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        # Get configuration from environment variables with defaults
        self.base_url = os.environ.get('DATA_BLACK_BOX_URL', 'http://localhost:8000')
        self.timeout = int(os.environ.get('DATA_BLACK_BOX_TIMEOUT', '5'))
        
        # By default, use retries and session connection pooling
        self.session = requests.Session()
        self.session.mount('http://', requests.adapters.HTTPAdapter(
            max_retries=3,
            pool_connections=10,
            pool_maxsize=20
        ))
        self.session.mount('https://', requests.adapters.HTTPAdapter(
            max_retries=3,
            pool_connections=10,
            pool_maxsize=20
        ))
        
        # Keep track of created topics for efficient storage
        self._known_topics = set()
        
        self._initialized = True
        logger.info(f"DataBlackBoxClient initialized with base URL: {self.base_url}")

    def _request(self, method: str, endpoint: str, **kwargs) -> Dict[str, Any]:
        """
        Send a request to the data black box API
        
        Args:
            method: HTTP method (GET, POST, etc.)
            endpoint: API endpoint (without base URL)
            **kwargs: Additional arguments to pass to requests
            
        Returns:
            Response JSON as dictionary
        """
        url = f"{self.base_url}/{endpoint.lstrip('/')}"
        
        # Ensure timeout is set
        if 'timeout' not in kwargs:
            kwargs['timeout'] = self.timeout
        
        try:
            response = self.session.request(method, url, **kwargs)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.HTTPError as e:
            logger.error(f"HTTP error from Data Black Box: {e}")
            if response.text:
                try:
                    error_details = response.json()
                    logger.error(f"Error details: {error_details}")
                except json.JSONDecodeError:
                    logger.error(f"Error response: {response.text}")
            raise
        except requests.exceptions.RequestException as e:
            logger.error(f"Request error to Data Black Box: {e}")
            raise
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse response from Data Black Box: {e}")
            raise ValueError(f"Invalid response from Data Black Box: {response.text}")

    def check_health(self) -> Dict[str, Any]:
        """
        Check the health of the data black box service
        
        Returns:
            Dictionary with health status information
        """
        try:
            response = self._request('GET', '/health')
            return {
                "status": response.get("status", "unknown"),
                "message": response.get("message", ""),
                "version": response.get("version", "unknown"),
                "timestamp": datetime.now().isoformat()
            }
        except Exception as e:
            logger.error(f"Health check failed: {e}")
            return {
                "status": "error",
                "message": str(e),
                "timestamp": datetime.now().isoformat()
            }

    def ensure_database_exists(self, database_name: str) -> bool:
        """
        Ensure a database exists, creating it if necessary
        
        Args:
            database_name: Name of the database to ensure exists
            
        Returns:
            True if database exists or was created, False on failure
        """
        try:
            # First check if database exists
            response = self._request('GET', '/databases')
            
            # If it exists, return True
            if database_name in [db.get("name") for db in response.get("databases", [])]:
                return True
            
            # If not, create it
            create_response = self._request('POST', '/create_database', json={
                "database_name": database_name
            })
            
            return create_response.get("success", False)
        except Exception as e:
            logger.error(f"Error ensuring database {database_name} exists: {e}")
            return False

    def ensure_topic_storage(self, topic_name: str, database_name: str = "wisevision_topics") -> bool:
        """
        Ensure storage is set up for a topic, creating it if necessary
        
        Args:
            topic_name: Name of the ROS2 topic
            database_name: Name of the database to store the topic data in
            
        Returns:
            True if storage exists or was created, False on failure
        """
        # Skip if we already know this topic has storage
        if topic_name in self._known_topics:
            return True
        
        try:
            # First ensure database exists
            if not self.ensure_database_exists(database_name):
                return False
            
            # Then add storage for this topic
            response = self._request('POST', '/add_storage_to_database', json={
                "database_name": database_name,
                "topic_name": topic_name
            })
            
            success = response.get("success", False)
            if success:
                self._known_topics.add(topic_name)
            
            return success
        except Exception as e:
            logger.error(f"Error ensuring storage for topic {topic_name} in database {database_name}: {e}")
            return False

    def store_topic_data(
        self, 
        topic_name: str, 
        data: Dict[str, Any], 
        timestamp: Optional[datetime] = None,
        database_name: str = "wisevision_topics"
    ) -> bool:
        """
        Store data for a ROS2 topic
        
        Args:
            topic_name: Name of the ROS2 topic
            data: Dictionary containing the message data
            timestamp: Timestamp for the data (defaults to now)
            database_name: Name of the database to store the data in
            
        Returns:
            True if data was stored, False on failure
        """
        # Ensure storage is set up for this topic
        if not self.ensure_topic_storage(topic_name, database_name):
            return False
        
        if timestamp is None:
            timestamp = datetime.now()
        
        try:
            response = self._request('POST', '/add_data_to_database', json={
                "database_name": database_name,
                "topic_name": topic_name,
                "timestamp": timestamp.isoformat(),
                "data": data
            })
            
            return response.get("success", False)
        except Exception as e:
            logger.error(f"Error storing data for topic {topic_name}: {e}")
            return False

    def query_topic_data(
        self,
        topic_name: str,
        start_time: datetime,
        end_time: datetime,
        limit: int = 1000,
        database_name: str = "wisevision_topics"
    ) -> List[Dict[str, Any]]:
        """
        Query historical data for a ROS2 topic
        
        Args:
            topic_name: Name of the ROS2 topic
            start_time: Start time for the query range
            end_time: End time for the query range
            limit: Maximum number of data points to return
            database_name: Name of the database to query
            
        Returns:
            List of data points as dictionaries
        """
        try:
            response = self._request('GET', '/get_messages', params={
                "database_name": database_name,
                "topic_name": topic_name,
                "start_time": start_time.isoformat(),
                "end_time": end_time.isoformat(),
                "limit": limit
            })
            
            return response.get("messages", [])
        except Exception as e:
            logger.error(f"Error querying data for topic {topic_name}: {e}")
            return []

    def query_metrics(
        self,
        metric_name: str,
        start_time: datetime,
        end_time: datetime,
        aggregation: str = "mean",
        interval: str = "1h",
        database_name: str = "wisevision_metrics"
    ) -> List[Dict[str, Any]]:
        """
        Query historical metrics with aggregation
        
        Args:
            metric_name: Name of the metric to query
            start_time: Start time for the query range
            end_time: End time for the query range
            aggregation: Aggregation function to use (mean, max, min, sum, count)
            interval: Time interval for aggregation (e.g., 1h, 30m, 1d)
            database_name: Name of the database to query
            
        Returns:
            List of aggregated data points as dictionaries
        """
        try:
            response = self._request('GET', '/get_metrics', params={
                "database_name": database_name,
                "metric_name": metric_name,
                "start_time": start_time.isoformat(),
                "end_time": end_time.isoformat(),
                "aggregation": aggregation,
                "interval": interval
            })
            
            return response.get("data_points", [])
        except Exception as e:
            logger.error(f"Error querying metrics for {metric_name}: {e}")
            return []

    def delete_topic_data(
        self,
        topic_name: str,
        start_time: Optional[datetime] = None,
        end_time: Optional[datetime] = None,
        database_name: str = "wisevision_topics"
    ) -> bool:
        """
        Delete data for a ROS2 topic within a time range
        
        Args:
            topic_name: Name of the ROS2 topic
            start_time: Start time for the deletion range (defaults to all time)
            end_time: End time for the deletion range (defaults to all time)
            database_name: Name of the database to delete from
            
        Returns:
            True if data was deleted, False on failure
        """
        try:
            params = {
                "database_name": database_name,
                "topic_name": topic_name
            }
            
            if start_time is not None:
                params["start_time"] = start_time.isoformat()
            if end_time is not None:
                params["end_time"] = end_time.isoformat()
            
            response = self._request('DELETE', '/delete_data_from_database', params=params)
            
            return response.get("success", False)
        except Exception as e:
            logger.error(f"Error deleting data for topic {topic_name}: {e}")
            return False