# API Endpoints


| Endpoint | Description | Call | Request parameters | Response parameters |
|----------|-------------|------|--------------------|---------------------|
| `/api/topic_types` | Get all available topic types | GET | None | JSON array of topic type names `[string]` |
| `/api/namespaces` | Get all namespaces and their nested structure | GET | None | Nested JSON object representing namespaces hierarchy |
| `/api/create_combined_automatic_action` | Create combined automatic action | POST | [SRV def](https://github.com/wise-vision/ros2_automatic_action_execution/blob/main/automatic_action_msgs/srv/AutomaticActionCombined.srv) | Bool status indicating if creation succeeded |
| `/api/delete_automatic_action` | Delete existing automatic action | POST | `listen_topic_to_delete` [string] | Bool status indicating if deletion succeeded |
| `/api/delete_combined_automatic_action` | Delete existing combined automatic action | POST | `name_of_combined_topics_publisher` [string] | Bool status indicating if deletion succeeded |
| `/api/available_topics` | Get available automatic action topics with parameters | GET | None | JSON array of objects with action topic details and parameters |
| `/api/available_topics_combined` | Get available combined automatic action topics | GET | None | JSON array of combined action topics and their parameters |
| `/api/change_automatic_action` | Modify existing automatic action | POST | Parameters for modification (listen_topic, trigger conditions, etc.) | Bool status indicating if modification succeeded |
| `/api/change_automatic_action_combined` | Modify existing combined automatic action | POST | Parameters for modification (listen_topics, logic expression, etc.) | Bool status indicating if modification succeeded |
| `/api/add_gps_device` | Add a new GPS device | POST | Device details (name, EUI, location) | Bool status indicating if addition succeeded |
| `/api/delete_gps_device` | Delete GPS device | POST | Device EUI details | Bool status indicating if deletion succeeded |
| `/api/modify_gps_device` | Modify existing GPS device details | POST | Device details to modify | Bool status indicating if modification succeeded |
| `/api/topic_echo_gps_devices` | Get data from all GPS devices | GET | None | JSON object with data from GPS devices |
| `/api/message_type/:topic_name` | Get message type for specific topic | GET | `topic_name` [string] | JSON containing message type `[string]` |
| `/api/message_structure/:message_type` | Get detailed structure of a ROS2 message type | GET | `message_type` [string] | JSON representing detailed ROS2 message structure |
| `/api/message_field_types/:message_type` | Get field types used in ROS2 message | GET | `message_type` [string] | JSON array of field types `[string]` |
| `/api/topic_echo/:topic_name` | Get the last message published on a specific topic | GET | `topic_name` [string], `type` [string] | JSON object containing the last published ROS 2 message |
| `/api/create_bucket` | Create new InfluxDB bucket | POST | `bucket_name` [string], `retention_days` [int], `description` [string] | `{ success: bool, error_message: string }` |
| `/api/get_influx_buckets` | Get all existing InfluxDB buckets | GET | None | `{ success: bool, influx_buckets: [string], error_message: string }` |
| `/api/start_record_topics` | Start recording one or more ROS 2 topics into InfluxDB | POST | `bucket_name` [string], `topics_names` [array of string] | `{ success: bool }` |
| `/api/stop_record_topics` | Stop recording specified ROS 2 topics | POST | `topics_names` [array of string] | `{ success: bool, error: string }` |
| `/api/get_recorded_topics` | Get list of all recorded topics with their bucket names | GET | None | `{ success: bool, topics: [string], bucket_names: [string], error_message: string }` |
| `/api/get_recorded_bags_by_topic` | Get list of recorded sessions (bags) for a given topic | POST | `topic_name` [string] | `{ success: bool, topic_name: string, bucket_names: [string], record_ids: [string], end_time_stamps: [string], error: string }` |
| `/api/get_messages` | Get recorded ROS 2 messages from InfluxDB | POST | `topic_name` [string], `bucket_name` [string], optional: `time_start` [FullDateTime or ISO-8601], `time_end` [FullDateTime or ISO-8601], `number_of_msgs` [int] | `{ success: bool, messages: [base64 string], timestamps: [string], message_type: string, error_message: string }` |
| `/api/play_recordings` | Replay (publish) recorded topics from InfluxDB | POST | `bucket_name` [string], `topic_names` [array of string], `record_ids` [array of string, same length as topic_names] | `{ success: bool, error: string }` |

**Note:** For endpoints with `:topic_name` containing `/`, replace `/` with `%`.