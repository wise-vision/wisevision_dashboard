# Build

``` bash
mkdir wisevision_dashboard_ws
cd wisevision_dashboard_ws
git clone git@github.com:wise-vision/wisevision-dashboard.git

sudo apt update
sudo apt install docker.io
sudo systemctl start docker
sudo systemctl enable docker

sudo apt install docker-compose
sudo apt install python3-pip

pip3 install --no-cache-dir -r requirements.txt

#build and install lora_msgs
mkdir -p ros2_ws/src
cd wisevision-dashboard
vsc import ../ros2_ws/src > msgs.repos
cd ../ros2_ws
rosdep install --from-paths src -i -y --rosdistro humble
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

# Run

```
cd wisevision-dashboard
GITHUB_TOKEN=<your_github_token> docker-compose up --build
```
- [How to create github personal-access-tokens](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens#creating-a-fine-grained-personal-access-token)

# Run local

```bash 
cd wisevision-dashboard
python3 -m app.server.run
```

## Test the api route

```bash
curl http://localhost:5000/api/topics
```


## API 

To be exported to a separate file


### Get Endpoints

| Endpoint         | Description                         | Call | Request parameters | Response parameters                                                     |
| ---------------- | ----------------------------------- | ---- | ------------------ | ----------------------------------------------------------------------- |
| /api/topics      | Get all topics                      | GET  | None               | JSON array of objects containing `name` [string] and `type` [string]    |
| /api/services    | Get all services                    | GET  | None               | JSON array of objects containing `name` [string] and `type` [string]    |
| /api/topic/:data | Get last data published for a topic | GET  | `data` [string]    | JSON object containing `ROS 2 message` [string] (one of ros2 msg types) |
| api/create_automatic_action            | Create automatic action            | POST | [SRV def](https://github.com/wise-vision/ros2_automatic_action_execution/blob/main/automatic_action_msgs/srv/AutomaticAction.srv) | Bool status informing if created succeeded or not |
| api/get_messages | Get messages from data base | POST | [SRV def](https://github.com/wise-vision/lora_msgs/blob/dev/srv/GetMessages.srv) | JSON object containing ROS 2 messages [MicroPublisher or Int32 with [TimeStamps](https://github.com/wise-vision/lora_msgs/blob/dev/msg/FullDateTime.msg)] |
| api/topic_echo_data_base/:data | Get last message from data base | GET | `data` [string]| JSON object containing ROS 2 message MicroPublisher or Int32 with [TimeStamps](https://github.com/wise-vision/lora_msgs/blob/dev/msg/FullDateTime.msg) [string] (one of ros2 msg types) |
| api/topic_echo_data_base_any/:data | Get any type from data base | GET | `data` [string] | JSON object containing ROS 2 messages of any type with [TimeStamps](https://github.com/wise-vision/lora_msgs/blob/dev/msg/FullDateTime.msg) [string] (one of messsage type & number_of & time_start & time_end)





### /api/topics 

```json
curl http://localhost:5000/api/topics
[
  {
    "name": "/parameter_events",
    "type": "rcl_interfaces/msg/ParameterEvent"
  },
  {
    "name": "/rosout",
    "type": "rcl_interfaces/msg/Log"
  }
]

```

### /api/services

```json
curl http://localhost:5000/api/services
[
  {
    "name": "/ros2_topic_list_node/describe_parameters",
    "type": "rcl_interfaces/srv/DescribeParameters"
  },
  {
    "name": "/ros2_topic_list_node/get_parameter_types",
    "type": "rcl_interfaces/srv/GetParameterTypes"
  },
  {
    "name": "/ros2_topic_list_node/get_parameters",
    "type": "rcl_interfaces/srv/GetParameters"
  },
  {
    "name": "/ros2_topic_list_node/list_parameters",
    "type": "rcl_interfaces/srv/ListParameters"
  },
  {
    "name": "/ros2_topic_list_node/set_parameters",
    "type": "rcl_interfaces/srv/SetParameters"
  },
  {
    "name": "/ros2_topic_list_node/set_parameters_atomically",
    "type": "rcl_interfaces/srv/SetParametersAtomically"
  }
]

```

### /api/topic/:data

On one terminal run the following command to publish data to a topic

```bash
ros2 topic pub /topic std_msgs/String 'data: Hello World'
```

And on another terminal run the following command to get the last data published to the topic

If topic name has `/` replace them with `%`
```json
curl "http://localhost:5000/api/topic_echo/topic?type=std_msgs/msg/String"
{
  "message": "Hello World"
}
```
### api/create_automatic_action (automatic_action_execution)

```bash
curl -X POST http://localhost:5000/api/create_automatic_action -H "Content-Type: application/json" -d '{
    "listen_topic": "/topic_input",
    "listen_message_type": "std_msgs/msg/Int32",
    "value": "data",
    "trigger_val": "50.0",
    "trigger_type": "LessThan",
    "action_and_publisher_name": "/topic_output",
    "pub_message_type": "std_msgs/msg/String",
    "trigger_text": "test",
    "data_validity_ms": 5000,
    "publication_method": 0
}'
{
  "success": true
}
```  

### api/create_combined_automatic_action (automatic_action_execution)

```bash
curl -X POST http://localhost:5000/api/create_combined_automatic_action -H "Content-Type: application/json" -d '{
    "listen_topics": ["/topic1", "/topic2", "/topic3"],
    "logic_expression": "/topic1 and /topic2 or /topic3",
    "action_and_publisher_name": "/combined_action",
    "trigger_text": "example_text",
    "publication_method": 2
}'
{
  "success": false
}
```


### api/delete_automatic_action (automatic_action_execution)
```bash
curl -X POST http://localhost:5000/api/delete_automatic_action -H "Content-Type: application/json" -d '{
    "listen_topic_to_delete": "/topic_input"
}'
{
  "success": true
}
```

### api/delete_combined_automatic_action (automatic_action_execution)

```bash
curl -X POST http://localhost:5000/api/delete_combined_automatic_action -H "Content-Type: application/json" -d '{
    "name_of_combined_topics_publisher": "combined_publisher_1"
}'
{
  "success": false
}
```

### api/available_topics (automatic_action_execution)

```bash
curl -X GET http://localhost:5000/api/available_topics
{
  "available_topics_with_parameters_and_time": [
    {
      "action_and_publisher_name": "/example_topic_1",
      "data_validity_ms": 5000,
      "date_and_time": {
        "day": 0,
        "hour": 0,
        "minute": 0,
        "month": 0,
        "nanosecond": 0,
        "second": 0,
        "year": 0
      },
      "listen_message_type": "std_msgs/msg/Int32",
      "listen_topic": "/topic_1",
      "pub_message_type": "std_msgs/msg/String",
      "publication_method": 0,
      "trigger_text": "test",
      "trigger_type": "LessThan",
      "trigger_val": "69.0",
      "value": "data"
    },
    {
      "action_and_publisher_name": "/example_topic_2",
      "data_validity_ms": 5000,
      "date_and_time": {
        "day": 0,
        "hour": 0,
        "minute": 0,
        "month": 0,
        "nanosecond": 0,
        "second": 0,
        "year": 0
      },
      "listen_message_type": "std_msgs/msg/Int32",
      "listen_topic": "/topic_2",
      "pub_message_type": "std_msgs/msg/String",
      "publication_method": 0,
      "trigger_text": "test",
      "trigger_type": "LessThan",
      "trigger_val": "50.0",
      "value": "data"
    }
  ]
}
```

### api/available_topics_combined (automatic_action_execution)
```bash
curl -X GET http://localhost:5000/api/available_topics_combined
{
  "available_combined_topics_with_parameters_and_time": [
    {
      "action_and_publisher_name": "example_topic_test",
      "date_and_time": {
        "day": 0,
        "hour": 0,
        "minute": 0,
        "month": 0,
        "nanosecond": 0,
        "second": 0,
        "year": 0
      },
      "listen_topics": [
        "/example_topic_1",
        "/example_topic_2"
      ],
      "logic_expression": "/example_topic_1 and /example_topic_2",
      "publication_method": 2,
      "trigger_text": "Updated action triggered!"
    }
  ]
}

```

### api/change_automatic_action (automatic_action_execution)
```bash
curl -X POST http://localhost:5000/api/change_automatic_action -H "Content-Type: application/json" -d '{
    "action_and_publisher_name_to_change": "/example_topic_2",
    "listen_topic": "/topic_2",
    "listen_message_type": "std_msgs/msg/Int32",
    "value": "data",
    "trigger_val": "130.0",
    "trigger_type": "LessThan",
    "new_action_and_publisher_name": "/example_topic_2",
    "pub_message_type": "std_msgs/msg/String",
    "trigger_text": "test",
    "data_validity_ms": 5000,
    "publication_method": 0
}'
{
  "success": true
}

```

### api/change_automatic_action_combined (automatic_action_execution)
```bash
curl -X POST http://localhost:5000/api/change_automatic_action_combined -H "Content-Type: application/json" -d '{
  "action_and_publisher_name_to_change": "example_topic_test",
  "listen_topics": ["/example_topic_1", "/example_topic_2"],
  "logic_expression": "/example_topic_1 and /example_topic_2",
  "new_action_and_publisher_name": "example_topic_test",
  "trigger_text": "Updated action triggered!",
  "publication_method": 2
}'
{
  "success": true
}
```

### api/get_messages

```bash
curl -X POST http://localhost:5000/api/get_messages -H "Content-Type: application/json" -d '{
  "topic_name": "sensor_publisher",
  "message_type": "lora_msgs/MicroPublisher",
  "number_of_msgs": 1
}'
{
  "int32_msgs": [],
  "micro_publisher_data": [
    {
      "sensors_data": [
        {
          "id": 111537764,
          "tpb_value": {
            "binary_value": true,
            "pressure": 0,
            "temperature": 0,
            "value_type": 2
          }
        },
        {
          "id": 438792350,
          "tpb_value": {
            "binary_value": false,
            "pressure": 25964,
            "temperature": 0,
            "value_type": 1
          }
        },
        {
          "id": 2142757034,
          "tpb_value": {
            "binary_value": false,
            "pressure": 24588,
            "temperature": 0,
            "value_type": 1
          }
        },
        {
          "id": 155324914,
          "tpb_value": {
            "binary_value": false,
            "pressure": 0,
            "temperature": 0,
            "value_type": 2
          }
        }
      ]
    }
  ],
  "timestamps": [
    {
      "day": 4,
      "hour": 9,
      "minute": 24,
      "month": 9,
      "nanosecond": 893372345,
      "second": 0,
      "year": 2024
    }
  ]
}


```

## api/topic_echo_data_base/<topic_name>?type=<topic_type>
``` bash
curl "http://localhost:5000/api/topic_echo_data_base/sensor_publisher?type=lora_msgs/MicroPublisher"
{
  "int32_msgs": [],
  "micro_publisher_data": [
    {
      "sensors_data": [
        {
          "id": 1998898814,
          "tpb_value": {
            "binary_value": false,
            "pressure": 0,
            "temperature": 34,
            "value_type": 0
          }
        }
      ]
    }
  ],
  "timestamps": [
    {
      "day": 5,
      "hour": 6,
      "minute": 31,
      "month": 9,
      "nanosecond": 632699344,
      "second": 3,
      "year": 2024
    }
  ]
}

```

## Premium: api/topic_echo_data_base_any/<string:topic_name>
If topic name has `/` replace them with `%`
``` bash
curl "http://localhost:5000/api/topic_echo_data_base_any/topic?type=std_msgs/msg/String&number_of_msgs=2"
{
  "messages": [
    {
      "data": "Hello World"
    },
    {
      "data": "Hello World"
    }
  ],
  "timestamps": [
    {
      "day": 5,
      "hour": 11,
      "minute": 7,
      "month": 9,
      "nanosecond": 800521846,
      "second": 15,
      "year": 2024
    },
    {
      "day": 5,
      "hour": 11,
      "minute": 7,
      "month": 9,
      "nanosecond": 742738910,
      "second": 16,
      "year": 2024
    }
  ]
}
```

## api/topic_echo_data_base_any_last_week/<string:topic_name>
```bash
curl "http://localhost:5000/api/topic_echo_data_base_any_last_week/sensor_publisher_temp?type=lora_msgs/msg/E5BoardUplink"
```


### Notifications
The application supports real-time notifications via WebSockets using Socket.IO. Notifications are triggered by events from the ROS2 system [Run this to trigger notications](https://github.com/wise-vision/test_env_notifications). These notifications are available to connected clients for immediate updates.

## api/add_gps_device
```bash
curl -X POST http://localhost:5000/api/add_gps_device -H "Content-Type: application/json" -d '{
    "device_name": "Test_Device",
    "device_eui": {
        "data": [0, 0, 0, 0, 0, 0, 0, 1]
    },
    "nav_value": {
        "latitude": 52.2297,
        "longitude": 21.0122,
        "altitude": 100.0
    },
    "is_moving": false
}'
{
  "success": true
}
```
## api/delete_gps_device
```bash
curl -X POST http://localhost:5000/api/delete_gps_device -H "Content-Type: application/json" -d '{
    "device_eui": {
        "data": [0, 0, 0, 0, 0, 0, 0, 1]
    }
}'
{
  "success": true
}
```

## api/topic_echo_gps_devices
```bash
curl "http://localhost:5000/api/topic_echo_gps_devices"
{
  "message": {
    "devices_data": [
      {
        "device_eui": {
          "data": [
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            1
          ]
        },
        "device_name": "Test_Device",
        "is_moving": false,
        "nav_value": {
          "altitude": 100.0,
          "header": {
            "frame_id": "",
            "stamp": {
              "nanosec": 0,
              "sec": 0
            }
          },
          "latitude": 52.2297,
          "longitude": 21.0122,
          "position_covariance": [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
          ],
          "position_covariance_type": 0,
          "status": {
            "service": 0,
            "status": 0
          }
        }
      }
    ]
  }
}
```

## api/modify_gps_device
```bash
curl -X POST http://localhost:5000/api/modify_gps_device -H "Content-Type: application/json" -d '{
    "device_name": "Updated_Device",
    "device_eui": {
        "data": [0, 0, 0, 0, 0, 0, 0, 1]
    },
    "nav_value": {
        "latitude": 52.4064,
        "longitude": 16.9252,
        "altitude": 120.0
    }
}'
{
  "success": true
}
```

# Admin api calls

## api/add_storage_to_database
```bash
curl -X POST http://localhost:5000/api/add_storage_to_database -H "Content-Type: application/json" -d '{
    "storage_name": "influxdb"
}'
{
  "success": true
}
```

## api/create_database

```bash
curl -X POST http://localhost:5000/api/create_database -H "Content-Type: application/json" -d '{
    "key_expr": "demo/example/**",
    "volume_id": "influxdb",
    "db_name": "zenoh_example_db",
    "create_db": true
}'
{
  "success": true
}
```

# Message type api calls

## api/message_type/<string:topic_name>
To call topic name add `%` before `topic_name`
```bash
curl -X GET http://localhost:5000/api/message_type/%notifications
{
  "message_type": "notification_msgs/msg/Notification"
}
```

## api/message_structure/<path:message_type>

If variable is array, type name is in `[]`, e.g.
```bash
"angular_velocity_covariance": [
    "double"
  ]
```

* `std_msgs/msg/Float32`
```bash
curl -X GET http://localhost:5000/api/message_structure/std_msgs/msg/Float32
{
  "data": "float"
}
```

* `sensor_msgs/msg/Imu`
``` bash
curl -X GET http://localhost:5000/api/message_structure/sensor_msgs/msg/Imu
{
  "angular_velocity": {
    "x": "double",
    "y": "double",
    "z": "double"
  },
  "angular_velocity_covariance": [
    "double"
  ],
  "header": {
    "frame_id": "string",
    "stamp": {
      "nanosec": "uint32",
      "sec": "int32"
    }
  },
  "linear_acceleration": "geometry_msgs/msg/Vector3",
  "linear_acceleration_covariance": [
    "double"
  ],
  "orientation": {
    "w": "double",
    "x": "double",
    "y": "double",
    "z": "double"
  },
  "orientation_covariance": [
    "double"
  ]
}
```