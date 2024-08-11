# Build

```
sudo apt update
sudo apt install nodejs npm

sudo apt update
sudo apt install docker.io
sudo systemctl start docker
sudo systemctl enable docker

sudo apt install docker-compose

cd wisevision-dashboard
npm install
npm install react-router-dom
```

# Run

```
cd wisevision-dashboard
docker build -t wisevision-dashboard .
docker run -p 3000:3000 wisevision-dashboard
```

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

```json
curl "http://localhost:5000/api/topic_echo/topic?type=std_msgs/msg/String"
{
  "message": "Hello World"
}
```
### api/create_automatic_action

```bash
curl -X POST http://localhost:5000/api/create_automatic_action -H "Content-Type: application/json" -d '{
    "listen_topic": "/topic_input",
    "listen_message_type": "std_msgs/msg/Int32",
    "value": "data",
    "trigger_val": "50.0",
    "trigger_type": "LessThan",
    "pub_topic": "/topic_output",
    "pub_message_type": "std_msgs/msg/String",
    "trigger_text": "test",
    "data_validity_ms": 5000
}'
{
  "success": true
}
```  
