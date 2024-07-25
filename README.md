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

| Endpoint | Description | Request parameters | Response parameters |
| --- | --- | --- | --- |
| /api/topics | Get all topics | None | JSON array of objects containing `name` [string] and `type` [string] |
| /api/services | Get all services | None | JSON array of objects containing `name` [string] and `type` [string] |
| /api/topic/:data | Get last data published for a topic | `data` [string] | JSON object containing `ROS 2 message` [string] (one of ros2 msg types)|

### Post Endpoints

| Endpoint | Description | Request parameters | Response parameters |
| --- | --- | --- | --- |
| /api/service/:name | Call a service | ROS 2 service request [string] | ROS 2 service response [string]|