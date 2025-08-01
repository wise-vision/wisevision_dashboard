# Configure and run

## Build local

``` bash
mkdir wisevision_dashboard_ws
cd wisevision_dashboard_ws
git clone git@github.com:wise-vision/wisevision-dashboard.git

sudo apt install python3-pip
pip3 install --no-cache-dir -r requirements.txt

#build and install wisevision_msgs
mkdir -p ros2_ws/src
cd wisevision-dashboard
vsc import ../ros2_ws/src > msgs.repos
cd ../ros2_ws
rosdep install --from-paths src -i -y --rosdistro humble
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run local

Backend:

```bash 
cd wisevision-dashboard
python3 -m app.server.run
```

Frontend:

Before run create `.env` file with adress to backend. (default->localhost)
```bash
cd app/client
cp .env_example .env
```

```bash
cd app/client
npm install
npm start
```

## Run in docker

```
cd wisevision-dashboard
docker-compose up --build
```
