# Thermal dashboard

This is a single-page dashboard for the `mlx90640_node` ROS 2 package. It starts and stops the package launch file, starts rosbridge for the browser stream, then renders `thermal/image_raw` as a live 32x24 heatmap.

## Run on the ROS 2 machine

Build the workspace and install rosbridge once:

```bash
cd /ros2-initiator-drone
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control

cd ../frontend-initiator-drone
npm start
```

Open `http://<robot-ip>:4173`. The Start button sources ROS 2, sources the built workspace, then launches the drone ROS graph with rosbridge enabled:

```bash
ros2 launch drone_control drone_launch.py start_rosbridge:=true
```

Stop sends SIGINT to the launch process and all of its ROS nodes.

`drone_control` is the top-level package for the drone. It starts the thermal sensor package and can start `rosbridge_websocket` on port `9090`; add future drone nodes to `src/drone_control/launch/drone_launch.py`.

Set `ROS2_WORKSPACE` when the ROS workspace is not beside this directory. The dashboard defaults to ROS 2 Jazzy; set `ROS_DISTRO` if you are using another distro, and `PORT` to change the dashboard port.
