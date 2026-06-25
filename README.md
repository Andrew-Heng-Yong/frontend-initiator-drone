# Thermal dashboard

This is a single-page dashboard for the `mlx90640_node` ROS 2 package. It starts and stops the package launch file, then renders `thermal/image_raw` as a live 32×24 heatmap.

## Run on the ROS 2 machine

Build the workspace and install rosbridge once:

```bash
cd ../ros2-initiator-drone
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-rosbridge-server
colcon build --packages-select mlx90640_node

cd ../frontend-initiator-drone
npm start
```

Open `http://<robot-ip>:4173`. The Start button launches `drone_launch.py`; Stop sends SIGINT to the launch process and all of its ROS nodes.

`drone_launch.py` is the top-level launch file for the drone. It currently starts only the thermal node; add future drone nodes to that file.

Set `ROS2_WORKSPACE` when the ROS workspace is not beside this directory. Set `ROS_DISTRO` if you are not using Humble, and `PORT` to change the dashboard port.
