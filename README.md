# Thermal dashboard

This is a single-page dashboard for the `mlx90640_node` ROS 2 package. It starts and stops the drone launch file, starts rosbridge for the browser stream, then renders `/camera/thermal_overlay/image_raw` when the optional Orbbec RGB overlay is available. If the overlay is not built or the camera setup is missing, it falls back to a crisp `/thermal/image_raw` MLX90640 view.

## Run on the ROS 2 machine

Build the workspace and install rosbridge once:

```bash
cd ~/ros2-initiator-drone
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control

cd ../frontend-initiator-drone
npm start
```

Open `http://<robot-ip>:4173`. The Start button sources ROS 2, sources the built workspace, then launches the drone ROS graph with rosbridge enabled. By default it tries:

```bash
ros2 launch drone_control drone_launch.py start_rosbridge:=true start_depth_camera:=true start_thermal_overlay:=true
```

If the optional overlay executable or Orbbec setup is missing, the server logs that and launches thermal-only mode instead.

Stop sends SIGINT to the launch process and all of its ROS nodes.

`drone_control` is the top-level package for the drone. It starts the thermal sensor package and can start `rosbridge_websocket` on port `9090`; add future drone nodes to `src/drone_control/launch/drone_launch.py`.

For RGB overlay, build the optional overlay executable and make sure the Orbbec workspace exists at `~/orbbec_ws/install/setup.bash`:

```bash
cd ~/ros2-initiator-drone
source /opt/ros/jazzy/setup.bash
sudo apt install -y ros-jazzy-cv-bridge libopencv-dev
colcon build --packages-up-to drone_control --cmake-args -DBUILD_THERMAL_OVERLAY=ON
source install/setup.bash
```

If the dashboard connects but no image appears, check the launch output. A healthy RGB overlay launch should include `component_container`, `mlx90640_node`, `thermal_overlay_node`, and `rosbridge_websocket`. A thermal fallback launch includes only `mlx90640_node` and `rosbridge_websocket`. If the launch package is missing new arguments, rebuild and source the ROS workspace on the Pi:

```bash
cd ~/ros2-initiator-drone
rm -rf build/mlx90640_node build/drone_control install/mlx90640_node install/drone_control
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control
source install/setup.bash
```

Set `ROS2_WORKSPACE` when the ROS workspace is not beside this directory. The dashboard defaults to ROS 2 Jazzy; set `ROS_DISTRO` if you are using another distro, and `PORT` to change the dashboard port.
