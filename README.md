# Thermal camera dashboard

This is a single-page dashboard for the thermal camera stack. It starts and stops the drone launch file, starts rosbridge for the browser stream, then renders `/thermal/image_raw`.

## Run on the ROS 2 machine

Build the workspace and install rosbridge plus `v4l2_camera` once:

```bash
sudo apt install ros-jazzy-rosbridge-server ros-jazzy-v4l2-camera
cd ~/ros2-initiator-drone
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control

cd ../frontend-initiator-drone
npm start
```

## Tune Without Rebuild

After one rebuild that installs the settings file, edit:

```bash
/home/andrew/ros2-initiator-drone/install/drone_control/share/drone_control/config/drone_settings.yaml
```

Then restart the dashboard launch. No `colcon build` is needed for changes such
as thermal device path, resolution, pixel format, or FPS.

To keep your own editable copy elsewhere:

```bash
DRONE_SETTINGS_FILE=/home/andrew/drone_settings.yaml npm start
```

Open `http://<robot-ip>:4173`. The Start button sources ROS 2, sources the built workspace, then launches only the thermal V4L2 camera node and rosbridge:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true \
  start_camera:=false \
  start_pose:=false \
  start_thermal_camera:=true
```

The dashboard subscribes only to the thermal stream. HikCamera-style 256x392 YUYV frames are cropped to the lower 256x192 thermal image for display. Stop sends SIGINT to the launch process and all of its ROS nodes.

Set `ROS2_WORKSPACE` when the ROS workspace is not beside this directory. The dashboard defaults to ROS 2 Jazzy; set `ROS_DISTRO` if you are using another distro, and `PORT` to change the dashboard port.
