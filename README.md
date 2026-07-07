# RGB human tracking dashboard

This is a single-page dashboard for the RGB human tracking stack. It starts and stops the drone launch file, starts rosbridge for the browser stream, then renders `/human_pose/debug_image` with tracked human boxes drawn by ROS.

## Run on the ROS 2 machine

Build the workspace and install rosbridge once. The RGB camera path uses the
Orbbec driver workspace pointed to by `ORBBEC_SETUP`.

```bash
sudo apt install ros-jazzy-rosbridge-server
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
as RGB resolution, crop pixels, detector threshold, or inference FPS.

To keep your own editable copy elsewhere:

```bash
DRONE_SETTINGS_FILE=/home/andrew/drone_settings.yaml npm start
```

Open `http://<robot-ip>:4173`. The Start button sources ROS 2, sources the built workspace, then launches the RGB camera, human tracker, and rosbridge:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true \
  start_camera:=true \
  start_pose:=true
```

The ROS launch captures RGB at 640x360 and the tracker crops away 29 px on each side before inference, producing a 582x360 debug stream with boxes. Stop sends SIGINT to the launch process and all of its ROS nodes.

Set `ROS2_WORKSPACE` when the ROS workspace is not beside this directory. The dashboard defaults to ROS 2 Jazzy; set `ROS_DISTRO` if you are using another distro, and `PORT` to change the dashboard port.
