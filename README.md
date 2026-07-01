# Pose dashboard

This is a single-page dashboard for the RGB human pose camera stack. It starts and stops the drone launch file, starts rosbridge for the browser stream, then renders `/human_pose/debug_image`.

## Run on the ROS 2 machine

Build the workspace and install rosbridge once:

```bash
cd ~/ros2-initiator-drone
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control

cd ../frontend-initiator-drone
MOVENET_MODEL_PATH=/path/to/movenet_lightning_int8.tflite npm start
```

Open `http://<robot-ip>:4173`. The Start button sources ROS 2, sources the built workspace, then launches the RGB camera, MoveNet pose node, and rosbridge:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true \
  start_camera:=true \
  pose_model_path:=/path/to/movenet_lightning_int8.tflite
```

The dashboard subscribes only to the RGB pose debug stream. Stop sends SIGINT to the launch process and all of its ROS nodes.

Set `ROS2_WORKSPACE` when the ROS workspace is not beside this directory. The dashboard defaults to ROS 2 Jazzy; set `ROS_DISTRO` if you are using another distro, and `PORT` to change the dashboard port.
