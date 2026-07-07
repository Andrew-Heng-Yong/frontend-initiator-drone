# Thermal dashboard

This is a single-page dashboard for the drone camera stack. It starts and stops the drone launch file, starts rosbridge for the browser stream, then renders `/camera/color/image_raw` as the window and blends `/thermal/image_raw` into the center. The Orbbec RGB camera is required; the dashboard does not start thermal-only mode.

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
ros2 launch drone_control drone_launch.py start_rosbridge:=true start_depth_camera:=true start_thermal_overlay:=false
```

If the Orbbec setup is missing, the server logs the missing setup path and exits instead of launching thermal-only mode.

Stop sends SIGINT to the launch process and all of its ROS nodes.

`drone_control` is the top-level package for the drone. It starts the thermal sensor package and can start `rosbridge_websocket` on port `9090`; add future drone nodes to `src/drone_control/launch/drone_launch.py`.

The dashboard can be pointed at the current I2C thermal accel path without editing the frontend:

```bash
DRONE_LAUNCH_COMMAND='ros2 launch <package> <launch-file> start_rosbridge:=true' \
THERMAL_IMAGE_TOPIC=/thermal/image_raw \
THERMAL_FOV_HORIZONTAL=55 \
THERMAL_FOV_VERTICAL=35 \
npm start
```

Use the actual launch command/topic for the active backend. The server logs both values on start, which makes it obvious if the old MLX node is still being launched.

For RGB overlay, make sure the Orbbec workspace exists at `~/orbbec_ws/install/setup.bash`:

```bash
cd ~/ros2-initiator-drone
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control
source install/setup.bash
```

If the dashboard connects but no image appears, check the launch output. A healthy RGB overlay launch should include `component_container`, the active thermal node, and `rosbridge_websocket`. If the launch package is missing new arguments, rebuild and source the ROS workspace on the Pi:

```bash
cd ~/ros2-initiator-drone
rm -rf build/<thermal_package> build/drone_control install/<thermal_package> install/drone_control
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control
source install/setup.bash
```

For MLX90640 hardware, `sudo i2cdetect -y 1` should normally show `0x33`; if it does not, check power, SDA/SCL, ground, and make sure the module `PS` pin is tied to ground for I2C mode.

If the thermal image is visible but does not line up with RGB, use a small hot target such as a candle or warm hand and tune the dashboard `X`, `Y`, and `Scale` controls until the thermal hot spot lands on the same RGB object. The dashboard saves the tuned values in `.thermal-alignment.json`; they can also be seeded with `THERMAL_OFFSET_X`, `THERMAL_OFFSET_Y`, and `THERMAL_SCALE`.

Set `ROS2_WORKSPACE` when the ROS workspace is not beside this directory. The dashboard defaults to ROS 2 Jazzy; set `ROS_DISTRO` if you are using another distro, and `PORT` to change the dashboard port.
