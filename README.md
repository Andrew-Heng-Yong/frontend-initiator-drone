# Thermal dashboard

This is a single-page dashboard for the drone camera stack. It starts and stops the drone launch file, starts rosbridge for the browser stream, then renders `/camera/depth/image_raw` as the window and blends `/thermal/image_raw` into the center. The Orbbec depth camera is required; the dashboard does not start thermal-only mode.

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
ros2 launch drone_control drone_launch.py start_rosbridge:=true start_depth_camera:=true start_imu:=true start_thermal_cropper:=false start_thermal_overlay:=false
```

Runtime defaults are loaded from `config/master_params.yaml`. Camera intrinsics, distortion, and camera-frame transforms live separately in `config/camera_calibrations.yaml`, and the master params file points to it with `camera_calibrations_params_file`. Environment variables still override YAML values, so one-off test runs do not require editing the params files.

If the Orbbec setup is missing, the server logs the missing setup path and exits instead of launching thermal-only mode.

Stop sends SIGINT to the launch process and all of its ROS nodes.

`drone_control` is the top-level package for the drone. It starts the thermal sensor package, starts the MPU6050 IMU when `start_imu:=true`, and can start `rosbridge_websocket` on port `9090`; add future drone nodes to `src/drone_control/launch/drone_launch.py`.

The dashboard can be pointed at the current I2C thermal accel path without editing the frontend:

```bash
DRONE_LAUNCH_COMMAND='ros2 launch <package> <launch-file> start_rosbridge:=true' \
DEPTH_IMAGE_TOPIC=/camera/depth/image_raw \
THERMAL_IMAGE_TOPIC=/thermal/image_raw \
DEPTH_FOV_HORIZONTAL=67 \
DEPTH_FOV_VERTICAL=53.6 \
THERMAL_FOV_HORIZONTAL=55 \
THERMAL_FOV_VERTICAL=35 \
npm start
```

Use the actual launch command/topic for the active backend. The server logs the active base/depth and thermal topics on start, which makes it obvious if the old MLX node or wrong camera topic is still being launched.

The dashboard also subscribes to `/camera/depth/camera_info` by default and displays the depth intrinsics-derived FOV for debugging. Overlay sizing uses the configured `depth_fov_horizontal` and `depth_fov_vertical` params unless `USE_CAMERA_INFO_FOV=true` is set. Set `DEPTH_CAMERA_INFO_TOPIC` if your Orbbec driver publishes camera info somewhere else.

By default `BASE_VIEW_MODE=full-depth`: the raw depth image is the main viewport and thermal is blended into the configured thermal FOV area. Set `BASE_VIEW_MODE=thermal-crop` only when you want the thermal FOV to define the main viewport.

The baked fallback thermal alignment is `X=10`, `Y=0`, `Scale=80`, `H=80`, `V=100`. This keeps vertical zoom unchanged and shrinks horizontal coverage from the right edge, matching the observed case where the right side aligned while the left side was too far left. The saved `.thermal-alignment.json` file still overrides these defaults after manual tuning.

For depth overlay, make sure the Orbbec workspace exists at `~/orbbec_ws/install/setup.bash`:

```bash
cd ~/ros2-initiator-drone
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control
source install/setup.bash
```

If the dashboard connects but no image appears, check the launch output. A healthy depth overlay launch should include `component_container`, the active thermal node, and `rosbridge_websocket`. If the launch package is missing new arguments, rebuild and source the ROS workspace on the Pi:

```bash
cd ~/ros2-initiator-drone
rm -rf build/<thermal_package> build/drone_control install/<thermal_package> install/drone_control
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control
source install/setup.bash
```

For MLX90640 hardware, `sudo i2cdetect -y 1` should normally show `0x33`; if it does not, check power, SDA/SCL, ground, and make sure the module `PS` pin is tied to ground for I2C mode.

For MPU6050 hardware, `sudo i2cdetect -y 1` should normally show `0x68`. The default launch command enables the IMU with `start_imu:=true`; it publishes `sensor_msgs/Imu` on `/imu/data_raw` and chip temperature on `/imu/temperature`.

If the thermal image is visible but does not line up with depth, use a small hot target such as a candle or warm hand and tune the dashboard `X`, `Y`, `Scale`, `H`, and `V` controls until the thermal hot spot lands on the same depth object. The dashboard saves the tuned values in `.thermal-alignment.json`; they can also be seeded with `THERMAL_OFFSET_X`, `THERMAL_OFFSET_Y`, `THERMAL_SCALE`, `THERMAL_STRETCH_X`, and `THERMAL_STRETCH_Y`.

The optional ROS cropper node uses highlighted thermal pixels to publish `/camera/depth/cropped/image_raw` and `/thermal/cropped/image_raw`. Both outputs keep the original stream dimensions; pixels outside the selected highlighted region are blacked out. Use those cropped topics only when you specifically want thermal-selected masking. Each crop unit covers `crop_unit_thermal_pixels` square thermal pixels, clusters count diagonal neighbors, `min_region_size` rejects small clusters, and `inflation_radius_thermal_pixels` expands the crop mask. The dashboard only toggles the cropper on or off at runtime after it has been launched; all cropper tuning values live in the `thermal_cropper` block in `config/master_params.yaml` and are passed to `thermal_cropper_node` at launch.

The built-in dashboard calibration defaults are `Overlay=50.0`, `X=25.0`, `Y=-10.0`, `Scale=100.0`, `H=79.6`, and `V=115.4`. Delete `.thermal-alignment.json` to return to these defaults after local tuning.

Set `ROS2_WORKSPACE` when the ROS workspace is not beside this directory. The dashboard defaults to ROS 2 Jazzy; set `ROS_DISTRO` if you are using another distro, and `PORT` to change the dashboard port.

Use `DRONE_MASTER_PARAMS=/path/to/master_params.yaml` to load a different master params file. Use `CAMERA_CALIBRATIONS_PARAMS=/path/to/camera_calibrations.yaml` to override only the calibration file referenced by the master params.
