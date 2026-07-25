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

Open `http://<robot-ip>:4173`. The Start button sources ROS 2, sources the built workspace, then launches the drone ROS graph with rosbridge enabled. The saved Cropper checkbox controls both cropper launch arguments. For example, when enabled it launches:

```bash
ros2 launch drone_control drone_launch.py start_rosbridge:=true start_depth_camera:=true start_imu:=true start_thermal_cropper:=true thermal_cropper_enabled:=true start_thermal_overlay:=false
```

When cleared, both cropper arguments are set to `false`, so the cropper node is not launched.

Runtime defaults are loaded from `config/master_params.yaml`. Camera intrinsics, distortion, and camera-frame transforms live separately in `config/camera_calibrations.yaml`, and the master params file points to it with `camera_calibrations_params_file`. Environment variables still override YAML values, so one-off test runs do not require editing the params files.

Set `camera_streams.ros__parameters.frontend_mode` to choose the browser display:

- `simple`: subscribe only to the thermal-cropped depth output and throttled IMU data, then place valid crops in a fixed `1024x768` depth window. Full-size passthrough frames are ignored while the cropper waits for a thermal region.
- `full`: subscribe to depth, thermal, camera info, and IMU topics for the full overlay/tuning dashboard.

If the Orbbec setup is missing, the server logs the missing setup path and exits instead of launching thermal-only mode.

Stop sends SIGINT to the launch process and all of its ROS nodes.

`drone_control` is the top-level package for the drone. It starts the MI0802 SenXor thermal driver, starts the MPU6050 IMU when `start_imu:=true`, and can start `rosbridge_websocket` on port `9090`; add future drone nodes to `src/drone_control/launch/drone_launch.py`.

The dashboard can be pointed at another compatible thermal backend without editing the frontend:

```bash
DRONE_LAUNCH_COMMAND='ros2 launch <package> <launch-file> start_rosbridge:=true' \
DEPTH_IMAGE_TOPIC=/camera/depth/image_raw \
THERMAL_IMAGE_TOPIC=/thermal/image_raw \
DEPTH_FOV_HORIZONTAL=79 \
DEPTH_FOV_VERTICAL=62 \
THERMAL_FOV_HORIZONTAL=90 \
THERMAL_FOV_VERTICAL=68 \
npm start
```

Use the actual launch command/topic for the active backend. The server logs the active base/depth and thermal topics on start, which makes it obvious if the old MLX node or wrong camera topic is still being launched.

The dashboard also subscribes to `/camera/depth/camera_info` by default and displays the depth intrinsics-derived FOV for debugging. Overlay sizing uses the configured `depth_fov_horizontal` and `depth_fov_vertical` params unless `USE_CAMERA_INFO_FOV=true` is set. Set `DEPTH_CAMERA_INFO_TOPIC` if your Orbbec driver publishes camera info somewhere else.

By default `BASE_VIEW_MODE=full-depth`: the depth image is the main viewport and thermal is blended into the configured thermal FOV area. The Cropper checkbox controls the next ROS launch and never changes the running graph. When selected, the next Start launches the cropper and subscribes to `/camera/depth/cropped/image_raw`, `/camera/depth/cropped/camera_info`, and `/thermal/cropped/image_raw`. When cleared, the next Start omits the cropper node and subscribes directly to the corresponding raw topics.

The thermal overlay defaults are `Blend=50`, `X=0`, `Y=0`, `Scale=100`, `Barrel=0`, `H=80`, and `V=90`. Its base size comes from the configured FOVs: depth is `79° x 62°`, and thermal is `90° x 68°`.

The thermal display is mirrored along the Y axis by default: `thermal_display.flip_x` is enabled and `thermal_display.flip_y` is disabled. Set `THERMAL_FLIP_X=false` to display the sensor-native orientation.

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

For MI0802 hardware, the default device is `/dev/ttyACM0`; the stable target path is `/dev/serial/by-id/usb-Nuvoton_USB_Virtual_COM-if00`. The ROS user normally needs membership in `dialout`. The MLX90640 package remains available as a fallback but is no longer the frontend thermal source.

For MPU6050 hardware, `sudo i2cdetect -y 1` should normally show `0x68`. The default launch command enables the IMU with `start_imu:=true`; it publishes `sensor_msgs/Imu` on `/imu/data_raw` and chip temperature on `/imu/temperature`.

If the thermal image is visible but does not line up with depth, use a small hot target such as a candle or warm hand and tune the dashboard `X`, `Y`, `Scale`, `Barrel`, `H`, and `V` controls until the thermal hot spot lands on the same depth object. `Barrel` applies signed radial distortion to the thermal overlay in Full mode: `0` disables it, positive values contract the image near the edges, and negative values expand it while leaving the center fixed. The dashboard saves the tuned values in `.thermal-alignment.json`; they can also be seeded with `THERMAL_OFFSET_X`, `THERMAL_OFFSET_Y`, `THERMAL_SCALE`, `THERMAL_BARREL_DISTORTION`, `THERMAL_STRETCH_X`, and `THERMAL_STRETCH_Y`.

The ROS cropper node uses highlighted thermal pixels to publish `/camera/depth/cropped/image_raw` and `/thermal/cropped/image_raw`. In simple mode it does not publish uncropped fallback frames while no thermal region is present. Valid regions publish rectangular crops around the selected thermal cluster and the frontend scales the crop into its fixed display. Each crop unit covers `crop_unit_thermal_pixels` square thermal pixels, clusters count diagonal neighbors, `min_region_size` rejects small clusters, and `inflation_radius_thermal_pixels` expands the crop region. Cropper tuning values live in the `thermal_cropper` block in `config/master_params.yaml` and are passed to `thermal_cropper_node` at launch.

The frontend and ROS cropper receive the same FOV, linear alignment, stretch, and axis-flip launch parameters. `Barrel` is a Full-mode frontend correction and is not passed to the cropper. Cropped thermal frames remain in their native 80×62 coordinates, so Full mode applies the configured transform instead of stretching the thermal mask across the complete depth frame.

Full mode exposes the crop unit, minimum region, inflation, temperature bounds, frame-relative delta bounds, and empty-frame passthrough settings in the Cropper panel. Changes are persisted to `.thermal-cropper.json` and applied only on the next ROS start.

Use **Save to parameter file** in the Full-mode tuning panel to write the current Blend, overlay alignment, and cropper controls into the loaded `master_params.yaml`. The button also keeps the alignment and cropper sidecar files synchronized with those values.

The built-in dashboard calibration defaults are `Overlay=50.0`, `X=0.0`, `Y=0.0`, `Scale=100.0`, `Barrel=0.000`, `H=80.0`, and `V=90.0`. Saved alignment files carry a geometry revision, so calibration from an older setup is ignored automatically.

Set `ROS2_WORKSPACE` when the ROS workspace is not beside this directory. The dashboard defaults to ROS 2 Jazzy; set `ROS_DISTRO` if you are using another distro, and `PORT` to change the dashboard port.

Use `DRONE_MASTER_PARAMS=/path/to/master_params.yaml` to load a different master params file. Use `CAMERA_CALIBRATIONS_PARAMS=/path/to/camera_calibrations.yaml` to override only the calibration file referenced by the master params.
