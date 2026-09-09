# DroneView

Native iPhone/iPad viewer for the `camera-and-gyro` branch of
`ros2-initiator-drone`. SwiftUI provides the interface; Metal draws the scene.
The Pi-viewer tabs use HTTP; the Thermal AR tab adds phone-side reconstruction.
No website, JavaScript or rosbridge is embedded.

## Run

Open `DroneView/DroneView.xcodeproj`, select **DroneView**, choose an iPhone or
iPad simulator (or set your signing team for a physical device), and Run.
Requires iOS 18+ and Xcode 26+; built and tested here with Xcode 27 beta.

The app opens the Thermal AR tab. It connects automatically to the
saved address, initially `http://192.168.1.6:8080`. The floating network button
changes the address or disconnects. Allow local-network access on a physical
phone. The module must run `bash scripts/run.sh --host 0.0.0.0` on the same network.

## Controls

- Drag to orbit, pinch to zoom. **Fit** frames the map and camera.
- **View** contains Follow camera, trajectory visibility, drag-to-pan and zoom.
- The system **Thermal AR**, **Scene**, **Cameras** and **Tracking** tab bar switches native
  screens. System toolbar buttons float over the full-screen Metal scene.
- Cameras switches among RGB, registered depth (0–6 m colour scale), and
  independent thermal imagery. Old frames are dimmed and marked stale.
- Tracking shows optical-frame position, inliers, frame processing, image timing,
  valid depth and gyro status. It also starts/stops module recordings (300-frame
  limit), exports a PLY map to Files, and requests a new map after confirmation.

A recording belongs to the module and continues if the app disconnects. A new
map clears the module's map; save it first if it needs to be retained. Simulation
is explicitly labelled. Local odometry drifts and is not gravity aligned.

## Data flow

The app reads `/api/state`, `/api/scene`, and `/api/image/{rgb,depth,thermal}`.
Commands use JSON POSTs to `/api/record` and `/api/reset`; map export uses
`/api/map.ply`. All endpoints are on the selected HTTP(S) origin.

State and images poll at most five times per second; changed maps fetch at
most once per second. Each stream permits only one outstanding request.
Scene decoding runs off the main actor. Polling cancels when the app enters the
background, changes server, or disconnects. The last accepted scene is retained
on connection loss, with a visible stale/disconnected status. Recording toggles
are never retried automatically after an ambiguous network error.

The binary point cloud is little-endian float32 XYZ/RGB. Metal renders point
primitives and line primitives with depth testing. GPU point buffers are replaced
only when a map arrives. Optical X/right, Y/down, Z/forward maps to display
X/right, Y/up, Z/back; pose, trajectory and map use the same conversion.

## Checks

```bash
# Pure decoding, coordinates, camera controls and HTTP contract checks:
bash Tests/run.sh

# Additional read-only checks against a running module:
bash Tests/run.sh http://192.168.1.6:8080

# Native model integration tests (use your installed simulator's name):
xcodebuild -project DroneView/DroneView.xcodeproj -scheme DroneView \
  -destination 'platform=iOS Simulator,name=iPhone 17 Pro' \
  CODE_SIGNING_ALLOWED=NO test
```

The native test uses an isolated URLSession fixture for polling, images, command
handling, export, stale states, feed switching, reconnects and cancellation.
It sends no recording/reset requests to the physical module.

## Phone reconstruction and thermal AR

The **Thermal AR** tab uses ARKit for the phone and native OpenCV 4.13 PnP for
the external rig. Xcode resolves the pinned OpenCV package automatically. The
existing Scene/Cameras/Tracking tabs retain their Pi-viewer behaviour.

Start the Pi with `--processing phone`, connect to its port 8080, then select
**Thermal AR**. Scanning starts automatically. Allow Camera and Local Network access. Point
both cameras at the same well-lit textured area with geometry at different depths.
Three consistent RGB/depth fits establish the shared frame. No markers are used.
LiDAR hardware is required; the simulator displays the unsupported-device state.

The overlay uses the previous FOV/offset/scale/stretch/barrel alignment; **Thermal**
opens its controls. Saved values are scoped to the server and current camera
profile. No measured thermal extrinsics are claimed. Changes reset the local map.
Thermal values use the same already-oriented image as the Pi preview; legacy
browser flips are not reapplied. The v2 orientation migration keeps saved offsets,
scale and distortion while resetting the erroneous extra flips.
For a saved adjustment, `Documents/ThermalAlignment.json` accepts a JSON dictionary
from the exact camera-profile key in the scan log to a `ThermalAlignment` object.
A matching valid profile is saved through the app's preferences and the import file
is consumed; mismatched or invalid files are retained and logged. The approximate
2026-09-08 fit and its manual correspondences are in `DroneView/thermal-alignment-2026-09-08.json`.
The AR heat scale defaults to a fixed 19–28 °C, matching the Pi preview. Manual
limits remain adjustable; automatic relative contrast is optional. In either mode,
changing the scale never changes stored Celsius values. Thermal values refresh
on every accepted RGB-D frame, including when the rig is stationary. The nearest
of eight recent thermal samples is selected by capture time, retaining the
150 ms rejection limit.

A 60,000-voxel map is rendered with Metal. Show through walls applies to both dots and the filled highlight; switching it off restores phone-depth occlusion.
Thermal observations over 150 ms from RGB are skipped. Cross-camera association
requires capture-time agreement within 100 ms including half the clock probe RTT.
Missing alignment, limited AR tracking or network loss hides the overlay and
stops map integration. Alignment persists when the cameras no longer share a view.
Rejected rig frames or missing timestamp pairs pause placement; accepted odometry
recovery in the same rig map resumes it. AR tracking loss, network failure, map
resets, and **Realign** still require another shared view. This first version is
for mostly static indoor scenes, not moving-object thermal reconstruction.
Thermal AR is the first/default tab and starts automatically. Switching tabs keeps
capture and reconstruction running. Backgrounding/locking suspends camera capture
as required by iOS; returning resumes automatically. Pi recording is independent.
A live filled heat surface highlights samples above 20 °C by default (adjustable).
The cutoff, visibility, through-wall setting and temperature scale persist across restarts.
Show through walls bypasses phone depth occlusion for dots and highlights; it shows
what the rig sees, not through-wall sensing. Warm objects also qualify. Highlights
expire after 500 ms without a fresh observation. Hot map samples are replaced each
accepted frame, and old foreground points are removed when current depth sees
farther surfaces, limiting trails after a person moves.
The Pi receives both poses and diagnostics, but not the reconstructed map.

### Native replay check

`Tests/prepare_replay.py BACKEND RECORDING OUTPUT` exports identical JPEG-decoded
frames and Python poses. Build `Tests/TrackingReplay.cpp` against OpenCV 4.13 and
run it with `OUTPUT/frames.yml` to obtain native CSV poses. Use Python OpenCV
4.13 for parity: a different OpenCV release can choose different RANSAC solutions.
This replay is visual-only; gyro integration also needs recorded/device checks.
Simulator tests cover binary decoding, transforms, thermal orientation, invalid JPEGs, and actual GPU rendering of dots and filled heat with wall occlusion both enabled and disabled.
Physical AR alignment, occlusion and sustained 10-fps processing/30-fps rendering
remain hardware acceptance checks, not guarantees from the replay.

Implementation checks (2026-09-08): signed iPhone 16 Pro build and installation,
simulator integration/stream tests, and native gyro interpolation/bias/gap checks
passed. Visual-only native/Python OpenCV 4.13 replay matched all 300 movement
frame decisions, with maximum transform-element difference below 5e-13.
The Pi passed 26 Python tests and six driver tests at backend commit 6ff1391.
Live AR and sustained FPS acceptance are not established by these checks.

Phone diagnostics: **Thermal → Diagnostics → Export scan log** shares JSONL with
camera stalls, clock offset/RTT, frame timing, tracking status and rates. It excludes
images and poses. Logs live in the app's Documents/ScanLogs, retain five files,
and cap each file at 4 MB. The AR history stores copied grayscale/depth snapshots,
not retained ARFrames, so a delayed network frame cannot exhaust ARKit's buffers.
