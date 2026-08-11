# Initiator — iPhone / iPad app

A native SwiftUI + ARKit app for watching the Initiator drone from a phone: the
robot's cropped depth stream, its VIO node status and pose, and a marker drawn
into the live camera view where the robot actually is in the room.

Visualisation and diagnostics only. **The app sends no flight-control commands.**

```
ios/
  InitiatorDrone/
    App/        app entry point and the object graph
    Core/       pure Swift: geometry, imaging, rosbridge wire layer  (unit tested)
    Services/   connection orchestration, HTTP client, image pipeline (unit tested)
    AR/         ARKit session, alignment, SceneKit scene            (iOS only)
    Views/      SwiftUI screens                                     (iOS only)
    Resources/  Info.plist and the mock rosbridge fixtures
  Tests/        XCTest suite
  Scripts/      project generator, fixture generator, headless test runner
```

## Build and run

```bash
cd ios
python3 Scripts/generate_xcodeproj.py     # only needed after adding/removing files
open InitiatorDrone.xcodeproj
```

Deployment target is iOS 16; iPhone and iPad, portrait and landscape.

ARKit needs a real device. In the simulator everything except the camera view
works, and the live view says so instead of showing a black rectangle.

## Install on your iPhone or iPad

Once installed the app is a normal app: it launches from the home screen and
talks to the robot over Wi-Fi with no Mac involved. The Mac is only needed to
build and install it, and again when the signature expires.

### 1. Configure signing (once)

Open the project, select the **InitiatorDrone** target ▸ **Signing &
Capabilities**, tick *Automatically manage signing*, and pick your team. A free
Apple ID works; it appears as *(Personal Team)*.

That is the whole step. Xcode issues a development certificate and works out the
Team ID for you — which matters, because **Xcode never displays the Team ID for a
Personal Team**, and it does not exist at all until that certificate is created.

Then pin it so it survives project regeneration:

```bash
cd ios
python3 Scripts/generate_xcodeproj.py
```

The generator recovers the team from the project, a provisioning profile, or the
`OU` field of your signing certificate, and prints it. Copy that value into
`Scripts/signing.local` (git-ignored):

```bash
cp Scripts/signing.local.example Scripts/signing.local
# replace the placeholders with the values the generator printed
```

Pinning it matters because setting the team in Xcode writes into
`project.pbxproj`, which the generator overwrites the next time anyone adds a
source file. Placeholder values from the example file are ignored rather than
baked in, so a half-edited `signing.local` fails loudly instead of producing a
signing error that names no cause.

Bundle identifiers are globally unique, so `com.initiatordrone.app` may already
be registered to someone else. If you see *"Failed to register bundle
identifier"*, change it in Xcode to your own reverse-DNS prefix, then re-run the
generator to pick it up.

### 2. Prepare the device (once)

- Connect it by USB and tap **Trust** on the device.
- iOS 16+: **Settings ▸ Privacy & Security ▸ Developer Mode**, turn it on, and
  restart when prompted. The device will not accept a development build without
  this.

### 3. Build a Release build

Debug builds are unoptimised and noticeably heavier — the difference is real for
a long AR session. In Xcode: **Product ▸ Scheme ▸ Edit Scheme ▸ Run ▸ Info ▸
Build Configuration → Release**, and untick *Debug executable* so the app does
not wait for the debugger when launched from the home screen.

Then pick your device in the run-destination menu at the top and press **⌘R**.

Equivalent from the command line:

```bash
xcodebuild -project InitiatorDrone.xcodeproj \
           -scheme InitiatorDrone \
           -configuration Release \
           -destination 'generic/platform=iOS' \
           build
```

### 4. Trust the certificate on the device

First launch will refuse with *"Untrusted Developer"*. On the device:
**Settings ▸ General ▸ VPN & Device Management** ▸ your Apple ID ▸ **Trust**.

### 5. Unplug

The app now runs standalone. Two prompts appear on first launch and both matter:

- **Camera** — required for the AR view.
- **Local Network** — required to reach the robot. Denying it makes every
  connection fail silently with no error the app can detect, which looks exactly
  like a robot that is switched off. If you tapped Don't Allow, re-enable it in
  **Settings ▸ Initiator ▸ Local Network**.

The phone and the robot must be on the same Wi-Fi network and subnet.

### How long it lasts

| Account | App works for | Notes |
| --- | --- | --- |
| Free Apple ID | **7 days** | Then it refuses to launch; re-run from Xcode to renew. Max 3 sideloaded apps per device. |
| Paid Developer Program ($99/yr) | **1 year** | Also unlocks TestFlight and ad-hoc `.ipa` distribution, so you can install without a cable. |

Re-signing on a free account does not lose anything: saved robots, colour maps
and settings live in `UserDefaults` and survive a reinstall over the top.

If you plan to keep this on a drone-flying phone for months, the paid account is
the difference between renewing weekly and renewing annually.

### Try it without the drone

On the **Robot** tab, tap **Run on fixtures**. A simulated rosbridge server
replays synthesised depth, odometry, IMU and status messages through the real
message path — same subscribe commands, same JSON envelopes, same decoders.
Everything except Start/Stop/Calibrate behaves as it does against a live robot,
including the reconnect logic (Diagnostics ▸ ⋯ ▸ *Simulate Wi-Fi drop*).

## Connecting to a robot

| What | Where |
| --- | --- |
| Dashboard HTTP | `http://<robot-ip>:4173` |
| rosbridge WebSocket | `ws://<robot-ip>:9090` |

Enter the address on the **Robot** tab. Paste style does not matter — a bare IP,
`http://10.0.0.5:4173/`, or a `.local` name all normalise to the same host.
**Test connection** round-trips `GET /api/state` and reports latency and whether
the ROS graph is running. Robots are saved, most-recent first, and the app
reconnects automatically after a drop with exponential backoff and jitter.

`NSAllowsLocalNetworking` is set in `Info.plist` so plain HTTP and `ws://` to a
local address work without weakening App Transport Security for the internet at
large.

## Topics and endpoints it expects

Subscriptions:

| Topic | Type |
| --- | --- |
| `/camera/depth/cropped/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/depth/cropped/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/vio/odometry` | `nav_msgs/msg/Odometry` |
| `/vio/calibrated` | `std_msgs/msg/Bool` |
| `/vio/visual_tracking` | `std_msgs/msg/Bool` |
| `/imu/data_calibrated` | `sensor_msgs/msg/Imu` |

Dashboard API: `GET /api/state`, `POST /api/start`, `POST /api/stop`,
`POST /api/vio/calibrate`.

All of these exist on this branch. `server.js` implements the four endpoints,
and `drone_launch.py` starts `vio_node`, the Orbbec depth camera, the thermal
cropper that produces the `cropped` topics, and `rosbridge_websocket` on 9090.
Bring them all up with:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true start_depth_camera:=true start_imu:=true \
  start_vio:=true start_thermal_cropper:=true thermal_cropper_enabled:=true
```

or just press **Start** in the app, which asks the dashboard to run its
configured launch command.

The cropped depth topics come from the thermal cropper, which uses hot regions
in the thermal image to decide *where* to crop. The thermal image itself is
never sent to the phone — the app subscribes to the depth output only.

## Screens

**Live** — the camera view with the robot drawn into it, plus the cropped depth
stream, Start, Stop, Calibrate, and Align. Status pills across the top cover the
link, the VIO node, robot tracking, phone AR tracking, and alignment. The metric
strip shows depth fps, odometry rate, VIO node state, and the robot's position
and orientation.

**Robot** — address entry, connection test, saved robots, fixtures mode.

**Diagnostics** — per-topic rate and last-message age, VIO node status, the full
VIO pose and twist, IMU values with a gravity sanity check, phone AR pose, every
ARKit video format the device offers, stream statistics including dropped frames,
and the raw rosbridge log.

**Settings** — depth colour map (fixed or auto range, six ramps), wire format,
image throttle, odometry staleness and extrapolation, the AR camera field of
view, and AR scene options.

## Depth only

Earlier versions of this app offered depth / thermal / blend / picture-in-picture
view modes. It now shows the cropped depth stream and nothing else: the thermal
subscription, the blend slider, the PiP inset, the thermal colour map and the
`mono16` scale and offset settings are all gone.

That is one subscription removed rather than one view hidden — the app no longer
asks rosbridge for `/thermal/cropped/image_raw` at all, so the robot never
serialises or sends those frames. `RosbridgeTests` pins this down: no topic in
`RobotTopic.allCases` may contain "thermal", and `.depthImage` must be the only
image topic.

The thermal sensor is still doing its job on the robot. It drives the cropper,
which is what makes the depth frames the app receives small and targeted.

## VIO node status

The **VIO node** pill answers a different question from **Robot track**. Robot
track asks "can I believe this pose". VIO node asks "is the estimator running at
all", which is the first thing worth knowing when the marker is missing.

| Shown as | Means |
| --- | --- |
| **Unknown** | Not connected to rosbridge. |
| **Graph stopped** | `GET /api/state` reports the launch is not running. Press Start. |
| **Not running** | Graph is up, but nothing has ever arrived on a `/vio` topic — check `start_vio:=true` and whether `vio_node` exited. |
| **Calibrating** | `/vio/calibrated` is false: it is collecting stationary samples and publishes no pose until it finishes. Keep the drone still. |
| **Silent** | The node was heard from, but odometry has stopped or never started. |
| **Running** | Publishing odometry. |

**Why it is derived rather than queried.** The obvious way to answer would be
`/rosapi/nodes`, but `drone_launch.py` starts `rosbridge_websocket` as a bare
node rather than through `rosbridge_websocket_launch.xml`, so `rosapi_node` is
never launched and that service does not exist. The next best evidence is the
node's own output: `vio_node` publishes `/vio/odometry` at IMU rate from the
moment it finishes initialising and stops the instant it dies.

Deriving it this way also survives a detail that would otherwise leave the app
permanently unsure. `/vio/calibrated` and `/vio/visual_tracking` are published
**only when they change**, so a phone that connects after calibration has already
finished may never see either flag. The status therefore does not depend on
them: odometry alone is enough to conclude the node is running, because the node
publishes none until it is initialised. A `false` flag, when one does arrive, is
the node explaining an odometry gap it is itself causing, so it outranks the gap.

(If `rosapi_node` is ever added to the launch, an authoritative node list would
be a strict improvement and would slot in behind the same `VIONodeStatus` type.)

## Camera field of view

**Settings ▸ AR camera ▸ Widest field of view** picks the video format that shows
the most of the room, so the robot marker stays on screen from closer and while
the phone is being moved around.

**ARKit does not offer the ultra-wide lens to world tracking.** Not on the
iPhone 14 Pro Max, not on any current iPhone. ARKit drives that camera itself as
part of tracking, but `ARWorldTrackingConfiguration.supportedVideoFormats`
publishes only `.builtInWideAngleCamera` entries, and there is no supported way
for an app to select the ultra-wide feed and keep world tracking. So the ceiling
on field of view is a property of ARKit, not of the setting or the hardware.

Within that ceiling there is still a real choice, and it is what the setting
makes. Every format from one lens is read from the same sensor: the 4:3 entries
are the full readout, and each 16:9 entry is that same image with the top and
bottom cropped away. Horizontal coverage is identical, so the taller aspect ratio
is strictly more of the scene at no cost. `VideoFormatSelection` prefers an
ultra-wide format if one is ever offered, then the tallest frame, then whatever
ARKit listed first — its own recommendation, and the safer resolution and frame
rate. The ranking is a pure function over resolution and lens type so it is
tested without a device.

**Diagnostics ▸ ARKit video formats** lists every format the device offers with
its lens, and marks the one running. That is the place to check what a given
phone can actually do, rather than trusting this paragraph.

Changing the setting restarts tracking, which moves the world origin, so the
robot alignment is cleared and the app says so.

## How the robot ends up in the right place

Three transforms, kept separate on purpose so a bug in one is visible.

**1. Axis convention.** ROS `odom`/`base_link` is REP-103: right-handed, metres,
`+X` forward, `+Y` left, `+Z` up. ARKit world is right-handed, metres, `+X`
right, `+Y` up, `+Z` toward the viewer. So:

```
ROS +X (forward) -> ARKit -Z
ROS +Y (left)    -> ARKit -X
ROS +Z (up)      -> ARKit +Y
```

Both frames are right-handed and metric, so this is a pure rotation with
determinant `+1` — no mirroring, no unit scaling. `FrameConversion` is the only
place this mapping exists, and `GeometryTests` pins down every claim in that
paragraph, including that the converted orientation lands a SceneKit node's `-Z`
forward axis exactly along the robot's `+X`.

**2. Alignment.** Nothing connects ARKit's world origin (wherever the session
started) to the robot's `odom` origin (wherever VIO initialised) until the
operator says so. Two ways to say it:

- **"The robot is here"** — stand at the robot, point the phone the way the
  robot faces, tap once. This is the intended flow for the "open the app at the
  robot, then walk around" workflow. Only the phone's heading is used, so the
  tilt of the phone in your hand does not tip the robot frame.
- **"Place on a surface"** — aim the crosshair at the floor where the robot
  started, tap to drop the origin, then dial in the heading.

Either produces a `RobotAlignment`: a position plus a yaw about the vertical
axis. Roll and pitch are deliberately not adjustable — both frames are already
gravity-aligned, and allowing them would let a sloppy placement tilt the whole
scene. Nudge controls and a height offset are there for touching up a placement
that is close but visibly off.

AprilTag or another automatic method is the obvious next step. It would produce
exactly the same `RobotAlignment` and nothing downstream would change.

**3. Time.** The odometry buffer is keyed to the robot's ROS clock; rendering
happens on the phone's. `ClockOffsetEstimator` recovers the difference with a
minimum-delay filter over a sliding window, which matters because a Raspberry Pi
without an RTC can be hours off. The SceneKit render callback then samples the
buffer at the exact instant it is about to draw and interpolates — linear on
position, shortest-path spherical on orientation. Drawing the newest message
directly instead makes the marker stutter and lag.

Past the newest sample the marker holds still by default. Extrapolation from the
reported twist is available in Settings but off, because a marker that keeps
gliding on invented motion is worse than one that visibly stops.

## Trusting the pose

`/vio/odometry` continuing to publish proves only that the VIO node is alive. An
estimator that has lost its features keeps dead-reckoning off the IMU and keeps
publishing a pose that drifts away from reality, smoothly and convincingly. So
the app reports the robot as tracking only when it is calibrated **and**
visually tracking **and** the messages are fresh:

| Condition | Shown as |
| --- | --- |
| No odometry for longer than the staleness threshold | **Stale**, marker hidden |
| `/vio/calibrated` false | **Not calibrated**, marker hidden |
| Calibrated, `/vio/visual_tracking` false | **Tracking lost**, marker amber |
| All three good | **Tracking**, marker green |

Staleness is checked before the flags: a "tracking is fine" message from thirty
seconds ago is not evidence of anything.

Calibrate is disabled unless `GET /api/state` reports the graph running, and the
live view says why any disabled control is disabled — including, ahead of
everything downstream of it, that the VIO node is not up.

## Keeping the stream from piling up

Four layers, outermost first:

1. **Throttle at the robot.** Subscriptions carry `throttle_rate` (66 ms by
   default, ~15 fps) and `queue_length: 1`. Frames that are never sent cannot
   queue anywhere.
2. **`LatestOnlySlot`.** A one-deep mailbox between the socket and the decoder.
   A `DispatchQueue.async` per frame is an unbounded queue; this replaces the
   pending frame instead of appending. Memory is bounded by one frame however
   far behind the decoder falls.
3. **Reusable buffers.** The colour-map renderer writes into a buffer it keeps
   between frames rather than allocating one per frame.
4. **Bounded history.** The odometry buffer is capped by both sample count and
   time span; rate trackers and the log ring are capped too.

Dropped-frame counts are on the Diagnostics screen. A number climbing steadily
means the throttle is set faster than this phone and link can keep up.

Phone motion comes from ARKit's visual-inertial fusion only. CoreMotion
acceleration is never double-integrated anywhere — that drifts metres within
seconds.

## Tests

```bash
Scripts/run-core-tests.sh
```

189 tests, no Xcode required. The suite compiles `Core/`, `Services/` and
`Tests/` for macOS with `swiftc` and runs them against a small XCTest shim; the
same files run unmodified under `⌘U` in Xcode, which additionally covers the AR
and SwiftUI layers by building them.

```
PASS GeometryTests            29 passed,   0 failed
PASS ROSImageDecoderTests     25 passed,   0 failed
PASS OdometryTests            28 passed,   0 failed
PASS RosbridgeTests           43 passed,   0 failed
PASS RenderingTests           28 passed,   0 failed
PASS ConnectionTests          33 passed,   0 failed
PASS ImageStreamSoakTests      3 passed,   0 failed
```

Coverage of the things most likely to be silently wrong:

- **Image decoding** — every supported encoding, row padding via `step`,
  big-endian payloads, unaligned row starts, truncated data, and the ROS
  convention that a zero in a 16-bit depth frame means "no return" and must
  become `NaN` rather than a wall at the lens.
- **Quaternions and coordinates** — algebra, matrix round-trips through all four
  trace branches, shortest-path slerp across the double cover, the full ROS ↔
  ARKit axis mapping including a handedness check, and the alignment pipeline
  round-tripping back to `odom`.
- **Odometry interpolation** — interpolation, clamping at both ends, twist
  extrapolation in the body frame, out-of-order and duplicate stamps, bounded
  history, staleness, and clock-offset recovery under varying latency.
- **VIO node status** — that odometry alone proves the node is running (the case
  a latched-flag-only design would get wrong), that a stopped launch is reported
  as such rather than as a crashed node, that `calibrating` outranks the odometry
  gap it causes, and that an unknown launch state is not read as a stopped one.
- **AR video format choice** — that the tallest frame wins when no ultra-wide is
  offered (the iPhone 14 Pro Max case), that an ultra-wide format would win
  outright if one ever appeared, and that equal aspect ratios fall back to
  ARKit's ordering rather than to resolution.
- **Reconnection** — the backoff curve and jitter bounds as pure functions, plus
  an end-to-end run where the fixture transport drops the link the way Wi-Fi
  does (no close handshake, just silence) and the client is required to notice,
  back off, reconnect, re-subscribe, and resume data.
- **Memory over a long stream** — `ImageStreamSoakTests` drives the decode →
  colour-map → `CGImage` path and asserts resident memory does not grow with
  frame count. It defaults to 1,500 frames; the full 30-minute equivalent runs
  with `INITIATOR_SOAK_FRAMES=27000 Scripts/run-core-tests.sh` and passes (a leak
  of even a few KB per frame fails at that scale). A separate test floods the
  pipeline and requires the drop counter to rise rather than the queue.

Fixtures in `InitiatorDrone/Resources/Fixtures/` are real rosbridge frames with
hand-picked payloads, so the tests assert exact decoded values. Regenerate with
`python3 Scripts/generate_fixtures.py`.

## Wire format

JSON by default. CBOR (Settings ▸ Image stream) is worth switching on if it
works with your rosbridge: `uint8[]` fields arrive as RFC 8746 tag-64 byte
strings instead of base64, which is about a third less traffic and skips a
decode pass. Needs `rosbridge_suite` 0.11 or newer — switch back to JSON if
frames stop arriving after changing it.

## Not built yet

- Point-cloud generation and `PointCloud2` — out of scope for this version. The
  camera frustum drawn from `CameraInfo` is the placeholder for it.
- AprilTag or automatic alignment.
- Depth reprojected into the AR scene. The alignment pipeline it depends on
  wants validating against real hardware first.
- An authoritative node list. Adding `rosapi_node` to `drone_launch.py` would
  let `VIONodeStatus` confirm what it currently infers.
- A genuinely wider camera view. It would need `AVCaptureSession` on the
  ultra-wide lens with pose estimation of our own, since ARKit will not give
  world tracking and that lens at the same time. That is a large piece of work
  to replace something Apple already does well.
