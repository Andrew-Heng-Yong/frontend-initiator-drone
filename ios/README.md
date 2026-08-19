# Initiator — iPhone / iPad app

A native SwiftUI + ARKit app for watching the Initiator drone from a phone: the
robot's depth stream, its odometry node status and pose, and a marker
drawn
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
Everything behaves as it does against a live robot, including the reconnect
logic (Diagnostics ▸ ⋯ ▸ *Simulate Wi-Fi drop*).

**Start, Stop and Calibrate work here too.** They drive the simulator instead of
the dashboard HTTP API, and it models the same states rather than merely
flipping a label:

- **Stop** silences every topic, the way killing the launch removes the nodes.
  The Odom node pill goes to *Graph stopped* and the depth panel stops updating.
- **Start** brings them back and runs a startup calibration first, as a real
  launch does.
- **Calibrate** publishes `calibrated = false`, withholds odometry while it
  "collects stationary samples", then publishes `true` and resumes — the exact
  sequence `odom_node` produces, so the *Calibrating → Running* transition is
  worth watching.

One deliberate difference: stopping the real launch also kills rosbridge, so the
socket drops. The simulator keeps the link up, because otherwise the client would
reconnect straight away and the stopped state would never stay on screen long
enough to inspect.

The fixture robot turns slowly on the spot and **never translates**, because
that is all `odom_node` can report — it publishes a 1e6 m² position variance,
which the app reads and shows as *Heading only*. This used to fly a
figure-of-eight, which made a better demo and a worse simulator: it taught the
operator to expect a marker that moves across the room, when the real one never
does.

**Settings ▸ Fixtures mode ▸ Hold the robot still** stops the rotation too,
while odometry keeps publishing at the same rate with the same timestamps — only
the pose stops changing. Hold it still when you are checking *where* the marker
and point cloud land, because a turning robot makes a yaw alignment error
indistinguishable from motion.

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
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/odom` | `nav_msgs/msg/Odometry` |
| `/odom/calibrated` | `std_msgs/msg/Bool` |
| `/imu/data_calibrated` | `sensor_msgs/msg/Imu` |

Dashboard API: `GET /api/state`, `POST /api/start`, `POST /api/stop`,
`POST /api/odom/calibrate` (falling back to `/api/vio/calibrate` on a 404, so a
phone updated ahead of the robot's dashboard still works).

All of these exist on this branch. `server.js` implements the four endpoints,
and `drone_launch.py` starts `odom_node`, the Orbbec depth camera, and
`rosbridge_websocket` on 9090. Bring them all up with:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true start_depth_camera:=true start_imu:=true \
  start_odom:=true
```

or just press **Start** in the app, which asks the dashboard to run its
configured launch command. That command may still start the thermal cropper for
the browser dashboard's benefit; the app is unaffected either way, because it
subscribes to the camera driver's own depth topics.

## Screens

**Live** — the camera view with the robot drawn into it, plus the depth
stream, Start, Stop, Calibrate, and Align. Status pills across the top cover the
link, the odometry node, robot tracking, the AprilTag fix, phone AR tracking,
and alignment. The metric strip shows depth fps, odometry rate, odometry node
state, and the robot's position and orientation.

Two levels of hiding, because they answer different questions. **Hide depth**
folds away the depth panel while the pills, metrics and controls stay up — for
when the stream is in the way but you are still driving the robot. The
**full-screen** button (⤢, next to it) clears everything: pills, metrics, depth
panel, control bar, and the tab bar, status bar and home indicator with them.
What is left is the camera and the robot marker, which is what you want when
checking whether the marker lands on the real robot, or pointing the phone at
something for someone else to look at.

A small ⤡ button in the top corner is the way back. It is a button rather than
tap-to-restore on purpose: a full-screen tap catcher would sit over the AR view
and swallow the taps that place the alignment origin.

**Robot** — address entry, connection test, saved robots, fixtures mode.

**Diagnostics** — per-topic rate and last-message age, odometry node status
including whether position is measured and its published variance, the full pose
and twist, AprilTag relocalisation state, IMU values, phone AR pose, every ARKit
video format the device offers, stream statistics including dropped frames, and
the raw rosbridge log.

The IMU rows adapt to what the publisher claims: `odom_node` marks linear
acceleration unavailable with `covariance[0] = -1`, so the acceleration and
gravity-check rows read *not published* instead of showing 0.00 m/s² and
flagging a healthy node as broken.

**Settings** — depth colour map (fixed or auto range, six ramps), wire format,
image throttle, odometry staleness and extrapolation, and AR scene options.

## Depth only

Earlier versions of this app offered depth / thermal / blend / picture-in-picture
view modes. It now shows the depth stream and nothing else: the thermal
subscription, the blend slider, the PiP inset, the thermal colour map and the
`mono16` scale and offset settings are all gone.

The thermal camera is out of the app's path entirely. It never asked rosbridge
for `/thermal/cropped/image_raw`, and it no longer takes the *cropped* depth
topics either — those are republished by the thermal cropper, so subscribing to
one made the depth view depend on the thermal sensor finding a hot region, and
made the visible window jump around with whatever the thermal camera saw. The
app now takes `/camera/depth/image_raw` straight from the camera driver.

`RosbridgeTests` pins this down: no topic in `RobotTopic.allCases` may contain
"thermal" or "cropped", and `.depthImage` must be the only image topic.

Frames are correspondingly larger than a crop, which is what the throttle and
the `queue_length: 1` backlog controls are for.

## What odom_node can and cannot tell you

`vio_node` was replaced by `odom_node`, and the change is larger than a rename.
The new node integrates the **gyro and nothing else**. It has no camera input,
no accelerometer fusion, and — the part that shapes the whole app —

> **it does not estimate position at all.**

Translation is held at zero and published with a variance of 1e6 m², which is
`nav_msgs/Odometry`'s way of saying "this number is not a measurement". So the
robot marker sits exactly where you aligned it and turns in place. That is not a
bug and not a lost fix; it is the entire estimate the robot currently has.

The app reads that covariance rather than being told which robot it is talking
to. If a flow sensor or GPS is added and the variance drops, **Robot track**
starts reading *Tracking* on its own, with no code change.

For stationary bench testing, `odom_static_override:=true` changes that contract:
the node publishes a fixed identity pose with low position covariance and reports
it calibrated. The app then shows **Robot track: Tracking**. This is an explicit
promise that the robot will not move, not a translation estimate; disable it and
restart before the robot can move.

`odom_quality_override:=true` keeps the live gyro orientation but publishes low
position covariance anyway. The app consequently shows **Tracking**, although
translation remains an unmeasured zero placeholder. This is deliberately a
reported-quality override and does not improve odometry.

`/vio/visual_tracking` and `/vio/video_working` did not survive the change because
the odometry node has no camera input. The old `vio_static_override` launch option
is now `odom_static_override`.

Renamed:

| Old | New |
| --- | --- |
| `/vio/odometry` | `/odom` |
| `/vio/calibrated` | `/odom/calibrated` |
| `/vio/calibrate` service | `/odom/calibrate` |
| `start_vio:=true` | `start_odom:=true` |
| `vio_static_override:=true` | `odom_static_override:=true` |

Calibration is now a stationary **gyro-bias** estimate rather than a gravity and
visual alignment. It rejects and restarts its own sample window if the drone
moves during it, so the *Calibrating* state can last longer than you expect on a
windy roof.

## Odom node status

The **Odom node** pill answers a different question from **Robot track**. Robot
track asks "how much of this pose can I believe". Odom node asks "is the
estimator running at all", which is the first thing worth knowing when the
marker is missing.

| Shown as | Means |
| --- | --- |
| **Unknown** | Not connected to rosbridge. |
| **Graph stopped** | `GET /api/state` reports the launch is not running. Press Start. |
| **Not running** | Graph is up, but nothing has arrived on `/odom` or `/imu/data_calibrated` — check `start_odom:=true` and whether `odom_node` exited. |
| **Calibrating** | `/odom/calibrated` is false: it is collecting stationary gyro samples and publishes no pose until it finishes. Keep the drone still. |
| **Silent** | The node was heard from, but odometry has stopped or never started. |
| **Running** | Publishing odometry. |

**Why it is derived rather than queried.** The obvious way to answer would be
`/rosapi/nodes`, but `drone_launch.py` starts `rosbridge_websocket` as a bare
node rather than through `rosbridge_websocket_launch.xml`, so `rosapi_node` is
never launched and that service does not exist. The next best evidence is the
node's own output: `odom_node` publishes `/odom` at IMU rate from the moment it
finishes calibrating and stops the instant it dies.

`/odom/calibrated` uses transient-local durability and is also republished once
per second. The heartbeat matters for rosbridge clients: a phone connecting after
startup still receives the current calibrated state. Odom-node presence is
derived from odometry/IMU traffic independently, so a missing status message does
not make a healthy node look absent.

(If `rosapi_node` is ever added to the launch, an authoritative node list would
be a strict improvement and would slot in behind the same `OdomNodeStatus` type.)

## Camera field of view

The session always runs the widest video format the device offers. This is not a
setting: nothing would be gained by choosing a cropped one.

**There is no ultra-wide option, and it was tried.** ARKit does not offer that
lens to world tracking on any current iPhone, including the 14 Pro Max —
`ARWorldTrackingConfiguration.supportedVideoFormats` publishes only
`.builtInWideAngleCamera` entries. ARKit drives the ultra-wide camera itself as
part of tracking but never exposes it as a format an app can select, and reaching
it through `AVCaptureDevice` means giving up camera poses, which is the one thing
this app cannot do without. An earlier version of this code had a preference for
it; on hardware it was dead UI, so it is gone.

What remains is a real choice. Every format comes from that one lens and one
sensor: the 4:3 entries are the full readout, and each 16:9 entry is the same
image with the top and bottom cropped away. Horizontal coverage is identical, so
the taller aspect ratio is strictly more of the scene at no cost.
`VideoFormatSelection` takes the tallest frame, tie-broken by ARKit's own
ordering — its recommendation, and so the safer resolution and frame rate. The
ranking is a pure function over resolution, so it is tested without a device.

On an iPhone 14 Pro Max that lands on 1920x1440 @ 60 fps, which is also what
ARKit would have picked by default. The value of choosing it explicitly is that
it stays true on a device whose default is a 16:9 crop.

**Diagnostics ▸ ARKit video formats** lists every format the device offers and
marks the one running. That is the place to check what a given phone can actually
do, rather than trusting this paragraph.

## The point cloud

The depth frame is deprojected on the phone and drawn in the AR scene, so the
room the robot has scanned appears anchored in the real room around you. No
`PointCloud2` topic is involved — the app already has the depth image and the
`CameraInfo` intrinsics, which is everything the pinhole model needs:

```text
Z = depth(u, v)
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

Those points are in the **camera optical frame** (`+X` right, `+Y` down, `+Z`
forward), which is a third convention on top of `base_link` and ARKit. With the
camera bolted straight to `base_link`, composing optical → `base_link` → ARKit
collapses to flipping Y and Z, and `DepthPointCloudTests` pins down each hop
separately so that shortcut cannot quietly rot.

The cloud is attached as a child of the robot marker node, so it inherits the
`odom`-to-ARKit pose and the alignment for free and needs no transform of its
own. One consequence worth knowing: **it only appears once you have aligned the
robot.** No alignment means no pose, which means no honest place to put the
points.

Building it costs nothing extra per frame: it happens on the depth pipeline's
existing worker, from the decode that was already being done, so it inherits the
same latest-only drop policy — a phone that cannot keep up skips whole frames
rather than queueing clouds. Rebuilding the SceneKit geometry is gated on a
generation counter, so a 60 Hz render loop against a 5 Hz depth stream does an
integer comparison rather than rebuilding tens of thousands of vertices.

Density, range and point size are in **Settings ▸ Point cloud**. The colours
come from the same depth colour map as the 2D panel, so the two always agree.

## The camera mount

The depth camera is not at `base_link`. It sits ahead of and below the body
origin, usually tilted down, and until that offset is entered the whole cloud
inherits the error — **a 15° pitch error lifts a wall 2 m away by about half a
metre**, which reads as a room that is subtly the wrong shape rather than as an
obvious fault.

**Settings ▸ Camera mount** takes the measured mounting:

| Field | Axis | Units | Sign |
| --- | --- | --- | --- |
| Forward | `base_link` `+X` | m | ahead of the origin |
| Left | `base_link` `+Y` | m | to the robot's left |
| Up | `base_link` `+Z` | m | above the origin |
| Pitch | about `+Y` | ° | **positive tilts the camera down** |
| Roll | about `+X` | ° | **positive drops the right side** |

These are typed rather than dragged on a slider, because they are numbers read
off a tape measure and a protractor: rounding `0.085 m` to the nearest slider
step is how a mount ends up a centimetre out for no reason anyone can see.

Both the point cloud and the frustum use it, by two different mechanisms that
are checked against each other. The frustum is a node moved by
`CameraExtrinsics.poseInARNode`, so changing the mount never rebuilds its mesh.
The cloud folds the mount into its vertices instead, because it is rebuilt every
frame anyway and a second transform node would be one more place for the two to
disagree. `CameraExtrinsicsTests` asserts they describe the same camera, that an
identity mount reproduces the flip-Y-and-Z shortcut exactly, and that each sign
goes the way the table says.

**Yaw is deliberately absent.** A camera rotated about the vertical is
indistinguishable from a robot pointing somewhere else, so a mis-measured yaw
entered here would hide a real heading error. A genuinely yawed camera needs the
URDF, which is also where all of this properly belongs — the honest long-term
fix is to subscribe to `/tf_static` and read the real extrinsic instead of
asking a person to type it.

## AprilTag relocalisation

`odom_node` does not estimate position, so until now the robot marker sat
wherever a person put it with **Align robot** and drifted from that moment on.
A tag on the robot fixes that: whenever the phone can see one, it knows where
the robot actually is, in its own world frame.

### Setting it up

**Settings ▸ Offsets ▸ AprilTags**. Add a tag and give it three things:

| Field | Notes |
| --- | --- |
| **ID** | 0–29. The family is always `tag16h5`; the row draws the actual tag so you can check it against the marker on the robot. |
| **Size** | The edge of the **outer black border**, in millimetres. Not the paper, not the payload inside the border. |
| **Offset** | Where the tag sits on the robot, in the same frame as the camera mount. |

Size is the one that bites. Range scales directly with it, so a tag entered 20%
too large puts the robot 20% too far away — and nothing else looks wrong.

The offset frame is the camera mount's, so there is one idea to hold: stand
behind the thing looking the way it faces, `+X` is out, `+Y` is your left, `+Z`
is up. A tag on the nose is all zeros; the tail is `yaw 180°`; facing the sky is
`pitch −90°`. **Yaw is present here and matters**, unlike the camera mount: it is
what decides which way the app thinks the robot is pointing.

Print tags with a white margin around the black border. The detector finds tags
by tracing the outline of dark blobs, so a tag butted against a dark background
merges into it and is never found.

### What happens then

A sighting is converted straight into a `RobotAlignment` — the same object
**Align robot** produces — so an automatic fix and a manual placement are
interchangeable and share every downstream path. When a tag is in view the robot
is placed from it; when none is, the marker holds its last fix and follows
odometry, which for `odom_node` means heading only.

Roll and pitch from the sighting are discarded. Both `odom` and the ARKit world
are gravity-aligned, so tilt between them is error by definition — and tilt is
exactly where a monocular tag pose is weakest. Throwing it away keeps the scene
level and drops the noisiest part of the measurement.

Manual alignment is never disabled: the tag can be obscured, unlit, or not
fitted. A manual placement also stops the app describing the alignment as
tag-derived, because the operator has overruled it.

### Why the app is so cautious about it

`tag16h5` has 30 codes in a 65,536-value space, and every code has four
rotations. Roughly one random 16-bit pattern in 550 decodes to *something*, and
a cluttered room offers the detector many quadrilaterals per frame. A false
positive here does not nudge the marker — it teleports the robot somewhere else,
and there is no way to tell a wrong relocalisation from a right one by looking
at it.

So there are five guards, and they are cheap in the order they run:

1. **Exact-match decoding.** No error correction by default. The family's
   distance of 5 makes correcting two bits sound safe, but every forgiven bit
   multiplies the space of random patterns that decode to a valid ID.
2. **The border check.** All twenty black-border cells must read black, before
   anything is decoded. This throws away almost every non-tag quad in a scene
   for the cost of twenty samples.
3. **Decision margin.** The gap between the darkest cell read as white and the
   brightest read as black. A coin-toss decode is refused.
4. **Reprojection.** The homography solve is exact, so it cannot check itself;
   reprojecting the *rigid pose* extracted from it can. A quad that is not
   really a square in the world fails here.
5. **Consecutive agreement.** The one that matters most. The same tag must be
   seen on several frames in a row (three by default). False positives are
   uncorrelated frame to frame, so agreement squares an already small
   probability, while a real tag in view satisfies it in a fraction of a second.

Plus a plausibility limit: a sighting that would jump the robot further than
**Largest correction** is ignored. The first fix of a session is exempt, because
there is nothing to disagree with yet.

**Diagnostics ▸ AprilTag relocalisation** shows which tags are configured, the
detector rate, how many frames were skipped, and how old the last fix is.
Skipped frames are expected and healthy: frames are taken latest-only, and a
queued frame would be paired with a stale phone pose — worse than no fix at all.

### The detector

Written in Swift rather than vendored, so the whole pipeline runs on the
headless test runner. `AprilTagTests` synthesises a tag at a known pose,
projects it with known intrinsics, puts it through the real pipeline, and
compares the recovered pose against the one it was drawn from — at several
ranges, tilts and all four quarter turns.

The stages: downscale, adaptive threshold, connected components, boundary
trace, polygon simplification, sub-pixel corner refinement, decode, pose from
homography. Two of those exist because of measurements rather than theory:

- **Sub-pixel refinement.** Douglas-Peucker can only return points that lie on
  the contour, so its corners are pixel-quantised. On a tag filling 50 pixels
  that is a two-to-three pixel error, and a border cell centre sits only four
  pixels inside the edge — so the decoder samples a payload cell where it
  expects black border and rejects the tag. Fitting lines to the traced edges
  and intersecting them fixes it. Before this, tags at a 20–30° tilt failed.
- **Half-pixel dilation.** The tracer returns the centres of the outermost dark
  pixels, but the tag's edge is half a pixel further out on every side.
  Uncorrected, a 27-pixel tag at 1.8 m reported 1.93 m — an error invisible up
  close that grows with distance, which is the worst shape an error can have.

## The localisation log

**Settings ▸ Localisation log ▸ Record localisation** writes a CSV with one row
per estimate: every accepted tag fix, and the odometry estimate a few times a
second.

Both are the robot's pose **in the AR world frame**. That is the point — the tag
pose starts out relative to the phone, and logging it that way would produce a
file measuring how the operator walked around the room. Differencing the two
sources at the same instant is the drift.

Tag rows also carry the odometry reading from that same moment, so a fix can be
compared against odometry without interpolating between the rows on either side
of it.

| Column group | Contents |
| --- | --- |
| `sequence`, `wall_time_iso`, `session_seconds`, `source` | `source` is `apriltag` or `odom` |
| `robot_*` | robot pose in the AR world frame, position + quaternion + yaw |
| `tag_*` | ID, range, decision margin, reprojection error (tag rows only) |
| `odom_*` | the raw `/odom` reading in the ROS `odom` frame |
| `phone_*` | the phone's own AR pose |
| `align_*` | the alignment transform in force |

Absent values are blank rather than zero, so a reader can tell "no tag on this
row" from "a tag exactly at the origin" — pandas reads a blank as `NaN`, which
is what it is. Headings are in degrees, and `odom_yaw_deg` is measured about ROS
`+Z` while `robot_yaw_deg` is about the AR up axis, because those are the frames
they live in.

Files are listed newest first with a share button, and also appear in the Files
app under **Initiator ▸ Localization**. Numbers are written with a full stop
whatever the phone's region is set to; a locale-aware formatter would write
`1,25` and silently shift every column one to the right.

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
started) to the robot's `odom` origin (wherever `odom_node` calibrated) until the
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

AprilTag relocalisation now does this automatically when a configured tag is in
view — see above. What follows is the manual path, which is still the fallback
whenever no tag is visible.

Another automatic method would produce
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

`/odom` continuing to publish proves only that `odom_node` is alive. An estimator
keeps publishing a pose whether or not it has anything to base it on — smoothly,
convincingly, and in the case of gyro integration, drifting the whole time. So
the app reports the robot as fully tracking only when it is calibrated **and**
the messages are fresh **and** the covariance claims a measured position:

| Condition | Shown as |
| --- | --- |
| No odometry for longer than the staleness threshold | **Stale**, marker hidden |
| `/odom/calibrated` false | **Not calibrated**, marker hidden |
| Calibrated and fresh, position variance ≥ 1e3 m² | **Heading only**, marker amber |
| All three good | **Tracking**, marker green |

**Heading only is the normal state when static override is disabled.** It is not a warning about a fault;
it is the app declining to pretend that a position it was never given is a
measurement. It shows in the pill rather than as a banner over the camera view,
because a banner that is always up is wallpaper.

Static override is the intentional exception: its low covariance makes the fixed
origin a complete tracked pose for visualization while the stationary promise is
in force.

Quality override is the explicit diagnostic exception: it also supplies low
covariance, but preserves live gyro rotation and makes no stationary guarantee.

Staleness is checked before the flags: a "calibrated" message from thirty
seconds ago is not evidence that anything is running now.

Calibrate is disabled unless `GET /api/state` reports the graph running and
static override is inactive, and the
live view says why any disabled control is disabled — including, ahead of
everything downstream of it, that the odometry node is not up.

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

209 tests, no Xcode required. The suite compiles `Core/`, `Services/` and
`Tests/` for macOS with `swiftc` and runs them against a small XCTest shim; the
same files run unmodified under `⌘U` in Xcode, which additionally covers the AR
and SwiftUI layers by building them.

```
PASS GeometryTests            29 passed,   0 failed
PASS ROSImageDecoderTests     25 passed,   0 failed
PASS OdometryTests            28 passed,   0 failed
PASS RosbridgeTests           43 passed,   0 failed
PASS RenderingTests           28 passed,   0 failed
PASS ConnectionTests          34 passed,   0 failed
PASS ImageStreamSoakTests      3 passed,   0 failed
PASS DepthPointCloudTests     19 passed,   0 failed
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
- **Odom node status** — that odometry alone proves the node is running (the
  case a latched-flag-only design would get wrong), that a stopped launch is
  reported as such rather than as a crashed node, that `calibrating` outranks
  the odometry gap it causes, and that an unknown launch state is not read as a
  stopped one.
- **The camera mount** — that each sign goes the way the settings screen says,
  that rotation is applied before the offset rather than after, that the
  precomputed transform matches going through `bodyPoint` and `FrameConversion`
  the slow way, that the frustum node and the cloud vertices describe the same
  camera, and that an identity mount reproduces the flip-Y-and-Z shortcut
  exactly.
- **Position observability** — that a 1e6 m² variance reads as unobserved, that
  an all-zero covariance means "not filled in" rather than "perfectly known",
  and that a missing covariance is trusted rather than second-guessed.
- **The AprilTag detector, end to end on synthesised frames** — a tag rendered
  at a known pose and recovered through the real pipeline, at several ranges,
  at tilt, and at all four quarter turns. Plus the rejections: a blank dark
  rectangle, a tag with no configured size, unusable intrinsics. The family's
  minimum Hamming distance of 5 is recomputed over every code and rotation,
  which is really a checksum on the transcribed code table.
- **Tag localisation conventions** — scenarios that derive the detector's tag
  frame from where a reader would have to stand, never from the constant being
  checked, so a wrong axis convention cannot pass. Plus that an alignment
  reproduces its robot pose through the existing render path, and that roll and
  pitch are dropped from a noisy sighting.
- **The fix gate** — that several consecutive frames are required, that a
  different tag restarts the count rather than adding to it, that the first fix
  of a session is never blocked as implausible, and that a refused jump keeps
  the streak alive so a genuinely large correction stays possible.
- **The CSV log** — schema width, blank-not-zero for absent values, a full stop
  as the decimal separator, contiguous sequence numbers, that a flush makes rows
  readable without stopping, and that the memory ceiling can never starve the
  file.
- **AR video format choice** — that the tallest frame wins over any 16:9 crop of
  it, and that equal aspect ratios fall back to ARKit's ordering rather than to
  resolution.
- **The fixture launch lifecycle** — that Stop makes the graph genuinely silent
  rather than only relabelling it, that Start brings it back, and that Calibrate
  withholds odometry and drops the calibrated flag until it completes, which is
  the sequence the node status is built to read.
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

- Subscribing to a `PointCloud2` topic. The cloud is deprojected on the phone
  from the depth image instead (see below), which needs no extra bandwidth and
  no extra node on the robot.
- An authoritative node list. Adding `rosapi_node` to `drone_launch.py` would
  let `OdomNodeStatus` confirm what it currently infers.
- **Reading the camera and tag extrinsics from `/tf_static`** instead of asking
  the operator to measure them into Settings ▸ Offsets. The offsets already live
  in the robot's URDF; the app simply does not subscribe to the topic that
  carries them.
- Fusing tag fixes rather than replacing with them. Each accepted sighting
  overwrites the alignment outright, so the marker steps when a fix arrives
  instead of easing onto it. A filter over recent fixes would smooth that and
  reject outliers on more evidence than a single frame.
- Tags fixed in the room rather than on the robot. The same detector would give
  the phone an absolute pose against a surveyed tag, which is the other half of
  the problem when the robot itself is out of sight.
- A camera view wider than ARKit's wide lens. It would need `AVCaptureSession`
  on the ultra-wide camera plus pose estimation of our own, since ARKit will not
  give world tracking and that lens at the same time — a large piece of work to
  reimplement something Apple already does well.
