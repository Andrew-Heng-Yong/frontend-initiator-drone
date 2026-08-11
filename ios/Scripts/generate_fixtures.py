#!/usr/bin/env python3
"""Generates the mock rosbridge JSON fixtures used by the tests and the offline mode.

The payloads are written from explicit value tables rather than captured from a
robot so the unit tests can assert exact decoded numbers. Run this after editing
the tables:

    python3 Scripts/generate_fixtures.py
"""

import base64
import json
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "InitiatorDrone", "Resources", "Fixtures")


def write(name, payload):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, name)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")
    print("wrote", os.path.relpath(path, os.path.join(HERE, "..")))


def header(sec, nanosec, frame_id):
    return {"stamp": {"sec": sec, "nanosec": nanosec}, "frame_id": frame_id}


def publish(topic, msg):
    return {"op": "publish", "topic": topic, "msg": msg}


# ---------------------------------------------------------------- depth 16UC1
# 4x3, deliberately padded: a row is 8 bytes of pixels but step is 10, so a
# decoder that assumes tight packing will read the wrong pixels from row 1 on.
DEPTH_MM = [
    [1000, 2000, 3000, 0],
    [1500, 2500, 3500, 4000],
    [500, 65535, 250, 750],
]
DEPTH_STEP = 10


def depth_bytes(big_endian):
    fmt = ">H" if big_endian else "<H"
    data = bytearray()
    for row in DEPTH_MM:
        for value in row:
            data += struct.pack(fmt, value)
        data += b"\xAB" * (DEPTH_STEP - len(row) * 2)  # padding, must be ignored
    return bytes(data)


for big_endian, name in ((False, "depth_16uc1.json"), (True, "depth_16uc1_bigendian.json")):
    write(
        name,
        publish(
            "/camera/depth/cropped/image_raw",
            {
                "header": header(1717430000, 250000000, "camera_depth_optical_frame"),
                "height": len(DEPTH_MM),
                "width": len(DEPTH_MM[0]),
                "encoding": "16UC1",
                "is_bigendian": 1 if big_endian else 0,
                "step": DEPTH_STEP,
                "data": base64.b64encode(depth_bytes(big_endian)).decode("ascii"),
            },
        ),
    )

# ---------------------------------------------------------------- depth 32FC1
# 0 and negative readings are "no return" and must decode to NaN.
DEPTH_M = [[0.5, 1.25], [0.0, -1.0]]
data = bytearray()
for row in DEPTH_M:
    for value in row:
        data += struct.pack("<f", value)
write(
    "depth_32fc1.json",
    publish(
        "/camera/depth/cropped/image_raw",
        {
            "header": header(1717430001, 0, "camera_depth_optical_frame"),
            "height": 2,
            "width": 2,
            "encoding": "32FC1",
            "is_bigendian": 0,
            "step": 8,
            "data": base64.b64encode(bytes(data)).decode("ascii"),
        },
    ),
)

# ------------------------------------------------------------------ colour rgb8
# 2x1 with a padded step, so the colour path exercises stride handling too.
RGB = bytes([255, 0, 0, 0, 255, 0]) + b"\x99\x99"
write(
    "color_rgb8.json",
    publish(
        "/camera/depth/cropped/image_raw",
        {
            "header": header(1717430003, 0, "camera_color_optical_frame"),
            "height": 1,
            "width": 2,
            "encoding": "rgb8",
            "is_bigendian": 0,
            "step": 8,
            "data": base64.b64encode(RGB).decode("ascii"),
        },
    ),
)

# BGR of the same two pixels: byte order swapped, decoded result identical.
BGR = bytes([0, 0, 255, 0, 255, 0])
write(
    "color_bgr8.json",
    publish(
        "/camera/depth/cropped/image_raw",
        {
            "header": header(1717430003, 0, "camera_color_optical_frame"),
            "height": 1,
            "width": 2,
            "encoding": "bgr8",
            "is_bigendian": 0,
            "step": 6,
            "data": base64.b64encode(BGR).decode("ascii"),
        },
    ),
)

# ------------------------------------------------------------------ camera_info
write(
    "camera_info.json",
    publish(
        "/camera/depth/cropped/camera_info",
        {
            "header": header(1717430000, 250000000, "camera_depth_optical_frame"),
            "height": 360,
            "width": 582,
            "distortion_model": "plumb_bob",
            "d": [0.0, 0.0, 0.0, 0.0, 0.0],
            "k": [520.0, 0.0, 291.0, 0.0, 520.0, 180.0, 0.0, 0.0, 1.0],
            "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "p": [520.0, 0.0, 291.0, 0.0, 0.0, 520.0, 180.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        },
    ),
)

# -------------------------------------------------------------------- odometry
# A 90 degree yaw about +Z: quaternion (0, 0, sin(45), cos(45)).
SQRT_HALF = 0.7071067811865476
write(
    "odometry.json",
    publish(
        "/vio/odometry",
        {
            "header": header(1717430010, 100000000, "odom"),
            "child_frame_id": "base_link",
            "pose": {
                "pose": {
                    "position": {"x": 1.5, "y": -0.25, "z": 0.75},
                    "orientation": {"x": 0.0, "y": 0.0, "z": SQRT_HALF, "w": SQRT_HALF},
                },
                "covariance": [0.0] * 36,
            },
            "twist": {
                "twist": {
                    "linear": {"x": 0.4, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.1},
                },
                "covariance": [0.0] * 36,
            },
        },
    ),
)

# ------------------------------------------------------------------- std_msgs
write("vio_calibrated_true.json", publish("/vio/calibrated", {"data": True}))
write("vio_visual_tracking_false.json", publish("/vio/visual_tracking", {"data": False}))

# ------------------------------------------------------------------------ imu
write(
    "imu.json",
    publish(
        "/imu/data_calibrated",
        {
            "header": header(1717430010, 120000000, "imu_link"),
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "orientation_covariance": [0.0] * 9,
            "angular_velocity": {"x": 0.001, "y": -0.002, "z": 0.15},
            "angular_velocity_covariance": [0.0] * 9,
            "linear_acceleration": {"x": 0.12, "y": -0.05, "z": 9.79},
            "linear_acceleration_covariance": [0.0] * 9,
        },
    ),
)

# --------------------------------------------------------------------- status
write(
    "status_error.json",
    {
        "op": "status",
        "level": "error",
        "msg": "Topic /vio/odometry does not exist",
        "id": "initiator-drone-/vio/odometry",
    },
)

# ----------------------------------------------------------- dashboard /api/state
write(
    "dashboard_state.json",
    {
        "running": True,
        "logs": ["[10:02:11] Starting launch", "[10:02:12] rosbridge up on 9090"],
        "cpu": [
            {"core": "cpu0", "load": 41},
            {"core": "cpu1", "load": 63},
            {"core": "cpu2", "load": 12},
            {"core": "cpu3", "load": 7},
        ],
        "cpuTemp": 58,
    },
)
