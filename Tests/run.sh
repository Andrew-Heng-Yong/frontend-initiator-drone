#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
check_dir="$(mktemp -d "${TMPDIR:-/tmp}/droneview-check.XXXXXX")"
trap 'rm -rf "$check_dir"' EXIT
swiftc -swift-version 5 -parse-as-library \
  DroneView/DroneView/TrackingAPI.swift DroneView/DroneView/SceneCamera.swift \
  Tests/CoreChecks.swift -o "$check_dir/check"
"$check_dir/check" "$@"
