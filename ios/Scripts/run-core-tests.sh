#!/usr/bin/env bash
#
# Runs the test suite without Xcode.
#
# The tests import XCTest only when it is available and fall back to
# Scripts/TestSupport/XCTestShim.swift otherwise, so the same assertions run
# here and in Xcode. Everything under InitiatorDrone/Core and
# InitiatorDrone/Services is platform-independent and compiles for macOS; the
# ARKit, SwiftUI and SceneKit layers are iOS-only and are covered by building
# the app in Xcode instead.
#
# Usage:
#   Scripts/run-core-tests.sh            # normal run
#   INITIATOR_SOAK_FRAMES=20000 \
#     Scripts/run-core-tests.sh          # full-length memory soak
#
set -euo pipefail

cd "$(dirname "$0")/.."
ROOT="$(pwd)"
BUILD_DIR="${TMPDIR:-/tmp}/initiator-drone-tests"
BINARY="$BUILD_DIR/InitiatorDroneTests"

mkdir -p "$BUILD_DIR"

SOURCES=()
while IFS= read -r file; do SOURCES+=("$file"); done < <(
  find "$ROOT/InitiatorDrone/Core" "$ROOT/InitiatorDrone/Services" -name '*.swift' | sort
)
while IFS= read -r file; do SOURCES+=("$file"); done < <(
  find "$ROOT/Tests" -name '*.swift' | sort
)
SOURCES+=("$ROOT/Scripts/TestSupport/XCTestShim.swift")
SOURCES+=("$ROOT/Scripts/TestSupport/main.swift")

echo "Compiling ${#SOURCES[@]} files…"
swiftc -O \
  -swift-version 5 \
  -target "$(uname -m)-apple-macosx11.0" \
  -o "$BINARY" \
  "${SOURCES[@]}"

echo "Running…"
INITIATOR_FIXTURES_DIR="$ROOT/InitiatorDrone/Resources/Fixtures" "$BINARY"
