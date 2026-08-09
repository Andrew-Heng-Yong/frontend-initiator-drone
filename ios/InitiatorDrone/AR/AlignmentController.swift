#if canImport(ARKit)
import ARKit
import Combine
import SwiftUI

/// Drives the "Align robot" workflow and owns the resulting transform.
///
/// Alignment is the one piece of information the app cannot get from either the
/// robot or the phone: ARKit's world origin is wherever the session happened to
/// start, and the robot's `odom` origin is wherever VIO happened to initialise.
/// Nothing connects them until the operator says so.
///
/// Two ways to say so are offered, matching how the tool is actually used:
///
/// - **"Robot is here"** — the operator stands at the robot, points the phone
///   the way the robot faces, and taps once. This is the intended flow for
///   launching the app at the robot and then walking away.
/// - **"Place on surface"** — the operator aims a crosshair at the floor where
///   the robot started, taps to drop the origin, then dials in the heading.
///
/// AprilTag or another automatic method would remove the human from this loop
/// and is the obvious next step; the transform it produces would slot into
/// exactly the same `RobotAlignment`.
@MainActor
public final class AlignmentController: ObservableObject {

    public enum Phase: Equatable {
        case idle
        /// Aiming the crosshair, waiting for a tap to fix the origin.
        case pickingPosition
        /// Origin fixed, dialling in the heading.
        case adjustingHeading(position: Vector3)
    }

    @Published public private(set) var alignment: RobotAlignment?
    @Published public private(set) var phase: Phase = .idle
    /// Heading being dialled in, in radians about the AR up axis.
    @Published public var pendingYaw: Double = 0
    /// Live raycast result while `phase == .pickingPosition`, for the preview.
    @Published public private(set) var previewPosition: Vector3?
    @Published public private(set) var statusMessage: String?

    /// Height of the robot's `odom` origin above whatever surface was picked,
    /// in metres. A quadrotor sitting on the floor has its `base_link` a little
    /// above it.
    @Published public var originHeightOffset: Double = 0.0

    public var isAligned: Bool { alignment != nil }
    public var isPlacing: Bool { phase != .idle }

    public init(alignment: RobotAlignment? = nil) {
        self.alignment = alignment
    }

    // MARK: - One-tap alignment

    /// Sets the robot's `odom` origin to the phone's current pose.
    ///
    /// Only the heading is taken from the phone's orientation; roll and pitch
    /// are dropped, so a phone held at any angle still produces a level robot
    /// frame. The height offset is applied along AR up.
    public func alignAtPhone(_ phonePose: Pose) {
        var position = phonePose.position
        position.y += originHeightOffset

        let placed = RobotAlignment(
            originInAR: position,
            yaw: phonePose.orientation.yawAroundY,
            capturedAt: Date()
        )
        commit(placed, message: "Aligned to the phone's position and heading.")
    }

    // MARK: - Surface placement

    public func beginSurfacePlacement() {
        phase = .pickingPosition
        previewPosition = nil
        statusMessage = "Aim at the floor where the robot started, then tap."
    }

    /// Called continuously with the crosshair raycast result.
    public func updatePreview(_ position: Vector3?) {
        guard case .pickingPosition = phase else { return }
        previewPosition = position
    }

    /// Fixes the origin at a raycast hit and moves on to the heading step.
    public func placeOrigin(at position: Vector3, initialYaw: Double) {
        guard case .pickingPosition = phase else { return }
        var placed = position
        placed.y += originHeightOffset
        pendingYaw = initialYaw
        phase = .adjustingHeading(position: placed)
        statusMessage = "Turn the dial until the arrow points the way the robot faces."
    }

    /// Snaps the pending heading to the phone's current heading, which is handy
    /// when the operator can stand behind the robot and sight along it.
    public func usePhoneHeading(_ phonePose: Pose) {
        guard case .adjustingHeading = phase else { return }
        pendingYaw = phonePose.orientation.yawAroundY
    }

    public func confirmPlacement() {
        guard case .adjustingHeading(let position) = phase else { return }
        let placed = RobotAlignment(originInAR: position, yaw: pendingYaw, capturedAt: Date())
        commit(placed, message: "Robot origin placed.")
    }

    public func cancel() {
        phase = .idle
        previewPosition = nil
        statusMessage = nil
    }

    public func clearAlignment() {
        alignment = nil
        phase = .idle
        previewPosition = nil
        statusMessage = "Alignment cleared. The robot marker is hidden until you align again."
    }

    /// Restores a saved alignment. Only meaningful when the AR session has not
    /// been reset since it was captured, which is why the UI asks first.
    public func restore(_ alignment: RobotAlignment) {
        commit(alignment, message: "Restored the saved alignment. Check that it still lines up.")
    }

    /// Nudges an existing alignment, for fixing up a placement that is close but
    /// visibly off.
    public func adjust(dx: Double = 0, dy: Double = 0, dz: Double = 0, dyaw: Double = 0) {
        guard var current = alignment else { return }
        current.originInAR = Vector3(
            current.originInAR.x + dx,
            current.originInAR.y + dy,
            current.originInAR.z + dz
        )
        current.yaw += dyaw
        current.capturedAt = Date()
        alignment = current
    }

    private func commit(_ value: RobotAlignment, message: String) {
        alignment = value
        phase = .idle
        previewPosition = nil
        statusMessage = message
    }

    /// A short human-readable description of the current alignment.
    public var summary: String {
        guard let alignment else { return "Not aligned" }
        let degrees = alignment.yaw * 180.0 / .pi
        return String(
            format: "origin (%.2f, %.2f, %.2f) m, heading %.0f°",
            alignment.originInAR.x,
            alignment.originInAR.y,
            alignment.originInAR.z,
            degrees
        )
    }
}
#endif
