import Foundation

/// The single conversion layer between ROS coordinates and ARKit coordinates.
///
/// Nothing else in the app is allowed to reinterpret axes. Every `/vio/odometry`
/// pose passes through `FrameConversion` exactly once, and the accompanying
/// tests pin the mapping down so a future refactor cannot silently flip a sign.
///
/// ## Conventions
///
/// **ROS** (`odom` / `base_link`, REP-103): right-handed, metres, radians.
/// `+X` forward, `+Y` left, `+Z` up. Quaternions are Hamilton, `(x, y, z, w)`
/// on the wire.
///
/// **ARKit world** (`ARSession` with `.gravity` or `.gravityAndHeading`):
/// right-handed, metres. `+X` right, `+Y` up, `+Z` toward the viewer, so `-Z`
/// is the forward direction. A node's local forward axis is `-Z`.
///
/// Both frames are right-handed and metric, so the conversion is a pure
/// rotation with determinant `+1` — no mirroring and no unit scaling. Anything
/// that looks like a handedness flip in this codebase is a bug.
///
/// ## Axis mapping
///
/// ```text
/// ROS +X (forward) -> ARKit -Z
/// ROS +Y (left)    -> ARKit -X
/// ROS +Z (up)      -> ARKit +Y
/// ```
public enum FrameConversion {
    /// The fixed axis-convention rotation taking ROS axes to ARKit axes.
    ///
    /// This is the quaternion form of the column-major matrix whose columns are
    /// the images of the ROS basis vectors: `x -> (0, 0, -1)`, `y -> (-1, 0, 0)`,
    /// `z -> (0, 1, 0)`.
    public static let arFromROSAxes = Quaternion(w: 0.5, x: -0.5, y: 0.5, z: 0.5)

    /// Inverse of `arFromROSAxes`.
    public static let rosFromARAxes = Quaternion(w: 0.5, x: 0.5, y: -0.5, z: -0.5)

    // MARK: - Positions

    /// Re-expresses a ROS position in ARKit axes. Units are metres in both.
    public static func arPosition(fromROS position: Vector3) -> Vector3 {
        Vector3(-position.y, position.z, -position.x)
    }

    /// Re-expresses an ARKit position in ROS axes.
    public static func rosPosition(fromAR position: Vector3) -> Vector3 {
        Vector3(-position.z, -position.x, position.y)
    }

    // MARK: - Orientations

    /// Re-expresses a ROS orientation in ARKit axes.
    ///
    /// This is a similarity transform (conjugation), not a plain multiplication:
    /// the rotation itself is unchanged, only the basis it is written in
    /// changes. A useful consequence is that the result lines up with the
    /// SceneKit/RealityKit `-Z` forward convention automatically, because the
    /// axis map already sends ROS forward (`+X`) to ARKit `-Z`.
    public static func arOrientation(fromROS orientation: Quaternion) -> Quaternion {
        (arFromROSAxes * orientation.normalized * rosFromARAxes).normalized
    }

    /// Re-expresses an ARKit orientation in ROS axes.
    public static func rosOrientation(fromAR orientation: Quaternion) -> Quaternion {
        (rosFromARAxes * orientation.normalized * arFromROSAxes).normalized
    }

    // MARK: - Poses

    /// Re-expresses a full ROS pose in ARKit axes, with no alignment applied.
    /// This is only the axis convention change; it still describes a pose in
    /// the robot's `odom` frame, just written in ARKit axes.
    public static func arPose(fromROS pose: Pose) -> Pose {
        Pose(
            position: arPosition(fromROS: pose.position),
            orientation: arOrientation(fromROS: pose.orientation)
        )
    }

    public static func rosPose(fromAR pose: Pose) -> Pose {
        Pose(
            position: rosPosition(fromAR: pose.position),
            orientation: rosOrientation(fromAR: pose.orientation)
        )
    }
}

/// Where the robot's `odom` origin sits inside the ARKit world, and how it is
/// rotated about the vertical axis.
///
/// Alignment is intentionally restricted to a translation plus a yaw. ARKit's
/// world frame is already gravity-aligned and so is the robot's `odom` frame,
/// so roll and pitch between them should be zero. Allowing only yaw keeps a
/// sloppy manual placement from tilting the whole scene.
public struct RobotAlignment: Equatable, Codable, Sendable {
    /// Position of the robot's `odom` origin in ARKit world coordinates.
    public var originInAR: Vector3
    /// Heading of the robot's `odom` `+X` axis, in radians, measured about the
    /// ARKit `+Y` (up) axis.
    public var yaw: Double
    /// When this alignment was captured, for staleness display.
    public var capturedAt: Date

    public init(originInAR: Vector3 = .zero, yaw: Double = 0, capturedAt: Date = Date()) {
        self.originInAR = originInAR
        self.yaw = yaw
        self.capturedAt = capturedAt
    }

    public static let identity = RobotAlignment(
        originInAR: .zero,
        yaw: 0,
        capturedAt: Date(timeIntervalSince1970: 0)
    )

    /// The `ar_world <- odom` transform, ready to compose with converted poses.
    public var arFromOdom: Pose {
        Pose(position: originInAR, orientation: Quaternion.aroundY(yaw))
    }

    /// Full pipeline: a raw ROS `odom -> base_link` pose to a pose in the
    /// ARKit world, ready to hand to a scene node.
    ///
    /// The two steps are kept separate on purpose. `FrameConversion.arPose`
    /// only rewrites axes; `arFromOdom` is the user-placed alignment. Mixing
    /// them into one matrix makes alignment bugs impossible to isolate.
    public func arPose(fromROSOdometry pose: Pose) -> Pose {
        arFromOdom * FrameConversion.arPose(fromROS: pose)
    }

    /// Inverse of `arPose(fromROSOdometry:)`, used to turn a point the user
    /// taps in AR back into robot `odom` coordinates.
    public func rosOdometryPose(fromAR pose: Pose) -> Pose {
        FrameConversion.rosPose(fromAR: arFromOdom.inverse * pose)
    }

    /// Builds an alignment that places the robot's `odom` origin at a known
    /// ARKit pose — for example the phone's own pose when the operator is
    /// standing at the robot and taps "Align here".
    ///
    /// Only the heading of the supplied pose is used; roll and pitch are
    /// dropped so a hand-held phone does not tilt the robot frame.
    public static func placingOrigin(atARPose pose: Pose, capturedAt: Date = Date()) -> RobotAlignment {
        RobotAlignment(
            originInAR: pose.position,
            yaw: pose.orientation.yawAroundY,
            capturedAt: capturedAt
        )
    }
}
