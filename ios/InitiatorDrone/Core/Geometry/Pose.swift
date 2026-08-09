import Foundation

/// A rigid-body transform: a rotation followed by a translation.
///
/// Read `Pose` as "the pose of some child frame expressed in a parent frame",
/// or equivalently as the transform that takes points from the child frame into
/// the parent frame.
public struct Pose: Equatable, Codable, Sendable {
    public var position: Vector3
    public var orientation: Quaternion

    public init(position: Vector3 = .zero, orientation: Quaternion = .identity) {
        self.position = position
        self.orientation = orientation
    }

    public static let identity = Pose()

    public var isFinite: Bool { position.isFinite && orientation.isFinite }

    /// Transforms a point from this pose's child frame into its parent frame.
    public func apply(to point: Vector3) -> Vector3 {
        orientation.rotate(point) + position
    }

    /// Rotates a direction (no translation) from child frame into parent frame.
    public func rotate(_ direction: Vector3) -> Vector3 {
        orientation.rotate(direction)
    }

    public var inverse: Pose {
        let inverseRotation = orientation.conjugate
        return Pose(
            position: inverseRotation.rotate(-position),
            orientation: inverseRotation
        )
    }

    /// Composition. `parentFromChild * childFromGrandchild` yields
    /// `parentFromGrandchild`.
    public static func * (lhs: Pose, rhs: Pose) -> Pose {
        Pose(
            position: lhs.apply(to: rhs.position),
            orientation: (lhs.orientation * rhs.orientation).normalized
        )
    }

    /// Interpolates between two poses: linear on position, shortest-path
    /// spherical on orientation. `t` is clamped to `0...1`.
    public static func interpolate(_ a: Pose, _ b: Pose, _ t: Double) -> Pose {
        let clamped = min(max(t, 0.0), 1.0)
        return Pose(
            position: Vector3.lerp(a.position, b.position, clamped),
            orientation: Quaternion.slerp(a.orientation, b.orientation, clamped)
        )
    }

    public func isApproximatelyEqual(to other: Pose, tolerance: Double = 1e-9) -> Bool {
        position.isApproximatelyEqual(to: other.position, tolerance: tolerance)
            && orientation.describesSameRotation(as: other.orientation, tolerance: tolerance)
    }
}

/// A pose stamped with the ROS message time it was measured at, in seconds
/// since the ROS epoch.
public struct StampedPose: Equatable, Sendable {
    public var stamp: Double
    public var pose: Pose
    /// Linear velocity from the odometry twist, in the child frame, m/s.
    public var linearVelocity: Vector3
    /// Angular velocity from the odometry twist, in the child frame, rad/s.
    public var angularVelocity: Vector3

    public init(
        stamp: Double,
        pose: Pose,
        linearVelocity: Vector3 = .zero,
        angularVelocity: Vector3 = .zero
    ) {
        self.stamp = stamp
        self.pose = pose
        self.linearVelocity = linearVelocity
        self.angularVelocity = angularVelocity
    }
}
