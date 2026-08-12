import Foundation

/// Where the depth camera sits on the robot, relative to `base_link`.
///
/// The app used to assume the camera was bolted to `base_link` itself, facing
/// straight forward. That assumption is wrong on a real airframe — the depth
/// camera is usually mounted ahead of and below the body origin, tilted down —
/// and every point in the cloud inherits the error, so a 15 degree mount tilt
/// puts a wall 2 m away roughly half a metre off the floor.
///
/// The true extrinsic lives in the robot's URDF and is published on `/tf_static`,
/// which this app does not subscribe to. Until it does, this is the operator's
/// way to enter the measured mounting by hand.
///
/// ## Conventions
///
/// Translation is in `base_link` axes (REP-103): `+X` forward, `+Y` left,
/// `+Z` up, metres.
///
/// Rotation is the ROS roll/pitch convention about those same axes, in degrees
/// because that is what a person measures with a protractor:
///
/// - **pitch** is about `+Y`, so **positive pitch tilts the camera down**.
/// - **roll** is about `+X`, so **positive roll drops the camera's right side**.
///
/// Applied as `R = R_y(pitch) · R_x(roll)`, the standard RPY composition with
/// yaw fixed at zero. Yaw is deliberately absent: a camera rotated about the
/// vertical is indistinguishable from a robot that is pointing somewhere else,
/// and folding that into the mount would let a mis-measured yaw hide a real
/// heading error. A genuinely yawed camera needs the URDF, not this screen.
public struct CameraExtrinsics: Equatable, Codable, Sendable {
    /// Forward of `base_link`, metres.
    public var x: Double
    /// Left of `base_link`, metres.
    public var y: Double
    /// Above `base_link`, metres.
    public var z: Double
    /// Rotation about `base_link` `+Y`; positive tilts the camera down.
    public var pitchDegrees: Double
    /// Rotation about `base_link` `+X`; positive drops the camera's right side.
    public var rollDegrees: Double

    public init(
        x: Double = 0,
        y: Double = 0,
        z: Double = 0,
        pitchDegrees: Double = 0,
        rollDegrees: Double = 0
    ) {
        self.x = x
        self.y = y
        self.z = z
        self.pitchDegrees = pitchDegrees
        self.rollDegrees = rollDegrees
    }

    /// The camera at `base_link`, facing forward — what the app assumed before
    /// this type existed, and still the default until someone measures.
    public static let identity = CameraExtrinsics()

    public var isIdentity: Bool { self == .identity }

    public var translationInBody: Vector3 { Vector3(x, y, z) }

    public var pitchRadians: Double { pitchDegrees * .pi / 180 }
    public var rollRadians: Double { rollDegrees * .pi / 180 }

    /// The mount rotation in `base_link` axes.
    public var rotationInBody: Quaternion {
        let roll = Quaternion(axis: Vector3(1, 0, 0), angle: rollRadians)
        let pitch = Quaternion(axis: Vector3(0, 1, 0), angle: pitchRadians)
        return (pitch * roll).normalized
    }

    /// The camera body frame's pose relative to `base_link`.
    public var poseInBody: Pose {
        Pose(position: translationInBody, orientation: rotationInBody)
    }

    /// The same pose written in the ARKit axes the robot marker node uses, so a
    /// SceneKit child node can be positioned with it directly.
    ///
    /// `FrameConversion`'s conjugation already sends ROS `+X` (forward) to a
    /// node's `-Z` (forward), so a node given this pose points where the camera
    /// points with no extra correction.
    public var poseInARNode: Pose {
        FrameConversion.arPose(fromROS: poseInBody)
    }

    /// Re-expresses a point from the camera **optical** frame in `base_link`.
    ///
    /// Two rotations, in this order: the fixed optical-to-body axis convention
    /// (REP-103 says optical is `+X` right, `+Y` down, `+Z` forward), then the
    /// mount rotation. Then the mount offset.
    public func bodyPoint(fromOptical point: Vector3) -> Vector3 {
        let axisAligned = Vector3(point.z, -point.x, -point.y)
        return rotationInBody.rotate(axisAligned) + translationInBody
    }

    /// The whole optical-to-node-local mapping, precomputed.
    ///
    /// Building a cloud applies this to every pixel, so the quaternion products
    /// are hoisted out of the loop into three basis images and an origin. It is
    /// also the honest way to write the transform down: the columns *are* where
    /// the optical axes end up.
    public var opticalToARNode: OpticalToNodeTransform {
        OpticalToNodeTransform(
            xAxis: arDirection(fromOptical: Vector3(1, 0, 0)),
            yAxis: arDirection(fromOptical: Vector3(0, 1, 0)),
            zAxis: arDirection(fromOptical: Vector3(0, 0, 1)),
            origin: FrameConversion.arPosition(fromROS: translationInBody)
        )
    }

    /// Where an optical-frame *direction* points in ARKit node axes. The
    /// translation is deliberately not applied, which is what makes these
    /// usable as the columns of the transform above.
    private func arDirection(fromOptical direction: Vector3) -> Vector3 {
        let axisAligned = Vector3(direction.z, -direction.x, -direction.y)
        return FrameConversion.arPosition(fromROS: rotationInBody.rotate(axisAligned))
    }
}

/// A camera-optical point mapped straight into the robot marker node's local
/// ARKit axes, with the mount already folded in.
///
/// With an identity mount this collapses to flipping Y and Z — the shortcut
/// `DepthPointCloud` documents. `CameraExtrinsicsTests` asserts that, so the
/// documented simple case and the general case cannot drift apart.
public struct OpticalToNodeTransform: Equatable, Sendable {
    /// Where optical `+X` (right) points.
    public var xAxis: Vector3
    /// Where optical `+Y` (down) points.
    public var yAxis: Vector3
    /// Where optical `+Z` (forward) points.
    public var zAxis: Vector3
    /// Where the camera itself sits.
    public var origin: Vector3

    public init(xAxis: Vector3, yAxis: Vector3, zAxis: Vector3, origin: Vector3) {
        self.xAxis = xAxis
        self.yAxis = yAxis
        self.zAxis = zAxis
        self.origin = origin
    }

    public static let identity = CameraExtrinsics.identity.opticalToARNode

    public func callAsFunction(_ point: Vector3) -> Vector3 {
        apply(x: point.x, y: point.y, z: point.z)
    }

    /// Scalar entry point, so the deprojection loop never builds a `Vector3` it
    /// is about to take apart again.
    public func apply(x: Double, y: Double, z: Double) -> Vector3 {
        Vector3(
            xAxis.x * x + yAxis.x * y + zAxis.x * z + origin.x,
            xAxis.y * x + yAxis.y * y + zAxis.y * z + origin.y,
            xAxis.z * x + yAxis.z * y + zAxis.z * z + origin.z
        )
    }
}
