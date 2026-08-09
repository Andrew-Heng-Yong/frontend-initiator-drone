import Foundation

/// Parsers from a decoded rosbridge `msg` value into typed Swift structs.
///
/// Every parser is total: it returns `nil` rather than throwing or trapping on
/// a malformed message, because a diagnostics tool must survive a robot that is
/// publishing something unexpected.
public enum ROSMessageParser {

    /// Reads a `builtin_interfaces/Time`, tolerating both the ROS 2 field names
    /// (`sec`/`nanosec`) and the ROS 1 names (`secs`/`nsecs`) that older
    /// rosbridge builds still emit.
    public static func time(from value: ROSValue?) -> Double? {
        guard let value else { return nil }
        let seconds = value["sec"]?.doubleValue ?? value["secs"]?.doubleValue
        let nanoseconds = value["nanosec"]?.doubleValue ?? value["nsecs"]?.doubleValue
        guard let seconds else { return nil }
        return seconds + (nanoseconds ?? 0) / 1_000_000_000.0
    }

    public static func header(from value: ROSValue?) -> (stamp: Double, frameId: String) {
        let stamp = time(from: value?["stamp"]) ?? 0
        let frameId = value?["frame_id"]?.stringValue ?? ""
        return (stamp, frameId)
    }

    public static func vector3(from value: ROSValue?) -> Vector3? {
        guard let value,
              let x = value["x"]?.doubleValue,
              let y = value["y"]?.doubleValue,
              let z = value["z"]?.doubleValue else { return nil }
        return Vector3(x, y, z)
    }

    /// Reads a `geometry_msgs/Quaternion`. ROS serialises the components by
    /// name, so the `(x, y, z, w)` wire order never has to be assumed.
    public static func quaternion(from value: ROSValue?) -> Quaternion? {
        guard let value,
              let x = value["x"]?.doubleValue,
              let y = value["y"]?.doubleValue,
              let z = value["z"]?.doubleValue,
              let w = value["w"]?.doubleValue else { return nil }
        let quaternion = Quaternion(w: w, x: x, y: y, z: z)
        // An all-zero quaternion is what an uninitialised message looks like;
        // normalising it would silently produce identity, so reject it.
        guard quaternion.length > 1e-6 else { return nil }
        return quaternion.normalized
    }

    public static func pose(from value: ROSValue?) -> Pose? {
        guard let value,
              let position = vector3(from: value["position"]),
              let orientation = quaternion(from: value["orientation"]) else { return nil }
        return Pose(position: position, orientation: orientation)
    }

    // MARK: - sensor_msgs/msg/Image

    public static func image(from value: ROSValue?) -> ROSImageMessage? {
        guard let value,
              let width = value["width"]?.intValue,
              let height = value["height"]?.intValue,
              let encoding = value["encoding"]?.stringValue,
              let data = value["data"]?.byteArrayValue else { return nil }

        let stamped = header(from: value["header"])
        let step = value["step"]?.intValue ?? 0
        let isBigEndian = (value["is_bigendian"]?.intValue ?? 0) != 0

        return ROSImageMessage(
            stamp: stamped.stamp,
            frameId: stamped.frameId,
            width: width,
            height: height,
            encoding: encoding,
            isBigEndian: isBigEndian,
            step: step,
            data: data
        )
    }

    // MARK: - sensor_msgs/msg/CameraInfo

    public static func cameraInfo(from value: ROSValue?) -> CameraInfoMessage? {
        guard let value,
              let width = value["width"]?.intValue,
              let height = value["height"]?.intValue else { return nil }

        let stamped = header(from: value["header"])
        let k = value["k"]?.doubleArrayValue ?? value["K"]?.doubleArrayValue ?? []
        let p = value["p"]?.doubleArrayValue ?? value["P"]?.doubleArrayValue ?? []
        let d = value["d"]?.doubleArrayValue ?? value["D"]?.doubleArrayValue ?? []

        return CameraInfoMessage(
            stamp: stamped.stamp,
            frameId: stamped.frameId,
            width: width,
            height: height,
            intrinsics: k.count == 9 ? k : nil,
            projection: p.count == 12 ? p : nil,
            distortion: d,
            distortionModel: value["distortion_model"]?.stringValue ?? ""
        )
    }

    // MARK: - nav_msgs/msg/Odometry

    public static func odometry(from value: ROSValue?) -> OdometryMessage? {
        guard let value else { return nil }
        let stamped = header(from: value["header"])
        guard let pose = pose(from: value["pose"]?["pose"]) else { return nil }

        let twist = value["twist"]?["twist"]
        let linear = vector3(from: twist?["linear"]) ?? .zero
        let angular = vector3(from: twist?["angular"]) ?? .zero

        return OdometryMessage(
            stamp: stamped.stamp,
            frameId: stamped.frameId,
            childFrameId: value["child_frame_id"]?.stringValue ?? "",
            pose: pose,
            linearVelocity: linear,
            angularVelocity: angular
        )
    }

    // MARK: - sensor_msgs/msg/Imu

    public static func imu(from value: ROSValue?) -> ImuMessage? {
        guard let value else { return nil }
        let stamped = header(from: value["header"])
        return ImuMessage(
            stamp: stamped.stamp,
            frameId: stamped.frameId,
            orientation: quaternion(from: value["orientation"]),
            angularVelocity: vector3(from: value["angular_velocity"]) ?? .zero,
            linearAcceleration: vector3(from: value["linear_acceleration"]) ?? .zero
        )
    }

    // MARK: - std_msgs/msg/Bool

    public static func boolean(from value: ROSValue?) -> Bool? {
        value?["data"]?.boolValue
    }
}

public struct CameraInfoMessage: Equatable, Sendable {
    public var stamp: Double
    public var frameId: String
    public var width: Int
    public var height: Int
    /// Row-major 3x3 intrinsics `K`, or `nil` when the publisher sent none.
    public var intrinsics: [Double]?
    /// Row-major 3x4 projection `P`.
    public var projection: [Double]?
    public var distortion: [Double]
    public var distortionModel: String

    public init(
        stamp: Double,
        frameId: String,
        width: Int,
        height: Int,
        intrinsics: [Double]?,
        projection: [Double]?,
        distortion: [Double],
        distortionModel: String
    ) {
        self.stamp = stamp
        self.frameId = frameId
        self.width = width
        self.height = height
        self.intrinsics = intrinsics
        self.projection = projection
        self.distortion = distortion
        self.distortionModel = distortionModel
    }

    public var focalLengthX: Double? { intrinsics.map { $0[0] } }
    public var focalLengthY: Double? { intrinsics.map { $0[4] } }
    public var principalPointX: Double? { intrinsics.map { $0[2] } }
    public var principalPointY: Double? { intrinsics.map { $0[5] } }

    /// Horizontal field of view in radians, derived from `fx` and the image
    /// width. Used to draw the robot's camera frustum in the AR scene.
    public var horizontalFieldOfView: Double? {
        guard let fx = focalLengthX, fx > 0, width > 0 else { return nil }
        return 2.0 * atan(Double(width) / (2.0 * fx))
    }

    public var verticalFieldOfView: Double? {
        guard let fy = focalLengthY, fy > 0, height > 0 else { return nil }
        return 2.0 * atan(Double(height) / (2.0 * fy))
    }
}

public struct OdometryMessage: Equatable, Sendable {
    public var stamp: Double
    /// The parent frame, normally `odom`.
    public var frameId: String
    /// The child frame, normally `base_link`.
    public var childFrameId: String
    public var pose: Pose
    public var linearVelocity: Vector3
    public var angularVelocity: Vector3

    public init(
        stamp: Double,
        frameId: String,
        childFrameId: String,
        pose: Pose,
        linearVelocity: Vector3,
        angularVelocity: Vector3
    ) {
        self.stamp = stamp
        self.frameId = frameId
        self.childFrameId = childFrameId
        self.pose = pose
        self.linearVelocity = linearVelocity
        self.angularVelocity = angularVelocity
    }

    public var stampedPose: StampedPose {
        StampedPose(
            stamp: stamp,
            pose: pose,
            linearVelocity: linearVelocity,
            angularVelocity: angularVelocity
        )
    }
}

public struct ImuMessage: Equatable, Sendable {
    public var stamp: Double
    public var frameId: String
    /// `nil` when the publisher marks orientation as unavailable.
    public var orientation: Quaternion?
    public var angularVelocity: Vector3
    public var linearAcceleration: Vector3

    public init(
        stamp: Double,
        frameId: String,
        orientation: Quaternion?,
        angularVelocity: Vector3,
        linearAcceleration: Vector3
    ) {
        self.stamp = stamp
        self.frameId = frameId
        self.orientation = orientation
        self.angularVelocity = angularVelocity
        self.linearAcceleration = linearAcceleration
    }

    /// Magnitude of the acceleration vector, in m/s². Close to 9.81 when the
    /// robot is still, which is a quick sanity check on the calibration.
    public var accelerationMagnitude: Double { linearAcceleration.length }
}
