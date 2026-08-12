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
            angularVelocity: angular,
            positionVariance: positionVariance(from: value["pose"]?["covariance"])
        )
    }

    /// Pulls the x/y/z variances off the diagonal of a 6x6 row-major pose
    /// covariance — elements 0, 7 and 14 — and returns the largest.
    ///
    /// An all-zero covariance means "not filled in" rather than "perfectly
    /// known", so it reads as absent. That is the ROS convention for every
    /// field except `orientation_covariance[0] = -1`, which is a different
    /// message's way of saying the same thing.
    static func positionVariance(from value: ROSValue?) -> Double? {
        guard let entries = value?.arrayValue, entries.count >= 15 else { return nil }
        let diagonal = [0, 7, 14].compactMap { entries[$0].doubleValue }
        guard diagonal.count == 3, let maximum = diagonal.max(), maximum > 0 else { return nil }
        return maximum
    }

    // MARK: - sensor_msgs/msg/Imu

    public static func imu(from value: ROSValue?) -> ImuMessage? {
        guard let value else { return nil }
        let stamped = header(from: value["header"])
        let hasAcceleration = isFieldAvailable(value["linear_acceleration_covariance"])
        return ImuMessage(
            stamp: stamped.stamp,
            frameId: stamped.frameId,
            orientation: quaternion(from: value["orientation"]),
            angularVelocity: vector3(from: value["angular_velocity"]) ?? .zero,
            linearAcceleration: hasAcceleration ? vector3(from: value["linear_acceleration"]) : nil
        )
    }

    /// `sensor_msgs/Imu` marks a field unavailable by setting the first element
    /// of its covariance to `-1`. An absent covariance is treated as available,
    /// since plenty of publishers leave it zeroed.
    static func isFieldAvailable(_ covariance: ROSValue?) -> Bool {
        guard let first = covariance?.arrayValue?.first?.doubleValue else { return true }
        return first != -1
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

    /// Largest of the three translational variances on the diagonal of
    /// `pose.covariance`, in m². `nil` when the publisher sends no covariance.
    ///
    /// This exists because `odom_node` publishes orientation only: it holds
    /// position at zero and marks it with a variance of 1e6 to say so. Reading
    /// the number the estimator publishes is better than hard-coding "this
    /// robot has no position", because the day a flow sensor lands the app
    /// starts believing the position without a code change.
    public var positionVariance: Double?

    public init(
        stamp: Double,
        frameId: String,
        childFrameId: String,
        pose: Pose,
        linearVelocity: Vector3,
        angularVelocity: Vector3,
        positionVariance: Double? = nil
    ) {
        self.stamp = stamp
        self.frameId = frameId
        self.childFrameId = childFrameId
        self.pose = pose
        self.linearVelocity = linearVelocity
        self.angularVelocity = angularVelocity
        self.positionVariance = positionVariance
    }

    /// A variance above this counts as "not measured". `odom_node` publishes
    /// 1e6 m², a kilometre of standard deviation; a real estimator's position
    /// variance is orders of magnitude below this even while drifting badly, so
    /// the threshold does not need to be tuned.
    public static let unobservedPositionVariance: Double = 1.0e3

    /// Whether the publisher claims to know where the robot is.
    ///
    /// Absent covariance is treated as observed. Plenty of publishers leave it
    /// zeroed, and refusing to believe them would be a worse default than
    /// trusting a value nobody filled in.
    public var isPositionObserved: Bool {
        guard let positionVariance else { return true }
        return positionVariance < Self.unobservedPositionVariance
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
    /// `nil` when the publisher marks linear acceleration as unavailable, which
    /// `odom_node` does — it is a gyro-only estimator and never touches the
    /// accelerometer.
    public var linearAcceleration: Vector3?

    public init(
        stamp: Double,
        frameId: String,
        orientation: Quaternion?,
        angularVelocity: Vector3,
        linearAcceleration: Vector3?
    ) {
        self.stamp = stamp
        self.frameId = frameId
        self.orientation = orientation
        self.angularVelocity = angularVelocity
        self.linearAcceleration = linearAcceleration
    }

    /// Magnitude of the acceleration vector, in m/s². Close to 9.81 when the
    /// robot is still, which is a quick sanity check on the calibration —
    /// `nil` when the publisher does not send acceleration at all, which is not
    /// the same thing as sending zero.
    public var accelerationMagnitude: Double? { linearAcceleration?.length }
}
