import Foundation

/// Wire compression requested from rosbridge at subscribe time.
public enum RosbridgeCompression: String, Codable, CaseIterable, Sendable, Identifiable {
    /// Plain JSON. `uint8[]` fields arrive base64-encoded. Works with every
    /// rosbridge build.
    case none
    /// CBOR with RFC 8746 typed arrays. Image payloads arrive as raw bytes,
    /// which is roughly a third less traffic and avoids a base64 pass.
    /// Supported by `rosbridge_suite` 0.11 and newer.
    case cbor

    public var id: String { rawValue }

    public var displayName: String {
        switch self {
        case .none: return "JSON (compatible)"
        case .cbor: return "CBOR (faster)"
        }
    }
}

/// A command sent to rosbridge.
public enum RosbridgeCommand: Equatable, Sendable {
    case subscribe(SubscribeOptions)
    case unsubscribe(topic: String, id: String)
    case callService(service: String, id: String, args: ROSValue?)

    public struct SubscribeOptions: Equatable, Sendable {
        public var topic: String
        public var messageType: String
        public var id: String
        /// Minimum milliseconds between messages, enforced on the robot. This
        /// is the first and most important line of defence against a backlog:
        /// frames that are never sent cannot queue up.
        public var throttleMilliseconds: Int
        /// rosbridge-side queue depth. One means "only ever hold the newest".
        public var queueLength: Int
        public var compression: RosbridgeCompression
        /// Maximum bytes per message before rosbridge fragments it. Zero
        /// disables fragmentation, which is what we want: reassembly costs
        /// memory and a dropped fragment strands the rest.
        public var fragmentSize: Int?

        public init(
            topic: String,
            messageType: String,
            id: String,
            throttleMilliseconds: Int,
            queueLength: Int = 1,
            compression: RosbridgeCompression = .none,
            fragmentSize: Int? = nil
        ) {
            self.topic = topic
            self.messageType = messageType
            self.id = id
            self.throttleMilliseconds = throttleMilliseconds
            self.queueLength = queueLength
            self.compression = compression
            self.fragmentSize = fragmentSize
        }
    }

    /// The JSON object rosbridge expects.
    public var payload: [String: Any] {
        switch self {
        case .subscribe(let options):
            var body: [String: Any] = [
                "op": "subscribe",
                "topic": options.topic,
                "type": options.messageType,
                "id": options.id,
                "throttle_rate": max(0, options.throttleMilliseconds),
                "queue_length": max(0, options.queueLength),
                "compression": options.compression.rawValue,
            ]
            if let fragmentSize = options.fragmentSize {
                body["fragment_size"] = fragmentSize
            }
            return body

        case .unsubscribe(let topic, let id):
            return ["op": "unsubscribe", "topic": topic, "id": id]

        case .callService(let service, let id, let args):
            var body: [String: Any] = ["op": "call_service", "service": service, "id": id]
            if let args { body["args"] = args.jsonObject }
            return body
        }
    }

    public func encoded() throws -> Data {
        try JSONSerialization.data(withJSONObject: payload, options: [])
    }
}

/// A message received from rosbridge.
public enum RosbridgeIncoming: Equatable, Sendable {
    case publish(topic: String, message: ROSValue)
    case serviceResponse(service: String, id: String?, values: ROSValue?, result: Bool)
    case status(level: String, message: String, id: String?)
    /// Anything the app does not model, kept so the diagnostics screen can show
    /// that something arrived rather than silently discarding it.
    case other(op: String, raw: ROSValue)

    public static func parse(_ value: ROSValue) -> RosbridgeIncoming? {
        guard let op = value["op"]?.stringValue else { return nil }
        switch op {
        case "publish":
            guard let topic = value["topic"]?.stringValue, let message = value["msg"] else { return nil }
            return .publish(topic: topic, message: message)

        case "service_response":
            return .serviceResponse(
                service: value["service"]?.stringValue ?? "",
                id: value["id"]?.stringValue,
                values: value["values"],
                result: value["result"]?.boolValue ?? true
            )

        case "status":
            return .status(
                level: value["level"]?.stringValue ?? "info",
                message: value["msg"]?.stringValue ?? "",
                id: value["id"]?.stringValue
            )

        default:
            return .other(op: op, raw: value)
        }
    }
}

/// The topics this app subscribes to, with the settings each one needs.
///
/// Depth comes straight from the camera driver, not from the thermal cropper's
/// `cropped` republished topics. The app never wanted the thermal image itself,
/// but subscribing to the cropped depth still put the thermal sensor in the
/// path: no hot region meant no depth frame, and the crop moved the window
/// around under whatever the thermal camera happened to see.
public enum RobotTopic: String, CaseIterable, Identifiable, Sendable {
    case depthImage = "/camera/depth/image_raw"
    case depthCameraInfo = "/camera/depth/camera_info"
    case odometry = "/odom"
    case odomCalibrated = "/odom/calibrated"
    case imu = "/imu/data_calibrated"

    public var id: String { rawValue }
    public var topicName: String { rawValue }

    public var messageType: String {
        switch self {
        case .depthImage: return "sensor_msgs/msg/Image"
        case .depthCameraInfo: return "sensor_msgs/msg/CameraInfo"
        case .odometry: return "nav_msgs/msg/Odometry"
        case .odomCalibrated: return "std_msgs/msg/Bool"
        case .imu: return "sensor_msgs/msg/Imu"
        }
    }

    public var displayName: String {
        switch self {
        case .depthImage: return "Depth image"
        case .depthCameraInfo: return "Depth camera info"
        case .odometry: return "Odometry"
        case .odomCalibrated: return "Gyro calibrated"
        case .imu: return "IMU (calibrated)"
        }
    }

    /// Topics published by `odom_node`, and therefore evidence that it is
    /// alive. `OdomNodeStatus` reads these rather than a node list, because the
    /// robot's rosbridge is launched without `rosapi_node` and so cannot answer
    /// `/rosapi/nodes`.
    public var isPublishedByOdomNode: Bool {
        switch self {
        case .odometry, .odomCalibrated, .imu: return true
        case .depthImage, .depthCameraInfo: return false
        }
    }

    /// The depth topic is throttled hard by default. The phone cannot usefully
    /// display more than ~15 fps of a depth frame, and anything the robot does
    /// not send is bandwidth and memory the app never has to manage.
    public var defaultThrottleMilliseconds: Int {
        switch self {
        case .depthImage: return 66                     // ~15 Hz ceiling
        case .depthCameraInfo: return 1000              // static; once a second is plenty
        case .odometry: return 0                        // unthrottled: needed for interpolation
        case .imu: return 20                            // ~50 Hz ceiling
        case .odomCalibrated: return 0                  // latched status flag
        }
    }

    /// Odometry keeps a slightly deeper queue so a brief scheduling hiccup does
    /// not punch a hole in the interpolation buffer. Everything else keeps only
    /// the newest message.
    public var defaultQueueLength: Int {
        switch self {
        case .odometry: return 5
        default: return 1
        }
    }

    public var isImageTopic: Bool {
        self == .depthImage
    }

    /// A topic that has published nothing for this long is reported as silent.
    public var silenceThreshold: TimeInterval {
        switch self {
        case .depthImage: return 2.0
        case .odometry: return 1.0
        case .imu: return 1.0
        case .depthCameraInfo: return 10.0
        case .odomCalibrated: return 15.0
        }
    }
}
