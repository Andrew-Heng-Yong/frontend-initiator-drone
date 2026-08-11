import Foundation

/// A rosbridge stand-in that plays fixtures and synthesised sensor data.
///
/// This exists so the whole app — subscriptions, decoding, colour mapping,
/// odometry buffering, alignment, AR rendering — can be run and demonstrated
/// with no robot on the network, and so the reconnection path can be tested
/// deterministically instead of by walking out of Wi-Fi range.
///
/// It speaks the real wire protocol: it parses the same `subscribe` commands
/// the client sends, honours `throttle_rate`, and emits genuine rosbridge
/// `publish` envelopes as JSON text frames. Nothing downstream knows the
/// difference.
public final class SimulatedRosbridgeTransport: NSObject, RosbridgeTransport, @unchecked Sendable {
    public weak var delegate: RosbridgeTransportDelegate?

    /// Depth frame size. Large enough to be a real workload for the decoder and
    /// the colour mapper, small enough to be kind to the simulator.
    public var depthSize = (width: 320, height: 240)

    public var depthFrameRate: Double = 10
    public var odometryRate: Double = 30
    public var imuRate: Double = 50

    /// Reports `/vio/visual_tracking` as false, to exercise the "calibrated but
    /// not tracking" presentation.
    public var simulatesTrackingLoss = false
    /// Reports `/vio/calibrated` as false until `calibrate()` is called.
    public var isCalibrated = true

    /// Parks the robot instead of driving it around a figure-of-eight.
    /// Safe to flip while connected.
    public var isStatic = false

    /// Where the parked robot sits, in the `odom` frame. The default is the
    /// origin itself, so the marker should land exactly on the alignment ring.
    public var staticPosition = Vector3.zero

    /// Heading of the parked robot about `odom` +Z, in radians.
    public var staticHeading: Double = 0

    private let queue = DispatchQueue(label: "com.initiatordrone.rosbridge.simulator")
    private var timer: DispatchSourceTimer?
    private var connected = false
    private var subscriptions: [String: Subscription] = [:]
    private var startTime = Date()
    private var lastSendTimes: [String: Double] = [:]
    private var tickCount = 0

    private struct Subscription {
        var topic: String
        var throttleSeconds: Double
        var compression: String
    }

    public var isConnected: Bool { queue.sync { connected } }

    public override init() {
        super.init()
    }

    // MARK: - Transport

    public func connect(to url: URL) {
        queue.async { [weak self] in
            guard let self else { return }
            self.stopTimerLocked()
            self.subscriptions.removeAll()
            self.lastSendTimes.removeAll()
            self.startTime = Date()
            self.tickCount = 0
            self.connected = true
            self.delegate?.transportDidConnect(self)
            self.sendStatus(level: "info", message: "simulated rosbridge ready")
            self.startTimerLocked()
        }
    }

    public func disconnect() {
        queue.async { [weak self] in
            guard let self else { return }
            self.stopTimerLocked()
            self.connected = false
        }
    }

    public func send(_ data: Data) {
        queue.async { [weak self] in
            guard let self, self.connected else { return }
            guard let value = try? ROSValue.fromJSON(data), let op = value["op"]?.stringValue else { return }

            switch op {
            case "subscribe":
                guard let topic = value["topic"]?.stringValue else { return }
                let throttleMilliseconds = value["throttle_rate"]?.doubleValue ?? 0
                self.subscriptions[topic] = Subscription(
                    topic: topic,
                    throttleSeconds: throttleMilliseconds / 1000.0,
                    compression: value["compression"]?.stringValue ?? "none"
                )
                self.sendStatus(level: "info", message: "subscribed \(topic)")

            case "unsubscribe":
                if let topic = value["topic"]?.stringValue {
                    self.subscriptions.removeValue(forKey: topic)
                }

            case "call_service":
                let service = value["service"]?.stringValue ?? ""
                self.emit([
                    "op": "service_response",
                    "service": service,
                    "id": value["id"]?.stringValue ?? "",
                    "result": true,
                    "values": [:] as [String: Any],
                ])

            default:
                break
            }
        }
    }

    // MARK: - Test hooks

    /// Drops the link the way a Wi-Fi loss does: no close handshake, just an
    /// error. Used to exercise automatic reconnection.
    public func simulateConnectionLoss(error: Error = RosbridgeTransportError.heartbeatTimeout) {
        queue.async { [weak self] in
            guard let self, self.connected else { return }
            self.stopTimerLocked()
            self.connected = false
            self.delegate?.transport(self, didDisconnectWith: error)
        }
    }

    /// Marks VIO as calibrated, mirroring what `POST /api/vio/calibrate` would
    /// eventually cause the robot to publish.
    public func calibrate() {
        queue.async { [weak self] in
            self?.isCalibrated = true
        }
    }

    /// Parks or releases the robot while connected.
    ///
    /// The plain `isStatic` property is only safe to set before `connect`;
    /// afterwards it is read on the playback queue, so live changes go through
    /// here.
    public func setStatic(_ isStatic: Bool) {
        queue.async { [weak self] in
            self?.isStatic = isStatic
        }
    }

    // MARK: - Playback

    private func startTimerLocked() {
        let timer = DispatchSource.makeTimerSource(queue: queue)
        // One base tick; each stream decimates from it via its own throttle.
        timer.schedule(deadline: .now() + 0.02, repeating: 0.02)
        timer.setEventHandler { [weak self] in self?.tick() }
        timer.resume()
        self.timer = timer
    }

    private func stopTimerLocked() {
        timer?.cancel()
        timer = nil
    }

    private func tick() {
        guard connected else { return }
        tickCount += 1
        let elapsed = Date().timeIntervalSince(startTime)

        publishIfDue(.odometry, interval: 1.0 / odometryRate, elapsed: elapsed) {
            self.odometryMessage(at: elapsed)
        }
        publishIfDue(.imu, interval: 1.0 / imuRate, elapsed: elapsed) {
            self.imuMessage(at: elapsed)
        }
        publishIfDue(.depthImage, interval: 1.0 / depthFrameRate, elapsed: elapsed) {
            self.depthImageMessage(at: elapsed)
        }
        publishIfDue(.depthCameraInfo, interval: 1.0, elapsed: elapsed) {
            self.cameraInfoMessage(at: elapsed)
        }
        publishIfDue(.vioCalibrated, interval: 1.0, elapsed: elapsed) {
            ["data": self.isCalibrated]
        }
        publishIfDue(.visualTracking, interval: 1.0, elapsed: elapsed) {
            ["data": !self.simulatesTrackingLoss]
        }
    }

    private func publishIfDue(
        _ topic: RobotTopic,
        interval: Double,
        elapsed: Double,
        build: () -> [String: Any]
    ) {
        let name = topic.topicName
        guard let subscription = subscriptions[name] else { return }

        // The natural rate and the client's requested throttle both apply,
        // exactly as they would on a real robot.
        let effectiveInterval = max(interval, subscription.throttleSeconds)
        let last = lastSendTimes[name] ?? -.greatestFiniteMagnitude
        guard elapsed - last >= effectiveInterval - 1e-6 else { return }
        lastSendTimes[name] = elapsed

        emit(["op": "publish", "topic": name, "msg": build()])
    }

    private func emit(_ payload: [String: Any]) {
        guard let data = try? JSONSerialization.data(withJSONObject: payload, options: []) else { return }
        delegate?.transport(self, didReceive: data, isBinary: false)
    }

    private func sendStatus(level: String, message: String) {
        emit(["op": "status", "level": level, "msg": message, "id": NSNull()])
    }

    // MARK: - Synthesised messages

    private func header(at elapsed: Double, frameId: String) -> [String: Any] {
        let stamp = startTime.timeIntervalSince1970 + elapsed
        let seconds = Int(stamp.rounded(.down))
        let nanoseconds = Int((stamp - Double(seconds)) * 1_000_000_000)
        return [
            "stamp": ["sec": seconds, "nanosec": nanoseconds],
            "frame_id": frameId,
        ]
    }

    /// A slow figure-of-eight around the origin at walking pace, with the robot
    /// facing along its own path — enough motion to make interpolation,
    /// alignment and the AR marker visibly correct or visibly wrong.
    ///
    /// With `isStatic` the robot sits at the `odom` origin instead. Odometry
    /// still publishes at the same rate with the same timestamps, so the link,
    /// the buffer and the staleness logic are all still exercised — only the
    /// pose stops changing. That is the mode to use when you are checking
    /// whether the marker lands in the right place, because a moving target
    /// makes an alignment error impossible to distinguish from motion.
    func simulatedPose(at elapsed: Double) -> Pose {
        if isStatic {
            return Pose(
                position: staticPosition,
                orientation: Quaternion.aroundZ(staticHeading)
            )
        }

        let omega = 0.25
        let x = 1.5 * sin(omega * elapsed)
        let y = 0.9 * sin(2 * omega * elapsed)
        let z = 0.05 * sin(0.7 * elapsed)

        let dx = 1.5 * omega * cos(omega * elapsed)
        let dy = 0.9 * 2 * omega * cos(2 * omega * elapsed)
        let heading = atan2(dy, dx)

        return Pose(
            position: Vector3(x, y, z),
            orientation: Quaternion.aroundZ(heading)
        )
    }

    private func odometryMessage(at elapsed: Double) -> [String: Any] {
        let pose = simulatedPose(at: elapsed)
        let ahead = simulatedPose(at: elapsed + 0.05)
        let velocity = (ahead.position - pose.position) * (1.0 / 0.05)
        // Report the twist in the body frame, as nav_msgs/Odometry specifies.
        let bodyVelocity = pose.orientation.conjugate.rotate(velocity)

        return [
            "header": header(at: elapsed, frameId: "odom"),
            "child_frame_id": "base_link",
            "pose": [
                "pose": [
                    "position": ["x": pose.position.x, "y": pose.position.y, "z": pose.position.z],
                    "orientation": [
                        "x": pose.orientation.x,
                        "y": pose.orientation.y,
                        "z": pose.orientation.z,
                        "w": pose.orientation.w,
                    ],
                ],
                "covariance": [Double](repeating: 0, count: 36),
            ],
            "twist": [
                "twist": [
                    "linear": ["x": bodyVelocity.x, "y": bodyVelocity.y, "z": bodyVelocity.z],
                    "angular": ["x": 0.0, "y": 0.0, "z": 0.0],
                ],
                "covariance": [Double](repeating: 0, count: 36),
            ],
        ]
    }

    private func imuMessage(at elapsed: Double) -> [String: Any] {
        let pose = simulatedPose(at: elapsed)
        return [
            "header": header(at: elapsed, frameId: "imu_link"),
            "orientation": [
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            ],
            "orientation_covariance": [Double](repeating: 0, count: 9),
            "angular_velocity": [
                "x": 0.01 * sin(elapsed),
                "y": 0.01 * cos(elapsed),
                "z": 0.2 * cos(0.5 * elapsed),
            ],
            "angular_velocity_covariance": [Double](repeating: 0, count: 9),
            "linear_acceleration": [
                "x": 0.05 * sin(1.3 * elapsed),
                "y": 0.05 * cos(1.1 * elapsed),
                "z": 9.81 + 0.03 * sin(2.0 * elapsed),
            ],
            "linear_acceleration_covariance": [Double](repeating: 0, count: 9),
        ]
    }

    private func cameraInfoMessage(at elapsed: Double) -> [String: Any] {
        let width = depthSize.width
        let height = depthSize.height
        let fx = Double(width) * 0.9
        let fy = fx
        let cx = Double(width) / 2.0
        let cy = Double(height) / 2.0
        return [
            "header": header(at: elapsed, frameId: "camera_depth_optical_frame"),
            "height": height,
            "width": width,
            "distortion_model": "plumb_bob",
            "d": [0.0, 0.0, 0.0, 0.0, 0.0],
            "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        ]
    }

    /// A 16UC1 depth frame in millimetres: a receding floor, a back wall, and a
    /// person-sized column that walks across the view.
    private func depthImageMessage(at elapsed: Double) -> [String: Any] {
        let width = depthSize.width
        let height = depthSize.height
        var bytes = [UInt8](repeating: 0, count: width * height * 2)

        let personCentre = Double(width) * (0.5 + 0.32 * sin(0.4 * elapsed))
        let personHalfWidth = Double(width) * 0.07
        let personDistance = 1.6 + 0.4 * sin(0.25 * elapsed)

        for y in 0..<height {
            let verticalFraction = Double(y) / Double(height - 1)
            // Floor sweeps from far at the horizon to near at the bottom edge.
            let floorDistance = 6.0 - 5.0 * verticalFraction
            for x in 0..<width {
                var metres = verticalFraction < 0.45 ? 6.0 : floorDistance

                let dx = abs(Double(x) - personCentre)
                if dx < personHalfWidth && verticalFraction > 0.18 && verticalFraction < 0.92 {
                    metres = personDistance
                }

                // A band of invalid samples, as a real sensor produces on dark
                // or specular surfaces. Exercises the NaN path end to end.
                if x % 97 == 0 && y % 53 == 0 { metres = 0 }

                let millimetres = UInt16(max(0, min(65535, (metres * 1000).rounded())))
                let index = (y * width + x) * 2
                bytes[index] = UInt8(millimetres & 0xFF)
                bytes[index + 1] = UInt8(millimetres >> 8)
            }
        }

        return [
            "header": header(at: elapsed, frameId: "camera_depth_optical_frame"),
            "height": height,
            "width": width,
            "encoding": "16UC1",
            "is_bigendian": 0,
            "step": width * 2,
            "data": Data(bytes).base64EncodedString(),
        ]
    }
}
