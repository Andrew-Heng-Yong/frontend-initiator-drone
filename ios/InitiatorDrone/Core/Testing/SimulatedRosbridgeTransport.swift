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

    /// Latest `/odom/calibrated` value. Driven by `calibrate()` and by the
    /// startup calibration a simulated launch performs.
    public var isCalibrated = true

    /// How long a simulated stationary calibration takes, in seconds. Short
    /// enough to watch, long enough to see the pill change.
    public var calibrationDuration: Double = 2.0

    /// Holds the robot completely still instead of letting it turn on the
    /// spot. Safe to flip while connected.
    public var isStatic = false

    /// Heading of the held robot about `odom` +Z, in radians.
    public var staticHeading: Double = 0

    private let queue = DispatchQueue(label: "com.initiatordrone.rosbridge.simulator")
    private var timer: DispatchSourceTimer?
    private var connected = false
    private var subscriptions: [String: Subscription] = [:]
    private var startTime = Date()
    private var lastSendTimes: [String: Double] = [:]
    private var tickCount = 0

    /// Whether the simulated ROS launch is up. A stopped graph publishes
    /// nothing at all, which is what killing the launch does.
    private var graphRunning = true
    /// Elapsed time at which the running calibration completes, or `nil`.
    private var calibrationEndsAt: Double?

    private struct Subscription {
        var topic: String
        var throttleSeconds: Double
        var compression: String
    }

    public var isConnected: Bool { queue.sync { connected } }

    /// Whether the simulated launch is running. `RobotConnection` reads this in
    /// place of `GET /api/state`, which has no counterpart here.
    public var isGraphRunning: Bool { queue.sync { graphRunning } }

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

    // MARK: - Simulated dashboard actions

    /// Stands in for `POST /api/start`.
    ///
    /// A real launch brings the nodes up and `odom_node` immediately runs its
    /// stationary gyro-bias calibration, so this does the same.
    public func startGraph() {
        queue.async { [weak self] in
            guard let self, !self.graphRunning else { return }
            self.graphRunning = true
            // Every stream restarts from now rather than pretending to have
            // been publishing all along while stopped.
            self.lastSendTimes.removeAll()
            self.beginCalibrationLocked()
            self.sendStatus(level: "info", message: "simulated launch started")
        }
    }

    /// Stands in for `POST /api/stop`: every topic goes quiet.
    ///
    /// Killing the real launch also takes rosbridge with it, so the socket
    /// drops. This keeps the link up on purpose — otherwise the client would
    /// immediately reconnect to a fresh simulator and the stopped state would
    /// never be visible long enough to look at.
    public func stopGraph() {
        queue.async { [weak self] in
            guard let self, self.graphRunning else { return }
            self.graphRunning = false
            self.calibrationEndsAt = nil
            // The next launch starts uncalibrated, as a fresh node does.
            self.isCalibrated = false
            self.sendStatus(level: "info", message: "simulated launch stopped")
        }
    }

    /// Stands in for `POST /api/odom/calibrate`.
    ///
    /// Mirrors what `odom_node` actually does: publish `calibrated = false`,
    /// stop publishing odometry while it collects stationary samples, then
    /// publish `calibrated = true` and resume. Nothing to calibrate while the
    /// graph is stopped.
    public func calibrate() {
        queue.async { [weak self] in
            guard let self, self.graphRunning else { return }
            self.beginCalibrationLocked()
        }
    }

    /// Must be called on `queue`.
    private func beginCalibrationLocked() {
        isCalibrated = false
        calibrationEndsAt = Date().timeIntervalSince(startTime) + calibrationDuration
        // Let the flags go out on the next tick rather than up to a second
        // later, so the button visibly does something.
        lastSendTimes.removeValue(forKey: RobotTopic.odomCalibrated.topicName)
    }

    /// Holds or releases the robot while connected.
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

        // A stopped launch has no nodes, so nothing publishes at all. This is
        // what makes `OdomNodeStatus` reach `.graphStopped` in fixtures mode.
        guard graphRunning else { return }

        if let endsAt = calibrationEndsAt, elapsed >= endsAt {
            calibrationEndsAt = nil
            isCalibrated = true
        }
        let isCalibrating = calibrationEndsAt != nil

        // The camera is a different node and keeps streaming through a gyro
        // calibration, exactly as on the robot.
        publishIfDue(.depthImage, interval: 1.0 / depthFrameRate, elapsed: elapsed) {
            self.depthImageMessage(at: elapsed)
        }
        publishIfDue(.depthCameraInfo, interval: 1.0, elapsed: elapsed) {
            self.cameraInfoMessage(at: elapsed)
        }
        publishIfDue(.odomCalibrated, interval: 1.0, elapsed: elapsed) {
            ["data": self.isCalibrated]
        }

        // No pose exists until initialisation finishes; the real node publishes
        // none either.
        guard !isCalibrating else { return }

        publishIfDue(.odometry, interval: 1.0 / odometryRate, elapsed: elapsed) {
            self.odometryMessage(at: elapsed)
        }
        publishIfDue(.imu, interval: 1.0 / imuRate, elapsed: elapsed) {
            self.imuMessage(at: elapsed)
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

    /// The robot turning slowly on the spot, with a little pitch and roll.
    ///
    /// **Position is always zero**, because that is all `odom_node` can report:
    /// it integrates the gyro and nothing else, so translation is a placeholder
    /// with a 1e6 m² variance. This used to fly a figure-of-eight, which made
    /// for a better demo and a worse simulator — it taught the operator to
    /// expect a marker that moves, when the real one never does.
    ///
    /// With `isStatic` even the rotation stops. Odometry still publishes at the
    /// same rate with the same timestamps, so the link, the buffer and the
    /// staleness logic are all still exercised. That is the mode to use when
    /// checking whether the marker and the cloud land in the right place,
    /// because a turning robot makes a yaw alignment error impossible to
    /// distinguish from motion.
    func simulatedPose(at elapsed: Double) -> Pose {
        if isStatic {
            return Pose(position: .zero, orientation: Quaternion.aroundZ(staticHeading))
        }

        // Yaw sweeps back and forth rather than spinning, so the marker stays
        // roughly in front of whoever is holding the phone. The pitch and roll
        // wobble is small and out of phase, which is enough to show that the
        // full orientation is being applied and not just a heading.
        let yaw = staticHeading + 0.6 * sin(0.25 * elapsed)
        let pitch = 0.08 * sin(0.41 * elapsed)
        let roll = 0.05 * sin(0.33 * elapsed + 1.1)

        let orientation = Quaternion.aroundZ(yaw)
            * Quaternion(axis: Vector3(0, 1, 0), angle: pitch)
            * Quaternion(axis: Vector3(1, 0, 0), angle: roll)

        return Pose(position: .zero, orientation: orientation.normalized)
    }

    private func odometryMessage(at elapsed: Double) -> [String: Any] {
        let pose = simulatedPose(at: elapsed)
        let ahead = simulatedPose(at: elapsed + 0.05)

        // Body-frame angular rate from the orientation difference, which is
        // what `odom_node` publishes in the twist. Position and linear velocity
        // stay at zero to match it.
        let delta = (pose.orientation.conjugate * ahead.orientation).normalized
        let angle = 2.0 * acos(min(1.0, max(-1.0, delta.w)))
        let sinHalf = sqrt(max(0.0, 1.0 - delta.w * delta.w))
        let axis = sinHalf > 1e-9
            ? Vector3(delta.x, delta.y, delta.z) * (1.0 / sinHalf)
            : Vector3.zero
        let angularVelocity = axis * (angle / 0.05)

        // Position and linear velocity are unobserved; `odom_node` says so with
        // a 1e6 variance on the translational diagonal, and the app reads that
        // rather than being told which robot it is talking to.
        var poseCovariance = [Double](repeating: 0, count: 36)
        for index in [0, 7, 14] {
            poseCovariance[index] = 1.0e6
        }
        for index in [21, 28, 35] {
            poseCovariance[index] = 0.01
        }

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
                "covariance": poseCovariance,
            ],
            "twist": [
                "twist": [
                    "linear": ["x": 0.0, "y": 0.0, "z": 0.0],
                    "angular": [
                        "x": angularVelocity.x,
                        "y": angularVelocity.y,
                        "z": angularVelocity.z,
                    ],
                ],
                "covariance": [Double](repeating: 0, count: 36),
            ],
        ]
    }

    /// `/imu/data_calibrated` as `odom_node` republishes it: the integrated
    /// orientation, the bias-corrected angular rate, and `base_link` as the
    /// frame.
    ///
    /// Linear acceleration is marked unavailable with `covariance[0] = -1`,
    /// which is the `sensor_msgs/Imu` way of saying "this estimator does not
    /// touch the accelerometer". Sending a plausible 9.81 here would be a lie
    /// the operator could not check against the real robot.
    private func imuMessage(at elapsed: Double) -> [String: Any] {
        let pose = simulatedPose(at: elapsed)
        let ahead = simulatedPose(at: elapsed + 0.05)
        let delta = (pose.orientation.conjugate * ahead.orientation).normalized
        let sinHalf = sqrt(max(0.0, 1.0 - delta.w * delta.w))
        let scale = sinHalf > 1e-9
            ? 2.0 * acos(min(1.0, max(-1.0, delta.w))) / (sinHalf * 0.05)
            : 0.0

        var accelerationCovariance = [Double](repeating: 0, count: 9)
        accelerationCovariance[0] = -1.0

        return [
            "header": header(at: elapsed, frameId: "base_link"),
            "orientation": [
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            ],
            "orientation_covariance": [Double](repeating: 0.01, count: 9),
            "angular_velocity": [
                "x": delta.x * scale,
                "y": delta.y * scale,
                "z": delta.z * scale,
            ],
            "angular_velocity_covariance": [Double](repeating: 0.02, count: 9),
            "linear_acceleration": ["x": 0.0, "y": 0.0, "z": 0.0],
            "linear_acceleration_covariance": accelerationCovariance,
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
