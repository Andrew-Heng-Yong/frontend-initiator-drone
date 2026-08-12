import Foundation

public enum RosbridgeConnectionState: Equatable, Sendable {
    case idle
    case connecting
    case connected
    /// Waiting out a backoff delay before attempt `attempt`.
    case reconnecting(attempt: Int, retryIn: Double)
    case failed(String)

    public var isConnected: Bool { self == .connected }

    public var shortDescription: String {
        switch self {
        case .idle: return "Offline"
        case .connecting: return "Connecting"
        case .connected: return "Connected"
        case .reconnecting(let attempt, let retryIn):
            return String(format: "Reconnecting (#%d in %.0fs)", attempt, retryIn)
        case .failed: return "Failed"
        }
    }
}

public struct RosbridgeLogEntry: Identifiable, Equatable, Sendable {
    public enum Level: String, Sendable {
        case info
        case warning
        case error
    }

    public let id = UUID()
    public var date: Date
    public var level: Level
    public var text: String

    public init(date: Date = Date(), level: Level, text: String) {
        self.date = date
        self.level = level
        self.text = text
    }
}

/// Everything the client hands upward.
public enum RosbridgeEvent: Sendable {
    case stateChanged(RosbridgeConnectionState)
    case image(topic: RobotTopic, message: ROSImageMessage)
    case cameraInfo(CameraInfoMessage)
    case odometry(OdometryMessage)
    case flag(topic: RobotTopic, value: Bool)
    case imu(ImuMessage)
    case log(RosbridgeLogEntry)
    /// Emitted roughly once a second with the current per-topic rates.
    case health([TopicHealth])
}

/// Subscribes to the robot's topics, parses what comes back, and keeps the link
/// alive across drops.
///
/// The client is transport-agnostic and does no rendering and no decoding
/// beyond turning a rosbridge frame into typed structs, which keeps it usable
/// from tests and from the fixture player.
public final class RosbridgeClient: RosbridgeTransportDelegate, @unchecked Sendable {

    /// Which topics to subscribe to, and how hard to throttle each one.
    public struct Configuration: Equatable, Sendable {
        public var topics: [RobotTopic]
        public var compression: RosbridgeCompression
        /// Per-topic overrides of `defaultThrottleMilliseconds`.
        public var throttleOverrides: [RobotTopic: Int]
        public var reconnectPolicy: ReconnectPolicy
        public var automaticallyReconnects: Bool

        public init(
            topics: [RobotTopic] = RobotTopic.allCases,
            compression: RosbridgeCompression = .none,
            throttleOverrides: [RobotTopic: Int] = [:],
            reconnectPolicy: ReconnectPolicy = ReconnectPolicy(),
            automaticallyReconnects: Bool = true
        ) {
            self.topics = topics
            self.compression = compression
            self.throttleOverrides = throttleOverrides
            self.reconnectPolicy = reconnectPolicy
            self.automaticallyReconnects = automaticallyReconnects
        }

        public func throttle(for topic: RobotTopic) -> Int {
            throttleOverrides[topic] ?? topic.defaultThrottleMilliseconds
        }
    }

    private let transport: RosbridgeTransport
    private let queue = DispatchQueue(label: "com.initiatordrone.rosbridge.client")
    private let now: @Sendable () -> Double

    /// Called on `callbackQueue` for every event.
    public var onEvent: ((RosbridgeEvent) -> Void)?
    public var callbackQueue: DispatchQueue = .main

    public private(set) var configuration: Configuration
    private var endpoint: RobotEndpoint?
    private var state: RosbridgeConnectionState = .idle
    private var rateTrackers: [RobotTopic: RateTracker] = [:]
    private var subscribedTopics: Set<RobotTopic> = []
    private var reconnectAttempt = 0
    private var reconnectWorkItem: DispatchWorkItem?
    private var healthTimer: DispatchSourceTimer?
    private var wantsConnection = false

    public init(
        transport: RosbridgeTransport,
        configuration: Configuration = Configuration(),
        now: @escaping @Sendable () -> Double = { Date().timeIntervalSince1970 }
    ) {
        self.transport = transport
        self.configuration = configuration
        self.now = now
        transport.delegate = self
    }

    deinit {
        healthTimer?.cancel()
        reconnectWorkItem?.cancel()
    }

    // MARK: - Lifecycle

    public func connect(to endpoint: RobotEndpoint) {
        queue.async { [weak self] in
            guard let self else { return }
            guard let url = endpoint.rosbridgeURL else {
                self.setState(.failed("Invalid rosbridge address"))
                return
            }
            self.endpoint = endpoint
            self.wantsConnection = true
            self.reconnectAttempt = 0
            self.startHealthTimer()
            self.setState(.connecting)
            self.transport.connect(to: url)
        }
    }

    public func disconnect() {
        queue.async { [weak self] in
            guard let self else { return }
            self.wantsConnection = false
            self.reconnectWorkItem?.cancel()
            self.reconnectWorkItem = nil
            self.healthTimer?.cancel()
            self.healthTimer = nil
            self.subscribedTopics.removeAll()
            for topic in self.rateTrackers.keys {
                self.rateTrackers[topic]?.reset()
            }
            self.transport.disconnect()
            self.setState(.idle)
        }
    }

    public func updateConfiguration(_ configuration: Configuration) {
        queue.async { [weak self] in
            guard let self else { return }
            let previous = self.configuration
            self.configuration = configuration
            guard self.state.isConnected else { return }

            // Compression and throttle live in the subscribe command, so a
            // change means re-subscribing rather than just remembering it.
            let needsResubscribe = previous.compression != configuration.compression
                || previous.throttleOverrides != configuration.throttleOverrides
                || previous.topics != configuration.topics

            if needsResubscribe {
                for topic in self.subscribedTopics {
                    self.send(.unsubscribe(topic: topic.topicName, id: self.subscriptionID(for: topic)))
                }
                self.subscribedTopics.removeAll()
                self.subscribeAll()
            }
        }
    }

    /// Calls a rosbridge service. Present for completeness; the app's own
    /// start/stop/calibrate actions go through the dashboard HTTP API instead,
    /// because those manage the ROS launch process rather than a running node.
    public func callService(_ service: String, args: ROSValue? = nil) {
        queue.async { [weak self] in
            guard let self else { return }
            self.send(.callService(service: service, id: UUID().uuidString, args: args))
        }
    }

    // MARK: - Subscriptions

    private func subscribeAll() {
        for topic in configuration.topics where !subscribedTopics.contains(topic) {
            let options = RosbridgeCommand.SubscribeOptions(
                topic: topic.topicName,
                messageType: topic.messageType,
                id: subscriptionID(for: topic),
                throttleMilliseconds: configuration.throttle(for: topic),
                queueLength: topic.defaultQueueLength,
                // Compression is only worth it for the bulky image topics; the
                // rest are a few hundred bytes and JSON keeps them debuggable.
                compression: topic.isImageTopic ? configuration.compression : .none,
                fragmentSize: nil
            )
            send(.subscribe(options))
            subscribedTopics.insert(topic)
        }
        log(.info, "Subscribed to \(subscribedTopics.count) topics")
    }

    private func subscriptionID(for topic: RobotTopic) -> String {
        "initiator-drone-\(topic.rawValue)"
    }

    private func send(_ command: RosbridgeCommand) {
        do {
            transport.send(try command.encoded())
        } catch {
            log(.error, "Could not encode rosbridge command: \(error.localizedDescription)")
        }
    }

    // MARK: - Transport delegate

    public func transportDidConnect(_ transport: RosbridgeTransport) {
        queue.async { [weak self] in
            guard let self else { return }
            self.reconnectAttempt = 0
            self.subscribedTopics.removeAll()
            self.setState(.connected)
            self.log(.info, "rosbridge connected")
            self.subscribeAll()
        }
    }

    public func transport(_ transport: RosbridgeTransport, didReceive data: Data, isBinary: Bool) {
        queue.async { [weak self] in
            self?.handle(data: data, isBinary: isBinary)
        }
    }

    public func transport(_ transport: RosbridgeTransport, didDisconnectWith error: Error?) {
        queue.async { [weak self] in
            guard let self else { return }
            self.subscribedTopics.removeAll()

            if let error {
                self.log(.error, "rosbridge disconnected: \(error.localizedDescription)")
            } else {
                self.log(.warning, "rosbridge disconnected")
            }

            guard self.wantsConnection, self.configuration.automaticallyReconnects else {
                self.setState(.idle)
                return
            }
            self.scheduleReconnect()
        }
    }

    // MARK: - Reconnection

    private func scheduleReconnect() {
        reconnectWorkItem?.cancel()

        let attempt = reconnectAttempt
        let delay = configuration.reconnectPolicy.delay(forAttempt: attempt)
        reconnectAttempt += 1
        setState(.reconnecting(attempt: attempt + 1, retryIn: delay))

        let work = DispatchWorkItem { [weak self] in
            guard let self, self.wantsConnection, let endpoint = self.endpoint,
                  let url = endpoint.rosbridgeURL else { return }
            self.setState(.connecting)
            self.transport.connect(to: url)
        }
        reconnectWorkItem = work
        queue.asyncAfter(deadline: .now() + delay, execute: work)
    }

    // MARK: - Inbound handling

    private func handle(data: Data, isBinary: Bool) {
        let value: ROSValue
        do {
            // rosbridge answers a CBOR subscription with binary frames and
            // everything else — including status replies to that same
            // subscription — with text, so the frame type decides the decoder
            // rather than the configured compression.
            value = isBinary ? try CBORDecoder.decode(data) : try ROSValue.fromJSON(data)
        } catch {
            log(.error, "Could not decode rosbridge frame (\(data.count) bytes): \(error)")
            return
        }

        guard let incoming = RosbridgeIncoming.parse(value) else {
            log(.warning, "Unrecognised rosbridge frame")
            return
        }

        switch incoming {
        case .publish(let topicName, let message):
            handlePublish(topicName: topicName, message: message)

        case .status(let level, let text, _):
            let mapped: RosbridgeLogEntry.Level
            switch level.lowercased() {
            case "error", "fatal": mapped = .error
            case "warning", "warn": mapped = .warning
            default: mapped = .info
            }
            log(mapped, "rosbridge: \(text)")

        case .serviceResponse(let service, _, _, let result):
            log(result ? .info : .error, "service \(service) -> \(result ? "ok" : "failed")")

        case .other(let op, _):
            log(.info, "rosbridge op '\(op)'")
        }
    }

    private func handlePublish(topicName: String, message: ROSValue) {
        guard let topic = RobotTopic(rawValue: topicName) else { return }

        let arrival = now()
        rateTrackers[topic, default: RateTracker()].record(at: arrival)

        switch topic {
        case .depthImage:
            guard let image = ROSMessageParser.image(from: message) else {
                log(.warning, "Malformed image on \(topicName)")
                return
            }
            emit(.image(topic: topic, message: image))

        case .depthCameraInfo:
            guard let info = ROSMessageParser.cameraInfo(from: message) else { return }
            emit(.cameraInfo(info))

        case .odometry:
            guard let odometry = ROSMessageParser.odometry(from: message) else {
                log(.warning, "Malformed odometry on \(topicName)")
                return
            }
            emit(.odometry(odometry))

        case .odomCalibrated:
            guard let flag = ROSMessageParser.boolean(from: message) else { return }
            emit(.flag(topic: topic, value: flag))

        case .imu:
            guard let imu = ROSMessageParser.imu(from: message) else { return }
            emit(.imu(imu))
        }
    }

    // MARK: - Health

    private func startHealthTimer() {
        healthTimer?.cancel()
        let timer = DispatchSource.makeTimerSource(queue: queue)
        timer.schedule(deadline: .now() + 1.0, repeating: 1.0)
        timer.setEventHandler { [weak self] in
            guard let self else { return }
            self.emit(.health(self.currentHealthLocked()))
        }
        timer.resume()
        healthTimer = timer
    }

    /// Must be called on `queue`.
    private func currentHealthLocked() -> [TopicHealth] {
        let time = now()
        return configuration.topics.map { topic in
            let tracker = rateTrackers[topic]
            return TopicHealth(
                topic: topic,
                rateHz: tracker?.rate(at: time) ?? 0,
                lastMessageAge: tracker?.age(at: time),
                totalCount: tracker?.totalCount ?? 0,
                isSubscribed: subscribedTopics.contains(topic)
            )
        }
    }

    // MARK: - Emission

    private func setState(_ newState: RosbridgeConnectionState) {
        guard state != newState else { return }
        state = newState
        emit(.stateChanged(newState))
    }

    private func log(_ level: RosbridgeLogEntry.Level, _ text: String) {
        emit(.log(RosbridgeLogEntry(level: level, text: text)))
    }

    private func emit(_ event: RosbridgeEvent) {
        guard let onEvent else { return }
        callbackQueue.async { onEvent(event) }
    }
}
