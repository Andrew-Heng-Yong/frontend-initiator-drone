import Foundation
import Combine

/// The app's single source of truth about the robot.
///
/// Owns the rosbridge client, the dashboard HTTP client and the two image
/// pipelines, and exposes everything the screens need as published state. All
/// mutation happens on the main actor; the expensive work happens on the
/// queues owned by the client and the pipelines.
@MainActor
public final class RobotConnection: ObservableObject {

    // MARK: - Published state

    @Published public private(set) var connectionState: RosbridgeConnectionState = .idle
    @Published public private(set) var endpoint: RobotEndpoint?
    @Published public private(set) var dashboardState: DashboardState?
    @Published public private(set) var dashboardError: String?

    @Published public private(set) var isCalibrated: Bool?
    @Published public private(set) var isVisualTracking: Bool?
    @Published public private(set) var trackingStatus: RobotTrackingStatus = .unknown
    /// Whether `vio_node` itself is up, independently of whether its pose can
    /// be believed. See `VIONodeStatus` for why this is derived from traffic
    /// rather than asked of `/rosapi/nodes`.
    @Published public private(set) var vioNodeStatus: VIONodeStatus = .unknown

    @Published public private(set) var latestOdometry: OdometryMessage?
    @Published public private(set) var latestIMU: ImuMessage?
    @Published public private(set) var depthCameraInfo: CameraInfoMessage?

    @Published public private(set) var depthFrame: RenderedFrame?

    @Published public private(set) var topicHealth: [TopicHealth] = []
    @Published public private(set) var logs: [RosbridgeLogEntry] = []

    /// Rendered frames per second, measured on the phone. Distinct from the
    /// topic rate: it is what actually reaches the screen after drops.
    @Published public private(set) var depthRenderFPS: Double = 0
    @Published public private(set) var odometryRateHz: Double = 0

    @Published public private(set) var droppedDepthFrames: Int = 0

    /// Estimated `phoneClock - robotClock`, in seconds.
    @Published public private(set) var clockOffset: Double?

    @Published public var isSimulated: Bool = false

    // MARK: - Internals

    private var client: RosbridgeClient?
    private var transport: RosbridgeTransport?
    private let dashboard = DashboardAPI()
    private var settings: AppSettings

    private var depthPipeline: ImageStreamPipeline?

    /// Shared with the SceneKit render loop, which samples it directly off the
    /// main actor. See `OdometrySampler` for why.
    public let sampler = OdometrySampler()

    /// Newest depth point cloud, handed straight from the image worker to the
    /// render thread without passing through the main actor.
    public let pointCloudStore = PointCloudStore()

    private var depthRenderRate = RateTracker(windowDuration: 2.0)

    /// Phone-clock time of the newest message on any topic `vio_node` publishes.
    /// Tracked here rather than read out of `topicHealth`, which is only
    /// refreshed once a second and would blur the node-status transitions.
    private var lastVIOMessageTime: Double?

    private var dashboardTimer: Timer?
    private var statusTimer: Timer?

    private static let maximumLogEntries = 400

    public init(settings: AppSettings = AppSettings()) {
        self.settings = settings
        rebuildPipelines()
        startStatusTimer()
    }

    /// Tears everything down. `RobotConnection` is held for the app's lifetime
    /// as a `StateObject`, so this exists for tests and for the rare case of a
    /// scene being discarded rather than for `deinit`, which cannot touch
    /// main-actor state.
    public func shutdown() {
        disconnect()
        statusTimer?.invalidate()
        statusTimer = nil
    }

    // MARK: - Settings

    public func apply(settings newSettings: AppSettings) {
        let needsClientUpdate = newSettings.compression != settings.compression
            || newSettings.imageThrottleMilliseconds != settings.imageThrottleMilliseconds
        settings = newSettings

        depthPipeline?.colorMapSettings = newSettings.depthColorMap
        depthPipeline?.pointCloudSettings = newSettings.pointCloud
        sampler.setStalenessThreshold(newSettings.odometryStalenessThreshold)
        sampler.setExtrapolationLimit(newSettings.odometryExtrapolationLimit)

        // Turning the cloud off should clear it, not freeze the last one in
        // mid-air.
        if !newSettings.pointCloud.isEnabled {
            pointCloudStore.reset()
        }

        if needsClientUpdate {
            client?.updateConfiguration(makeClientConfiguration())
        }
        refreshTrackingStatus()
    }

    private func makeClientConfiguration() -> RosbridgeClient.Configuration {
        RosbridgeClient.Configuration(
            topics: RobotTopic.allCases,
            compression: settings.compression,
            throttleOverrides: [.depthImage: settings.imageThrottleMilliseconds],
            reconnectPolicy: ReconnectPolicy(),
            automaticallyReconnects: true
        )
    }

    private func rebuildPipelines() {
        let currentSettings = settings

        let depth = ImageStreamPipeline(
            topic: .depthImage,
            colorMapSettings: currentSettings.depthColorMap,
            interpretationProvider: { encoding in
                ScalarInterpretation.depthDefault(for: encoding)
            }
        )
        depth.onFrame = { [weak self] frame in
            self?.handleDepthFrame(frame)
        }
        depth.onError = { [weak self] text in
            self?.append(log: RosbridgeLogEntry(level: .error, text: text))
        }
        depth.pointCloudSettings = currentSettings.pointCloud
        // Called on the pipeline's worker queue; the store is the thread-safe
        // hand-off, so there is no hop to main on this path.
        depth.onPointCloud = { [weak self] cloud in
            self?.pointCloudStore.store(cloud)
        }

        depthPipeline = depth
    }

    // MARK: - Connection lifecycle

    /// Connects to a real robot.
    public func connect(to endpoint: RobotEndpoint) {
        connect(to: endpoint, transport: WebSocketRosbridgeTransport(), simulated: false)
    }

    /// Connects to the fixture player, so the app runs with no drone present.
    public func connectToSimulator(_ simulator: SimulatedRosbridgeTransport) {
        let fake = RobotEndpoint(name: "Simulator", host: "simulated.local")
        connect(to: fake, transport: simulator, simulated: true)
    }

    private func connect(to endpoint: RobotEndpoint, transport: RosbridgeTransport, simulated: Bool) {
        disconnect()

        self.endpoint = endpoint
        self.isSimulated = simulated
        self.transport = transport

        let client = RosbridgeClient(transport: transport, configuration: makeClientConfiguration())
        client.callbackQueue = .main
        client.onEvent = { [weak self] event in
            self?.enqueue(event)
        }
        self.client = client
        client.connect(to: endpoint)

        if simulated {
            updateSimulatedDashboardState()
        } else {
            startDashboardPolling()
            Task {
                await dashboard.setEndpoint(endpoint)
                await self.refreshDashboardState()
            }
        }
    }

    /// Hops an event onto the main actor.
    ///
    /// Strict ordering between hops is not guaranteed by the runtime, and
    /// nothing downstream needs it: the odometry buffer sorts by stamp, the
    /// image pipelines keep only the newest frame, and health is a whole
    /// snapshot each time.
    nonisolated private func enqueue(_ event: RosbridgeEvent) {
        Task { @MainActor [weak self] in
            self?.handle(event)
        }
    }

    public func disconnect() {
        client?.disconnect()
        client = nil
        transport = nil
        dashboardTimer?.invalidate()
        dashboardTimer = nil

        depthPipeline?.reset()
        pointCloudStore.reset()
        sampler.reset()
        depthRenderRate.reset()

        connectionState = .idle
        // Nothing is known about the graph once the link is gone, and a stale
        // "running" would leave Stop and Calibrate enabled against a backend
        // that is no longer there.
        dashboardState = nil
        dashboardError = nil
        depthFrame = nil
        latestOdometry = nil
        latestIMU = nil
        isCalibrated = nil
        isVisualTracking = nil
        lastVIOMessageTime = nil
        clockOffset = nil
        depthRenderFPS = 0
        odometryRateHz = 0
        topicHealth = []
        trackingStatus = .unknown
        vioNodeStatus = .unknown
        isSimulated = false
    }

    // MARK: - Dashboard actions

    public func startGraph() async {
        await performDashboardAction(
            "Start",
            onSimulator: { $0.startGraph() },
            onRobot: { try await self.dashboard.start() }
        )
    }

    public func stopGraph() async {
        await performDashboardAction(
            "Stop",
            onSimulator: { $0.stopGraph() },
            onRobot: { try await self.dashboard.stop() }
        )
    }

    /// Requests a VIO calibration. Refused while the graph is stopped, because
    /// there is nothing running to calibrate.
    public func calibrateVIO() async {
        await performDashboardAction(
            "Calibrate",
            onSimulator: { $0.calibrate() },
            onRobot: { try await self.dashboard.calibrateVIO() }
        )
    }

    public var canCalibrate: Bool {
        dashboardState?.isRunning ?? false
    }

    /// The fixture player, when one is driving this connection.
    private var simulator: SimulatedRosbridgeTransport? {
        transport as? SimulatedRosbridgeTransport
    }

    /// Runs a dashboard action against whichever backend is connected.
    ///
    /// Fixtures mode used to refuse these outright, which left the operator
    /// with three dead buttons and no way to exercise the launch lifecycle
    /// without a robot. The simulator models the same states instead, so the
    /// controls behave the same way and only the thing being controlled
    /// differs.
    private func performDashboardAction(
        _ name: String,
        onSimulator: (SimulatedRosbridgeTransport) -> Void,
        onRobot: @escaping () async throws -> Bool
    ) async {
        if let simulator {
            onSimulator(simulator)
            append(log: RosbridgeLogEntry(level: .info, text: "\(name) requested (fixtures)"))
            dashboardError = nil
            updateSimulatedDashboardState()
            refreshTrackingStatus()
            return
        }
        do {
            _ = try await onRobot()
            append(log: RosbridgeLogEntry(level: .info, text: "\(name) requested"))
            dashboardError = nil
        } catch {
            let message = (error as? LocalizedError)?.errorDescription ?? error.localizedDescription
            dashboardError = message
            append(log: RosbridgeLogEntry(level: .error, text: "\(name) failed: \(message)"))
        }
        await refreshDashboardState()
    }

    public func refreshDashboardState() async {
        guard endpoint != nil else { return }
        guard simulator == nil else {
            updateSimulatedDashboardState()
            return
        }
        do {
            dashboardState = try await dashboard.fetchState()
            dashboardError = nil
        } catch {
            dashboardState = nil
            dashboardError = (error as? LocalizedError)?.errorDescription ?? error.localizedDescription
        }
    }

    /// Fixtures mode has no HTTP dashboard, so the simulator itself is the
    /// authority on whether the graph is up.
    private func updateSimulatedDashboardState() {
        guard let simulator else { return }
        dashboardState = DashboardState(isRunning: simulator.isGraphRunning)
        dashboardError = nil
    }

    private func startDashboardPolling() {
        dashboardTimer?.invalidate()
        let timer = Timer.scheduledTimer(withTimeInterval: 3.0, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in
                await self?.refreshDashboardState()
            }
        }
        dashboardTimer = timer
    }

    // MARK: - Pose access

    /// The robot's pose in ARKit world coordinates for a given phone time.
    ///
    /// - Parameters:
    ///   - localTime: the render frame's time on the phone's clock.
    ///   - alignment: the operator's robot-to-AR alignment.
    /// - Returns: the pose plus how it was obtained, or `nil` when no odometry
    ///   has been buffered.
    public func robotPoseInAR(atLocalTime localTime: Double) -> (pose: Pose, sample: PoseSample)? {
        sampler.poseInAR(atLocalTime: localTime)
    }

    /// Raw robot pose in the ROS `odom` frame, sampled the same way.
    public func robotPoseInOdom(atLocalTime localTime: Double) -> PoseSample? {
        sampler.poseInOdom(atLocalTime: localTime)
    }

    /// Installs the operator's robot-to-AR alignment, or clears it.
    public func setAlignment(_ alignment: RobotAlignment?) {
        sampler.setAlignment(alignment)
    }

    /// Recent poses in the `odom` frame, for drawing a trail.
    public func odometryTrail(maximumCount: Int = 120) -> [Pose] {
        sampler.trailInOdom(maximumCount: maximumCount)
    }

    // MARK: - Event handling

    private func handle(_ event: RosbridgeEvent) {
        switch event {
        case .stateChanged(let state):
            connectionState = state
            if case .connected = state {
                // A reconnect means a new rosbridge session; anything buffered
                // from before it belongs to a different continuity of time.
                sampler.reset()
                depthPipeline?.reset()
        pointCloudStore.reset()
                lastVIOMessageTime = nil
            }
            refreshTrackingStatus()

        case .image(let topic, let message):
            observeClock(rosStamp: message.stamp)
            switch topic {
            case .depthImage: depthPipeline?.submit(message)
            default: break
            }

        case .cameraInfo(let info):
            depthCameraInfo = info
            // The deprojection needs intrinsics; without them the pipeline
            // emits an empty cloud rather than guessing a focal length.
            depthPipeline?.cameraInfo = info

        case .odometry(let odometry):
            latestOdometry = odometry
            noteVIONodeMessage()
            sampler.append(odometry.stampedPose, localTime: Date().timeIntervalSince1970)
            clockOffset = sampler.clockOffset
            refreshTrackingStatus()

        case .flag(let topic, let value):
            switch topic {
            case .vioCalibrated: isCalibrated = value
            case .visualTracking: isVisualTracking = value
            default: break
            }
            noteVIONodeMessage()
            refreshTrackingStatus()

        case .imu(let imu):
            latestIMU = imu
            noteVIONodeMessage()
            observeClock(rosStamp: imu.stamp)

        case .log(let entry):
            append(log: entry)

        case .health(let health):
            topicHealth = health
            odometryRateHz = health.first { $0.topic == .odometry }?.rateHz ?? 0
        }
    }

    /// Records that `vio_node` was heard from. `/imu/data_calibrated` counts:
    /// the VIO node republishes it, so it is the node's output rather than the
    /// MPU6050 driver's.
    private func noteVIONodeMessage() {
        lastVIOMessageTime = Date().timeIntervalSince1970
    }

    private func observeClock(rosStamp: Double) {
        guard rosStamp > 0 else { return }
        sampler.observeClock(rosStamp: rosStamp, localTime: Date().timeIntervalSince1970)
        clockOffset = sampler.clockOffset
    }

    private func handleDepthFrame(_ frame: RenderedFrame) {
        depthFrame = frame
        depthRenderRate.record(at: Date().timeIntervalSince1970)
        droppedDepthFrames = depthPipeline?.statistics.dropped ?? 0
    }

    private func append(log entry: RosbridgeLogEntry) {
        logs.append(entry)
        if logs.count > Self.maximumLogEntries {
            logs.removeFirst(logs.count - Self.maximumLogEntries)
        }
    }

    public func clearLogs() {
        logs.removeAll()
    }

    // MARK: - Derived status

    private func startStatusTimer() {
        let timer = Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in
                self?.refreshDerivedRates()
            }
        }
        statusTimer = timer
    }

    private func refreshDerivedRates() {
        let now = Date().timeIntervalSince1970
        depthRenderFPS = depthRenderRate.rate(at: now)
        refreshTrackingStatus()
    }

    private func refreshTrackingStatus() {
        let now = Date().timeIntervalSince1970
        let odometryAge = sampler.age(atLocalTime: now)

        trackingStatus = RobotTrackingStatus.evaluate(
            isCalibrated: isCalibrated,
            isVisualTracking: isVisualTracking,
            odometryAge: odometryAge,
            stalenessThreshold: settings.odometryStalenessThreshold
        )

        vioNodeStatus = VIONodeStatus.evaluate(
            isLinkConnected: connectionState.isConnected,
            // The simulator answers this too, so Stop in fixtures mode reaches
            // `.graphStopped` rather than looking like a crashed node.
            isGraphRunning: dashboardState?.isRunning,
            isCalibrated: isCalibrated,
            vioMessageAge: lastVIOMessageTime.map { now - $0 },
            odometryAge: odometryAge,
            silenceThreshold: RobotTopic.odometry.silenceThreshold
        )
    }
}
