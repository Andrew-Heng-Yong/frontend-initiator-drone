import Combine
import SwiftUI
#if canImport(ARKit)
import ARKit
#endif

/// Wires the stores, the connection and the AR session together, and holds the
/// few pieces of state that do not belong to any single screen.
@MainActor
final class AppModel: ObservableObject {
    let robots = RobotStore()
    let settings = SettingsStore()
    let connection: RobotConnection
    let alignmentStore = AlignmentStore()

    let recording = LocalizationRecordingService()

    #if canImport(ARKit)
    let arSession = ARSessionController()
    let alignment = AlignmentController()
    let tagDetection: TagDetectionController
    #endif

    /// Which tab is showing, so the connection screen can hand off to the live
    /// view once a robot is connected.
    @Published var selectedTab: Tab = .live
    @Published var lastActionMessage: String?

    enum Tab: Hashable {
        case live
        case connection
        case diagnostics
        case settings
    }

    private var cancellables = Set<AnyCancellable>()
    private var recordingTimer: Timer?

    init() {
        let initialSettings = settings.settings
        connection = RobotConnection(settings: initialSettings)
        #if canImport(ARKit)
        tagDetection = TagDetectionController(settings: initialSettings)
        #endif

        // Settings changes flow one way: store -> connection -> pipelines.
        settings.$settings
            .receive(on: DispatchQueue.main)
            .sink { [weak self] newSettings in
                guard let self else { return }
                self.connection.apply(settings: newSettings)
                // The fixtures robot can be parked and released while
                // connected, so the flag is pushed rather than only read at
                // connect time.
                self.simulatedTransport?.setStatic(newSettings.fixtureRobotIsStatic)
                #if canImport(ARKit)
                self.tagDetection.apply(settings: newSettings)
                #endif
            }
            .store(in: &cancellables)

        #if canImport(ARKit)
        // The alignment controller owns the transform; the sampler needs it on
        // the render thread, and the store keeps it across launches.
        alignment.$alignment
            .receive(on: DispatchQueue.main)
            .sink { [weak self] value in
                guard let self else { return }
                self.connection.setAlignment(value)
                if let value {
                    self.alignmentStore.store(value, for: self.connection.endpoint)
                }
            }
            .store(in: &cancellables)

        // ARKit frame -> tag detector. The session hands over the frame and the
        // pose it was captured at together, which is the only reason a fix can
        // be trusted: a sighting composed with a phone pose from a different
        // instant is wrong by however far the operator moved in between.
        arSession.onFrame = { [weak self] frame in
            self?.tagDetection.submit(frame: frame)
        }

        // Accepted fix -> alignment. `TagLocalization` already solved for the
        // transform the existing render path consumes, so there is nothing to
        // add here beyond installing it and logging it.
        tagDetection.onFix = { [weak self] robotPoseInAR, tagID, range, phonePose in
            guard let self else { return }
            let odometry = self.connection.latestOdometry
            let solved = TagLocalization.alignment(
                placingRobotAt: robotPoseInAR,
                reportedOdometry: odometry?.pose ?? .identity
            )
            self.alignment.applyTagFix(solved, tagID: tagID)
            self.recording.recordTagFix(
                robotPoseInWorld: robotPoseInAR,
                tagID: tagID,
                range: range,
                odometryPose: odometry?.pose,
                odometryStamp: odometry?.stamp,
                phonePoseInWorld: phonePose,
                alignment: solved
            )
        }
        #endif

        startRecordingSampler()
    }

    /// Samples the odometry estimate for the CSV log.
    ///
    /// Driven by a timer rather than by `/odom` arriving, because the log wants
    /// an even time series and `/odom` runs at IMU rate. The service applies its
    /// own interval on top; this only has to tick often enough not to be the
    /// limit.
    private func startRecordingSampler() {
        let timer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in
                self?.sampleOdometryForRecording()
            }
        }
        recordingTimer = timer
    }

    private func sampleOdometryForRecording() {
        guard recording.isRecording else { return }
        #if canImport(ARKit)
        // The rendered pose, so the log records what the operator was actually
        // shown rather than a second estimate computed only for the file.
        guard let sample = connection.robotPoseInAR(atLocalTime: Date().timeIntervalSince1970)
        else { return }
        recording.recordOdometry(
            robotPoseInWorld: sample.pose,
            odometryPose: connection.latestOdometry?.pose ?? .identity,
            odometryStamp: connection.latestOdometry?.stamp,
            phonePoseInWorld: arSession.currentPose,
            alignment: alignment.alignment
        )
        #endif
    }

    // MARK: - Connection actions

    func connect(to endpoint: RobotEndpoint) {
        let saved = robots.save(endpoint)
        connection.connect(to: saved)
        #if canImport(ARKit)
        startARIfNeeded()
        #endif
        selectedTab = .live
    }

    func connectToFixtures() {
        #if canImport(ARKit)
        startARIfNeeded()
        #endif
        let simulator = SimulatedRosbridgeTransport()
        // Set before connecting, so the very first odometry message already
        // reflects the choice.
        simulator.isStatic = settings.settings.fixtureRobotIsStatic
        connection.connectToSimulator(simulator)
        simulatedTransport = simulator
        selectedTab = .live
    }

    /// Retained so the diagnostics screen can trigger a simulated Wi-Fi drop.
    private(set) var simulatedTransport: SimulatedRosbridgeTransport?

    func simulateConnectionLoss() {
        simulatedTransport?.simulateConnectionLoss()
    }

    func disconnect() {
        connection.disconnect()
        simulatedTransport = nil
    }

    #if canImport(ARKit)
    func startARIfNeeded() {
        guard !arSession.isRunning else { return }
        arSession.start()
    }

    /// Resetting ARKit moves the world origin, which invalidates any alignment
    /// expressed in it. Clearing it is the honest thing to do rather than
    /// leaving a marker floating in the wrong place.
    func resetARTracking() {
        arSession.resetTracking()
        alignment.clearAlignment()
        // World coordinates from before a reset describe a world that no longer
        // exists, so a fix computed in them must not survive into the new one.
        tagDetection.reset()
        lastActionMessage = "AR tracking reset. Align the robot again."
    }

    /// Offers a previously saved alignment for the current robot, if the
    /// operator wants it back.
    var savedAlignmentForCurrentRobot: RobotAlignment? {
        alignmentStore.alignment(for: connection.endpoint)
    }
    #endif
}
