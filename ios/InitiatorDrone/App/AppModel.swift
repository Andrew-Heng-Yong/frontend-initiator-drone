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

    #if canImport(ARKit)
    let arSession = ARSessionController()
    let alignment = AlignmentController()
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

    init() {
        let initialSettings = settings.settings
        connection = RobotConnection(settings: initialSettings)

        // Settings changes flow one way: store -> connection -> pipelines.
        settings.$settings
            .receive(on: DispatchQueue.main)
            .sink { [weak self] newSettings in
                self?.connection.apply(settings: newSettings)
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
        lastActionMessage = "AR tracking reset. Align the robot again."
    }

    /// Offers a previously saved alignment for the current robot, if the
    /// operator wants it back.
    var savedAlignmentForCurrentRobot: RobotAlignment? {
        alignmentStore.alignment(for: connection.endpoint)
    }
    #endif
}
