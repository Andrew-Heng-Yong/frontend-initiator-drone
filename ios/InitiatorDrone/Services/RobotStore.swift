import Foundation
import Combine

/// The operator's saved robots, most recently used first.
@MainActor
public final class RobotStore: ObservableObject {
    private static let storageKey = "com.initiatordrone.robots.v1"
    private static let selectionKey = "com.initiatordrone.robots.selected.v1"
    private static let maximumSaved = 12

    @Published public private(set) var robots: [RobotEndpoint] = []
    @Published public var selectedID: UUID? {
        didSet { defaults.set(selectedID?.uuidString, forKey: Self.selectionKey) }
    }

    private let defaults: UserDefaults

    public init(defaults: UserDefaults = .standard) {
        self.defaults = defaults
        load()
    }

    public var selected: RobotEndpoint? {
        guard let selectedID else { return robots.first }
        return robots.first { $0.id == selectedID } ?? robots.first
    }

    /// Adds a robot, or updates the existing entry with the same host and
    /// ports. Editing an address the operator already saved should not leave a
    /// near-duplicate behind.
    @discardableResult
    public func save(_ endpoint: RobotEndpoint) -> RobotEndpoint {
        var stored = endpoint
        stored.lastUsed = Date()

        if let index = robots.firstIndex(where: { $0.id == endpoint.id }) {
            robots[index] = stored
        } else if let index = robots.firstIndex(where: {
            $0.normalizedHost.caseInsensitiveCompare(stored.normalizedHost) == .orderedSame
                && $0.dashboardPort == stored.dashboardPort
                && $0.rosbridgePort == stored.rosbridgePort
        }) {
            stored.id = robots[index].id
            robots[index] = stored
        } else {
            robots.append(stored)
        }

        sortAndTrim()
        selectedID = stored.id
        persist()
        return stored
    }

    public func remove(_ endpoint: RobotEndpoint) {
        robots.removeAll { $0.id == endpoint.id }
        if selectedID == endpoint.id { selectedID = robots.first?.id }
        persist()
    }

    public func remove(atOffsets offsets: IndexSet) {
        let removed = offsets.compactMap { robots.indices.contains($0) ? robots[$0] : nil }
        // Removing back to front keeps the earlier indices valid.
        for index in offsets.sorted(by: >) where robots.indices.contains(index) {
            robots.remove(at: index)
        }
        if let selectedID, removed.contains(where: { $0.id == selectedID }) {
            self.selectedID = robots.first?.id
        }
        persist()
    }

    /// Marks a robot as just used, which floats it to the top of the list.
    public func touch(_ endpoint: RobotEndpoint) {
        guard let index = robots.firstIndex(where: { $0.id == endpoint.id }) else { return }
        robots[index].lastUsed = Date()
        sortAndTrim()
        persist()
    }

    private func sortAndTrim() {
        robots.sort { $0.lastUsed > $1.lastUsed }
        if robots.count > Self.maximumSaved {
            robots = Array(robots.prefix(Self.maximumSaved))
        }
    }

    private func load() {
        if let data = defaults.data(forKey: Self.storageKey),
           let decoded = try? JSONDecoder().decode([RobotEndpoint].self, from: data) {
            robots = decoded.sorted { $0.lastUsed > $1.lastUsed }
        }
        if let raw = defaults.string(forKey: Self.selectionKey) {
            selectedID = UUID(uuidString: raw)
        } else {
            selectedID = robots.first?.id
        }
    }

    private func persist() {
        guard let data = try? JSONEncoder().encode(robots) else { return }
        defaults.set(data, forKey: Self.storageKey)
    }
}

/// Persistence for the robot-to-AR alignment, keyed by robot.
///
/// Alignment is stored per robot but is only offered as a starting point after
/// a relaunch, never applied silently: ARKit's world origin is wherever the
/// session started, so a saved transform is meaningless in a new session. The
/// UI makes the operator confirm it.
@MainActor
public final class AlignmentStore: ObservableObject {
    private static let storageKey = "com.initiatordrone.alignments.v1"

    @Published public private(set) var alignments: [UUID: RobotAlignment] = [:]

    private let defaults: UserDefaults

    public init(defaults: UserDefaults = .standard) {
        self.defaults = defaults
        if let data = defaults.data(forKey: Self.storageKey),
           let decoded = try? JSONDecoder().decode([UUID: RobotAlignment].self, from: data) {
            alignments = decoded
        }
    }

    public func alignment(for robot: RobotEndpoint?) -> RobotAlignment? {
        guard let robot else { return nil }
        return alignments[robot.id]
    }

    public func store(_ alignment: RobotAlignment, for robot: RobotEndpoint?) {
        guard let robot else { return }
        alignments[robot.id] = alignment
        persist()
    }

    public func clear(for robot: RobotEndpoint?) {
        guard let robot else { return }
        alignments.removeValue(forKey: robot.id)
        persist()
    }

    private func persist() {
        guard let data = try? JSONEncoder().encode(alignments) else { return }
        defaults.set(data, forKey: Self.storageKey)
    }
}
