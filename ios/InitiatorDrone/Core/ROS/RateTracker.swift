import Foundation

/// Measures a topic's publish rate over a sliding window.
///
/// A window rather than an exponential average, because the diagnostics screen
/// needs to show a rate that visibly drops to zero when a topic stops, not one
/// that decays slowly and looks like a slowdown.
public struct RateTracker: Sendable {
    private var timestamps: [Double] = []

    /// Length of the sliding window in seconds.
    public var windowDuration: Double

    /// Cap on retained timestamps, so a 200 Hz IMU cannot grow the array.
    public var capacity: Int

    public private(set) var lastMessageAt: Double?
    public private(set) var totalCount: Int = 0

    /// When the first message ever arrived, used to avoid under-reporting the
    /// rate before the window has had time to fill.
    private var startedAt: Double?

    public init(windowDuration: Double = 3.0, capacity: Int = 1024) {
        self.windowDuration = max(0.5, windowDuration)
        self.capacity = max(2, capacity)
    }

    public mutating func record(at time: Double) {
        if startedAt == nil { startedAt = time }
        timestamps.append(time)
        lastMessageAt = time
        totalCount += 1

        let cutoff = time - windowDuration
        if let firstKept = timestamps.firstIndex(where: { $0 >= cutoff }), firstKept > 0 {
            timestamps.removeFirst(firstKept)
        }
        if timestamps.count > capacity {
            timestamps.removeFirst(timestamps.count - capacity)
        }
    }

    /// Messages per second over the window, evaluated at `time`.
    ///
    /// Rate is computed against the elapsed window rather than against the
    /// span between the first and last timestamps. Using the span would report
    /// a healthy rate forever after a topic dies, since the two remaining
    /// samples stay a fixed distance apart.
    ///
    /// Note that `capacity` caps the reportable rate at
    /// `capacity / windowDuration`: a topic faster than that is under-reported
    /// rather than allowed to grow the array. The defaults leave headroom for
    /// everything this app subscribes to.
    public func rate(at time: Double) -> Double {
        guard !timestamps.isEmpty else { return 0 }
        let cutoff = time - windowDuration
        let recent = timestamps.filter { $0 >= cutoff }
        guard !recent.isEmpty else { return 0 }

        // Before the window has filled, divide by how long we have actually
        // been listening, so the first reading is not artificially low. The
        // reference is the first message ever seen, not the oldest retained
        // one — using the latter would treat samples evicted by `capacity` as
        // if the topic had only just started and inflate the rate wildly.
        let elapsedSinceStart = time - (startedAt ?? time)
        let effectiveWindow = min(windowDuration, max(elapsedSinceStart, 1e-3))
        return Double(recent.count) / effectiveWindow
    }

    /// Seconds since the last message, or `nil` if nothing has arrived.
    public func age(at time: Double) -> Double? {
        guard let lastMessageAt else { return nil }
        return time - lastMessageAt
    }

    public mutating func reset() {
        timestamps.removeAll(keepingCapacity: true)
        lastMessageAt = nil
        startedAt = nil
        totalCount = 0
    }
}

/// A snapshot of one topic's health, for the diagnostics screen.
public struct TopicHealth: Equatable, Identifiable, Sendable {
    public var topic: RobotTopic
    public var rateHz: Double
    public var lastMessageAge: Double?
    public var totalCount: Int
    public var isSubscribed: Bool

    public var id: String { topic.rawValue }

    public init(
        topic: RobotTopic,
        rateHz: Double,
        lastMessageAge: Double?,
        totalCount: Int,
        isSubscribed: Bool
    ) {
        self.topic = topic
        self.rateHz = rateHz
        self.lastMessageAge = lastMessageAge
        self.totalCount = totalCount
        self.isSubscribed = isSubscribed
    }

    /// Never received anything, or nothing recently enough to trust.
    public var isSilent: Bool {
        guard let lastMessageAge else { return true }
        return lastMessageAge > topic.silenceThreshold
    }
}
