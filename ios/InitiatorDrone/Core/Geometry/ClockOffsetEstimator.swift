import Foundation

/// Estimates the offset between the robot's ROS clock and the phone's clock.
///
/// The odometry buffer is keyed by ROS message stamps, but rendering happens on
/// the phone's timeline. Without an offset estimate the two clocks can be
/// seconds or hours apart — the Raspberry Pi may have no RTC at all — and every
/// pose lookup would clamp or return nothing.
///
/// The estimator uses the classic minimum-delay filter. For each message,
/// `delay = localArrival - rosStamp = transportLatency + clockDifference`.
/// Transport latency is non-negative and noisy, so the smallest delay seen in a
/// recent window is the best available estimate of the clock difference alone.
/// A sliding window keeps it tracking slow drift instead of locking onto one
/// lucky early packet forever.
public struct ClockOffsetEstimator: Sendable {
    /// Observations of `localArrival - rosStamp`, paired with their local time
    /// so the window can be aged out.
    private var window: [(localTime: Double, delay: Double)] = []

    /// How long an observation stays in the window, in seconds.
    public var windowDuration: Double

    /// Cap on retained observations, so a fast topic cannot grow the window.
    public var capacity: Int

    /// Current estimate of `localClock - rosClock`, in seconds, or `nil` before
    /// the first observation.
    public private(set) var offset: Double?

    public init(windowDuration: Double = 30.0, capacity: Int = 600) {
        self.windowDuration = max(1.0, windowDuration)
        self.capacity = max(1, capacity)
    }

    public var hasEstimate: Bool { offset != nil }

    /// Records one message arrival.
    ///
    /// - Parameters:
    ///   - rosStamp: the message's header stamp, in seconds since the ROS epoch.
    ///   - localTime: when the phone received it, on the phone's monotonic-ish
    ///     reference clock.
    public mutating func observe(rosStamp: Double, localTime: Double) {
        guard rosStamp.isFinite, localTime.isFinite, rosStamp > 0 else { return }

        window.append((localTime: localTime, delay: localTime - rosStamp))

        let cutoff = localTime - windowDuration
        if let firstKept = window.firstIndex(where: { $0.localTime >= cutoff }), firstKept > 0 {
            window.removeFirst(firstKept)
        }
        if window.count > capacity {
            window.removeFirst(window.count - capacity)
        }

        offset = window.map(\.delay).min()
    }

    /// Converts a phone timestamp to the robot's ROS timeline.
    ///
    /// Falls back to returning `localTime` unchanged when no estimate exists
    /// yet, which is correct on the rare setup where both clocks are already
    /// NTP-synchronised and harmless otherwise, since the buffer will simply
    /// clamp until the first message lands.
    public func rosTime(fromLocal localTime: Double) -> Double {
        guard let offset else { return localTime }
        return localTime - offset
    }

    /// Converts a ROS timestamp to the phone's timeline.
    public func localTime(fromROS rosTime: Double) -> Double {
        guard let offset else { return rosTime }
        return rosTime + offset
    }

    public mutating func reset() {
        window.removeAll(keepingCapacity: true)
        offset = nil
    }
}
