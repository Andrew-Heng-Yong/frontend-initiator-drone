import Foundation

/// How a pose was produced for a requested time.
public enum PoseSampleKind: Equatable, Sendable {
    /// The request landed between two buffered samples and was interpolated.
    case interpolated
    /// The request matched a buffered sample time exactly.
    case exact
    /// The request was older than everything buffered; the oldest sample was
    /// returned unchanged.
    case clampedToOldest
    /// The request was newer than everything buffered and extrapolation was not
    /// allowed (or not permitted this far); the newest sample was returned.
    case clampedToNewest
    /// The request was newer than everything buffered and was extrapolated
    /// forward using the odometry twist.
    case extrapolated
}

/// The result of asking the buffer for a pose at a particular time.
public struct PoseSample: Equatable, Sendable {
    public var pose: Pose
    public var kind: PoseSampleKind
    /// How far the requested time sits beyond the newest buffered sample, in
    /// seconds. Zero when the request fell inside the buffered span.
    public var ageBeyondNewest: Double

    public init(pose: Pose, kind: PoseSampleKind, ageBeyondNewest: Double) {
        self.pose = pose
        self.kind = kind
        self.ageBeyondNewest = ageBeyondNewest
    }
}

/// A time-ordered ring of robot odometry poses that can be sampled at an
/// arbitrary instant.
///
/// Odometry arrives at whatever rate the robot publishes, while rendering runs
/// at the display refresh rate. Drawing the most recent message directly makes
/// the marker jitter and lag; sampling this buffer at the render frame's
/// timestamp gives a smooth, correctly timed pose.
///
/// The buffer bounds itself by both sample count and time span so a long
/// session cannot grow without limit.
public struct OdometryBuffer: Sendable {
    /// Samples in ascending stamp order.
    public private(set) var samples: [StampedPose] = []

    /// Hard cap on retained samples.
    public var capacity: Int

    /// Samples older than this many seconds behind the newest one are dropped.
    public var historyDuration: Double

    /// A pose older than this is reported as stale by `isStale(at:)`.
    public var stalenessThreshold: Double

    public init(capacity: Int = 240, historyDuration: Double = 5.0, stalenessThreshold: Double = 0.5) {
        self.capacity = max(2, capacity)
        self.historyDuration = max(0.1, historyDuration)
        self.stalenessThreshold = max(0.0, stalenessThreshold)
    }

    public var isEmpty: Bool { samples.isEmpty }
    public var count: Int { samples.count }
    public var newest: StampedPose? { samples.last }
    public var oldest: StampedPose? { samples.first }

    /// Time span currently covered by the buffer, in seconds.
    public var span: Double {
        guard let first = samples.first, let last = samples.last else { return 0 }
        return last.stamp - first.stamp
    }

    /// Inserts a sample, keeping the buffer sorted.
    ///
    /// The common case is a sample newer than everything held, which appends in
    /// constant time. Out-of-order arrivals are inserted at the right index
    /// rather than dropped, because rosbridge does not guarantee ordering
    /// across a reconnect. A sample whose stamp duplicates an existing one
    /// replaces it, so a republished message cannot create a zero-length
    /// interpolation interval.
    public mutating func append(_ sample: StampedPose) {
        guard sample.stamp.isFinite, sample.pose.isFinite else { return }

        if let last = samples.last, sample.stamp > last.stamp {
            samples.append(sample)
        } else if samples.isEmpty {
            samples.append(sample)
        } else {
            let index = insertionIndex(for: sample.stamp)
            if index < samples.count && samples[index].stamp == sample.stamp {
                samples[index] = sample
            } else {
                samples.insert(sample, at: index)
            }
        }

        prune()
    }

    /// Drops every buffered sample. Called on disconnect so a stale pose from a
    /// previous session cannot be drawn against a fresh one.
    public mutating func reset() {
        samples.removeAll(keepingCapacity: true)
    }

    private mutating func prune() {
        if let newestStamp = samples.last?.stamp {
            let cutoff = newestStamp - historyDuration
            if let firstKept = samples.firstIndex(where: { $0.stamp >= cutoff }), firstKept > 0 {
                samples.removeFirst(firstKept)
            }
        }
        if samples.count > capacity {
            samples.removeFirst(samples.count - capacity)
        }
    }

    /// Binary search for the index where `stamp` belongs.
    private func insertionIndex(for stamp: Double) -> Int {
        var low = 0
        var high = samples.count
        while low < high {
            let mid = (low + high) / 2
            if samples[mid].stamp < stamp {
                low = mid + 1
            } else {
                high = mid
            }
        }
        return low
    }

    /// Samples the buffer at `time`, in the same clock as the buffered stamps.
    ///
    /// - Parameter maxExtrapolation: how far past the newest sample the twist
    ///   may be integrated forward, in seconds. Zero disables extrapolation and
    ///   clamps to the newest pose instead, which is the safe default for a
    ///   diagnostics tool: a clamped marker stops moving, while an
    ///   over-extrapolated one drifts away convincingly and lies.
    public func pose(at time: Double, maxExtrapolation: Double = 0) -> PoseSample? {
        guard let first = samples.first, let last = samples.last else { return nil }

        if time <= first.stamp {
            let kind: PoseSampleKind = time == first.stamp ? .exact : .clampedToOldest
            return PoseSample(pose: first.pose, kind: kind, ageBeyondNewest: 0)
        }

        if time >= last.stamp {
            let age = time - last.stamp
            if age == 0 {
                return PoseSample(pose: last.pose, kind: .exact, ageBeyondNewest: 0)
            }
            if maxExtrapolation > 0 && age <= maxExtrapolation {
                return PoseSample(
                    pose: extrapolate(last, by: age),
                    kind: .extrapolated,
                    ageBeyondNewest: age
                )
            }
            return PoseSample(pose: last.pose, kind: .clampedToNewest, ageBeyondNewest: age)
        }

        // `time` is strictly inside the buffered span, so there is an index
        // whose stamp is >= time and a predecessor whose stamp is < time.
        let upper = insertionIndex(for: time)
        let after = samples[upper]
        if after.stamp == time {
            return PoseSample(pose: after.pose, kind: .exact, ageBeyondNewest: 0)
        }
        let before = samples[upper - 1]

        let interval = after.stamp - before.stamp
        guard interval > 0 else {
            return PoseSample(pose: after.pose, kind: .exact, ageBeyondNewest: 0)
        }
        let t = (time - before.stamp) / interval
        return PoseSample(
            pose: Pose.interpolate(before.pose, after.pose, t),
            kind: .interpolated,
            ageBeyondNewest: 0
        )
    }

    /// Integrates a sample's body-frame twist forward by `dt` seconds.
    private func extrapolate(_ sample: StampedPose, by dt: Double) -> Pose {
        let linearInParent = sample.pose.orientation.rotate(sample.linearVelocity * dt)
        let angularMagnitude = sample.angularVelocity.length
        let rotation: Quaternion
        if angularMagnitude > 1e-9 {
            let delta = Quaternion(axis: sample.angularVelocity, angle: angularMagnitude * dt)
            rotation = (sample.pose.orientation * delta).normalized
        } else {
            rotation = sample.pose.orientation
        }
        return Pose(position: sample.pose.position + linearInParent, orientation: rotation)
    }

    /// Whether the newest sample is older than `stalenessThreshold` relative to
    /// `time`. An empty buffer counts as stale.
    public func isStale(at time: Double) -> Bool {
        guard let last = samples.last else { return true }
        return (time - last.stamp) > stalenessThreshold
    }

    /// Seconds between the newest sample and `time`, or `nil` when empty.
    public func age(at time: Double) -> Double? {
        guard let last = samples.last else { return nil }
        return time - last.stamp
    }
}
