import Foundation

/// Thread-safe access to the odometry buffer, the clock estimate and the
/// alignment.
///
/// The SceneKit render loop runs on its own thread and needs the robot's pose
/// for the exact instant it is about to draw. It cannot hop to the main actor
/// to get it without adding a frame of latency and a data race, so the pieces
/// the renderer needs live here behind a lock instead of inside the
/// main-actor-isolated `RobotConnection`.
public final class OdometrySampler: @unchecked Sendable {
    private let lock = NSLock()
    private var buffer: OdometryBuffer
    private var clock = ClockOffsetEstimator()
    private var alignment = RobotAlignment.identity
    private var hasAlignmentValue = false
    private var extrapolationLimit: Double = 0

    public init(buffer: OdometryBuffer = OdometryBuffer()) {
        self.buffer = buffer
    }

    // MARK: - Ingest

    /// Records an odometry message and the phone time it arrived.
    public func append(_ sample: StampedPose, localTime: Double) {
        lock.lock()
        defer { lock.unlock() }
        buffer.append(sample)
        clock.observe(rosStamp: sample.stamp, localTime: localTime)
    }

    /// Feeds a timestamp from a non-odometry topic into the clock estimate.
    /// More observations across more topics make the minimum-delay filter
    /// converge faster.
    public func observeClock(rosStamp: Double, localTime: Double) {
        lock.lock()
        defer { lock.unlock() }
        clock.observe(rosStamp: rosStamp, localTime: localTime)
    }

    public func reset() {
        lock.lock()
        defer { lock.unlock() }
        buffer.reset()
        clock.reset()
    }

    // MARK: - Configuration

    public func setAlignment(_ alignment: RobotAlignment?) {
        lock.lock()
        defer { lock.unlock() }
        if let alignment {
            self.alignment = alignment
            hasAlignmentValue = true
        } else {
            self.alignment = .identity
            hasAlignmentValue = false
        }
    }

    public func setExtrapolationLimit(_ limit: Double) {
        lock.lock()
        defer { lock.unlock() }
        extrapolationLimit = max(0, limit)
    }

    public func setStalenessThreshold(_ threshold: Double) {
        lock.lock()
        defer { lock.unlock() }
        buffer.stalenessThreshold = threshold
    }

    public var hasAlignment: Bool {
        lock.lock()
        defer { lock.unlock() }
        return hasAlignmentValue
    }

    public var currentAlignment: RobotAlignment? {
        lock.lock()
        defer { lock.unlock() }
        return hasAlignmentValue ? alignment : nil
    }

    public var clockOffset: Double? {
        lock.lock()
        defer { lock.unlock() }
        return clock.offset
    }

    // MARK: - Sampling

    /// The robot's pose in the ROS `odom` frame at a phone-clock instant.
    public func poseInOdom(atLocalTime localTime: Double) -> PoseSample? {
        lock.lock()
        defer { lock.unlock() }
        let rosTime = clock.rosTime(fromLocal: localTime)
        return buffer.pose(at: rosTime, maxExtrapolation: extrapolationLimit)
    }

    /// The robot's pose in ARKit world coordinates, with alignment applied.
    /// Returns `nil` when there is no odometry or no alignment.
    public func poseInAR(atLocalTime localTime: Double) -> (pose: Pose, sample: PoseSample)? {
        lock.lock()
        defer { lock.unlock() }
        guard hasAlignmentValue else { return nil }
        let rosTime = clock.rosTime(fromLocal: localTime)
        guard let sample = buffer.pose(at: rosTime, maxExtrapolation: extrapolationLimit) else {
            return nil
        }
        return (alignment.arPose(fromROSOdometry: sample.pose), sample)
    }

    /// Seconds since the newest odometry message, on the ROS timeline.
    public func age(atLocalTime localTime: Double) -> Double? {
        lock.lock()
        defer { lock.unlock() }
        return buffer.age(at: clock.rosTime(fromLocal: localTime))
    }

    /// Recent poses in the `odom` frame, decimated for drawing a trail.
    public func trailInOdom(maximumCount: Int = 120) -> [Pose] {
        lock.lock()
        defer { lock.unlock() }
        let samples = buffer.samples
        guard samples.count > maximumCount, maximumCount > 0 else { return samples.map(\.pose) }
        let step = max(1, samples.count / maximumCount)
        return stride(from: 0, to: samples.count, by: step).map { samples[$0].pose }
    }

    /// Recent poses already transformed into ARKit world coordinates.
    public func trailInAR(maximumCount: Int = 120) -> [Vector3] {
        lock.lock()
        defer { lock.unlock() }
        guard hasAlignmentValue else { return [] }
        let samples = buffer.samples
        let step = samples.count > maximumCount && maximumCount > 0
            ? max(1, samples.count / maximumCount)
            : 1
        return stride(from: 0, to: samples.count, by: step).map {
            alignment.arPose(fromROSOdometry: samples[$0].pose).position
        }
    }

    public var sampleCount: Int {
        lock.lock()
        defer { lock.unlock() }
        return buffer.count
    }
}
