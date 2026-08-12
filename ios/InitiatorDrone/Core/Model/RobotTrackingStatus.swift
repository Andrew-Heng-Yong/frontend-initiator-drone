import Foundation

/// Whether the robot's pose can currently be believed, and how much of it.
///
/// The distinction this type exists to enforce: `/odom` continuing to publish
/// proves only that `odom_node` is alive. An estimator keeps publishing a pose
/// whether or not it has anything to base it on — smoothly, convincingly, and
/// in the case of gyro integration, drifting the whole time.
///
/// Since the move from `vio_node` to `odom_node` there is a second and blunter
/// reason not to believe a pose in full: **position is not estimated at all**.
/// The node integrates gyro only, holds translation at zero, and flags it with
/// a 1e6 m² variance. So the interesting question stopped being "is vision
/// working" and became "how much of this pose is measured".
public enum RobotTrackingStatus: Equatable, Sendable {
    /// No status flags received yet.
    case unknown
    /// `/odom/calibrated` is false. The gyro bias is still being estimated and
    /// the orientation has no meaningful reference.
    case notCalibrated
    /// Calibrated and fresh, but the publisher marks position as unobserved:
    /// the orientation is real, the position is a placeholder at the origin.
    /// This is the normal state with `odom_node`.
    case orientationOnly
    /// Flags look good but odometry has stopped arriving.
    case stale(age: Double)
    /// Calibrated, fresh, and the publisher claims to know where the robot is.
    /// Unreachable until a translation source (flow sensor, GPS) is added.
    case tracking

    /// Whether the full pose — position included — should be believed.
    public var isTrustworthy: Bool { self == .tracking }

    /// Whether the orientation can be believed, whatever the position is doing.
    public var isOrientationTrustworthy: Bool {
        self == .tracking || self == .orientationOnly
    }

    /// Whether a pose exists at all, even if it should be drawn as suspect.
    /// A stale or position-less pose is still worth showing, greyed out: it
    /// tells the operator which way the robot was facing.
    public var hasUsablePose: Bool {
        switch self {
        case .tracking, .orientationOnly, .stale: return true
        case .unknown, .notCalibrated: return false
        }
    }

    public var shortLabel: String {
        switch self {
        case .unknown: return "No data"
        case .notCalibrated: return "Not calibrated"
        case .orientationOnly: return "Heading only"
        case .stale: return "Stale"
        case .tracking: return "Tracking"
        }
    }

    public var detailLabel: String {
        switch self {
        case .unknown:
            return "Waiting for odometry status."
        case .notCalibrated:
            return "The gyro is not calibrated. Hold the robot still and calibrate."
        case .orientationOnly:
            return "Heading is live, position is not measured. odom_node integrates the gyro only, so the marker holds the position you aligned it to."
        case .stale(let age):
            return String(format: "No odometry for %.1f s.", age)
        case .tracking:
            return "Calibrated, fresh, and reporting a measured position."
        }
    }

    /// Derives the status from the three independent signals.
    ///
    /// Order matters and is deliberate: staleness is checked before the flags,
    /// because a `calibrated = true` from thirty seconds ago is not evidence
    /// that anything is running now.
    ///
    /// - Parameters:
    ///   - isCalibrated: latest `/odom/calibrated`, or `nil` if never received.
    ///   - isPositionObserved: whether the newest odometry message claims a
    ///     usable position, from its covariance. `nil` if none has arrived.
    ///   - odometryAge: seconds since the newest `/odom`, or `nil` if none has
    ///     arrived.
    ///   - stalenessThreshold: how old odometry may be before it is stale.
    public static func evaluate(
        isCalibrated: Bool?,
        isPositionObserved: Bool?,
        odometryAge: Double?,
        stalenessThreshold: Double = 0.5
    ) -> RobotTrackingStatus {
        guard let odometryAge else {
            guard let isCalibrated else { return .unknown }
            return isCalibrated ? .stale(age: .infinity) : .notCalibrated
        }

        if odometryAge > stalenessThreshold {
            return .stale(age: odometryAge)
        }
        guard let isCalibrated else { return .unknown }
        guard isCalibrated else { return .notCalibrated }
        guard let isPositionObserved else { return .unknown }
        return isPositionObserved ? .tracking : .orientationOnly
    }
}

/// Whether the robot's `odom` frame has been tied to the AR world yet.
public enum AlignmentStatus: Equatable, Sendable {
    case notAligned
    case aligned(at: Date)

    public var isAligned: Bool {
        if case .aligned = self { return true }
        return false
    }
}
