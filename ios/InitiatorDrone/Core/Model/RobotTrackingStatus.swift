import Foundation

/// Whether the robot's pose can currently be believed.
///
/// The distinction this type exists to enforce: `/vio/odometry` continuing to
/// publish proves only that the VIO node is alive. A visual-inertial estimator
/// that has lost its features keeps dead-reckoning off the IMU and keeps
/// publishing a pose that drifts away from reality, smoothly and convincingly.
/// So a pose is only trustworthy when the estimator says it is calibrated
/// *and* says visual tracking is active *and* the messages are fresh.
public enum RobotTrackingStatus: Equatable, Sendable {
    /// No status flags received yet.
    case unknown
    /// `/vio/calibrated` is false. The pose has no meaningful origin.
    case notCalibrated
    /// Calibrated, but `/vio/visual_tracking` is false: the estimator is
    /// coasting on inertial data and drifting.
    case visualTrackingLost
    /// Flags look good but odometry has stopped arriving.
    case stale(age: Double)
    /// Calibrated, visually tracking, and fresh.
    case tracking

    /// Whether the robot marker should be drawn as a live pose.
    public var isTrustworthy: Bool { self == .tracking }

    /// Whether a pose exists at all, even if it should be drawn as suspect.
    /// A lost-tracking pose is still worth showing, greyed out, because it
    /// tells the operator where the robot was when tracking failed.
    public var hasUsablePose: Bool {
        switch self {
        case .tracking, .visualTrackingLost, .stale: return true
        case .unknown, .notCalibrated: return false
        }
    }

    public var shortLabel: String {
        switch self {
        case .unknown: return "No data"
        case .notCalibrated: return "Not calibrated"
        case .visualTrackingLost: return "Tracking lost"
        case .stale: return "Stale"
        case .tracking: return "Tracking"
        }
    }

    public var detailLabel: String {
        switch self {
        case .unknown:
            return "Waiting for VIO status."
        case .notCalibrated:
            return "VIO has not been calibrated. Hold the robot still and calibrate."
        case .visualTrackingLost:
            return "VIO is calibrated but visual tracking is inactive; the pose is dead-reckoned and drifting."
        case .stale(let age):
            return String(format: "No odometry for %.1f s.", age)
        case .tracking:
            return "VIO calibrated and visually tracking."
        }
    }

    /// Derives the status from the three independent signals.
    ///
    /// Order matters and is deliberate: staleness is checked before the flags,
    /// because a stale `visual_tracking = true` from thirty seconds ago is not
    /// evidence of anything.
    ///
    /// - Parameters:
    ///   - isCalibrated: latest `/vio/calibrated`, or `nil` if never received.
    ///   - isVisualTracking: latest `/vio/visual_tracking`, or `nil`.
    ///   - odometryAge: seconds since the newest `/vio/odometry`, or `nil` if
    ///     none has arrived.
    ///   - stalenessThreshold: how old odometry may be before it is stale.
    public static func evaluate(
        isCalibrated: Bool?,
        isVisualTracking: Bool?,
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
        guard let isVisualTracking else { return .unknown }
        return isVisualTracking ? .tracking : .visualTrackingLost
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
