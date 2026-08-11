import Foundation

/// Whether the robot's `vio_node` is up, and what it is doing.
///
/// This answers a different question from `RobotTrackingStatus`. That type asks
/// "can I believe this pose"; this one asks "is the estimator running at all",
/// which is the first thing an operator needs when the marker is missing and it
/// is not obvious whether the node crashed, was never launched, or is simply
/// still collecting its stationary calibration samples.
///
/// **Why this is derived rather than queried.** The honest way to answer would
/// be `/rosapi/nodes`, but the robot launches `rosbridge_websocket` as a bare
/// node rather than through `rosbridge_websocket_launch.xml`, so `rosapi_node`
/// is never started and that service does not exist. The next best evidence is
/// the node's own output: `vio_node` publishes `/vio/odometry` at IMU rate the
/// moment it finishes initialising and stops the instant it dies, so traffic on
/// the topics it owns is a sound liveness proxy.
///
/// It also survives a detail of the real robot that would otherwise leave the
/// app permanently unsure: `/vio/calibrated` and `/vio/visual_tracking` are
/// published only on transition. A phone that connects after calibration
/// finished may never see either flag, so the status must not depend on them —
/// odometry alone is enough to conclude the node is running, because the node
/// publishes none until it is initialised.
public enum VIONodeStatus: Equatable, Sendable {
    /// Not connected to rosbridge, so nothing can be concluded.
    case unknown
    /// The dashboard reports the ROS launch is stopped. Nothing is running.
    case graphStopped
    /// Connected, the graph is up, but nothing has ever arrived on a VIO topic:
    /// the node is absent, crashed at start, or `start_vio` was false.
    case notRunning
    /// The node is up and has reported `/vio/calibrated = false`: it is
    /// collecting stationary samples and publishes no pose until it finishes.
    case calibrating
    /// The node was heard from, but odometry has stopped or never started.
    case silent(age: Double)
    /// Publishing odometry now.
    case running

    /// Whether the node can be assumed present, whatever it is doing.
    public var isNodePresent: Bool {
        switch self {
        case .calibrating, .silent, .running: return true
        case .unknown, .graphStopped, .notRunning: return false
        }
    }

    public var shortLabel: String {
        switch self {
        case .unknown: return "Unknown"
        case .graphStopped: return "Graph stopped"
        case .notRunning: return "Not running"
        case .calibrating: return "Calibrating"
        case .silent: return "Silent"
        case .running: return "Running"
        }
    }

    public var detailLabel: String {
        switch self {
        case .unknown:
            return "Not connected to rosbridge, so the VIO node cannot be seen."
        case .graphStopped:
            return "The ROS launch is stopped. Start it to bring up VIO."
        case .notRunning:
            return "Nothing has published on any /vio topic. Check that the launch ran with start_vio:=true and that vio_node did not exit."
        case .calibrating:
            return "VIO is collecting stationary samples. Keep the drone still; no pose is published until it finishes."
        case .silent(let age):
            return age.isFinite
                ? String(format: "The node was heard from but has published no odometry for %.1f s.", age)
                : "The node was heard from but has never published odometry."
        case .running:
            return "vio_node is publishing odometry."
        }
    }

    /// Derives the status from what the app has actually received.
    ///
    /// Order matters. The link is checked first because nothing else is
    /// meaningful without it, then the launch state, then evidence of the node
    /// existing at all, and only then what it is doing.
    ///
    /// - Parameters:
    ///   - isLinkConnected: whether rosbridge is currently connected.
    ///   - isGraphRunning: `GET /api/state`'s `running`, or `nil` when the
    ///     dashboard has not answered — including fixtures mode, where the
    ///     topics are the only evidence available.
    ///   - isCalibrated: latest `/vio/calibrated`, or `nil` if never received.
    ///   - vioMessageAge: seconds since the newest message on any topic
    ///     `vio_node` publishes, or `nil` if none has ever arrived.
    ///   - odometryAge: seconds since the newest `/vio/odometry`, or `nil`.
    ///   - silenceThreshold: how old odometry may be before the node counts as
    ///     silent.
    public static func evaluate(
        isLinkConnected: Bool,
        isGraphRunning: Bool?,
        isCalibrated: Bool?,
        vioMessageAge: Double?,
        odometryAge: Double?,
        silenceThreshold: Double = 2.0
    ) -> VIONodeStatus {
        guard isLinkConnected else { return .unknown }
        if isGraphRunning == false { return .graphStopped }
        guard vioMessageAge != nil || odometryAge != nil else { return .notRunning }

        // A `false` flag is the node telling us directly what it is doing, so it
        // outranks the odometry gap it is itself causing.
        if isCalibrated == false { return .calibrating }

        guard let odometryAge else { return .silent(age: .infinity) }
        if odometryAge > silenceThreshold { return .silent(age: odometryAge) }
        return .running
    }
}
