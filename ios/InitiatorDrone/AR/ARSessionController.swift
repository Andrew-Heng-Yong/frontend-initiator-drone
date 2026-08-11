#if canImport(ARKit)
import ARKit
import Combine
import SwiftUI

/// Owns the ARKit session and publishes the phone's pose.
///
/// ARKit is the app's only source of phone motion. `CoreMotion` acceleration is
/// deliberately not integrated anywhere: double-integrating accelerometer data
/// produces metres of drift within seconds, while ARKit's visual-inertial fusion
/// is already doing that job correctly with the camera in the loop.
@MainActor
public final class ARSessionController: NSObject, ObservableObject {

    @Published public private(set) var isSupported: Bool = ARWorldTrackingConfiguration.isSupported
    @Published public private(set) var isRunning: Bool = false
    @Published public private(set) var trackingStateLabel: String = "Not started"
    @Published public private(set) var trackingStateDetail: String = ""
    @Published public private(set) var isTrackingUsable: Bool = false

    /// Resolution and frame rate of the chosen video format, or empty.
    @Published public private(set) var videoFormatLabel: String = ""
    /// Index into `supportedFormatSummaries` of the format actually running.
    @Published public private(set) var activeFormatIndex: Int?

    /// Every world-tracking format this device offers, in ARKit's own order,
    /// described for the diagnostics screen. Claims about what the hardware
    /// will and will not do should be checkable on the hardware.
    public let supportedFormatSummaries: [String] = ARWorldTrackingConfiguration
        .supportedVideoFormats
        .map(ARSessionController.summarise)

    /// Phone pose in ARKit world coordinates, republished a few times a second
    /// for the numeric readouts. The scene renderer reads `currentPose`
    /// directly at render time instead, so nothing is throttled that matters.
    @Published public private(set) var phonePose: Pose = .identity

    /// Live phone pose, updated on every ARKit frame.
    public private(set) var currentPose: Pose = .identity
    /// Timestamp of the most recent ARKit frame, on ARKit's own clock.
    public private(set) var currentFrameTime: TimeInterval = 0

    public let session = ARSession()

    private var lastPublish: TimeInterval = 0
    private let publishInterval: TimeInterval = 0.1

    public override init() {
        super.init()
        session.delegate = self
    }

    public func start() {
        guard ARWorldTrackingConfiguration.isSupported else {
            isSupported = false
            trackingStateLabel = "Unsupported"
            trackingStateDetail = "This device does not support ARKit world tracking."
            return
        }

        // Re-assert the delegate: the session outlives any particular view, and
        // attaching it to an ARSCNView is the kind of thing that could replace
        // it. Losing it silently would freeze `currentPose` while the camera
        // feed carried on looking perfectly healthy.
        session.delegate = self
        session.run(makeConfiguration(), options: [.resetTracking, .removeExistingAnchors])
        isRunning = true
    }

    public func pause() {
        session.pause()
        isRunning = false
        trackingStateLabel = "Paused"
        trackingStateDetail = ""
        isTrackingUsable = false
    }

    /// Restarts tracking from a fresh origin. Any existing alignment refers to
    /// the old origin and is invalidated by the caller.
    public func resetTracking() {
        guard isRunning else {
            start()
            return
        }
        session.run(makeConfiguration(), options: [.resetTracking, .removeExistingAnchors])
    }

    // MARK: - Configuration

    private func makeConfiguration() -> ARWorldTrackingConfiguration {
        let configuration = ARWorldTrackingConfiguration()
        // `.gravity` keeps +Y along true up without waiting on the compass.
        // `.gravityAndHeading` would tie -Z to north, which sounds useful for
        // aligning to a robot's odom frame but in practice indoor magnetic
        // heading is bad enough to make manual alignment the better default.
        configuration.worldAlignment = .gravity
        configuration.planeDetection = [.horizontal]
        configuration.environmentTexturing = .none
        // Nothing here needs a depth map from the phone; leaving scene
        // reconstruction off saves a large amount of power and thermal budget
        // on a session that is expected to run for half an hour.
        configuration.isAutoFocusEnabled = true

        // Always the widest format available. Nothing would be gained by
        // offering a cropped one, so this is not a setting.
        if let format = Self.widestFormat() {
            configuration.videoFormat = format
        }
        describeFormat(configuration.videoFormat)
        return configuration
    }

    /// The format showing the most of the room. See `VideoFormatSelection` for
    /// the rule; the ranking lives there so it can be tested without a device.
    private static func widestFormat() -> ARConfiguration.VideoFormat? {
        let formats = ARWorldTrackingConfiguration.supportedVideoFormats
        let candidates = formats.map { format in
            VideoFormatCandidate(
                width: Int(format.imageResolution.width),
                height: Int(format.imageResolution.height),
                framesPerSecond: format.framesPerSecond
            )
        }
        guard let index = VideoFormatSelection.widestFieldOfView(among: candidates) else {
            return nil
        }
        return formats[index]
    }

    private static func summarise(_ format: ARConfiguration.VideoFormat) -> String {
        String(
            format: "%.0fx%.0f @ %ld fps",
            format.imageResolution.width,
            format.imageResolution.height,
            format.framesPerSecond
        )
    }

    private func describeFormat(_ format: ARConfiguration.VideoFormat?) {
        guard let format else {
            videoFormatLabel = ""
            activeFormatIndex = nil
            return
        }
        activeFormatIndex = ARWorldTrackingConfiguration.supportedVideoFormats
            .firstIndex(of: format)
        videoFormatLabel = Self.summarise(format)
    }

    /// Raycasts from a point in view coordinates onto a horizontal surface.
    /// Used by "place on surface" alignment.
    public func raycastHorizontal(from point: CGPoint, in view: ARSCNView) -> Pose? {
        guard let query = view.raycastQuery(
            from: point,
            allowing: .estimatedPlane,
            alignment: .horizontal
        ) else { return nil }
        guard let result = session.raycast(query).first else { return nil }
        return ARKitBridge.pose(from: result.worldTransform)
    }
}

extension ARSessionController: ARSessionDelegate {
    nonisolated public func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let pose = ARKitBridge.pose(from: frame.camera.transform)
        let timestamp = frame.timestamp

        Task { @MainActor [weak self] in
            guard let self else { return }
            self.currentPose = pose
            self.currentFrameTime = timestamp

            // The published copy drives text labels only; republishing at 60 Hz
            // would re-render the whole SwiftUI tree for no benefit.
            if timestamp - self.lastPublish >= self.publishInterval {
                self.lastPublish = timestamp
                self.phonePose = pose
            }
        }
    }

    nonisolated public func session(_ session: ARSession, cameraDidChangeTrackingState camera: ARCamera) {
        let state = camera.trackingState
        Task { @MainActor [weak self] in
            self?.trackingStateLabel = state.shortLabel
            self?.trackingStateDetail = state.detailLabel
            self?.isTrackingUsable = state.isUsable
        }
    }

    nonisolated public func session(_ session: ARSession, didFailWithError error: Error) {
        Task { @MainActor [weak self] in
            self?.isRunning = false
            self?.trackingStateLabel = "Failed"
            self?.trackingStateDetail = error.localizedDescription
            self?.isTrackingUsable = false
        }
    }

    nonisolated public func sessionWasInterrupted(_ session: ARSession) {
        Task { @MainActor [weak self] in
            self?.trackingStateLabel = "Interrupted"
            self?.trackingStateDetail = "The AR session paused. Alignment may no longer be valid."
            self?.isTrackingUsable = false
        }
    }

    nonisolated public func sessionInterruptionEnded(_ session: ARSession) {
        Task { @MainActor [weak self] in
            self?.trackingStateLabel = "Resuming"
            self?.trackingStateDetail = "Re-establishing tracking. Check the robot alignment."
        }
    }
}
#endif
