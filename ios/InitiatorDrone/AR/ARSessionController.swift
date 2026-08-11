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

    /// Which physical lens the session is running on, for the settings and
    /// diagnostics screens.
    @Published public private(set) var lensLabel: String = "Not started"
    /// Resolution and frame rate of the chosen video format, or empty.
    @Published public private(set) var videoFormatLabel: String = ""
    /// Whether the ultra-wide lens is actually in use. False on devices that do
    /// not offer an ultra-wide ARKit video format, whatever the preference says.
    @Published public private(set) var isUsingUltraWide: Bool = false
    /// Whether this device offers an ultra-wide format at all, so the settings
    /// screen can say why the toggle did nothing.
    public let supportsUltraWide: Bool = ARSessionController.ultraWideFormats().isEmpty == false

    /// Operator preference. The session honours it only when the device has an
    /// ultra-wide format to switch to.
    private var prefersUltraWide = true

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

    /// Applies the operator's lens preference.
    ///
    /// The video format is fixed for the life of a `run(_:)`, so switching lens
    /// means restarting tracking — which moves the world origin and therefore
    /// invalidates any alignment. The caller is responsible for clearing it,
    /// the same as for `resetTracking()`.
    ///
    /// - Returns: whether the session was restarted.
    @discardableResult
    public func setPrefersUltraWide(_ prefers: Bool) -> Bool {
        guard prefers != prefersUltraWide else { return false }
        prefersUltraWide = prefers
        guard isRunning, supportsUltraWide else {
            // Nothing to restart, but the labels should still reflect what the
            // next session will do.
            if !isRunning { describeLens(nil) }
            return false
        }
        session.run(makeConfiguration(), options: [.resetTracking, .removeExistingAnchors])
        return true
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

        if prefersUltraWide, let format = Self.ultraWideFormats().first {
            configuration.videoFormat = format
        }
        describeLens(configuration.videoFormat)
        return configuration
    }

    /// The device's ultra-wide world-tracking video formats, best first.
    ///
    /// ARKit publishes `supportedVideoFormats` in its own preference order and
    /// the first entry is the one it would have picked, so filtering rather than
    /// re-ranking keeps Apple's choice of resolution and frame rate intact and
    /// only overrides the lens. The list is empty on every device without an
    /// ultra-wide camera exposed to ARKit, which is what makes this safe to ask
    /// for unconditionally.
    private static func ultraWideFormats() -> [ARConfiguration.VideoFormat] {
        ARWorldTrackingConfiguration.supportedVideoFormats.filter {
            $0.captureDeviceType == .builtInUltraWideCamera
        }
    }

    private func describeLens(_ format: ARConfiguration.VideoFormat?) {
        guard let format else {
            lensLabel = supportsUltraWide && prefersUltraWide ? "Ultra-wide (pending)" : "Wide (pending)"
            videoFormatLabel = ""
            isUsingUltraWide = false
            return
        }

        let ultraWide = format.captureDeviceType == .builtInUltraWideCamera
        isUsingUltraWide = ultraWide
        if ultraWide {
            lensLabel = "Ultra-wide"
        } else if prefersUltraWide && !supportsUltraWide {
            lensLabel = "Wide (no ultra-wide on this device)"
        } else {
            lensLabel = "Wide"
        }
        videoFormatLabel = String(
            format: "%.0fx%.0f @ %ld fps",
            format.imageResolution.width,
            format.imageResolution.height,
            format.framesPerSecond
        )
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
