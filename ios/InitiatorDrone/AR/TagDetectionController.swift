#if canImport(ARKit)
import ARKit
import Combine
import CoreVideo
import Foundation

/// Runs the AprilTag detector on the ARKit camera feed and turns accepted
/// sightings into robot alignments.
///
/// This is the piece that makes the robot's position observable at all. Nothing
/// on the robot measures translation — `odom_node` integrates the gyro and holds
/// position at zero — so until now the marker sat wherever a human put it and
/// drifted from that moment. A tag on the robot, seen by the phone, is an
/// absolute fix in the phone's own world frame.
///
/// ## Keeping up with the camera
///
/// ARKit delivers 60 frames a second and a detection pass costs far more than
/// 16 ms. Frames are therefore taken **latest-only**: a frame arriving while the
/// worker is busy replaces the pending one rather than queueing behind it. A
/// backlog would be worse than useless here — a pose computed from a frame two
/// seconds old is composed with a phone pose two seconds old, and the answer is
/// wrong by however far the operator has walked since.
///
/// The luma plane is copied out of the pixel buffer before the frame is
/// released, and nothing downstream touches ARKit or Core Video.
@MainActor
public final class TagDetectionController: ObservableObject {

    /// What the app is currently getting out of the tag pipeline. Drives the
    /// status pill, and is deliberately specific: "no tags configured" and
    /// "confirming" are entirely different problems for the operator.
    public enum Status: Equatable {
        case disabled
        case noTagsConfigured
        case searching
        case confirming(tagID: Int, count: Int, required: Int)
        case fixed(tagID: Int, range: Double, at: Date)
        case rejected(String)

        public var shortLabel: String {
            switch self {
            case .disabled: return "Off"
            case .noTagsConfigured: return "No tags"
            case .searching: return "Searching"
            case .confirming: return "Confirming"
            case .fixed: return "Fixed"
            case .rejected: return "Rejected"
            }
        }

        public var detailLabel: String {
            switch self {
            case .disabled:
                return "Tag relocalisation is switched off in Settings ▸ AprilTags."
            case .noTagsConfigured:
                return "No AprilTags are configured. Add one in Settings ▸ AprilTags with its ID, size and offset."
            case .searching:
                return "Looking for a configured tag. Point the phone at the robot."
            case .confirming(let tagID, let count, let required):
                return "Confirming tag \(tagID) (\(count)/\(required) frames)."
            case .fixed(let tagID, let range, let at):
                let age = Date().timeIntervalSince(at)
                return String(format: "Fixed on tag %d at %.2f m, %.0f s ago.", tagID, range, age)
            case .rejected(let reason):
                return reason
            }
        }
    }

    @Published public private(set) var status: Status = .searching
    @Published public private(set) var lastFixAt: Date?
    @Published public private(set) var lastFixTagID: Int?
    @Published public private(set) var lastFixRange: Double?
    @Published public private(set) var lastRobotPoseInAR: Pose?
    /// Detections per second, measured on the phone.
    @Published public private(set) var detectionRateHz: Double = 0
    @Published public private(set) var framesProcessed: Int = 0
    @Published public private(set) var framesDropped: Int = 0

    /// Called on the main actor when a sighting is accepted and believed.
    public var onFix: ((_ robotPoseInAR: Pose, _ tagID: Int, _ range: Double, _ phonePose: Pose) -> Void)?

    private var settings: AppSettings
    private var gate: TagFixGate
    private let worker = DispatchQueue(label: "com.initiatordrone.apriltag", qos: .userInitiated)
    /// The slot owns the drain loop: `offer` reports whether a worker needs
    /// starting, and `take` returning nil is what ends it. Keeping a separate
    /// "busy" flag alongside it would be a second source of truth for the same
    /// thing, and the two would disagree the first time a frame arrived in the
    /// window between the last `take` and the worker finishing.
    private let pending = LatestOnlySlot<PendingFrame>()
    private var rate = RateTracker(windowDuration: 3.0)

    /// A frame lifted out of ARKit and detached from it.
    private struct PendingFrame {
        var image: GrayImage
        var intrinsics: CameraIntrinsics
        var phonePose: Pose
    }

    public init(settings: AppSettings = AppSettings()) {
        self.settings = settings
        self.gate = TagFixGate(settings: settings.tagLocalization)
        refreshIdleStatus()
    }

    public func apply(settings newSettings: AppSettings) {
        let tagsChanged = newSettings.aprilTags != settings.aprilTags
            || newSettings.tagLocalization != settings.tagLocalization
        settings = newSettings
        gate.settings = newSettings.tagLocalization
        // Evidence gathered under the old configuration says nothing about the
        // new one, so the streak starts again rather than carrying over.
        if tagsChanged { gate.reset() }
        refreshIdleStatus()
    }

    /// Clears everything a fix was based on. Called when the AR session resets,
    /// because world coordinates from before the reset mean nothing after it.
    public func reset() {
        gate.reset()
        pending.reset()
        lastFixAt = nil
        lastFixTagID = nil
        lastFixRange = nil
        lastRobotPoseInAR = nil
        rate.reset()
        framesProcessed = 0
        framesDropped = 0
        refreshIdleStatus()
    }

    private func refreshIdleStatus() {
        guard settings.tagLocalization.isEnabled else {
            status = .disabled
            return
        }
        guard !settings.tagSizesByID.isEmpty else {
            status = .noTagsConfigured
            return
        }
        if case .fixed = status { return }
        status = .searching
    }

    // MARK: - Frame intake

    /// Offers an ARKit frame to the detector.
    ///
    /// Cheap to call at 60 Hz and safe to call from the session delegate: it
    /// copies the luma plane and returns. Everything expensive happens on the
    /// worker.
    public func submit(frame: ARFrame) {
        guard settings.isTagLocalizationUsable else { return }
        guard let image = GrayImage(lumaOf: frame.capturedImage) else { return }

        let intrinsics = frame.camera.intrinsics
        let referenceSize = frame.camera.imageResolution
        // ARKit reports intrinsics against `imageResolution`; the captured
        // buffer is normally the same size, but scaling by the actual buffer
        // keeps them consistent if it ever is not.
        let scale = Double(image.width) / Double(referenceSize.width)
        let scaled = CameraIntrinsics(
            fx: Double(intrinsics.columns.0.x) * scale,
            fy: Double(intrinsics.columns.1.y) * scale,
            cx: Double(intrinsics.columns.2.x) * scale,
            cy: Double(intrinsics.columns.2.y) * scale
        )
        guard scaled.isUsable else { return }

        let candidate = PendingFrame(
            image: image,
            intrinsics: scaled,
            phonePose: ARKitBridge.pose(from: frame.camera.transform)
        )
        if pending.offer(candidate) { schedule() }
        framesDropped = pending.statistics.dropped
    }

    private func schedule() {
        guard let frame = pending.take() else { return }

        let tagSizes = settings.tagSizesByID
        let options = detectorOptions
        worker.async { [weak self] in
            let detections = AprilTagDetector.detect(
                in: frame.image,
                intrinsics: frame.intrinsics,
                tagSizes: tagSizes,
                options: options
            )
            Task { @MainActor [weak self] in
                self?.finish(detections: detections, phonePose: frame.phonePose)
            }
        }
    }

    /// Detector thresholds that mirror the localisation policy.
    ///
    /// Applying the margin and reprojection limits inside the detector as well
    /// as in the gate is not redundant: the detector drops a bad candidate
    /// before it costs a pose solve, while the gate is what the operator sees a
    /// reason from. Keeping them equal means a sighting never disappears
    /// without any explanation reaching the screen.
    private var detectorOptions: AprilTagDetector.Options {
        var options = AprilTagDetector.Options.default
        options.minimumDecisionMargin = min(
            options.minimumDecisionMargin, settings.tagLocalization.minimumDecisionMargin
        )
        options.maximumReprojectionError = max(
            options.maximumReprojectionError, settings.tagLocalization.maximumReprojectionError
        )
        return options
    }

    private func finish(detections: [AprilTagDetection], phonePose: Pose) {
        framesProcessed += 1
        rate.record(at: Date().timeIntervalSince1970)
        detectionRateHz = rate.rate(at: Date().timeIntervalSince1970)

        if let detection = detections.first {
            let outcome = gate.evaluate(
                detection: detection,
                mounts: settings.aprilTags,
                phonePoseInAR: phonePose,
                currentRobotPoseInAR: lastRobotPoseInAR
            )
            switch outcome {
            case .accepted(let tagID, let pose, let range):
                lastRobotPoseInAR = pose
                lastFixAt = Date()
                lastFixTagID = tagID
                lastFixRange = range
                status = .fixed(tagID: tagID, range: range, at: Date())
                onFix?(pose, tagID, range, phonePose)
            case .rejected(.awaitingAgreement(let count, let required)):
                status = .confirming(tagID: detection.id, count: count, required: required)
            case .rejected(let reason):
                status = .rejected(reason.label)
            }
        } else if !hasRecentFix {
            refreshIdleStatus()
        }

        // A frame may have arrived while the worker was busy.
        schedule()
    }

    /// Whether the last fix is recent enough that the marker is still standing
    /// on measured ground rather than coasting on odometry.
    public var hasRecentFix: Bool {
        guard let lastFixAt else { return false }
        return Date().timeIntervalSince(lastFixAt) <= settings.tagLocalization.fixValidityDuration
    }

    public var secondsSinceFix: Double? {
        lastFixAt.map { Date().timeIntervalSince($0) }
    }
}

extension GrayImage {
    /// Copies the luma plane out of an ARKit capture buffer.
    ///
    /// ARKit hands over bi-planar YCbCr, and plane 0 is already the greyscale
    /// image the detector wants — no colour conversion, no Accelerate, just a
    /// strided copy. The copy is the point: the pixel buffer belongs to ARKit
    /// and is recycled the moment the frame is released, so holding a pointer
    /// into it across a queue hop would read whatever the camera wrote next.
    init?(lumaOf pixelBuffer: CVPixelBuffer) {
        guard CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly) == kCVReturnSuccess else {
            return nil
        }
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        let planeIndex = CVPixelBufferIsPlanar(pixelBuffer) ? 0 : 0
        guard let base = CVPixelBufferIsPlanar(pixelBuffer)
            ? CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, planeIndex)
            : CVPixelBufferGetBaseAddress(pixelBuffer)
        else { return nil }

        let width = CVPixelBufferIsPlanar(pixelBuffer)
            ? CVPixelBufferGetWidthOfPlane(pixelBuffer, planeIndex)
            : CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferIsPlanar(pixelBuffer)
            ? CVPixelBufferGetHeightOfPlane(pixelBuffer, planeIndex)
            : CVPixelBufferGetHeight(pixelBuffer)
        let bytesPerRow = CVPixelBufferIsPlanar(pixelBuffer)
            ? CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, planeIndex)
            : CVPixelBufferGetBytesPerRow(pixelBuffer)
        guard width > 0, height > 0, bytesPerRow >= width else { return nil }

        var pixels = [UInt8](repeating: 0, count: width * height)
        let source = base.assumingMemoryBound(to: UInt8.self)
        pixels.withUnsafeMutableBufferPointer { destination in
            guard let target = destination.baseAddress else { return }
            for row in 0..<height {
                // Row by row rather than one memcpy: the plane is padded to a
                // hardware-friendly stride that is usually wider than the image.
                target.advanced(by: row * width)
                    .update(from: source.advanced(by: row * bytesPerRow), count: width)
            }
        }
        self.init(width: width, height: height, pixels: pixels)
    }
}
#endif
