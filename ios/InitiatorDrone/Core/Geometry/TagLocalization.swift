import Foundation

/// Turns a tag sighting into a robot pose, and a robot pose into an alignment.
///
/// This is the piece that makes the app's central problem go away. `odom_node`
/// estimates orientation only, so the robot's position has always come from a
/// human standing next to it and tapping "align", and it drifts from that moment
/// on. A tag bolted to the robot gives an absolute fix in the phone's own world
/// frame, whenever the phone can see it.
///
/// ## The chain
///
/// ```text
/// T_world_camera   ARKit, the phone's own pose
/// T_camera_tag     the detector
/// T_base_tag       the mount, measured by hand (ROS axes)
///
/// T_world_base = T_world_camera · T_camera_tag · T_base_tag⁻¹
/// ```
///
/// ## The two tag conventions
///
/// A detected tag's frame is the AprilTag one: `+X` right and `+Y` down as the
/// tag is read, `+Z` through the tag away from the reader. A *mounted* tag's
/// frame is the one the settings screen uses, matching the camera mount: `+X`
/// out of the face, `+Y` left, `+Z` up.
///
/// Written in ARKit axes — which is what `FrameConversion` has already done to
/// the mount by this point — the two differ by a **half turn about the vertical**
/// and nothing more. The detector's frame faces back at whoever is reading the
/// tag; the settings frame faces outward with it. Up is shared, and both "out"
/// and "right" flip.
///
/// It is easy to talk oneself into something more elaborate here, because the
/// ROS and ARKit axis conventions are also in play. They are not part of this
/// step: `FrameConversion.arPose(fromROS:)` has already relabelled them, and
/// what is left over is only the reader-versus-behind flip. The hand-derived
/// scenarios in `TagLocalizationTests` exist to keep that honest — each one
/// works out the detector frame from where the reader would have to stand,
/// without referring to the constant below.
public enum TagLocalization {

    /// Detector tag frame to settings tag frame, in ARKit axes: a half turn
    /// about the tag's own up axis.
    ///
    /// Self-inverse, so the same constant converts either way.
    public static let settingsTagFromDetectorTag = Quaternion(w: 0, x: 0, y: 1, z: 0)

    /// Where the robot's `base_link` is in the ARKit world, from one sighting.
    ///
    /// - Parameters:
    ///   - detection: as returned by `AprilTagDetector`.
    ///   - mount: where that tag is bolted to the robot.
    ///   - phonePoseInAR: `ARCamera.transform` as a pose.
    public static func robotPoseInAR(
        detection: AprilTagDetection,
        mount: AprilTagMount,
        phonePoseInAR: Pose
    ) -> Pose {
        let tagInAR = phonePoseInAR * detection.poseInCameraNode
        // Same origin, different axes: rotation only.
        let settingsTagInAR = Pose(
            position: tagInAR.position,
            orientation: (tagInAR.orientation * settingsTagFromDetectorTag).normalized
        )
        let mountInAR = FrameConversion.arPose(fromROS: mount.poseInBody)
        return settingsTagInAR * mountInAR.inverse
    }

    /// The alignment that would put the robot marker at `robotPoseInAR` given
    /// what odometry currently reports.
    ///
    /// The rendering path is `arFromOdom · arPose(fromROS: odom)`, so this
    /// solves that for `arFromOdom` and hands back something the existing
    /// pipeline consumes unchanged — no second transform, no separate render
    /// path, and the manual and automatic alignments stay interchangeable.
    ///
    /// **Roll and pitch are dropped**, and that is a feature rather than a
    /// concession. `RobotAlignment` only carries a translation and a yaw because
    /// both `odom` and the ARKit world are gravity-aligned, so any tilt between
    /// them is error by definition — and tilt is exactly where a monocular tag
    /// pose is weakest. Projecting onto translation-and-yaw throws away the
    /// noisiest component of the measurement and keeps the scene level.
    public static func alignment(
        placingRobotAt robotPoseInAR: Pose,
        reportedOdometry odometryPose: Pose,
        capturedAt: Date = Date()
    ) -> RobotAlignment {
        let odometryInAR = FrameConversion.arPose(fromROS: odometryPose)
        let transform = robotPoseInAR * odometryInAR.inverse
        return RobotAlignment(
            originInAR: transform.position,
            yaw: transform.orientation.yawAroundY,
            capturedAt: capturedAt
        )
    }

    /// Convenience for the whole path: sighting to alignment.
    public static func alignment(
        from detection: AprilTagDetection,
        mount: AprilTagMount,
        phonePoseInAR: Pose,
        reportedOdometry odometryPose: Pose,
        capturedAt: Date = Date()
    ) -> RobotAlignment {
        alignment(
            placingRobotAt: robotPoseInAR(
                detection: detection, mount: mount, phonePoseInAR: phonePoseInAR
            ),
            reportedOdometry: odometryPose,
            capturedAt: capturedAt
        )
    }
}

/// Decides whether a sighting is good enough to move the robot, and holds the
/// consecutive-agreement count.
///
/// Separated from the math above so the policy — every threshold, and the
/// requirement that several frames agree — can be tested without synthesising
/// an image, and reasoned about without reading any geometry.
public struct TagFixGate {

    public enum Rejection: Equatable, Sendable {
        case disabled
        case noMountConfigured(tagID: Int)
        case mountDisabled(tagID: Int)
        case tooFar(range: Double)
        case lowDecisionMargin(margin: Double)
        case highReprojectionError(error: Double)
        /// Seen, believed, but not yet seen often enough in a row.
        case awaitingAgreement(count: Int, required: Int)
        /// A jump so large it is more likely a wrong tag than a real correction.
        case implausibleCorrection(distance: Double)

        public var label: String {
            switch self {
            case .disabled:
                return "Tag relocalisation is switched off."
            case .noMountConfigured(let tagID):
                return "Saw tag \(tagID), which is not in Settings ▸ AprilTags."
            case .mountDisabled(let tagID):
                return "Tag \(tagID) is configured but switched off."
            case .tooFar(let range):
                return String(format: "Tag is %.1f m away, beyond the range limit.", range)
            case .lowDecisionMargin(let margin):
                return String(format: "Decode margin %.0f is too low to trust.", margin)
            case .highReprojectionError(let error):
                return String(format: "Corner fit is off by %.1f px; not a flat square.", error)
            case .awaitingAgreement(let count, let required):
                return "Confirming tag (\(count)/\(required) frames)."
            case .implausibleCorrection(let distance):
                return String(format: "Sighting would jump the robot %.1f m; ignored.", distance)
            }
        }
    }

    public enum Outcome: Equatable, Sendable {
        case accepted(tagID: Int, robotPoseInAR: Pose, range: Double)
        case rejected(Rejection)
    }

    public var settings: TagLocalizationSettings
    private var streakTagID: Int?
    private var streakCount = 0

    public init(settings: TagLocalizationSettings = .default) {
        self.settings = settings
    }

    /// Number of consecutive frames the current tag has been seen on.
    public var currentStreak: Int { streakCount }
    public var currentStreakTagID: Int? { streakTagID }

    /// Drops the streak. Called when the link, the AR session or the tag list
    /// changes, so evidence gathered under different conditions is not counted
    /// toward a fix made under new ones.
    public mutating func reset() {
        streakTagID = nil
        streakCount = 0
    }

    /// Evaluates one sighting.
    ///
    /// - Parameter currentRobotPoseInAR: where the robot is believed to be, for
    ///   the plausibility check. `nil` on the first fix of a session, which is
    ///   the case that must always be allowed through — there is nothing to
    ///   disagree with yet.
    public mutating func evaluate(
        detection: AprilTagDetection,
        mounts: [AprilTagMount],
        phonePoseInAR: Pose,
        currentRobotPoseInAR: Pose?
    ) -> Outcome {
        guard settings.isEnabled else {
            reset()
            return .rejected(.disabled)
        }
        guard let mount = mounts.first(where: { $0.tagID == detection.id && $0.isValid }) else {
            reset()
            return .rejected(.noMountConfigured(tagID: detection.id))
        }
        guard mount.isEnabled else {
            reset()
            return .rejected(.mountDisabled(tagID: detection.id))
        }
        guard detection.range <= settings.maximumRange else {
            reset()
            return .rejected(.tooFar(range: detection.range))
        }
        guard detection.decisionMargin >= settings.minimumDecisionMargin else {
            reset()
            return .rejected(.lowDecisionMargin(margin: detection.decisionMargin))
        }
        guard detection.reprojectionError <= settings.maximumReprojectionError else {
            reset()
            return .rejected(.highReprojectionError(error: detection.reprojectionError))
        }

        // A different tag restarts the count rather than adding to it: two
        // frames of tag 4 and one of tag 9 is not three frames of evidence.
        if streakTagID == detection.id {
            streakCount += 1
        } else {
            streakTagID = detection.id
            streakCount = 1
        }
        guard streakCount >= settings.requiredConsecutiveSightings else {
            return .rejected(.awaitingAgreement(
                count: streakCount, required: settings.requiredConsecutiveSightings
            ))
        }

        let pose = TagLocalization.robotPoseInAR(
            detection: detection, mount: mount, phonePoseInAR: phonePoseInAR
        )
        if settings.maximumCorrection > 0, let current = currentRobotPoseInAR {
            let distance = pose.position.distance(to: current.position)
            if distance > settings.maximumCorrection {
                // The streak is kept: if the tag really has moved this far, the
                // operator will keep seeing it and the next check against the
                // updated estimate will pass. Resetting here would make a large
                // genuine correction impossible to ever apply.
                return .rejected(.implausibleCorrection(distance: distance))
            }
        }
        return .accepted(tagID: detection.id, robotPoseInAR: pose, range: detection.range)
    }
}
