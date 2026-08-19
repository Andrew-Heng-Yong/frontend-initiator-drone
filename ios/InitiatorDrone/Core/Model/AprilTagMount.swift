import Foundation

/// One AprilTag mounted on the robot: which tag it is, how big it is, and
/// where it sits.
///
/// ## The frame
///
/// The offset uses the same convention as `CameraExtrinsics`, deliberately, so
/// there is one mental model for both: stand behind the thing looking the way
/// it faces, and `+X` is out, `+Y` is your left, `+Z` is up — all relative to
/// `base_link`, in metres and degrees.
///
/// For a tag, "the way it faces" is the direction you have to look from to read
/// it. So a tag on the nose of the drone facing forward is all zeros; one on the
/// tail is `yaw 180°`; one on the top deck facing the sky is `pitch -90°`.
///
/// Unlike the camera mount, **yaw is here and matters**. A camera's yaw is
/// indistinguishable from the robot pointing elsewhere, so offering it would
/// only let a mis-measurement hide a heading error. A tag's yaw is the opposite:
/// it is the thing that determines which way the app decides the robot is
/// facing, so it cannot be left out.
public struct AprilTagMount: Equatable, Codable, Sendable, Identifiable {
    /// Stable across edits, so SwiftUI list rows keep their identity while the
    /// tag ID is being retyped — including the moment the field is empty.
    public var id: UUID
    /// `tag16h5` ID, 0...29.
    public var tagID: Int
    /// Edge length of the **outer black border**, in metres.
    ///
    /// This is the measurement people get wrong, and it is not forgiving: range
    /// scales linearly with it, so a tag entered 20% too large puts the robot
    /// 20% further away with no other symptom. Measure the black square, not the
    /// paper, and not the payload inside the border.
    public var sizeMetres: Double

    /// Forward of `base_link`, metres.
    public var x: Double
    /// Left of `base_link`, metres.
    public var y: Double
    /// Above `base_link`, metres.
    public var z: Double
    /// Rotation about `base_link` `+X`; positive drops the tag's right side.
    public var rollDegrees: Double
    /// Rotation about `base_link` `+Y`; positive tips the tag's face downward.
    public var pitchDegrees: Double
    /// Rotation about `base_link` `+Z`; positive turns the tag's face to the
    /// robot's left.
    public var yawDegrees: Double

    /// Lets a tag be configured but ignored, which beats deleting it and
    /// re-measuring when a marker is temporarily obscured or removed.
    public var isEnabled: Bool

    public init(
        id: UUID = UUID(),
        tagID: Int = 0,
        sizeMetres: Double = 0.10,
        x: Double = 0,
        y: Double = 0,
        z: Double = 0,
        rollDegrees: Double = 0,
        pitchDegrees: Double = 0,
        yawDegrees: Double = 0,
        isEnabled: Bool = true
    ) {
        self.id = id
        self.tagID = tagID
        self.sizeMetres = sizeMetres
        self.x = x
        self.y = y
        self.z = z
        self.rollDegrees = rollDegrees
        self.pitchDegrees = pitchDegrees
        self.yawDegrees = yawDegrees
        self.isEnabled = isEnabled
    }

    public var translationInBody: Vector3 { Vector3(x, y, z) }

    /// Mount rotation in `base_link` axes, composed as the standard
    /// `R = R_z(yaw) · R_y(pitch) · R_x(roll)`.
    public var rotationInBody: Quaternion {
        let roll = Quaternion(axis: Vector3(1, 0, 0), angle: rollDegrees * .pi / 180)
        let pitch = Quaternion(axis: Vector3(0, 1, 0), angle: pitchDegrees * .pi / 180)
        let yaw = Quaternion(axis: Vector3(0, 0, 1), angle: yawDegrees * .pi / 180)
        return (yaw * pitch * roll).normalized
    }

    /// The tag frame's pose relative to `base_link`, in ROS axes.
    public var poseInBody: Pose {
        Pose(position: translationInBody, orientation: rotationInBody)
    }

    public var isValid: Bool {
        AprilTagFamily.isValidID(tagID) && sizeMetres > 0.005 && sizeMetres < 5
    }

    /// A one-line summary for the settings list.
    public var summary: String {
        String(
            format: "%.0f mm  ·  (%.2f, %.2f, %.2f) m  ·  rpy %.0f/%.0f/%.0f°",
            sizeMetres * 1000, x, y, z, rollDegrees, pitchDegrees, yawDegrees
        )
    }
}

/// How willing the app is to move the robot on the strength of a tag sighting.
///
/// Every one of these is a false-positive or accuracy guard, because the failure
/// they exist to prevent is severe: a bad sighting does not nudge the marker, it
/// teleports it, and the operator has no way to tell a wrong relocalisation from
/// a right one by looking at it.
public struct TagLocalizationSettings: Equatable, Codable, Sendable {
    public var isEnabled: Bool
    /// Ignore sightings beyond this range, in metres. Corner noise turns into
    /// range error in proportion to distance, and into heading error faster
    /// than that.
    public var maximumRange: Double
    /// Minimum decode decision margin, in grey levels.
    public var minimumDecisionMargin: Double
    /// Largest tolerated mean corner reprojection error, in pixels.
    public var maximumReprojectionError: Double
    /// How many consecutive frames must agree on the same tag before the robot
    /// is moved. The single most effective guard available: `tag16h5` false
    /// positives are essentially uncorrelated frame to frame, so requiring
    /// agreement squares an already small probability, while a real tag in view
    /// satisfies it in a fraction of a second.
    public var requiredConsecutiveSightings: Int
    /// How far a new sighting may move the robot from the current estimate
    /// before it is treated as a mistake rather than a correction, in metres.
    /// Zero disables the check, which is what the first fix of a session needs.
    public var maximumCorrection: Double
    /// How long a fix stays authoritative before the app admits it is coasting
    /// on odometry, in seconds. Display only; it changes no transform.
    public var fixValidityDuration: Double

    public init(
        isEnabled: Bool = true,
        maximumRange: Double = 4.0,
        minimumDecisionMargin: Double = 25,
        maximumReprojectionError: Double = 2.0,
        requiredConsecutiveSightings: Int = 3,
        maximumCorrection: Double = 1.5,
        fixValidityDuration: Double = 10.0
    ) {
        self.isEnabled = isEnabled
        self.maximumRange = maximumRange
        self.minimumDecisionMargin = minimumDecisionMargin
        self.maximumReprojectionError = maximumReprojectionError
        self.requiredConsecutiveSightings = max(1, requiredConsecutiveSightings)
        self.maximumCorrection = maximumCorrection
        self.fixValidityDuration = fixValidityDuration
    }

    public static let `default` = TagLocalizationSettings()
}
