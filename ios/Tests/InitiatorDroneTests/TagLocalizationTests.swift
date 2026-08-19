#if canImport(XCTest)
import XCTest
// In Xcode the tests compile as their own module; the headless runner in
// Scripts/ compiles them alongside the sources instead, so neither import
// exists there.
@testable import InitiatorDrone
#endif
import Foundation

/// Turning a tag sighting into a robot pose.
///
/// The scenarios below derive the detector's tag frame from **where a reader
/// would have to stand** — which way is out of the tag, which way is up, which
/// way is right as it is read — and never from
/// `TagLocalization.settingsTagFromDetectorTag`. That is the point: a test that
/// built the input with the same constant it is checking would pass whatever
/// that constant said.
final class TagLocalizationTests: XCTestCase {

    /// Builds a detection whose `poseInCameraNode` is the pose given.
    ///
    /// `AprilTagDetection` stores the optical-frame pose and derives the node
    /// one, so this inverts that: the position flip is its own inverse, and a
    /// half turn about X is its own inverse too.
    private func detection(
        id: Int,
        nodePose: Pose,
        decisionMargin: Double = 100,
        reprojectionError: Double = 0.4
    ) -> AprilTagDetection {
        let flip = Quaternion(axis: Vector3(1, 0, 0), angle: .pi)
        return AprilTagDetection(
            id: id,
            corners: [.zero, .zero, .zero, .zero],
            hammingDistance: 0,
            decisionMargin: decisionMargin,
            poseInCameraOptical: Pose(
                position: Vector3(nodePose.position.x, -nodePose.position.y, -nodePose.position.z),
                orientation: (flip * nodePose.orientation).normalized
            ),
            reprojectionError: reprojectionError
        )
    }

    private func rotationY(_ degrees: Double) -> Quaternion {
        Quaternion(axis: Vector3(0, 1, 0), angle: degrees * .pi / 180)
    }

    /// A tag on the robot's nose, 0.2 m forward, facing the way the robot faces.
    private var noseMount: AprilTagMount {
        AprilTagMount(tagID: 4, sizeMetres: 0.1, x: 0.2)
    }

    // MARK: - Hand-derived scenarios

    /// Robot at the AR origin facing `-Z` (which is what ROS `+X` forward maps
    /// to), phone at the origin looking at it.
    ///
    /// The tag is 0.2 m along `-Z` and faces `-Z`, so the reader stands further
    /// along `-Z` looking back toward `+Z`. From there: out-toward-reader is
    /// `-Z`, up is `+Y`, and right-as-read is `-X` — a half turn about Y.
    func testRobotAtOriginFacingAwayFromTheCamera() {
        let observed = detection(id: 4, nodePose: Pose(
            position: Vector3(0, 0, -0.2),
            orientation: rotationY(180)
        ))
        let pose = TagLocalization.robotPoseInAR(
            detection: observed, mount: noseMount, phonePoseInAR: .identity
        )
        XCTAssertTrue(
            pose.isApproximatelyEqual(to: .identity, tolerance: 1e-9),
            "expected the robot at the origin, got \(pose)"
        )
    }

    /// The same geometry moved three metres away and one to the side: the mount
    /// offset must be subtracted in the world, not in the camera.
    func testRobotTranslatedAwayFromTheCamera() {
        let expected = Pose(position: Vector3(1, 0, -3))
        let observed = detection(id: 4, nodePose: Pose(
            position: Vector3(1, 0, -3.2),
            orientation: rotationY(180)
        ))
        let pose = TagLocalization.robotPoseInAR(
            detection: observed, mount: noseMount, phonePoseInAR: .identity
        )
        XCTAssertTrue(
            pose.isApproximatelyEqual(to: expected, tolerance: 1e-9),
            "expected \(expected), got \(pose)"
        )
    }

    /// Robot yawed 90°, so it faces AR `-X` and its nose tag is at `(-0.2,0,0)`.
    ///
    /// The reader now stands on the `-X` side looking toward `+X`: out is `-X`,
    /// up is `+Y`, and right-as-read is `+Z`. That basis is a quarter turn about
    /// Y the other way from the robot's own.
    func testRobotYawedNinetyDegrees() {
        let expected = Pose(orientation: rotationY(90))
        let observed = detection(id: 4, nodePose: Pose(
            position: Vector3(-0.2, 0, 0),
            orientation: rotationY(-90)
        ))
        let pose = TagLocalization.robotPoseInAR(
            detection: observed, mount: noseMount, phonePoseInAR: .identity
        )
        XCTAssertTrue(
            pose.isApproximatelyEqual(to: expected, tolerance: 1e-9),
            "expected \(expected), got \(pose)"
        )
    }

    /// The phone's own pose must be composed in, not ignored: the same sighting
    /// from a moved phone describes a different place in the world.
    func testThePhonePoseIsComposedIn() {
        let phone = Pose(position: Vector3(2, 1, 5), orientation: rotationY(30))
        let observed = detection(id: 4, nodePose: Pose(
            position: Vector3(0, 0, -0.2),
            orientation: rotationY(180)
        ))
        let pose = TagLocalization.robotPoseInAR(
            detection: observed, mount: noseMount, phonePoseInAR: phone
        )
        // The robot sits where the phone's own frame puts a point 0.2 m ahead,
        // plus the mount taken back off — which lands exactly on the phone.
        XCTAssertTrue(
            pose.isApproximatelyEqual(to: phone, tolerance: 1e-9),
            "expected \(phone), got \(pose)"
        )
    }

    /// A tag on top of the robot facing the sky. Pitching the mount `-90°` tips
    /// its face from forward to up, and the reader is then overhead looking
    /// down. Nothing about the robot's own pose changes.
    func testTagMountedOnTopFacingUp() {
        let mount = AprilTagMount(tagID: 7, sizeMetres: 0.1, z: 0.15, pitchDegrees: -90)
        // Pitching the mount -90° sends the tag's own "out" to the robot's up
        // and its own "up" to the robot's rear. With the robot facing AR -Z,
        // that puts out at AR +Y, up-as-read at AR +Z, and right-as-read at
        // AR -X. Columns (-1,0,0), (0,0,1), (0,1,0) — and their cross product
        // checks out: X × Y = +Y = Z.
        let readerBasis = Quaternion(rotationMatrixColumnMajor: [
            -1, 0, 0,
            0, 0, 1,
            0, 1, 0,
        ])
        let observed = detection(id: 7, nodePose: Pose(
            position: Vector3(0, 0.15, 0),
            orientation: readerBasis.normalized
        ))
        let pose = TagLocalization.robotPoseInAR(
            detection: observed, mount: mount, phonePoseInAR: .identity
        )
        XCTAssertTrue(
            pose.isApproximatelyEqual(to: .identity, tolerance: 1e-6),
            "expected the robot at the origin facing -Z, got \(pose)"
        )
    }

    /// The bridge rotation is its own inverse, which the code relies on by using
    /// one constant for both directions.
    func testTheTagFrameBridgeIsItsOwnInverse() {
        let bridge = TagLocalization.settingsTagFromDetectorTag
        let round = (bridge * bridge).normalized
        XCTAssertEqual(abs(round.w), 1, accuracy: 1e-12)
    }

    // MARK: - Alignment

    /// The whole reason a sighting is expressed as a `RobotAlignment`: it has to
    /// reproduce the pose through the existing render path, which is
    /// `arFromOdom · arPose(fromROS: odom)`.
    func testAlignmentReproducesTheRobotPoseThroughTheRenderPath() {
        let odometry = Pose(
            position: .zero,
            orientation: Quaternion.aroundZ(0.7)
        )
        let robotInAR = Pose(position: Vector3(1.2, 0, -2.4), orientation: rotationY(25))

        let alignment = TagLocalization.alignment(
            placingRobotAt: robotInAR, reportedOdometry: odometry
        )
        let rendered = alignment.arPose(fromROSOdometry: odometry)
        XCTAssertTrue(
            rendered.isApproximatelyEqual(to: robotInAR, tolerance: 1e-9),
            "render path gave \(rendered), wanted \(robotInAR)"
        )
    }

    /// Tag pose is weakest in tilt, and both frames are gravity-aligned, so any
    /// roll or pitch in a sighting is error by construction. It must not reach
    /// the scene.
    func testAlignmentDropsRollAndPitchFromANoisySighting() {
        let tilted = Quaternion(axis: Vector3(1, 0, 0), angle: 0.15)
            * Quaternion(axis: Vector3(0, 0, 1), angle: -0.1)
        let robotInAR = Pose(position: Vector3(0.5, 0.2, -1.0), orientation: tilted.normalized)

        let alignment = TagLocalization.alignment(
            placingRobotAt: robotInAR, reportedOdometry: .identity
        )
        let rendered = alignment.arPose(fromROSOdometry: .identity)
        // Level: the marker's own up axis stays the world's up axis.
        let up = rendered.orientation.rotate(Vector3(0, 1, 0))
        XCTAssertEqual(up.x, 0, accuracy: 1e-9)
        XCTAssertEqual(up.z, 0, accuracy: 1e-9)
        XCTAssertEqual(up.y, 1, accuracy: 1e-9)
        // The position is kept exactly; only the tilt is discarded.
        XCTAssertTrue(rendered.position.isApproximatelyEqual(to: robotInAR.position, tolerance: 1e-9))
    }

    // MARK: - The gate

    private func acceptingGate() -> TagFixGate {
        TagFixGate(settings: TagLocalizationSettings(requiredConsecutiveSightings: 3))
    }

    private var goodSighting: AprilTagDetection {
        detection(id: 4, nodePose: Pose(position: Vector3(0, 0, -0.2), orientation: rotationY(180)))
    }

    func testAFixNeedsSeveralConsecutiveFramesOfTheSameTag() {
        var gate = acceptingGate()
        for attempt in 1...2 {
            let outcome = gate.evaluate(
                detection: goodSighting, mounts: [noseMount],
                phonePoseInAR: .identity, currentRobotPoseInAR: nil
            )
            XCTAssertEqual(outcome, .rejected(.awaitingAgreement(count: attempt, required: 3)))
        }
        let final = gate.evaluate(
            detection: goodSighting, mounts: [noseMount],
            phonePoseInAR: .identity, currentRobotPoseInAR: nil
        )
        guard case .accepted(let tagID, _, _) = final else {
            return XCTFail("third consecutive frame should be accepted, got \(final)")
        }
        XCTAssertEqual(tagID, 4)
    }

    /// Two frames of one tag and one of another is not three frames of evidence.
    func testADifferentTagRestartsTheStreakRatherThanAddingToIt() {
        var gate = acceptingGate()
        let other = detection(id: 9, nodePose: Pose(position: Vector3(0, 0, -0.2), orientation: rotationY(180)))
        let mounts = [noseMount, AprilTagMount(tagID: 9, sizeMetres: 0.1, x: 0.2)]

        _ = gate.evaluate(detection: goodSighting, mounts: mounts, phonePoseInAR: .identity, currentRobotPoseInAR: nil)
        _ = gate.evaluate(detection: goodSighting, mounts: mounts, phonePoseInAR: .identity, currentRobotPoseInAR: nil)
        let switched = gate.evaluate(detection: other, mounts: mounts, phonePoseInAR: .identity, currentRobotPoseInAR: nil)
        XCTAssertEqual(switched, .rejected(.awaitingAgreement(count: 1, required: 3)))
    }

    func testAnUnconfiguredTagIsNeverUsed() {
        var gate = acceptingGate()
        let outcome = gate.evaluate(
            detection: goodSighting, mounts: [], phonePoseInAR: .identity, currentRobotPoseInAR: nil
        )
        XCTAssertEqual(outcome, .rejected(.noMountConfigured(tagID: 4)))
    }

    func testADisabledMountIsNeverUsed() {
        var gate = acceptingGate()
        var mount = noseMount
        mount.isEnabled = false
        XCTAssertEqual(
            gate.evaluate(detection: goodSighting, mounts: [mount], phonePoseInAR: .identity, currentRobotPoseInAR: nil),
            .rejected(.mountDisabled(tagID: 4))
        )
    }

    func testDistantWeakAndBadlyFittedSightingsAreRejected() {
        var gate = TagFixGate(settings: TagLocalizationSettings(
            maximumRange: 2.0, minimumDecisionMargin: 25,
            maximumReprojectionError: 2.0, requiredConsecutiveSightings: 1
        ))

        let far = detection(id: 4, nodePose: Pose(position: Vector3(0, 0, -5), orientation: rotationY(180)))
        guard case .rejected(.tooFar) = gate.evaluate(
            detection: far, mounts: [noseMount], phonePoseInAR: .identity, currentRobotPoseInAR: nil
        ) else { return XCTFail("a tag 5 m away must be refused at a 2 m limit") }

        let faint = detection(id: 4, nodePose: Pose(position: Vector3(0, 0, -0.2), orientation: rotationY(180)), decisionMargin: 5)
        guard case .rejected(.lowDecisionMargin) = gate.evaluate(
            detection: faint, mounts: [noseMount], phonePoseInAR: .identity, currentRobotPoseInAR: nil
        ) else { return XCTFail("a marginal decode must be refused") }

        let skewed = detection(id: 4, nodePose: Pose(position: Vector3(0, 0, -0.2), orientation: rotationY(180)), reprojectionError: 9)
        guard case .rejected(.highReprojectionError) = gate.evaluate(
            detection: skewed, mounts: [noseMount], phonePoseInAR: .identity, currentRobotPoseInAR: nil
        ) else { return XCTFail("a quad that is not a flat square must be refused") }
    }

    /// The first fix of a session has nothing to disagree with, so the
    /// plausibility check must not block it — otherwise tag localisation could
    /// never start.
    func testTheFirstFixIsNeverBlockedAsImplausible() {
        var gate = TagFixGate(settings: TagLocalizationSettings(
            requiredConsecutiveSightings: 1, maximumCorrection: 0.1
        ))
        guard case .accepted = gate.evaluate(
            detection: goodSighting, mounts: [noseMount],
            phonePoseInAR: .identity, currentRobotPoseInAR: nil
        ) else { return XCTFail("the first fix must be allowed through") }
    }

    func testAWildJumpFromTheCurrentEstimateIsRefused() {
        var gate = TagFixGate(settings: TagLocalizationSettings(
            requiredConsecutiveSightings: 1, maximumCorrection: 0.5
        ))
        let outcome = gate.evaluate(
            detection: goodSighting, mounts: [noseMount],
            phonePoseInAR: .identity,
            currentRobotPoseInAR: Pose(position: Vector3(0, 0, -40))
        )
        guard case .rejected(.implausibleCorrection) = outcome else {
            return XCTFail("a 40 m jump must be refused, got \(outcome)")
        }
    }

    /// A refused jump must not also destroy the streak, or a correction that is
    /// genuinely large could never be applied at all.
    func testARefusedJumpKeepsTheStreakAlive() {
        var gate = TagFixGate(settings: TagLocalizationSettings(
            requiredConsecutiveSightings: 2, maximumCorrection: 0.5
        ))
        _ = gate.evaluate(detection: goodSighting, mounts: [noseMount], phonePoseInAR: .identity, currentRobotPoseInAR: nil)
        _ = gate.evaluate(
            detection: goodSighting, mounts: [noseMount], phonePoseInAR: .identity,
            currentRobotPoseInAR: Pose(position: Vector3(0, 0, -40))
        )
        XCTAssertEqual(gate.currentStreak, 2)
        guard case .accepted = gate.evaluate(
            detection: goodSighting, mounts: [noseMount], phonePoseInAR: .identity, currentRobotPoseInAR: nil
        ) else { return XCTFail("the streak should have survived the refused jump") }
    }

    func testDisablingTagLocalisationRefusesEverything() {
        var gate = TagFixGate(settings: TagLocalizationSettings(isEnabled: false))
        XCTAssertEqual(
            gate.evaluate(detection: goodSighting, mounts: [noseMount], phonePoseInAR: .identity, currentRobotPoseInAR: nil),
            .rejected(.disabled)
        )
    }

    // MARK: - Mount

    func testMountValidityMatchesTheFamilyAndSensibleSizes() {
        XCTAssertTrue(AprilTagMount(tagID: 0, sizeMetres: 0.1).isValid)
        XCTAssertTrue(AprilTagMount(tagID: 29, sizeMetres: 0.1).isValid)
        XCTAssertFalse(AprilTagMount(tagID: 30, sizeMetres: 0.1).isValid)
        XCTAssertFalse(AprilTagMount(tagID: -1, sizeMetres: 0.1).isValid)
        XCTAssertFalse(AprilTagMount(tagID: 0, sizeMetres: 0).isValid)
    }

    func testMountYawTurnsTheFaceTowardTheRobotsLeft() {
        let mount = AprilTagMount(tagID: 0, yawDegrees: 90)
        // The face direction is the mount frame's +X, in ROS axes.
        let face = mount.rotationInBody.rotate(Vector3(1, 0, 0))
        XCTAssertTrue(face.isApproximatelyEqual(to: Vector3(0, 1, 0), tolerance: 1e-9))
    }

    func testMountPitchOfMinusNinetyPointsTheFaceUp() {
        let mount = AprilTagMount(tagID: 0, pitchDegrees: -90)
        let face = mount.rotationInBody.rotate(Vector3(1, 0, 0))
        XCTAssertTrue(face.isApproximatelyEqual(to: Vector3(0, 0, 1), tolerance: 1e-9))
    }

    static var allTests: [(String, (TagLocalizationTests) -> () throws -> Void)] {
        [
        ("testRobotAtOriginFacingAwayFromTheCamera", testRobotAtOriginFacingAwayFromTheCamera),
        ("testRobotTranslatedAwayFromTheCamera", testRobotTranslatedAwayFromTheCamera),
        ("testRobotYawedNinetyDegrees", testRobotYawedNinetyDegrees),
        ("testThePhonePoseIsComposedIn", testThePhonePoseIsComposedIn),
        ("testTagMountedOnTopFacingUp", testTagMountedOnTopFacingUp),
        ("testTheTagFrameBridgeIsItsOwnInverse", testTheTagFrameBridgeIsItsOwnInverse),
        ("testAlignmentReproducesTheRobotPoseThroughTheRenderPath", testAlignmentReproducesTheRobotPoseThroughTheRenderPath),
        ("testAlignmentDropsRollAndPitchFromANoisySighting", testAlignmentDropsRollAndPitchFromANoisySighting),
        ("testAFixNeedsSeveralConsecutiveFramesOfTheSameTag", testAFixNeedsSeveralConsecutiveFramesOfTheSameTag),
        ("testADifferentTagRestartsTheStreakRatherThanAddingToIt", testADifferentTagRestartsTheStreakRatherThanAddingToIt),
        ("testAnUnconfiguredTagIsNeverUsed", testAnUnconfiguredTagIsNeverUsed),
        ("testADisabledMountIsNeverUsed", testADisabledMountIsNeverUsed),
        ("testDistantWeakAndBadlyFittedSightingsAreRejected", testDistantWeakAndBadlyFittedSightingsAreRejected),
        ("testTheFirstFixIsNeverBlockedAsImplausible", testTheFirstFixIsNeverBlockedAsImplausible),
        ("testAWildJumpFromTheCurrentEstimateIsRefused", testAWildJumpFromTheCurrentEstimateIsRefused),
        ("testARefusedJumpKeepsTheStreakAlive", testARefusedJumpKeepsTheStreakAlive),
        ("testDisablingTagLocalisationRefusesEverything", testDisablingTagLocalisationRefusesEverything),
        ("testMountValidityMatchesTheFamilyAndSensibleSizes", testMountValidityMatchesTheFamilyAndSensibleSizes),
        ("testMountYawTurnsTheFaceTowardTheRobotsLeft", testMountYawTurnsTheFaceTowardTheRobotsLeft),
        ("testMountPitchOfMinusNinetyPointsTheFaceUp", testMountPitchOfMinusNinetyPointsTheFaceUp),
        ]
    }
}
