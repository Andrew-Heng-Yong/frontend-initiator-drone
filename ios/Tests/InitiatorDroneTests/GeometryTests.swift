#if canImport(XCTest)
import XCTest
// In Xcode the tests compile as their own module; the headless runner in
// Scripts/ compiles them alongside the sources instead, so neither import
// exists there.
@testable import InitiatorDrone
#endif
import Foundation

/// Quaternion algebra and the ROS <-> ARKit conversion layer.
///
/// This is the file to read first when a marker ends up in the wrong place.
/// Every claim the conversion layer's documentation makes is asserted here.
final class GeometryTests: XCTestCase {

    // MARK: - Quaternion basics

    func testIdentityRotationLeavesVectorsAlone() {
        let vector = Vector3(1, 2, 3)
        let rotated = Quaternion.identity.rotate(vector)
        XCTAssertTrue(rotated.isApproximatelyEqual(to: vector, tolerance: 1e-12))
    }

    func testQuarterTurnAboutZTakesXToY() {
        let rotation = Quaternion.aroundZ(.pi / 2)
        let rotated = rotation.rotate(Vector3(1, 0, 0))
        XCTAssertTrue(rotated.isApproximatelyEqual(to: Vector3(0, 1, 0), tolerance: 1e-9))
    }

    func testMultiplicationAppliesRightHandSideFirst() {
        // Rotate about Z by 90 degrees, then about X by 90 degrees.
        let aboutZ = Quaternion.aroundZ(.pi / 2)
        let aboutX = Quaternion(axis: Vector3(1, 0, 0), angle: .pi / 2)
        let combined = aboutX * aboutZ

        let viaCombined = combined.rotate(Vector3(1, 0, 0))
        let viaSequence = aboutX.rotate(aboutZ.rotate(Vector3(1, 0, 0)))
        XCTAssertTrue(viaCombined.isApproximatelyEqual(to: viaSequence, tolerance: 1e-9))
        XCTAssertTrue(viaCombined.isApproximatelyEqual(to: Vector3(0, 0, 1), tolerance: 1e-9))
    }

    func testConjugateUndoesRotation() {
        let rotation = Quaternion(axis: Vector3(0.3, -0.5, 0.8), angle: 1.1)
        let vector = Vector3(0.4, 1.2, -0.7)
        let roundTripped = rotation.conjugate.rotate(rotation.rotate(vector))
        XCTAssertTrue(roundTripped.isApproximatelyEqual(to: vector, tolerance: 1e-9))
    }

    func testRotationMatrixRoundTrip() {
        let original = Quaternion(axis: Vector3(0.2, 0.9, -0.4), angle: 2.3).normalized
        let rebuilt = Quaternion(rotationMatrixColumnMajor: original.rotationMatrixColumnMajor)
        XCTAssertTrue(original.describesSameRotation(as: rebuilt, tolerance: 1e-9))
    }

    func testRotationMatrixRoundTripAtEveryTraceBranch() {
        // The matrix-to-quaternion conversion picks one of four branches based
        // on the trace and the largest diagonal element; each needs coverage.
        let rotations = [
            Quaternion.identity,
            Quaternion(axis: Vector3(1, 0, 0), angle: .pi),
            Quaternion(axis: Vector3(0, 1, 0), angle: .pi),
            Quaternion(axis: Vector3(0, 0, 1), angle: .pi),
            Quaternion(axis: Vector3(1, 1, 0), angle: .pi),
        ]
        for rotation in rotations {
            let rebuilt = Quaternion(rotationMatrixColumnMajor: rotation.rotationMatrixColumnMajor)
            XCTAssertTrue(
                rotation.normalized.describesSameRotation(as: rebuilt, tolerance: 1e-7),
                "failed for \(rotation)"
            )
        }
    }

    func testRollPitchYawMatchesConstruction() {
        let roll = 0.3
        let pitch = -0.4
        let yaw = 1.9
        // tf2's convention: intrinsic Z-Y-X, so yaw is applied first.
        let rotation = Quaternion.aroundZ(yaw)
            * Quaternion(axis: Vector3(0, 1, 0), angle: pitch)
            * Quaternion(axis: Vector3(1, 0, 0), angle: roll)

        let angles = rotation.rollPitchYaw
        XCTAssertEqual(angles.roll, roll, accuracy: 1e-9)
        XCTAssertEqual(angles.pitch, pitch, accuracy: 1e-9)
        XCTAssertEqual(angles.yaw, yaw, accuracy: 1e-9)
    }

    func testYawAroundYIsTheInverseOfAroundY() {
        for degrees in stride(from: -170.0, through: 170.0, by: 17.0) {
            let radians = degrees * .pi / 180
            XCTAssertEqual(Quaternion.aroundY(radians).yawAroundY, radians, accuracy: 1e-9)
        }
    }

    // MARK: - Slerp

    func testSlerpEndpointsAreExact() {
        let start = Quaternion.aroundZ(0.2)
        let end = Quaternion.aroundZ(1.4)
        XCTAssertTrue(Quaternion.slerp(start, end, 0).describesSameRotation(as: start, tolerance: 1e-9))
        XCTAssertTrue(Quaternion.slerp(start, end, 1).describesSameRotation(as: end, tolerance: 1e-9))
    }

    func testSlerpMidpointIsHalfTheAngle() {
        let start = Quaternion.aroundZ(0)
        let end = Quaternion.aroundZ(1.0)
        let middle = Quaternion.slerp(start, end, 0.5)
        XCTAssertEqual(middle.yawAroundZ, 0.5, accuracy: 1e-9)
    }

    func testSlerpTakesTheShortPathAcrossTheDoubleCover() {
        // q and -q are the same rotation; interpolating naively would swing the
        // long way around and spin the marker through 300 degrees.
        let start = Quaternion.aroundZ(0.1)
        let end = -Quaternion.aroundZ(0.3)
        let middle = Quaternion.slerp(start, end, 0.5)
        XCTAssertEqual(middle.yawAroundZ, 0.2, accuracy: 1e-9)
    }

    func testSlerpClampsOutOfRangeParameters() {
        let start = Quaternion.aroundZ(0.2)
        let end = Quaternion.aroundZ(0.9)
        XCTAssertTrue(Quaternion.slerp(start, end, -3).describesSameRotation(as: start, tolerance: 1e-9))
        XCTAssertTrue(Quaternion.slerp(start, end, 7).describesSameRotation(as: end, tolerance: 1e-9))
    }

    func testSlerpOfNearlyIdenticalRotationsStaysUnit() {
        let start = Quaternion.aroundZ(0.5000000)
        let end = Quaternion.aroundZ(0.5000001)
        let middle = Quaternion.slerp(start, end, 0.5)
        XCTAssertEqual(middle.length, 1.0, accuracy: 1e-9)
    }

    // MARK: - Pose

    func testPoseCompositionMatchesSequentialApplication() {
        let outer = Pose(position: Vector3(1, 2, 3), orientation: Quaternion.aroundZ(0.7))
        let inner = Pose(position: Vector3(-0.5, 0.25, 2), orientation: Quaternion.aroundY(-0.4))
        let point = Vector3(0.3, -1.1, 0.6)

        let composed = (outer * inner).apply(to: point)
        let sequential = outer.apply(to: inner.apply(to: point))
        XCTAssertTrue(composed.isApproximatelyEqual(to: sequential, tolerance: 1e-9))
    }

    func testPoseInverseRoundTrips() {
        let pose = Pose(position: Vector3(3, -1, 0.5), orientation: Quaternion(axis: Vector3(0.2, 1, 0.3), angle: 0.9))
        let identity = pose * pose.inverse
        XCTAssertTrue(identity.isApproximatelyEqual(to: .identity, tolerance: 1e-9))
    }

    // MARK: - ROS <-> ARKit axes

    func testRosAxesMapOntoARKitAxes() {
        // The mapping the whole app depends on, stated as three facts.
        XCTAssertTrue(
            FrameConversion.arPosition(fromROS: Vector3(1, 0, 0))
                .isApproximatelyEqual(to: Vector3(0, 0, -1)),
            "ROS forward must become ARKit -Z"
        )
        XCTAssertTrue(
            FrameConversion.arPosition(fromROS: Vector3(0, 1, 0))
                .isApproximatelyEqual(to: Vector3(-1, 0, 0)),
            "ROS left must become ARKit -X"
        )
        XCTAssertTrue(
            FrameConversion.arPosition(fromROS: Vector3(0, 0, 1))
                .isApproximatelyEqual(to: Vector3(0, 1, 0)),
            "ROS up must become ARKit +Y"
        )
    }

    func testAxisConversionPreservesHandednessAndLength() {
        let x = FrameConversion.arPosition(fromROS: Vector3(1, 0, 0))
        let y = FrameConversion.arPosition(fromROS: Vector3(0, 1, 0))
        let z = FrameConversion.arPosition(fromROS: Vector3(0, 0, 1))

        // A right-handed triad must stay right-handed: x cross y == z. A sign
        // flip here would mirror the entire scene, which is exactly the class
        // of bug this conversion layer exists to prevent.
        XCTAssertTrue(x.cross(y).isApproximatelyEqual(to: z, tolerance: 1e-12))

        // Both frames are metric, so lengths are untouched.
        let arbitrary = Vector3(0.3, -1.7, 2.2)
        XCTAssertEqual(
            FrameConversion.arPosition(fromROS: arbitrary).length,
            arbitrary.length,
            accuracy: 1e-12
        )
    }

    func testPositionConversionRoundTrips() {
        let original = Vector3(1.25, -3.5, 0.75)
        let roundTripped = FrameConversion.rosPosition(
            fromAR: FrameConversion.arPosition(fromROS: original)
        )
        XCTAssertTrue(roundTripped.isApproximatelyEqual(to: original, tolerance: 1e-12))
    }

    func testAxisQuaternionAgreesWithThePositionMapping() {
        // The quaternion form and the hand-written component swap must describe
        // the same rotation, or orientation and position would disagree.
        for vector in [Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1), Vector3(0.4, -0.9, 2.1)] {
            let viaQuaternion = FrameConversion.arFromROSAxes.rotate(vector)
            let viaMapping = FrameConversion.arPosition(fromROS: vector)
            XCTAssertTrue(
                viaQuaternion.isApproximatelyEqual(to: viaMapping, tolerance: 1e-12),
                "mismatch for \(vector)"
            )
        }
    }

    func testInverseAxisQuaternionIsTheConjugate() {
        let product = FrameConversion.arFromROSAxes * FrameConversion.rosFromARAxes
        XCTAssertTrue(product.describesSameRotation(as: .identity, tolerance: 1e-12))
    }

    func testOrientationConversionRoundTrips() {
        let original = Quaternion(axis: Vector3(0.3, -0.4, 0.86), angle: 1.7).normalized
        let roundTripped = FrameConversion.rosOrientation(
            fromAR: FrameConversion.arOrientation(fromROS: original)
        )
        XCTAssertTrue(roundTripped.describesSameRotation(as: original, tolerance: 1e-9))
    }

    func testConvertedOrientationPointsSceneKitForwardAlongRobotForward() {
        // The property that makes the marker work without a corrective rotation:
        // a node given the converted orientation has its local -Z along the
        // robot's +X.
        for yaw in stride(from: -3.0, through: 3.0, by: 0.5) {
            let rosOrientation = Quaternion.aroundZ(yaw)
            let arOrientation = FrameConversion.arOrientation(fromROS: rosOrientation)

            let nodeForward = arOrientation.rotate(Vector3(0, 0, -1))
            let robotForwardInAR = FrameConversion.arPosition(
                fromROS: rosOrientation.rotate(Vector3(1, 0, 0))
            )
            XCTAssertTrue(
                nodeForward.isApproximatelyEqual(to: robotForwardInAR, tolerance: 1e-9),
                "mismatch at yaw \(yaw)"
            )
        }
    }

    func testRosYawBecomesArYawOfTheSameMagnitude() {
        // A robot turning left in ROS must turn left on screen.
        let rosYaw = 0.9
        let arOrientation = FrameConversion.arOrientation(fromROS: Quaternion.aroundZ(rosYaw))
        XCTAssertEqual(arOrientation.yawAroundY, rosYaw, accuracy: 1e-9)
    }

    // MARK: - Alignment

    func testIdentityAlignmentIsJustTheAxisChange() {
        let alignment = RobotAlignment(originInAR: .zero, yaw: 0)
        let rosPose = Pose(position: Vector3(2, 1, 0.5), orientation: Quaternion.aroundZ(0.4))

        let viaAlignment = alignment.arPose(fromROSOdometry: rosPose)
        let viaConversion = FrameConversion.arPose(fromROS: rosPose)
        XCTAssertTrue(viaAlignment.isApproximatelyEqual(to: viaConversion, tolerance: 1e-12))
    }

    func testAlignmentTranslatesTheOrigin() {
        let alignment = RobotAlignment(originInAR: Vector3(5, 1, -2), yaw: 0)
        let atOrigin = alignment.arPose(fromROSOdometry: Pose())
        XCTAssertTrue(atOrigin.position.isApproximatelyEqual(to: Vector3(5, 1, -2), tolerance: 1e-12))
    }

    func testAlignmentYawRotatesTheRobotFrame() {
        // With the odom frame yawed by 90 degrees, a robot one metre along its
        // own +X should appear 90 degrees round from ARKit's -Z.
        let alignment = RobotAlignment(originInAR: .zero, yaw: .pi / 2)
        let oneMetreForward = Pose(position: Vector3(1, 0, 0))
        let placed = alignment.arPose(fromROSOdometry: oneMetreForward)

        // ARKit -Z rotated about +Y by +90 degrees lands on -X.
        XCTAssertTrue(placed.position.isApproximatelyEqual(to: Vector3(-1, 0, 0), tolerance: 1e-9))
    }

    func testAlignmentRoundTripsBackToOdom() {
        let alignment = RobotAlignment(originInAR: Vector3(1.5, 0.2, -3), yaw: 0.87)
        let rosPose = Pose(
            position: Vector3(2.5, -1.25, 0.4),
            orientation: Quaternion.aroundZ(-0.6)
        )
        let recovered = alignment.rosOdometryPose(fromAR: alignment.arPose(fromROSOdometry: rosPose))
        XCTAssertTrue(recovered.isApproximatelyEqual(to: rosPose, tolerance: 1e-9))
    }

    func testPlacingOriginAtPhoneUsesHeadingOnly() {
        // A phone held at a steep angle must still produce a level robot frame:
        // roll and pitch have to be discarded.
        let tilted = Quaternion.aroundY(0.75)
            * Quaternion(axis: Vector3(1, 0, 0), angle: 0.6)
        let phonePose = Pose(position: Vector3(1, 1.4, -2), orientation: tilted)

        let alignment = RobotAlignment.placingOrigin(atARPose: phonePose)
        XCTAssertTrue(alignment.originInAR.isApproximatelyEqual(to: phonePose.position, tolerance: 1e-12))

        // The robot's forward axis must be level, whatever the phone's pitch.
        let forward = alignment.arPose(fromROSOdometry: Pose(position: Vector3(1, 0, 0))).position
            - alignment.originInAR
        XCTAssertEqual(forward.y, 0, accuracy: 1e-9)
    }

    func testAlignAtPhoneMakesRobotForwardMatchPhoneHeading() {
        // The "open the app standing at the robot" flow: after aligning, one
        // metre of robot forward motion must appear one metre ahead of where
        // the phone was pointing.
        let heading = 1.2
        let phonePose = Pose(position: Vector3(3, 1.5, -1), orientation: Quaternion.aroundY(heading))
        let alignment = RobotAlignment.placingOrigin(atARPose: phonePose)

        let placed = alignment.arPose(fromROSOdometry: Pose(position: Vector3(1, 0, 0)))
        let displacement = placed.position - phonePose.position
        let phoneForward = phonePose.orientation.rotate(Vector3(0, 0, -1))

        XCTAssertEqual(displacement.length, 1.0, accuracy: 1e-9)
        XCTAssertTrue(displacement.isApproximatelyEqual(to: phoneForward, tolerance: 1e-9))
    }

    static var allTests: [(String, (GeometryTests) -> () throws -> Void)] {
        [
            ("testIdentityRotationLeavesVectorsAlone", testIdentityRotationLeavesVectorsAlone),
            ("testQuarterTurnAboutZTakesXToY", testQuarterTurnAboutZTakesXToY),
            ("testMultiplicationAppliesRightHandSideFirst", testMultiplicationAppliesRightHandSideFirst),
            ("testConjugateUndoesRotation", testConjugateUndoesRotation),
            ("testRotationMatrixRoundTrip", testRotationMatrixRoundTrip),
            ("testRotationMatrixRoundTripAtEveryTraceBranch", testRotationMatrixRoundTripAtEveryTraceBranch),
            ("testRollPitchYawMatchesConstruction", testRollPitchYawMatchesConstruction),
            ("testYawAroundYIsTheInverseOfAroundY", testYawAroundYIsTheInverseOfAroundY),
            ("testSlerpEndpointsAreExact", testSlerpEndpointsAreExact),
            ("testSlerpMidpointIsHalfTheAngle", testSlerpMidpointIsHalfTheAngle),
            ("testSlerpTakesTheShortPathAcrossTheDoubleCover", testSlerpTakesTheShortPathAcrossTheDoubleCover),
            ("testSlerpClampsOutOfRangeParameters", testSlerpClampsOutOfRangeParameters),
            ("testSlerpOfNearlyIdenticalRotationsStaysUnit", testSlerpOfNearlyIdenticalRotationsStaysUnit),
            ("testPoseCompositionMatchesSequentialApplication", testPoseCompositionMatchesSequentialApplication),
            ("testPoseInverseRoundTrips", testPoseInverseRoundTrips),
            ("testRosAxesMapOntoARKitAxes", testRosAxesMapOntoARKitAxes),
            ("testAxisConversionPreservesHandednessAndLength", testAxisConversionPreservesHandednessAndLength),
            ("testPositionConversionRoundTrips", testPositionConversionRoundTrips),
            ("testAxisQuaternionAgreesWithThePositionMapping", testAxisQuaternionAgreesWithThePositionMapping),
            ("testInverseAxisQuaternionIsTheConjugate", testInverseAxisQuaternionIsTheConjugate),
            ("testOrientationConversionRoundTrips", testOrientationConversionRoundTrips),
            ("testConvertedOrientationPointsSceneKitForwardAlongRobotForward", testConvertedOrientationPointsSceneKitForwardAlongRobotForward),
            ("testRosYawBecomesArYawOfTheSameMagnitude", testRosYawBecomesArYawOfTheSameMagnitude),
            ("testIdentityAlignmentIsJustTheAxisChange", testIdentityAlignmentIsJustTheAxisChange),
            ("testAlignmentTranslatesTheOrigin", testAlignmentTranslatesTheOrigin),
            ("testAlignmentYawRotatesTheRobotFrame", testAlignmentYawRotatesTheRobotFrame),
            ("testAlignmentRoundTripsBackToOdom", testAlignmentRoundTripsBackToOdom),
            ("testPlacingOriginAtPhoneUsesHeadingOnly", testPlacingOriginAtPhoneUsesHeadingOnly),
            ("testAlignAtPhoneMakesRobotForwardMatchPhoneHeading", testAlignAtPhoneMakesRobotForwardMatchPhoneHeading),
        ]
    }
}
