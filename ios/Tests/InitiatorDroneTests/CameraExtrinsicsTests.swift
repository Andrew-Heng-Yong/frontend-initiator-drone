#if canImport(XCTest)
import XCTest
// In Xcode the tests compile as their own module; the headless runner in
// Scripts/ compiles them alongside the sources instead, so neither import
// exists there.
@testable import InitiatorDrone
#endif
import Foundation

/// The camera mount: sign conventions, the composed optical-to-node transform,
/// and its effect on a built cloud.
///
/// Signs are the whole risk here. A mount entered as "10 degrees down" that
/// silently tilts the cloud up is worse than no mount setting at all, because
/// the operator will trust it and adjust the wrong way.
final class CameraExtrinsicsTests: XCTestCase {

    private func info(width: Int, height: Int, fx: Double, fy: Double, cx: Double, cy: Double) -> CameraInfoMessage {
        CameraInfoMessage(
            stamp: 1,
            frameId: "camera_depth_optical_frame",
            width: width,
            height: height,
            intrinsics: [fx, 0, cx, 0, fy, cy, 0, 0, 1],
            projection: nil,
            distortion: [],
            distortionModel: "plumb_bob"
        )
    }

    private func white(_ depth: Double) -> RGBColor { RGBColor(255, 255, 255) }

    // MARK: - Sign conventions

    func testPositivePitchTiltsTheCameraDown() {
        let mount = CameraExtrinsics(pitchDegrees: 90)
        // Optical +Z is the camera's forward ray. Pitched fully down, it must
        // point along body -Z, which is down in REP-103.
        let forward = mount.bodyPoint(fromOptical: Vector3(0, 0, 1))
        XCTAssertTrue(
            forward.isApproximatelyEqual(to: Vector3(0, 0, -1), tolerance: 1e-9),
            "positive pitch must aim the camera at the floor, got \(forward)"
        )
    }

    func testPositiveRollDropsTheRightSide() {
        let mount = CameraExtrinsics(rollDegrees: 90)
        // Optical +X points out of the camera's right edge. Rolled fully to the
        // right, the right edge must end up pointing down: body -Z.
        let right = mount.bodyPoint(fromOptical: Vector3(1, 0, 0))
        XCTAssertTrue(
            right.isApproximatelyEqual(to: Vector3(0, 0, -1), tolerance: 1e-9),
            "positive roll must drop the camera's right side, got \(right)"
        )
    }

    func testTranslationIsInBodyAxes() {
        let mount = CameraExtrinsics(x: 0.12, y: -0.03, z: 0.08)
        // A point at the camera's own origin sits exactly at the mount offset.
        let origin = mount.bodyPoint(fromOptical: .zero)
        XCTAssertTrue(origin.isApproximatelyEqual(to: Vector3(0.12, -0.03, 0.08), tolerance: 1e-9))
    }

    func testRotationHappensBeforeTheOffsetIsAdded() {
        // Getting this backwards — offsetting in optical axes then rotating —
        // moves the whole cloud instead of just the camera, and the error grows
        // with the tilt rather than staying constant.
        let mount = CameraExtrinsics(x: 1.0, pitchDegrees: 90)
        let point = mount.bodyPoint(fromOptical: Vector3(0, 0, 2))
        // 2 m along a fully-downward ray, from a camera 1 m ahead of the origin.
        XCTAssertTrue(
            point.isApproximatelyEqual(to: Vector3(1.0, 0, -2.0), tolerance: 1e-9),
            "expected (1, 0, -2), got \(point)"
        )
    }

    // MARK: - The composed transform

    /// The identity mount must reproduce exactly the shortcut `DepthPointCloud`
    /// documents, or the simple case and the general case have drifted apart.
    func testIdentityMountCollapsesToFlippingYAndZ() {
        let transform = CameraExtrinsics.identity.opticalToARNode
        for sample in [
            Vector3(0, 0, 1), Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0.3, -0.7, 2.1),
        ] {
            let mapped = transform(sample)
            let shortcut = Vector3(sample.x, -sample.y, -sample.z)
            XCTAssertTrue(
                mapped.isApproximatelyEqual(to: shortcut, tolerance: 1e-9),
                "optical \(sample): transform gave \(mapped), shortcut \(shortcut)"
            )
        }
    }

    /// The precomputed transform is an optimisation of `bodyPoint` followed by
    /// `FrameConversion`. If the two ever disagree, the cloud and everything
    /// else that reasons in body coordinates are drawing different worlds.
    func testPrecomputedTransformAgreesWithGoingThroughTheFrames() {
        let mounts = [
            CameraExtrinsics.identity,
            CameraExtrinsics(x: 0.1, y: 0.02, z: -0.05),
            CameraExtrinsics(pitchDegrees: 12),
            CameraExtrinsics(rollDegrees: -7),
            CameraExtrinsics(x: 0.14, y: -0.03, z: 0.09, pitchDegrees: 18, rollDegrees: 4),
        ]
        let samples = [
            Vector3(0, 0, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
            Vector3(0.4, -0.9, 3.2), Vector3(-1.1, 0.6, 0.8),
        ]

        for mount in mounts {
            let transform = mount.opticalToARNode
            for sample in samples {
                let viaFrames = FrameConversion.arPosition(
                    fromROS: mount.bodyPoint(fromOptical: sample)
                )
                XCTAssertTrue(
                    transform(sample).isApproximatelyEqual(to: viaFrames, tolerance: 1e-9),
                    "mount \(mount) sample \(sample): \(transform(sample)) vs \(viaFrames)"
                )
            }
        }
    }

    /// The frustum is posed with `poseInARNode` while the cloud folds the mount
    /// into its vertices. Two different mechanisms, one geometry — so a point
    /// placed by the transform must land where the frustum node would put it.
    func testFrustumPoseAndCloudTransformDescribeTheSameCamera() {
        let mount = CameraExtrinsics(x: 0.14, y: -0.03, z: 0.09, pitchDegrees: 18, rollDegrees: 4)
        let pose = mount.poseInARNode
        let transform = mount.opticalToARNode

        for sample in [Vector3(0, 0, 1), Vector3(0.5, 0.2, 2.0), Vector3(-0.3, -0.4, 1.1)] {
            // What the frustum node would produce: the point expressed in the
            // node's own axes, then moved by the node's pose.
            let inNodeAxes = Vector3(sample.x, -sample.y, -sample.z)
            let viaNode = pose.apply(to: inNodeAxes)
            XCTAssertTrue(
                transform(sample).isApproximatelyEqual(to: viaNode, tolerance: 1e-9),
                "sample \(sample): cloud \(transform(sample)) vs frustum \(viaNode)"
            )
        }
    }

    func testCameraForwardPointsAlongTheNodeMinusZWithNoMount() {
        // FrameConversion sends ROS forward to node -Z, so an unmounted camera
        // looking straight ahead must too.
        let forward = CameraExtrinsics.identity.opticalToARNode(Vector3(0, 0, 1))
        XCTAssertTrue(forward.isApproximatelyEqual(to: Vector3(0, 0, -1), tolerance: 1e-9))
    }

    // MARK: - Effect on a built cloud

    func testMountOffsetShiftsTheWholeCloud() {
        let image = ScalarImage(width: 1, height: 1, values: [2.0], unit: .metres)
        let cameraInfo = info(width: 1, height: 1, fx: 100, fy: 100, cx: 0, cy: 0)
        let settings = PointCloudSettings(pixelStride: 1, maximumPoints: 10)

        let unmounted = DepthPointCloud.build(
            from: image, cameraInfo: cameraInfo, settings: settings, colorFor: white
        )
        // 0.5 m above base_link. ROS +Z (up) maps to ARKit +Y (up).
        let raised = DepthPointCloud.build(
            from: image,
            cameraInfo: cameraInfo,
            settings: settings,
            extrinsics: CameraExtrinsics(z: 0.5),
            colorFor: white
        )

        XCTAssertEqual(unmounted.count, 1)
        XCTAssertEqual(raised.count, 1)
        XCTAssertEqual(raised.positions[0] - unmounted.positions[0], 0, accuracy: 1e-5)
        XCTAssertEqual(raised.positions[1] - unmounted.positions[1], 0.5, accuracy: 1e-5)
        XCTAssertEqual(raised.positions[2] - unmounted.positions[2], 0, accuracy: 1e-5)
    }

    /// The concrete version of the warning on the settings screen: a camera
    /// tilted down puts distant points below the robot, not level with it.
    func testDownwardPitchPutsDistantPointsBelowTheRobot() {
        let cloud = DepthPointCloud.build(
            from: ScalarImage(width: 1, height: 1, values: [2.0], unit: .metres),
            cameraInfo: info(width: 1, height: 1, fx: 100, fy: 100, cx: 0, cy: 0),
            settings: PointCloudSettings(pixelStride: 1, maximumPoints: 10),
            extrinsics: CameraExtrinsics(pitchDegrees: 15),
            colorFor: white
        )
        XCTAssertEqual(cloud.count, 1)
        // ARKit +Y is up, so a downward tilt must give a negative Y.
        XCTAssertEqual(Double(cloud.positions[1]), -2.0 * sin(15 * .pi / 180), accuracy: 1e-5)
        XCTAssertEqual(Double(cloud.positions[2]), -2.0 * cos(15 * .pi / 180), accuracy: 1e-5)
    }

    func testIdentityMountLeavesTheCloudExactlyWhereItWas() {
        let values = [Float](repeating: 1.5, count: 16)
        let image = ScalarImage(width: 4, height: 4, values: values, unit: .metres)
        let cameraInfo = info(width: 4, height: 4, fx: 100, fy: 100, cx: 2, cy: 2)
        let settings = PointCloudSettings(pixelStride: 1, maximumPoints: 100)

        let implicit = DepthPointCloud.build(
            from: image, cameraInfo: cameraInfo, settings: settings, colorFor: white
        )
        let explicit = DepthPointCloud.build(
            from: image,
            cameraInfo: cameraInfo,
            settings: settings,
            extrinsics: .identity,
            colorFor: white
        )
        XCTAssertEqual(implicit.positions, explicit.positions)
    }

    // MARK: - Persistence

    func testExtrinsicsSurviveASettingsRoundTrip() throws {
        var settings = AppSettings()
        settings.cameraExtrinsics = CameraExtrinsics(
            x: 0.135, y: -0.02, z: 0.071, pitchDegrees: 12.5, rollDegrees: -1.5
        )
        let decoded = try JSONDecoder().decode(
            AppSettings.self,
            from: try JSONEncoder().encode(settings)
        )
        XCTAssertEqual(decoded.cameraExtrinsics, settings.cameraExtrinsics)
        XCTAssertFalse(decoded.cameraExtrinsics.isIdentity)
    }

    func testDefaultMountIsIdentity() {
        XCTAssertTrue(AppSettings().cameraExtrinsics.isIdentity)
    }

    static var allTests: [(String, (CameraExtrinsicsTests) -> () throws -> Void)] {
        [
        ("testPositivePitchTiltsTheCameraDown", testPositivePitchTiltsTheCameraDown),
        ("testPositiveRollDropsTheRightSide", testPositiveRollDropsTheRightSide),
        ("testTranslationIsInBodyAxes", testTranslationIsInBodyAxes),
        ("testRotationHappensBeforeTheOffsetIsAdded", testRotationHappensBeforeTheOffsetIsAdded),
        ("testIdentityMountCollapsesToFlippingYAndZ", testIdentityMountCollapsesToFlippingYAndZ),
        ("testPrecomputedTransformAgreesWithGoingThroughTheFrames", testPrecomputedTransformAgreesWithGoingThroughTheFrames),
        ("testFrustumPoseAndCloudTransformDescribeTheSameCamera", testFrustumPoseAndCloudTransformDescribeTheSameCamera),
        ("testCameraForwardPointsAlongTheNodeMinusZWithNoMount", testCameraForwardPointsAlongTheNodeMinusZWithNoMount),
        ("testMountOffsetShiftsTheWholeCloud", testMountOffsetShiftsTheWholeCloud),
        ("testDownwardPitchPutsDistantPointsBelowTheRobot", testDownwardPitchPutsDistantPointsBelowTheRobot),
        ("testIdentityMountLeavesTheCloudExactlyWhereItWas", testIdentityMountLeavesTheCloudExactlyWhereItWas),
        ("testExtrinsicsSurviveASettingsRoundTrip", testExtrinsicsSurviveASettingsRoundTrip),
        ("testDefaultMountIsIdentity", testDefaultMountIsIdentity),
        ]
    }
}
