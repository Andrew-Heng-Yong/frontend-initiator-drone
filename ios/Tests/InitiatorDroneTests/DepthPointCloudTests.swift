#if canImport(XCTest)
import XCTest
// In Xcode the tests compile as their own module; the headless runner in
// Scripts/ compiles them alongside the sources instead, so neither import
// exists there.
@testable import InitiatorDrone
#endif
import Foundation

/// Deprojection of a depth frame into a point cloud, and the frame hops it
/// depends on.
final class DepthPointCloudTests: XCTestCase {

    // MARK: - Helpers

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

    private func image(_ values: [Float], width: Int, height: Int) -> ScalarImage {
        ScalarImage(width: width, height: height, values: values, unit: .metres)
    }

    private func white(_ depth: Double) -> RGBColor { RGBColor(255, 255, 255) }

    // MARK: - Pinhole maths

    func testCentrePixelProjectsStraightAhead() {
        let point = DepthPointCloud.opticalPoint(u: 320, v: 240, depth: 2.0, fx: 500, fy: 500, cx: 320, cy: 240)
        XCTAssertEqual(point.x, 0, accuracy: 1e-9)
        XCTAssertEqual(point.y, 0, accuracy: 1e-9)
        XCTAssertEqual(point.z, 2.0, accuracy: 1e-9)
    }

    func testOffCentrePixelScalesWithDepth() {
        let near = DepthPointCloud.opticalPoint(u: 420, v: 240, depth: 1.0, fx: 500, fy: 500, cx: 320, cy: 240)
        let far = DepthPointCloud.opticalPoint(u: 420, v: 240, depth: 2.0, fx: 500, fy: 500, cx: 320, cy: 240)
        // Same ray, twice the distance.
        XCTAssertEqual(near.x, 0.2, accuracy: 1e-9)
        XCTAssertEqual(far.x, 0.4, accuracy: 1e-9)
        XCTAssertEqual(far.z, 2 * near.z, accuracy: 1e-9)
    }

    func testPixelRightAndBelowCentreIsPositiveXAndY() {
        // The optical frame is +X right, +Y DOWN. A pixel below the principal
        // point must therefore have positive Y, not negative.
        let point = DepthPointCloud.opticalPoint(u: 400, v: 300, depth: 1.0, fx: 500, fy: 500, cx: 320, cy: 240)
        XCTAssertGreaterThan(point.x, 0)
        XCTAssertGreaterThan(point.y, 0)
    }

    // MARK: - Frame hops

    func testOpticalToBodyMatchesREP103() {
        // Optical +Z (forward) must become body +X (forward).
        let forward = DepthPointCloud.bodyPoint(fromOptical: Vector3(0, 0, 1))
        XCTAssertTrue(forward.isApproximatelyEqual(to: Vector3(1, 0, 0)))

        // Optical +X (right) must become body -Y, because body +Y is left.
        let right = DepthPointCloud.bodyPoint(fromOptical: Vector3(1, 0, 0))
        XCTAssertTrue(right.isApproximatelyEqual(to: Vector3(0, -1, 0)))

        // Optical +Y (down) must become body -Z, because body +Z is up.
        let down = DepthPointCloud.bodyPoint(fromOptical: Vector3(0, 1, 0))
        XCTAssertTrue(down.isApproximatelyEqual(to: Vector3(0, 0, -1)))
    }

    /// The whole reason the build loop can get away with `(X, -Y, -Z)`.
    func testComposedOpticalToARKitEqualsFlippingYAndZ() {
        for sample in [Vector3(0, 0, 1), Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0.3, -0.7, 2.1)] {
            let viaFrames = FrameConversion.arPosition(
                fromROS: DepthPointCloud.bodyPoint(fromOptical: sample)
            )
            let shortcut = Vector3(sample.x, -sample.y, -sample.z)
            XCTAssertTrue(
                viaFrames.isApproximatelyEqual(to: shortcut, tolerance: 1e-9),
                "optical \(sample): frames gave \(viaFrames), shortcut gave \(shortcut)"
            )
        }
    }

    func testForwardDepthLandsOnARKitMinusZ() {
        // A pixel at the principal point, 2 m away, must end up 2 m along the
        // node's forward axis, which in SceneKit is -Z.
        let cloud = DepthPointCloud.build(
            from: image([2.0], width: 1, height: 1),
            cameraInfo: info(width: 1, height: 1, fx: 100, fy: 100, cx: 0, cy: 0),
            settings: PointCloudSettings(pixelStride: 1, maximumPoints: 10),
            colorFor: white
        )
        XCTAssertEqual(cloud.count, 1)
        XCTAssertEqual(cloud.positions[0], 0, accuracy: 1e-6)
        XCTAssertEqual(cloud.positions[1], 0, accuracy: 1e-6)
        XCTAssertEqual(cloud.positions[2], -2.0, accuracy: 1e-6)
    }

    // MARK: - Filtering

    func testInvalidSamplesAreSkipped() {
        let cloud = DepthPointCloud.build(
            from: image([.nan, 1.0, .nan, 2.0], width: 4, height: 1),
            cameraInfo: info(width: 4, height: 1, fx: 100, fy: 100, cx: 2, cy: 0),
            settings: PointCloudSettings(pixelStride: 1, maximumPoints: 100),
            colorFor: white
        )
        XCTAssertEqual(cloud.count, 2, "NaN samples must not become points at the origin")
    }

    func testRangeLimitsAreApplied() {
        let cloud = DepthPointCloud.build(
            from: image([0.05, 1.0, 50.0], width: 3, height: 1),
            cameraInfo: info(width: 3, height: 1, fx: 100, fy: 100, cx: 1, cy: 0),
            settings: PointCloudSettings(
                pixelStride: 1, maximumPoints: 100, minimumDepth: 0.2, maximumDepth: 8.0
            ),
            colorFor: white
        )
        XCTAssertEqual(cloud.count, 1)
        XCTAssertEqual(cloud.positions[2], -1.0, accuracy: 1e-6)
    }

    func testStrideWidensToRespectTheMaximumPointCount() {
        // 100x100 at stride 1 would be 10,000 points; the cap is 500.
        let values = [Float](repeating: 2.0, count: 100 * 100)
        let cloud = DepthPointCloud.build(
            from: image(values, width: 100, height: 100),
            cameraInfo: info(width: 100, height: 100, fx: 100, fy: 100, cx: 50, cy: 50),
            settings: PointCloudSettings(pixelStride: 1, maximumPoints: 500),
            colorFor: white
        )
        XCTAssertGreaterThan(cloud.count, 0)
        XCTAssertLessThanOrEqual(cloud.count, 500)
    }

    func testDisabledSettingsProduceNothing() {
        let cloud = DepthPointCloud.build(
            from: image([1.0], width: 1, height: 1),
            cameraInfo: info(width: 1, height: 1, fx: 100, fy: 100, cx: 0, cy: 0),
            settings: PointCloudSettings(isEnabled: false),
            colorFor: white
        )
        XCTAssertTrue(cloud.isEmpty)
    }

    func testBuffersStayInStepWithTheCount() {
        let values: [Float] = [1.0, .nan, 2.0, 3.0]
        let cloud = DepthPointCloud.build(
            from: image(values, width: 4, height: 1),
            cameraInfo: info(width: 4, height: 1, fx: 100, fy: 100, cx: 2, cy: 0),
            settings: PointCloudSettings(pixelStride: 1, maximumPoints: 100),
            colorFor: white
        )
        // SceneKit reads both sources with the same vector count; a mismatch
        // here would be an out-of-bounds read on the GPU.
        XCTAssertEqual(cloud.positions.count, cloud.count * 3)
        XCTAssertEqual(cloud.colors.count, cloud.count * 3)
    }

    // MARK: - Intrinsics

    func testIntrinsicsAreUsedDirectlyWhenSizesMatch() {
        let resolved = DepthPointCloud.scaledIntrinsics(
            for: image([Float](repeating: 1, count: 64 * 48), width: 64, height: 48),
            cameraInfo: info(width: 64, height: 48, fx: 50, fy: 50, cx: 32, cy: 24)
        )
        XCTAssertEqual(resolved?.fx, 50)
        XCTAssertEqual(resolved?.cx, 32)
    }

    func testIntrinsicsScaleWhenTheImageWasResized() {
        let resolved = DepthPointCloud.scaledIntrinsics(
            for: image([Float](repeating: 1, count: 32 * 24), width: 32, height: 24),
            cameraInfo: info(width: 64, height: 48, fx: 50, fy: 50, cx: 32, cy: 24)
        )
        XCTAssertEqual(resolved?.fx, 25)
        XCTAssertEqual(resolved?.cy, 12)
    }

    func testMissingIntrinsicsProduceNoCloud() {
        // Better an empty scene than a plausible-looking cloud built on a
        // made-up focal length.
        let noIntrinsics = CameraInfoMessage(
            stamp: 1, frameId: "f", width: 4, height: 1,
            intrinsics: nil, projection: nil, distortion: [], distortionModel: ""
        )
        let cloud = DepthPointCloud.build(
            from: image([1, 1, 1, 1], width: 4, height: 1),
            cameraInfo: noIntrinsics,
            settings: PointCloudSettings(pixelStride: 1),
            colorFor: white
        )
        XCTAssertTrue(cloud.isEmpty)
    }

    // MARK: - Store

    func testStoreOnlyReportsNewGenerations() {
        let store = PointCloudStore()
        XCTAssertNil(store.take(ifNewerThan: 0), "an untouched store has nothing new")

        store.store(PointCloudBuffer(positions: [0, 0, 0], colors: [1, 2, 3], count: 1, stamp: 5))
        guard let first = store.take(ifNewerThan: 0) else {
            return XCTFail("expected the stored cloud")
        }
        XCTAssertEqual(first.cloud.count, 1)
        XCTAssertNil(store.take(ifNewerThan: first.generation), "same generation must not rebuild")

        store.store(PointCloudBuffer(positions: [1, 1, 1], colors: [4, 5, 6], count: 1, stamp: 6))
        XCTAssertNotNil(store.take(ifNewerThan: first.generation))
    }

    func testResetClearsAndBumpsGeneration() {
        let store = PointCloudStore()
        store.store(PointCloudBuffer(positions: [0, 0, 0], colors: [1, 2, 3], count: 1, stamp: 1))
        let generation = store.currentGeneration
        store.reset()
        guard let update = store.take(ifNewerThan: generation) else {
            return XCTFail("reset must be observable, or stale points hang in the air")
        }
        XCTAssertEqual(update.cloud.count, 0)
    }

    // MARK: - Fixtures static robot

    func testStaticFixtureRobotHoldsItsPose() {
        let transport = SimulatedRosbridgeTransport()
        transport.isStatic = true

        let first = transport.simulatedPose(at: 0)
        let later = transport.simulatedPose(at: 37.5)
        XCTAssertTrue(
            first.isApproximatelyEqual(to: later),
            "a parked robot must not drift; got \(first.position) then \(later.position)"
        )
        XCTAssertTrue(first.position.isApproximatelyEqual(to: .zero))
    }

    func testMovingFixtureRobotStillMoves() {
        let transport = SimulatedRosbridgeTransport()
        transport.isStatic = false
        let first = transport.simulatedPose(at: 0)
        let later = transport.simulatedPose(at: 3.0)
        XCTAssertGreaterThan(first.position.distance(to: later.position), 0.1)
    }

    func testStaticPoseIsConfigurable() {
        let transport = SimulatedRosbridgeTransport()
        transport.isStatic = true
        transport.staticPosition = Vector3(1.5, -0.5, 0.2)
        transport.staticHeading = .pi / 2

        let pose = transport.simulatedPose(at: 12.0)
        XCTAssertTrue(pose.position.isApproximatelyEqual(to: Vector3(1.5, -0.5, 0.2)))
        XCTAssertEqual(pose.orientation.yawAroundZ, .pi / 2, accuracy: 1e-9)
    }

    static var allTests: [(String, (DepthPointCloudTests) -> () throws -> Void)] {
        [
        ("testCentrePixelProjectsStraightAhead", testCentrePixelProjectsStraightAhead),
        ("testOffCentrePixelScalesWithDepth", testOffCentrePixelScalesWithDepth),
        ("testPixelRightAndBelowCentreIsPositiveXAndY", testPixelRightAndBelowCentreIsPositiveXAndY),
        ("testOpticalToBodyMatchesREP103", testOpticalToBodyMatchesREP103),
        ("testComposedOpticalToARKitEqualsFlippingYAndZ", testComposedOpticalToARKitEqualsFlippingYAndZ),
        ("testForwardDepthLandsOnARKitMinusZ", testForwardDepthLandsOnARKitMinusZ),
        ("testInvalidSamplesAreSkipped", testInvalidSamplesAreSkipped),
        ("testRangeLimitsAreApplied", testRangeLimitsAreApplied),
        ("testStrideWidensToRespectTheMaximumPointCount", testStrideWidensToRespectTheMaximumPointCount),
        ("testDisabledSettingsProduceNothing", testDisabledSettingsProduceNothing),
        ("testBuffersStayInStepWithTheCount", testBuffersStayInStepWithTheCount),
        ("testIntrinsicsAreUsedDirectlyWhenSizesMatch", testIntrinsicsAreUsedDirectlyWhenSizesMatch),
        ("testIntrinsicsScaleWhenTheImageWasResized", testIntrinsicsScaleWhenTheImageWasResized),
        ("testMissingIntrinsicsProduceNoCloud", testMissingIntrinsicsProduceNoCloud),
        ("testStoreOnlyReportsNewGenerations", testStoreOnlyReportsNewGenerations),
        ("testResetClearsAndBumpsGeneration", testResetClearsAndBumpsGeneration),
        ("testStaticFixtureRobotHoldsItsPose", testStaticFixtureRobotHoldsItsPose),
        ("testMovingFixtureRobotStillMoves", testMovingFixtureRobotStillMoves),
        ("testStaticPoseIsConfigurable", testStaticPoseIsConfigurable),
        ]
    }
}
