#if canImport(XCTest)
import XCTest
// In Xcode the tests compile as their own module; the headless runner in
// Scripts/ compiles them alongside the sources instead, so neither import
// exists there.
@testable import InitiatorDrone
#endif
import Foundation

/// Time-based pose sampling: the buffer, the clock estimate, and the sampler
/// the render loop reads.
final class OdometryTests: XCTestCase {

    private func pose(_ x: Double, yaw: Double = 0) -> Pose {
        Pose(position: Vector3(x, 0, 0), orientation: Quaternion.aroundZ(yaw))
    }

    private func sample(_ stamp: Double, _ x: Double, yaw: Double = 0) -> StampedPose {
        StampedPose(stamp: stamp, pose: pose(x, yaw: yaw))
    }

    // MARK: - Interpolation

    func testEmptyBufferReturnsNothing() {
        let buffer = OdometryBuffer()
        XCTAssertNil(buffer.pose(at: 100))
        XCTAssertNil(buffer.age(at: 100))
        XCTAssertTrue(buffer.isStale(at: 100))
    }

    func testInterpolatesPositionLinearlyBetweenSamples() {
        var buffer = OdometryBuffer()
        buffer.append(sample(10.0, 0.0))
        buffer.append(sample(11.0, 2.0))

        let result = buffer.pose(at: 10.25)
        XCTAssertEqual(result?.kind, .interpolated)
        XCTAssertEqual(result?.pose.position.x ?? -1, 0.5, accuracy: 1e-9)
    }

    func testInterpolatesOrientationSpherically() {
        var buffer = OdometryBuffer()
        buffer.append(sample(0.0, 0.0, yaw: 0.0))
        buffer.append(sample(1.0, 0.0, yaw: 1.0))

        let result = buffer.pose(at: 0.5)
        XCTAssertEqual(result?.pose.orientation.yawAroundZ ?? 0, 0.5, accuracy: 1e-9)
    }

    func testExactStampReturnsTheSampleUnchanged() {
        var buffer = OdometryBuffer()
        buffer.append(sample(5.0, 1.0))
        buffer.append(sample(6.0, 2.0))

        let result = buffer.pose(at: 6.0)
        XCTAssertEqual(result?.kind, .exact)
        XCTAssertEqual(result?.pose.position.x ?? -1, 2.0, accuracy: 1e-12)
    }

    func testInterpolationIsCorrectAcrossManySamples() {
        var buffer = OdometryBuffer()
        for index in 0..<50 {
            buffer.append(sample(Double(index) * 0.1, Double(index)))
        }
        // Halfway between samples 20 and 21.
        let result = buffer.pose(at: 2.05)
        XCTAssertEqual(result?.kind, .interpolated)
        XCTAssertEqual(result?.pose.position.x ?? -1, 20.5, accuracy: 1e-9)
    }

    // MARK: - Clamping and extrapolation

    func testRequestBeforeTheBufferClampsToTheOldest() {
        var buffer = OdometryBuffer()
        buffer.append(sample(10.0, 3.0))
        buffer.append(sample(11.0, 4.0))

        let result = buffer.pose(at: 5.0)
        XCTAssertEqual(result?.kind, .clampedToOldest)
        XCTAssertEqual(result?.pose.position.x ?? -1, 3.0, accuracy: 1e-12)
    }

    func testRequestAfterTheBufferHoldsTheNewestByDefault() {
        var buffer = OdometryBuffer()
        buffer.append(sample(10.0, 3.0))
        buffer.append(sample(11.0, 4.0))

        let result = buffer.pose(at: 11.4)
        XCTAssertEqual(result?.kind, .clampedToNewest)
        XCTAssertEqual(result?.pose.position.x ?? -1, 4.0, accuracy: 1e-12)
        XCTAssertEqual(result?.ageBeyondNewest ?? 0, 0.4, accuracy: 1e-9)
    }

    func testExtrapolationIntegratesTheBodyTwistWhenAllowed() {
        var buffer = OdometryBuffer()
        buffer.append(StampedPose(
            stamp: 10.0,
            pose: Pose(position: Vector3(1, 0, 0), orientation: .identity),
            linearVelocity: Vector3(2, 0, 0)
        ))

        let result = buffer.pose(at: 10.1, maxExtrapolation: 0.2)
        XCTAssertEqual(result?.kind, .extrapolated)
        XCTAssertEqual(result?.pose.position.x ?? -1, 1.2, accuracy: 1e-9)
    }

    func testExtrapolationUsesTheBodyFrameNotTheWorldFrame() {
        // The twist in nav_msgs/Odometry is expressed in the child frame, so a
        // robot facing +Y that reports forward velocity must move along +Y.
        var buffer = OdometryBuffer()
        buffer.append(StampedPose(
            stamp: 0,
            pose: Pose(position: .zero, orientation: Quaternion.aroundZ(.pi / 2)),
            linearVelocity: Vector3(1, 0, 0)
        ))

        let result = buffer.pose(at: 1.0, maxExtrapolation: 2.0)
        XCTAssertEqual(result?.pose.position.x ?? -1, 0.0, accuracy: 1e-9)
        XCTAssertEqual(result?.pose.position.y ?? -1, 1.0, accuracy: 1e-9)
    }

    func testExtrapolationStopsAtTheLimit() {
        var buffer = OdometryBuffer()
        buffer.append(StampedPose(
            stamp: 10.0,
            pose: Pose(position: Vector3(1, 0, 0)),
            linearVelocity: Vector3(2, 0, 0)
        ))

        let result = buffer.pose(at: 10.5, maxExtrapolation: 0.2)
        XCTAssertEqual(result?.kind, .clampedToNewest)
        XCTAssertEqual(result?.pose.position.x ?? -1, 1.0, accuracy: 1e-12)
    }

    func testExtrapolationRotatesWithAngularVelocity() {
        var buffer = OdometryBuffer()
        buffer.append(StampedPose(
            stamp: 0,
            pose: Pose(),
            angularVelocity: Vector3(0, 0, 1.0)
        ))
        let result = buffer.pose(at: 0.5, maxExtrapolation: 1.0)
        XCTAssertEqual(result?.pose.orientation.yawAroundZ ?? 0, 0.5, accuracy: 1e-9)
    }

    // MARK: - Ordering and bounds

    func testOutOfOrderSamplesAreInsertedNotDropped() {
        var buffer = OdometryBuffer()
        buffer.append(sample(3.0, 3.0))
        buffer.append(sample(1.0, 1.0))
        buffer.append(sample(2.0, 2.0))

        XCTAssertEqual(buffer.count, 3)
        XCTAssertEqual(buffer.oldest?.stamp, 1.0)
        XCTAssertEqual(buffer.newest?.stamp, 3.0)
        XCTAssertEqual(buffer.pose(at: 1.5)?.pose.position.x ?? -1, 1.5, accuracy: 1e-9)
    }

    func testDuplicateStampReplacesRatherThanCreatingAZeroLengthInterval() {
        var buffer = OdometryBuffer()
        buffer.append(sample(1.0, 1.0))
        buffer.append(sample(1.0, 9.0))

        XCTAssertEqual(buffer.count, 1)
        XCTAssertEqual(buffer.newest?.pose.position.x, 9.0)
    }

    func testNonFiniteSamplesAreRejected() {
        var buffer = OdometryBuffer()
        buffer.append(StampedPose(stamp: .nan, pose: Pose()))
        buffer.append(StampedPose(stamp: 1.0, pose: Pose(position: Vector3(.infinity, 0, 0))))
        XCTAssertEqual(buffer.count, 0)
    }

    func testHistoryHorizonBoundsTheBuffer() {
        var buffer = OdometryBuffer(capacity: 10_000, historyDuration: 1.0)
        for index in 0..<1000 {
            buffer.append(sample(Double(index) * 0.01, Double(index)))
        }
        XCTAssertLessThanOrEqual(buffer.span, 1.0 + 1e-9)
        XCTAssertLessThanOrEqual(buffer.count, 102)
    }

    func testCapacityBoundsTheBuffer() {
        var buffer = OdometryBuffer(capacity: 20, historyDuration: 10_000)
        for index in 0..<500 {
            buffer.append(sample(Double(index), Double(index)))
        }
        XCTAssertEqual(buffer.count, 20)
        XCTAssertEqual(buffer.newest?.stamp, 499.0)
    }

    func testResetEmptiesTheBuffer() {
        var buffer = OdometryBuffer()
        buffer.append(sample(1, 1))
        buffer.reset()
        XCTAssertTrue(buffer.isEmpty)
        XCTAssertNil(buffer.pose(at: 1))
    }

    // MARK: - Staleness

    func testStalenessUsesTheConfiguredThreshold() {
        var buffer = OdometryBuffer(stalenessThreshold: 0.5)
        buffer.append(sample(100.0, 0))

        XCTAssertFalse(buffer.isStale(at: 100.4))
        XCTAssertTrue(buffer.isStale(at: 100.6))
        XCTAssertEqual(buffer.age(at: 100.6) ?? 0, 0.6, accuracy: 1e-9)
    }

    // MARK: - Clock estimation

    func testClockEstimatorFindsTheOffsetFromTheLowestDelay() {
        var estimator = ClockOffsetEstimator()
        // The robot's clock is 120 s behind the phone's; latency varies.
        let trueOffset = 120.0
        for index in 0..<20 {
            let rosStamp = 1000.0 + Double(index) * 0.1
            let latency = index == 7 ? 0.002 : 0.05 + Double(index % 5) * 0.01
            estimator.observe(rosStamp: rosStamp, localTime: rosStamp + trueOffset + latency)
        }

        let offset = try? XCTUnwrap(estimator.offset)
        XCTAssertEqual(offset ?? 0, trueOffset + 0.002, accuracy: 1e-9)
    }

    func testClockEstimatorMapsBothDirections() {
        var estimator = ClockOffsetEstimator()
        estimator.observe(rosStamp: 1000, localTime: 1010)
        XCTAssertEqual(estimator.rosTime(fromLocal: 1010), 1000, accuracy: 1e-9)
        XCTAssertEqual(estimator.localTime(fromROS: 1000), 1010, accuracy: 1e-9)
    }

    func testClockEstimatorPassesTimeThroughBeforeItHasAnEstimate() {
        let estimator = ClockOffsetEstimator()
        XCTAssertFalse(estimator.hasEstimate)
        XCTAssertEqual(estimator.rosTime(fromLocal: 42), 42, accuracy: 1e-12)
    }

    func testClockEstimatorIgnoresUnsetStamps() {
        var estimator = ClockOffsetEstimator()
        estimator.observe(rosStamp: 0, localTime: 100)
        XCTAssertFalse(estimator.hasEstimate)
    }

    func testClockEstimatorTracksDriftAsTheWindowMovesOn() {
        var estimator = ClockOffsetEstimator(windowDuration: 5.0)
        // An early, very low-latency sample would otherwise pin the estimate
        // forever; it must age out of the window.
        estimator.observe(rosStamp: 100, localTime: 200)
        for index in 1...100 {
            let ros = 100.0 + Double(index) * 0.2
            estimator.observe(rosStamp: ros, localTime: ros + 105.0)
        }
        XCTAssertEqual(estimator.offset ?? 0, 105.0, accuracy: 1e-6)
    }

    // MARK: - Sampler

    func testSamplerNeedsAnAlignmentBeforeItReturnsAnARPose() {
        let sampler = OdometrySampler()
        sampler.append(sample(1000, 1.0), localTime: 1000)
        XCTAssertNil(sampler.poseInAR(atLocalTime: 1000))
        XCTAssertNotNil(sampler.poseInOdom(atLocalTime: 1000))
    }

    func testSamplerAppliesAlignmentAndClockOffsetTogether() {
        let sampler = OdometrySampler()
        // Robot clock 500 s behind the phone.
        sampler.append(StampedPose(stamp: 1000, pose: Pose(position: Vector3(0, 0, 0))), localTime: 1500)
        sampler.append(StampedPose(stamp: 1001, pose: Pose(position: Vector3(2, 0, 0))), localTime: 1501)
        sampler.setAlignment(RobotAlignment(originInAR: Vector3(10, 0, 0), yaw: 0))

        // Phone time 1500.5 maps to robot time 1000.5, halfway between samples.
        let result = sampler.poseInAR(atLocalTime: 1500.5)
        XCTAssertEqual(result?.sample.kind, .interpolated)
        // 1 m of ROS forward becomes 1 m along ARKit -Z, offset by the origin.
        XCTAssertEqual(result?.pose.position.x ?? -1, 10.0, accuracy: 1e-9)
        XCTAssertEqual(result?.pose.position.z ?? -1, -1.0, accuracy: 1e-9)
    }

    func testSamplerTrailIsExpressedInARCoordinates() {
        let sampler = OdometrySampler()
        for index in 0..<5 {
            sampler.append(
                StampedPose(stamp: Double(index), pose: Pose(position: Vector3(Double(index), 0, 0))),
                localTime: Double(index)
            )
        }
        sampler.setAlignment(RobotAlignment(originInAR: .zero, yaw: 0))

        let trail = sampler.trailInAR(maximumCount: 100)
        XCTAssertEqual(trail.count, 5)
        // ROS +X becomes ARKit -Z.
        XCTAssertEqual(trail.last?.z ?? 0, -4.0, accuracy: 1e-9)
    }

    func testSamplerReturnsNoTrailWithoutAlignment() {
        let sampler = OdometrySampler()
        sampler.append(sample(1, 1), localTime: 1)
        XCTAssertTrue(sampler.trailInAR().isEmpty)
    }

    func testSamplerResetClearsEverything() {
        let sampler = OdometrySampler()
        sampler.append(sample(1, 1), localTime: 1)
        sampler.reset()
        XCTAssertEqual(sampler.sampleCount, 0)
        XCTAssertNil(sampler.clockOffset)
    }

    static var allTests: [(String, (OdometryTests) -> () throws -> Void)] {
        [
            ("testEmptyBufferReturnsNothing", testEmptyBufferReturnsNothing),
            ("testInterpolatesPositionLinearlyBetweenSamples", testInterpolatesPositionLinearlyBetweenSamples),
            ("testInterpolatesOrientationSpherically", testInterpolatesOrientationSpherically),
            ("testExactStampReturnsTheSampleUnchanged", testExactStampReturnsTheSampleUnchanged),
            ("testInterpolationIsCorrectAcrossManySamples", testInterpolationIsCorrectAcrossManySamples),
            ("testRequestBeforeTheBufferClampsToTheOldest", testRequestBeforeTheBufferClampsToTheOldest),
            ("testRequestAfterTheBufferHoldsTheNewestByDefault", testRequestAfterTheBufferHoldsTheNewestByDefault),
            ("testExtrapolationIntegratesTheBodyTwistWhenAllowed", testExtrapolationIntegratesTheBodyTwistWhenAllowed),
            ("testExtrapolationUsesTheBodyFrameNotTheWorldFrame", testExtrapolationUsesTheBodyFrameNotTheWorldFrame),
            ("testExtrapolationStopsAtTheLimit", testExtrapolationStopsAtTheLimit),
            ("testExtrapolationRotatesWithAngularVelocity", testExtrapolationRotatesWithAngularVelocity),
            ("testOutOfOrderSamplesAreInsertedNotDropped", testOutOfOrderSamplesAreInsertedNotDropped),
            ("testDuplicateStampReplacesRatherThanCreatingAZeroLengthInterval", testDuplicateStampReplacesRatherThanCreatingAZeroLengthInterval),
            ("testNonFiniteSamplesAreRejected", testNonFiniteSamplesAreRejected),
            ("testHistoryHorizonBoundsTheBuffer", testHistoryHorizonBoundsTheBuffer),
            ("testCapacityBoundsTheBuffer", testCapacityBoundsTheBuffer),
            ("testResetEmptiesTheBuffer", testResetEmptiesTheBuffer),
            ("testStalenessUsesTheConfiguredThreshold", testStalenessUsesTheConfiguredThreshold),
            ("testClockEstimatorFindsTheOffsetFromTheLowestDelay", testClockEstimatorFindsTheOffsetFromTheLowestDelay),
            ("testClockEstimatorMapsBothDirections", testClockEstimatorMapsBothDirections),
            ("testClockEstimatorPassesTimeThroughBeforeItHasAnEstimate", testClockEstimatorPassesTimeThroughBeforeItHasAnEstimate),
            ("testClockEstimatorIgnoresUnsetStamps", testClockEstimatorIgnoresUnsetStamps),
            ("testClockEstimatorTracksDriftAsTheWindowMovesOn", testClockEstimatorTracksDriftAsTheWindowMovesOn),
            ("testSamplerNeedsAnAlignmentBeforeItReturnsAnARPose", testSamplerNeedsAnAlignmentBeforeItReturnsAnARPose),
            ("testSamplerAppliesAlignmentAndClockOffsetTogether", testSamplerAppliesAlignmentAndClockOffsetTogether),
            ("testSamplerTrailIsExpressedInARCoordinates", testSamplerTrailIsExpressedInARCoordinates),
            ("testSamplerReturnsNoTrailWithoutAlignment", testSamplerReturnsNoTrailWithoutAlignment),
            ("testSamplerResetClearsEverything", testSamplerResetClearsEverything),
        ]
    }
}
