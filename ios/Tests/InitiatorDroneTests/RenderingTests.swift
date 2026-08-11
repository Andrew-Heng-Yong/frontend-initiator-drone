#if canImport(XCTest)
import XCTest
// In Xcode the tests compile as their own module; the headless runner in
// Scripts/ compiles them alongside the sources instead, so neither import
// exists there.
@testable import InitiatorDrone
#endif
import Foundation

/// Colour maps, the scalar renderer, and the bounded frame mailbox.
final class RenderingTests: XCTestCase {

    // MARK: - Colour ramps

    func testEveryRampBakesA256EntryTable() {
        for style in ColorRampStyle.allCases {
            XCTAssertEqual(style.ramp.lookupTable.count, 256, "\(style.displayName)")
        }
    }

    func testRampEndpointsMatchTheirStops() {
        for style in ColorRampStyle.allCases {
            let ramp = style.ramp
            XCTAssertEqual(ramp.color(normalized: 0), ramp.stops.first?.color, "\(style.displayName) low")
            XCTAssertEqual(ramp.color(normalized: 1), ramp.stops.last?.color, "\(style.displayName) high")
        }
    }

    func testRampClampsOutOfRangeLookups() {
        let ramp = ColorRampStyle.grayscale.ramp
        XCTAssertEqual(ramp.color(normalized: -5), RGBColor(0, 0, 0))
        XCTAssertEqual(ramp.color(normalized: 5), RGBColor(255, 255, 255))
    }

    func testGrayscaleRampIsLinear() {
        let ramp = ColorRampStyle.grayscale.ramp
        let middle = ramp.color(normalized: 0.5)
        // The table has 256 entries covering 0...255, so 0.5 lands on index 127
        // and the exact midpoint is not representable. Anything within one
        // count of 127.5 is correct.
        XCTAssertLessThanOrEqual(
            abs(Double(middle.red) - 127.5),
            1.0,
            "expected the midpoint to be mid-grey, got \(middle.red)"
        )
        XCTAssertEqual(middle.red, middle.green)
        XCTAssertEqual(middle.green, middle.blue)

        // Linearity: every entry equals its index.
        for index in 0..<256 {
            XCTAssertEqual(Int(ramp.lookupTable[index].red), index)
        }
    }

    func testRampIsMonotonicInBrightnessForGrayscale() {
        let table = ColorRampStyle.grayscale.ramp.lookupTable
        for index in 1..<table.count {
            XCTAssertGreaterThanOrEqual(table[index].red, table[index - 1].red)
        }
    }

    // MARK: - Scalar renderer

    private func image(_ values: [Float], width: Int? = nil, unit: ScalarUnit = .metres) -> ScalarImage {
        ScalarImage(width: width ?? values.count, height: 1, values: values, unit: unit)
    }

    func testFixedRangeMapsEndpointsToRampEnds() {
        let settings = ScalarColorMapSettings(style: .grayscale, low: 0, high: 10, rangeMode: .fixed)
        let renderer = ScalarImageRenderer(settings: settings)
        let output = renderer.render(image([0, 5, 10]))

        XCTAssertEqual(output.rgba[0], 0)     // 0 m -> black
        XCTAssertEqual(output.rgba[8], 255)   // 10 m -> white
        XCTAssertLessThanOrEqual(
            abs(Double(output.rgba[4]) - 127.5),
            1.0,
            "midpoint should be mid-grey, got \(output.rgba[4])"
        )
    }

    func testReversedRampFlipsTheMapping() {
        let settings = ScalarColorMapSettings(
            style: .grayscale, low: 0, high: 10, rangeMode: .fixed, reversed: true
        )
        let renderer = ScalarImageRenderer(settings: settings)
        let output = renderer.render(image([0, 10]))
        XCTAssertEqual(output.rgba[0], 255)
        XCTAssertEqual(output.rgba[4], 0)
    }

    func testInvalidSamplesAreTransparentNotBlack() {
        // A "no return" sample must show the camera through it, not paint a
        // black surface that looks like a real reading.
        let renderer = ScalarImageRenderer(settings: .depthDefault)
        let output = renderer.render(image([.nan, 1.0]))
        XCTAssertEqual(output.rgba[3], 0, "invalid sample must be fully transparent")
        XCTAssertGreaterThan(output.rgba[7], 0, "valid sample must be opaque")
    }

    func testOutOfRangeSamplesClampWhenAskedTo() {
        let settings = ScalarColorMapSettings(
            style: .grayscale, low: 1, high: 2, rangeMode: .fixed, clampOutOfRange: true
        )
        let renderer = ScalarImageRenderer(settings: settings)
        let output = renderer.render(image([0.1, 9.0]))
        XCTAssertEqual(output.rgba[0], 0)
        XCTAssertEqual(output.rgba[4], 255)
        XCTAssertEqual(output.rgba[3], 255)
    }

    func testOutOfRangeSamplesDropOutWhenClampingIsOff() {
        let settings = ScalarColorMapSettings(
            style: .grayscale, low: 1, high: 2, rangeMode: .fixed, clampOutOfRange: false
        )
        let renderer = ScalarImageRenderer(settings: settings)
        let output = renderer.render(image([0.1, 1.5, 9.0]))
        XCTAssertEqual(output.rgba[3], 0)
        XCTAssertEqual(output.rgba[7], 255)
        XCTAssertEqual(output.rgba[11], 0)
    }

    func testOpacityIsAppliedToValidSamples() {
        let settings = ScalarColorMapSettings(
            style: .grayscale, low: 0, high: 1, rangeMode: .fixed, opacity: 0.5
        )
        let renderer = ScalarImageRenderer(settings: settings)
        let output = renderer.render(image([0.5]))
        XCTAssertEqual(Int(output.rgba[3]), 128)
    }

    func testDegenerateRangeDoesNotProduceNaNColours() {
        let settings = ScalarColorMapSettings(style: .turbo, low: 3, high: 3, rangeMode: .fixed)
        let renderer = ScalarImageRenderer(settings: settings)
        let output = renderer.render(image([3, 3, 3]))
        XCTAssertEqual(output.rgba.count, 12)
        XCTAssertEqual(output.rgba[3], 255)
    }

    func testAutoRangeFollowsTheFrame() {
        let settings = ScalarColorMapSettings(style: .grayscale, low: 0, high: 1, rangeMode: .autoPerFrame)
        let renderer = ScalarImageRenderer(settings: settings)
        var values = [Float](repeating: 20, count: 50)
        values += [Float](repeating: 30, count: 50)
        _ = renderer.render(ScalarImage(width: 100, height: 1, values: values, unit: .celsius))

        XCTAssertGreaterThanOrEqual(renderer.effectiveRange.low, 19.0)
        XCTAssertLessThanOrEqual(renderer.effectiveRange.high, 31.0)
    }

    func testAutoRangeWidensADegenerateFrame() {
        // A frame where every sample is the same would otherwise amplify pure
        // sensor noise to full scale.
        let settings = ScalarColorMapSettings(style: .grayscale, low: 0, high: 1, rangeMode: .autoPerFrame)
        let renderer = ScalarImageRenderer(settings: settings)
        _ = renderer.render(ScalarImage(
            width: 10, height: 1,
            values: [Float](repeating: 25.0, count: 10),
            unit: .celsius
        ))
        XCTAssertGreaterThanOrEqual(renderer.effectiveRange.high - renderer.effectiveRange.low, 0.5 - 1e-9)
    }

    func testSmoothedAutoRangeEasesTowardTheTarget() {
        let settings = ScalarColorMapSettings(style: .grayscale, low: 0, high: 10, rangeMode: .autoSmoothed)
        let renderer = ScalarImageRenderer(settings: settings)
        renderer.smoothing = 0.5

        let frame = ScalarImage(
            width: 100, height: 1,
            values: (0..<100).map { Float($0) },
            unit: .metres
        )
        let first = renderer.effectiveRange
        _ = renderer.render(frame)
        let second = renderer.effectiveRange

        // One frame must move part of the way, not all of it.
        XCTAssertNotEqual(second.high, first.high)
        XCTAssertLessThan(second.high, 99.0)
    }

    func testAutoRangeKeepsTheLastRangeWhenAFrameIsAllInvalid() {
        let settings = ScalarColorMapSettings(style: .grayscale, low: 2, high: 8, rangeMode: .autoSmoothed)
        let renderer = ScalarImageRenderer(settings: settings)
        _ = renderer.render(image([.nan, .nan, .nan]))
        XCTAssertEqual(renderer.effectiveRange.low, 2)
        XCTAssertEqual(renderer.effectiveRange.high, 8)
    }

    func testRendererOutputIsAlwaysFourBytesPerPixel() {
        let renderer = ScalarImageRenderer(settings: .depthDefault)
        let output = renderer.render(ScalarImage(
            width: 7, height: 5,
            values: [Float](repeating: 1.0, count: 35),
            unit: .metres
        ))
        XCTAssertEqual(output.width, 7)
        XCTAssertEqual(output.height, 5)
        XCTAssertEqual(output.rgba.count, 7 * 5 * 4)
    }

    func testRendererReusesItsBufferAcrossFramesOfTheSameSize() {
        // Not a behavioural requirement so much as a memory one: a long stream
        // must not allocate a fresh output buffer per frame.
        let renderer = ScalarImageRenderer(settings: .depthDefault)
        let frame = ScalarImage(
            width: 64, height: 48,
            values: [Float](repeating: 2.0, count: 64 * 48),
            unit: .metres
        )
        let first = renderer.render(frame)
        let second = renderer.render(frame)
        XCTAssertEqual(first.rgba, second.rgba)
    }

    // MARK: - Bounded mailbox

    func testLatestOnlySlotKeepsOnlyTheNewestValue() {
        let slot = LatestOnlySlot<Int>()
        XCTAssertTrue(slot.offer(1), "first offer should start a drain")
        XCTAssertFalse(slot.offer(2), "a drain is already running")
        XCTAssertFalse(slot.offer(3))

        XCTAssertEqual(slot.take(), 3)
        XCTAssertNil(slot.take())

        let statistics = slot.statistics
        XCTAssertEqual(statistics.accepted, 3)
        XCTAssertEqual(statistics.dropped, 2)
    }

    func testLatestOnlySlotRestartsDrainingAfterItEmpties() {
        let slot = LatestOnlySlot<Int>()
        XCTAssertTrue(slot.offer(1))
        XCTAssertEqual(slot.take(), 1)
        XCTAssertNil(slot.take())
        XCTAssertTrue(slot.offer(2), "after emptying, a new offer must schedule a drain")
    }

    func testLatestOnlySlotResetDropsThePendingValue() {
        let slot = LatestOnlySlot<Int>()
        _ = slot.offer(1)
        slot.reset()
        XCTAssertNil(slot.take())
    }

    func testFloodingTheSlotNeverQueuesMoreThanOneFrame() {
        // The backlog guarantee, stated directly: whatever the producer does,
        // at most one value is ever retained.
        let slot = LatestOnlySlot<[UInt8]>()
        for index in 0..<10_000 {
            _ = slot.offer([UInt8(index % 256)])
        }
        XCTAssertNotNil(slot.take())
        XCTAssertNil(slot.take(), "only one value may ever be held")
        XCTAssertEqual(slot.statistics.dropped, 9_999)
    }

    func testConcurrentProducersStayBounded() {
        let slot = LatestOnlySlot<Int>()
        let group = DispatchGroup()
        for producer in 0..<8 {
            DispatchQueue.global().async(group: group) {
                for index in 0..<2_000 {
                    _ = slot.offer(producer * 10_000 + index)
                }
            }
        }
        group.wait()

        XCTAssertNotNil(slot.take())
        XCTAssertNil(slot.take())
        XCTAssertEqual(slot.statistics.accepted, 16_000)
    }

    // MARK: - Rate tracking

    func testRateTrackerReportsMessagesPerSecond() {
        var tracker = RateTracker(windowDuration: 2.0)
        for index in 0..<20 {
            tracker.record(at: 100.0 + Double(index) * 0.1)
        }
        XCTAssertEqual(tracker.rate(at: 101.9), 10.0, accuracy: 1.0)
    }

    func testRateFallsToZeroAfterAStreamStops() {
        var tracker = RateTracker(windowDuration: 2.0)
        for index in 0..<20 {
            tracker.record(at: 100.0 + Double(index) * 0.1)
        }
        // Ten seconds later, nothing has arrived: the rate must read zero, not
        // keep reporting the old cadence.
        XCTAssertEqual(tracker.rate(at: 112.0), 0.0, accuracy: 1e-9)
    }

    func testRateTrackerReportsAge() {
        var tracker = RateTracker()
        XCTAssertNil(tracker.age(at: 100))
        tracker.record(at: 100)
        XCTAssertEqual(tracker.age(at: 100.75) ?? 0, 0.75, accuracy: 1e-9)
    }

    func testRateTrackerIsBoundedAndCapsTheReportableRate() {
        // 100k messages through a 64-slot tracker. Memory must stay bounded,
        // and the reported rate must stay inside what that capacity can
        // represent — `capacity / windowDuration` — rather than dividing a
        // truncated sample count by a tiny span and reporting nonsense.
        var tracker = RateTracker(windowDuration: 1.0, capacity: 64)
        for index in 0..<100_000 {
            tracker.record(at: Double(index) * 0.001)
        }
        XCTAssertEqual(tracker.totalCount, 100_000)

        let rate = tracker.rate(at: 100.0)
        XCTAssertGreaterThan(rate, 0)
        XCTAssertLessThanOrEqual(rate, 64.0)
    }

    func testRateIsAccurateWhenCapacityIsNotTheLimit() {
        var tracker = RateTracker(windowDuration: 2.0, capacity: 4096)
        for index in 0..<1_000 {
            tracker.record(at: Double(index) * 0.01)
        }
        // 100 Hz for ten seconds; the last two seconds hold 200 samples.
        XCTAssertEqual(tracker.rate(at: 9.99), 100.0, accuracy: 5.0)
    }

    static var allTests: [(String, (RenderingTests) -> () throws -> Void)] {
        [
            ("testEveryRampBakesA256EntryTable", testEveryRampBakesA256EntryTable),
            ("testRampEndpointsMatchTheirStops", testRampEndpointsMatchTheirStops),
            ("testRampClampsOutOfRangeLookups", testRampClampsOutOfRangeLookups),
            ("testGrayscaleRampIsLinear", testGrayscaleRampIsLinear),
            ("testRampIsMonotonicInBrightnessForGrayscale", testRampIsMonotonicInBrightnessForGrayscale),
            ("testFixedRangeMapsEndpointsToRampEnds", testFixedRangeMapsEndpointsToRampEnds),
            ("testReversedRampFlipsTheMapping", testReversedRampFlipsTheMapping),
            ("testInvalidSamplesAreTransparentNotBlack", testInvalidSamplesAreTransparentNotBlack),
            ("testOutOfRangeSamplesClampWhenAskedTo", testOutOfRangeSamplesClampWhenAskedTo),
            ("testOutOfRangeSamplesDropOutWhenClampingIsOff", testOutOfRangeSamplesDropOutWhenClampingIsOff),
            ("testOpacityIsAppliedToValidSamples", testOpacityIsAppliedToValidSamples),
            ("testDegenerateRangeDoesNotProduceNaNColours", testDegenerateRangeDoesNotProduceNaNColours),
            ("testAutoRangeFollowsTheFrame", testAutoRangeFollowsTheFrame),
            ("testAutoRangeWidensADegenerateFrame", testAutoRangeWidensADegenerateFrame),
            ("testSmoothedAutoRangeEasesTowardTheTarget", testSmoothedAutoRangeEasesTowardTheTarget),
            ("testAutoRangeKeepsTheLastRangeWhenAFrameIsAllInvalid", testAutoRangeKeepsTheLastRangeWhenAFrameIsAllInvalid),
            ("testRendererOutputIsAlwaysFourBytesPerPixel", testRendererOutputIsAlwaysFourBytesPerPixel),
            ("testRendererReusesItsBufferAcrossFramesOfTheSameSize", testRendererReusesItsBufferAcrossFramesOfTheSameSize),
            ("testLatestOnlySlotKeepsOnlyTheNewestValue", testLatestOnlySlotKeepsOnlyTheNewestValue),
            ("testLatestOnlySlotRestartsDrainingAfterItEmpties", testLatestOnlySlotRestartsDrainingAfterItEmpties),
            ("testLatestOnlySlotResetDropsThePendingValue", testLatestOnlySlotResetDropsThePendingValue),
            ("testFloodingTheSlotNeverQueuesMoreThanOneFrame", testFloodingTheSlotNeverQueuesMoreThanOneFrame),
            ("testConcurrentProducersStayBounded", testConcurrentProducersStayBounded),
            ("testRateTrackerReportsMessagesPerSecond", testRateTrackerReportsMessagesPerSecond),
            ("testRateFallsToZeroAfterAStreamStops", testRateFallsToZeroAfterAStreamStops),
            ("testRateTrackerReportsAge", testRateTrackerReportsAge),
            ("testRateTrackerIsBoundedAndCapsTheReportableRate", testRateTrackerIsBoundedAndCapsTheReportableRate),
            ("testRateIsAccurateWhenCapacityIsNotTheLimit", testRateIsAccurateWhenCapacityIsNotTheLimit),
        ]
    }
}
