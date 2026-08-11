#if canImport(XCTest)
import XCTest
// In Xcode the tests compile as their own module; the headless runner in
// Scripts/ compiles them alongside the sources instead, so neither import
// exists there.
@testable import InitiatorDrone
#endif
import Foundation

/// Pixel decoding for every encoding the robot can publish.
final class ROSImageDecoderTests: XCTestCase {

    // MARK: - Helpers

    private func message(
        width: Int,
        height: Int,
        encoding: String,
        step: Int,
        bigEndian: Bool = false,
        bytes: [UInt8]
    ) -> ROSImageMessage {
        ROSImageMessage(
            stamp: 1.0,
            frameId: "test",
            width: width,
            height: height,
            encoding: encoding,
            isBigEndian: bigEndian,
            step: step,
            data: Data(bytes)
        )
    }

    private func littleEndian16(_ values: [UInt16]) -> [UInt8] {
        values.flatMap { [UInt8($0 & 0xFF), UInt8($0 >> 8)] }
    }

    private func bigEndian16(_ values: [UInt16]) -> [UInt8] {
        values.flatMap { [UInt8($0 >> 8), UInt8($0 & 0xFF)] }
    }

    private func littleEndianFloat(_ values: [Float]) -> [UInt8] {
        values.flatMap { value -> [UInt8] in
            let bits = value.bitPattern
            return [
                UInt8(bits & 0xFF),
                UInt8((bits >> 8) & 0xFF),
                UInt8((bits >> 16) & 0xFF),
                UInt8((bits >> 24) & 0xFF),
            ]
        }
    }

    private func scalar(_ decoded: DecodedROSImage) throws -> ScalarImage {
        guard case .scalar(let image) = decoded else {
            throw UnwrapFailureMarker.notScalar
        }
        return image
    }

    private func color(_ decoded: DecodedROSImage) throws -> ColorImage {
        guard case .color(let image) = decoded else {
            throw UnwrapFailureMarker.notColor
        }
        return image
    }

    private enum UnwrapFailureMarker: Error {
        case notScalar
        case notColor
    }

    // MARK: - Encoding recognition

    func testEncodingParsingIsCaseInsensitiveAndHandlesAliases() {
        XCTAssertEqual(ROSImageEncoding(rosEncoding: "16UC1"), .uint16Single)
        XCTAssertEqual(ROSImageEncoding(rosEncoding: "16uc1"), .uint16Single)
        XCTAssertEqual(ROSImageEncoding(rosEncoding: " mono16 "), .mono16)
        XCTAssertEqual(ROSImageEncoding(rosEncoding: "32FC1"), .float32Single)
        XCTAssertEqual(ROSImageEncoding(rosEncoding: "mono8"), .mono8)
        XCTAssertEqual(ROSImageEncoding(rosEncoding: "8UC1"), .mono8)
        XCTAssertEqual(ROSImageEncoding(rosEncoding: "rgb8"), .rgb8)
        XCTAssertEqual(ROSImageEncoding(rosEncoding: "bgr8"), .bgr8)
        XCTAssertEqual(ROSImageEncoding(rosEncoding: "rgba8"), .rgba8)
        XCTAssertEqual(ROSImageEncoding(rosEncoding: "bgra8"), .bgra8)
        XCTAssertNil(ROSImageEncoding(rosEncoding: "yuyv"))
        XCTAssertNil(ROSImageEncoding(rosEncoding: ""))
    }

    func testBytesPerPixel() {
        XCTAssertEqual(ROSImageEncoding.mono8.bytesPerPixel, 1)
        XCTAssertEqual(ROSImageEncoding.mono16.bytesPerPixel, 2)
        XCTAssertEqual(ROSImageEncoding.uint16Single.bytesPerPixel, 2)
        XCTAssertEqual(ROSImageEncoding.float32Single.bytesPerPixel, 4)
        XCTAssertEqual(ROSImageEncoding.rgb8.bytesPerPixel, 3)
        XCTAssertEqual(ROSImageEncoding.bgra8.bytesPerPixel, 4)
    }

    // MARK: - 16UC1 depth

    func test16UC1DepthConvertsMillimetresToMetres() throws {
        let input = message(
            width: 3,
            height: 1,
            encoding: "16UC1",
            step: 6,
            bytes: littleEndian16([1000, 2500, 65535])
        )
        let image = try scalar(try ROSImageDecoder.decode(input, interpretation: .depthMillimetres))

        XCTAssertEqual(image.width, 3)
        XCTAssertEqual(image.height, 1)
        XCTAssertEqual(image.unit, .metres)
        XCTAssertEqual(Double(image.value(x: 0, y: 0)), 1.0, accuracy: 1e-6)
        XCTAssertEqual(Double(image.value(x: 1, y: 0)), 2.5, accuracy: 1e-6)
        XCTAssertEqual(Double(image.value(x: 2, y: 0)), 65.535, accuracy: 1e-4)
    }

    func testZeroDepthBecomesNaNRatherThanAWallAtTheLens() throws {
        // ROS uses 0 for "no return" in 16-bit depth. Passing it through as
        // 0 m would paint a surface right at the camera.
        let input = message(
            width: 2,
            height: 1,
            encoding: "16UC1",
            step: 4,
            bytes: littleEndian16([0, 1200])
        )
        let image = try scalar(try ROSImageDecoder.decode(input, interpretation: .depthMillimetres))
        XCTAssertTrue(image.value(x: 0, y: 0).isNaN)
        XCTAssertEqual(Double(image.value(x: 1, y: 0)), 1.2, accuracy: 1e-6)
    }

    func testStepPaddingIsRespected() throws {
        // Two pixels per row need 4 bytes, but the publisher pads rows to 8.
        // A decoder that ignores `step` reads garbage from the second row on.
        let row0 = littleEndian16([1000, 2000]) + [0xFF, 0xFF, 0xFF, 0xFF]
        let row1 = littleEndian16([3000, 4000]) + [0xFF, 0xFF, 0xFF, 0xFF]
        let input = message(
            width: 2,
            height: 2,
            encoding: "16UC1",
            step: 8,
            bytes: row0 + row1
        )
        let image = try scalar(try ROSImageDecoder.decode(input, interpretation: .depthMillimetres))

        XCTAssertEqual(Double(image.value(x: 0, y: 0)), 1.0, accuracy: 1e-6)
        XCTAssertEqual(Double(image.value(x: 1, y: 0)), 2.0, accuracy: 1e-6)
        XCTAssertEqual(Double(image.value(x: 0, y: 1)), 3.0, accuracy: 1e-6)
        XCTAssertEqual(Double(image.value(x: 1, y: 1)), 4.0, accuracy: 1e-6)
    }

    func testBigEndianAndLittleEndianAgree() throws {
        let values: [UInt16] = [1, 258, 4095, 60000]
        let little = message(
            width: 4, height: 1, encoding: "16UC1", step: 8,
            bigEndian: false, bytes: littleEndian16(values)
        )
        let big = message(
            width: 4, height: 1, encoding: "16UC1", step: 8,
            bigEndian: true, bytes: bigEndian16(values)
        )

        let a = try scalar(try ROSImageDecoder.decode(little, interpretation: .raw))
        let b = try scalar(try ROSImageDecoder.decode(big, interpretation: .raw))
        XCTAssertEqual(a.values, b.values)
        XCTAssertEqual(Double(a.value(x: 1, y: 0)), 258.0, accuracy: 1e-6)
    }

    func testUnalignedRowStartsDoNotTrap() throws {
        // A step of 5 puts every second row on an odd byte offset. Reading a
        // UInt16 through a plain pointer load there is undefined; the decoder
        // assembles bytes by hand to stay safe.
        let row0 = littleEndian16([1000, 2000]) + [0x00]
        let row1 = littleEndian16([3000, 4000]) + [0x00]
        let input = message(width: 2, height: 2, encoding: "16UC1", step: 5, bytes: row0 + row1)
        let image = try scalar(try ROSImageDecoder.decode(input, interpretation: .depthMillimetres))
        XCTAssertEqual(Double(image.value(x: 1, y: 1)), 4.0, accuracy: 1e-6)
    }

    // MARK: - mono16

    func testMono16WithLinearInterpretationProducesCelsius() throws {
        let input = message(
            width: 3, height: 1, encoding: "mono16", step: 6,
            bytes: littleEndian16([2100, 3670, 0])
        )
        let image = try scalar(try ROSImageDecoder.decode(
            input,
            interpretation: .linear(scale: 0.01, offset: 0, unit: .celsius)
        ))

        XCTAssertEqual(image.unit, .celsius)
        XCTAssertEqual(Double(image.value(x: 0, y: 0)), 21.0, accuracy: 1e-4)
        XCTAssertEqual(Double(image.value(x: 1, y: 0)), 36.7, accuracy: 1e-4)
        // Zero is a real reading for a thermal sensor, not a dropout.
        XCTAssertEqual(Double(image.value(x: 2, y: 0)), 0.0, accuracy: 1e-6)
    }

    func testLinearInterpretationAppliesOffset() throws {
        let input = message(
            width: 1, height: 1, encoding: "mono16", step: 2,
            bytes: littleEndian16([1000])
        )
        let image = try scalar(try ROSImageDecoder.decode(
            input,
            interpretation: .linear(scale: 0.1, offset: -50, unit: .celsius)
        ))
        XCTAssertEqual(Double(image.value(x: 0, y: 0)), 50.0, accuracy: 1e-4)
    }

    // MARK: - 32FC1

    func test32FC1DepthKeepsMetresAndRejectsNonPositive() throws {
        let input = message(
            width: 4, height: 1, encoding: "32FC1", step: 16,
            bytes: littleEndianFloat([0.5, 3.25, 0.0, -2.0])
        )
        let image = try scalar(try ROSImageDecoder.decode(input, interpretation: .depthMetres))

        XCTAssertEqual(Double(image.value(x: 0, y: 0)), 0.5, accuracy: 1e-6)
        XCTAssertEqual(Double(image.value(x: 1, y: 0)), 3.25, accuracy: 1e-6)
        XCTAssertTrue(image.value(x: 2, y: 0).isNaN)
        XCTAssertTrue(image.value(x: 3, y: 0).isNaN)
    }

    func test32FC1PropagatesNaNAndInfinity() throws {
        let input = message(
            width: 3, height: 1, encoding: "32FC1", step: 12,
            bytes: littleEndianFloat([.nan, .infinity, 1.5])
        )
        let image = try scalar(try ROSImageDecoder.decode(input, interpretation: .depthMetres))
        XCTAssertTrue(image.value(x: 0, y: 0).isNaN)
        XCTAssertTrue(image.value(x: 1, y: 0).isNaN)
        XCTAssertEqual(Double(image.value(x: 2, y: 0)), 1.5, accuracy: 1e-6)
    }

    func test32FC1BigEndianMatchesLittleEndian() throws {
        let little: [UInt8] = littleEndianFloat([1.5, -0.25])
        var big: [UInt8] = []
        var start = 0
        while start < little.count {
            big.append(little[start + 3])
            big.append(little[start + 2])
            big.append(little[start + 1])
            big.append(little[start])
            start += 4
        }
        let a = try scalar(try ROSImageDecoder.decode(
            message(width: 2, height: 1, encoding: "32FC1", step: 8, bytes: little),
            interpretation: .raw
        ))
        let b = try scalar(try ROSImageDecoder.decode(
            message(width: 2, height: 1, encoding: "32FC1", step: 8, bigEndian: true, bytes: big),
            interpretation: .raw
        ))
        XCTAssertEqual(a.values, b.values)
    }

    // MARK: - mono8

    func testMono8ReadsRawCounts() throws {
        let input = message(width: 3, height: 1, encoding: "mono8", step: 3, bytes: [0, 128, 255])
        let image = try scalar(try ROSImageDecoder.decode(input, interpretation: .raw))
        XCTAssertEqual(image.value(x: 0, y: 0), 0)
        XCTAssertEqual(image.value(x: 1, y: 0), 128)
        XCTAssertEqual(image.value(x: 2, y: 0), 255)
    }

    // MARK: - Colour

    func testRGB8AndBGR8DecodeToTheSamePixels() throws {
        let rgb = try color(try ROSImageDecoder.decode(
            message(width: 2, height: 1, encoding: "rgb8", step: 6, bytes: [255, 0, 0, 0, 255, 0])
        ))
        let bgr = try color(try ROSImageDecoder.decode(
            message(width: 2, height: 1, encoding: "bgr8", step: 6, bytes: [0, 0, 255, 0, 255, 0])
        ))

        XCTAssertEqual(rgb.rgba, [255, 0, 0, 255, 0, 255, 0, 255])
        XCTAssertEqual(rgb.rgba, bgr.rgba)
    }

    func testRGBA8AndBGRA8PreserveAlpha() throws {
        let rgba = try color(try ROSImageDecoder.decode(
            message(width: 1, height: 1, encoding: "rgba8", step: 4, bytes: [10, 20, 30, 40])
        ))
        let bgra = try color(try ROSImageDecoder.decode(
            message(width: 1, height: 1, encoding: "bgra8", step: 4, bytes: [30, 20, 10, 40])
        ))
        XCTAssertEqual(rgba.rgba, [10, 20, 30, 40])
        XCTAssertEqual(rgba.rgba, bgra.rgba)
    }

    func testColourRespectsStepPadding() throws {
        let row0: [UInt8] = [1, 2, 3, 0, 0]
        let row1: [UInt8] = [4, 5, 6, 0, 0]
        let image = try color(try ROSImageDecoder.decode(
            message(width: 1, height: 2, encoding: "rgb8", step: 5, bytes: row0 + row1)
        ))
        XCTAssertEqual(image.rgba, [1, 2, 3, 255, 4, 5, 6, 255])
    }

    // MARK: - Failure modes

    func testUnsupportedEncodingThrows() {
        let input = message(width: 1, height: 1, encoding: "yuyv", step: 2, bytes: [0, 0])
        XCTAssertThrowsError(try ROSImageDecoder.decode(input)) { error in
            XCTAssertEqual(error as? ROSImageDecodeError, .unsupportedEncoding("yuyv"))
        }
    }

    func testZeroDimensionsThrow() {
        let input = message(width: 0, height: 4, encoding: "mono8", step: 0, bytes: [])
        XCTAssertThrowsError(try ROSImageDecoder.decode(input)) { error in
            XCTAssertEqual(error as? ROSImageDecodeError, .invalidDimensions(width: 0, height: 4))
        }
    }

    func testStepSmallerThanARowThrows() {
        let input = message(width: 4, height: 1, encoding: "16UC1", step: 4, bytes: [UInt8](repeating: 0, count: 8))
        XCTAssertThrowsError(try ROSImageDecoder.decode(input)) { error in
            XCTAssertEqual(error as? ROSImageDecodeError, .stepTooSmall(step: 4, required: 8))
        }
    }

    func testTruncatedPayloadThrowsInsteadOfReadingPastTheEnd() {
        let input = message(width: 4, height: 4, encoding: "16UC1", step: 8, bytes: [UInt8](repeating: 0, count: 12))
        XCTAssertThrowsError(try ROSImageDecoder.decode(input)) { error in
            XCTAssertEqual(error as? ROSImageDecodeError, .truncatedData(available: 12, required: 32))
        }
    }

    func testMissingStepIsTreatedAsTightlyPacked() throws {
        // rosbridge sends step 0 for some publishers; assume no padding rather
        // than rejecting the frame.
        let input = message(width: 2, height: 2, encoding: "mono8", step: 0, bytes: [1, 2, 3, 4])
        let image = try scalar(try ROSImageDecoder.decode(input, interpretation: .raw))
        XCTAssertEqual(image.values, [1, 2, 3, 4])
    }

    func testTrailingRowPaddingMayBeAbsent() throws {
        // The last row's padding is not required to be present, which is what a
        // publisher that trims the final row produces.
        let bytes: [UInt8] = [1, 2, 0, 0, 3, 4]
        let input = message(width: 2, height: 2, encoding: "mono8", step: 4, bytes: bytes)
        let image = try scalar(try ROSImageDecoder.decode(input, interpretation: .raw))
        XCTAssertEqual(image.values, [1, 2, 3, 4])
    }

    // MARK: - Statistics

    func testFiniteRangeIgnoresInvalidSamples() {
        let image = ScalarImage(
            width: 4, height: 1,
            values: [.nan, 2.0, 5.0, .nan],
            unit: .metres
        )
        let range = image.finiteRange
        XCTAssertEqual(range?.min, 2.0)
        XCTAssertEqual(range?.max, 5.0)
    }

    func testFiniteRangeIsNilWhenEverySampleIsInvalid() {
        let image = ScalarImage(width: 2, height: 1, values: [.nan, .nan], unit: .metres)
        XCTAssertNil(image.finiteRange)
    }

    func testPercentileRangeRejectsOutliers() {
        // One hot pixel must not stretch the colour map over the whole frame.
        var values = [Float](repeating: 20.0, count: 99)
        values.append(500.0)
        let image = ScalarImage(width: 100, height: 1, values: values, unit: .celsius)
        let range = image.finitePercentileRange(low: 0.02, high: 0.98)
        XCTAssertEqual(range?.max, 20.0)
    }

    static var allTests: [(String, (ROSImageDecoderTests) -> () throws -> Void)] {
        [
            ("testEncodingParsingIsCaseInsensitiveAndHandlesAliases", testEncodingParsingIsCaseInsensitiveAndHandlesAliases),
            ("testBytesPerPixel", testBytesPerPixel),
            ("test16UC1DepthConvertsMillimetresToMetres", test16UC1DepthConvertsMillimetresToMetres),
            ("testZeroDepthBecomesNaNRatherThanAWallAtTheLens", testZeroDepthBecomesNaNRatherThanAWallAtTheLens),
            ("testStepPaddingIsRespected", testStepPaddingIsRespected),
            ("testBigEndianAndLittleEndianAgree", testBigEndianAndLittleEndianAgree),
            ("testUnalignedRowStartsDoNotTrap", testUnalignedRowStartsDoNotTrap),
            ("testMono16WithLinearInterpretationProducesCelsius", testMono16WithLinearInterpretationProducesCelsius),
            ("testLinearInterpretationAppliesOffset", testLinearInterpretationAppliesOffset),
            ("test32FC1DepthKeepsMetresAndRejectsNonPositive", test32FC1DepthKeepsMetresAndRejectsNonPositive),
            ("test32FC1PropagatesNaNAndInfinity", test32FC1PropagatesNaNAndInfinity),
            ("test32FC1BigEndianMatchesLittleEndian", test32FC1BigEndianMatchesLittleEndian),
            ("testMono8ReadsRawCounts", testMono8ReadsRawCounts),
            ("testRGB8AndBGR8DecodeToTheSamePixels", testRGB8AndBGR8DecodeToTheSamePixels),
            ("testRGBA8AndBGRA8PreserveAlpha", testRGBA8AndBGRA8PreserveAlpha),
            ("testColourRespectsStepPadding", testColourRespectsStepPadding),
            ("testUnsupportedEncodingThrows", testUnsupportedEncodingThrows),
            ("testZeroDimensionsThrow", testZeroDimensionsThrow),
            ("testStepSmallerThanARowThrows", testStepSmallerThanARowThrows),
            ("testTruncatedPayloadThrowsInsteadOfReadingPastTheEnd", testTruncatedPayloadThrowsInsteadOfReadingPastTheEnd),
            ("testMissingStepIsTreatedAsTightlyPacked", testMissingStepIsTreatedAsTightlyPacked),
            ("testTrailingRowPaddingMayBeAbsent", testTrailingRowPaddingMayBeAbsent),
            ("testFiniteRangeIgnoresInvalidSamples", testFiniteRangeIgnoresInvalidSamples),
            ("testFiniteRangeIsNilWhenEverySampleIsInvalid", testFiniteRangeIsNilWhenEverySampleIsInvalid),
            ("testPercentileRangeRejectsOutliers", testPercentileRangeRejectsOutliers),
        ]
    }
}
