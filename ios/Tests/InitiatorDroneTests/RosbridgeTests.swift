#if canImport(XCTest)
import XCTest
// In Xcode the tests compile as their own module; the headless runner in
// Scripts/ compiles them alongside the sources instead, so neither import
// exists there.
@testable import InitiatorDrone
#endif
import Foundation

/// The rosbridge wire layer: value tree, CBOR, command encoding, inbound
/// parsing, and the typed message parsers, checked against the JSON fixtures.
final class RosbridgeTests: XCTestCase {

    // MARK: - ROSValue

    func testJSONBooleansSurviveAsBooleansNotNumbers() throws {
        // NSNumber erases Bool; if that leaks through, /odom/calibrated stops
        // meaning anything.
        let value = try ROSValue.fromJSON(Data(#"{"data": true, "count": 1}"#.utf8))
        XCTAssertEqual(value["data"], .bool(true))
        XCTAssertEqual(value["count"], .integer(1))
    }

    func testNumbersKeepIntegerAndFloatingPointApart() throws {
        let value = try ROSValue.fromJSON(Data(#"{"i": 42, "f": 42.5, "neg": -7}"#.utf8))
        XCTAssertEqual(value["i"]?.intValue, 42)
        XCTAssertEqual(value["f"]?.doubleValue, 42.5)
        XCTAssertEqual(value["neg"]?.intValue, -7)
    }

    func testByteArrayAcceptsBase64AndPlainArrays() throws {
        let base64 = ROSValue.string(Data([1, 2, 3]).base64EncodedString())
        XCTAssertEqual(base64.byteArrayValue, Data([1, 2, 3]))

        let array = ROSValue.array([.integer(1), .integer(2), .integer(3)])
        XCTAssertEqual(array.byteArrayValue, Data([1, 2, 3]))

        let raw = ROSValue.bytes(Data([9, 9]))
        XCTAssertEqual(raw.byteArrayValue, Data([9, 9]))
    }

    func testDoubleArrayReadsCameraInfoMatrices() {
        let value = ROSValue.array([.number(1.5), .integer(2), .number(3.25)])
        XCTAssertEqual(value.doubleArrayValue, [1.5, 2.0, 3.25])
    }

    func testNullAndMissingKeysAreDistinguishable() throws {
        let value = try ROSValue.fromJSON(Data(#"{"present": null}"#.utf8))
        XCTAssertEqual(value["present"], ROSValue.null)
        XCTAssertNil(value["absent"])
    }

    // MARK: - CBOR

    func testCBORUnsignedIntegers() throws {
        // RFC 8949 Appendix A vectors.
        XCTAssertEqual(try CBORDecoder.decode(Data([0x00])), .integer(0))
        XCTAssertEqual(try CBORDecoder.decode(Data([0x17])), .integer(23))
        XCTAssertEqual(try CBORDecoder.decode(Data([0x18, 0x18])), .integer(24))
        XCTAssertEqual(try CBORDecoder.decode(Data([0x19, 0x03, 0xE8])), .integer(1000))
        XCTAssertEqual(try CBORDecoder.decode(Data([0x1A, 0x00, 0x0F, 0x42, 0x40])), .integer(1_000_000))
    }

    func testCBORNegativeIntegers() throws {
        XCTAssertEqual(try CBORDecoder.decode(Data([0x20])), .integer(-1))
        XCTAssertEqual(try CBORDecoder.decode(Data([0x29])), .integer(-10))
        XCTAssertEqual(try CBORDecoder.decode(Data([0x38, 0x63])), .integer(-100))
    }

    func testCBORSimpleValues() throws {
        XCTAssertEqual(try CBORDecoder.decode(Data([0xF4])), .bool(false))
        XCTAssertEqual(try CBORDecoder.decode(Data([0xF5])), .bool(true))
        XCTAssertEqual(try CBORDecoder.decode(Data([0xF6])), .null)
    }

    func testCBORFloats() throws {
        // 1.0 as half, single and double precision.
        XCTAssertEqual(try CBORDecoder.decode(Data([0xF9, 0x3C, 0x00])).doubleValue ?? 0, 1.0, accuracy: 1e-6)
        XCTAssertEqual(
            try CBORDecoder.decode(Data([0xFA, 0x47, 0xC3, 0x50, 0x00])).doubleValue ?? 0,
            100000.0,
            accuracy: 1e-3
        )
        XCTAssertEqual(
            try CBORDecoder.decode(Data([0xFB, 0x40, 0x09, 0x21, 0xFB, 0x54, 0x44, 0x2D, 0x18])).doubleValue ?? 0,
            3.14159265358979,
            accuracy: 1e-12
        )
    }

    func testCBORStringsAndByteStrings() throws {
        XCTAssertEqual(try CBORDecoder.decode(Data([0x63, 0x61, 0x62, 0x63])), .string("abc"))
        XCTAssertEqual(try CBORDecoder.decode(Data([0x44, 0x01, 0x02, 0x03, 0x04])), .bytes(Data([1, 2, 3, 4])))
        XCTAssertEqual(try CBORDecoder.decode(Data([0x60])), .string(""))
    }

    func testCBORArraysAndMaps() throws {
        XCTAssertEqual(
            try CBORDecoder.decode(Data([0x83, 0x01, 0x02, 0x03])),
            .array([.integer(1), .integer(2), .integer(3)])
        )
        XCTAssertEqual(
            try CBORDecoder.decode(Data([0xA1, 0x61, 0x61, 0x01])),
            .object(["a": .integer(1)])
        )
    }

    func testCBORIndefiniteLengthContainers() throws {
        // 0x9F ... 0xFF is an indefinite array; rosbridge's encoder can emit
        // these for streamed payloads.
        XCTAssertEqual(
            try CBORDecoder.decode(Data([0x9F, 0x01, 0x02, 0xFF])),
            .array([.integer(1), .integer(2)])
        )
        XCTAssertEqual(
            try CBORDecoder.decode(Data([0xBF, 0x61, 0x61, 0x01, 0xFF])),
            .object(["a": .integer(1)])
        )
        XCTAssertEqual(
            try CBORDecoder.decode(Data([0x5F, 0x42, 0x01, 0x02, 0x43, 0x03, 0x04, 0x05, 0xFF])),
            .bytes(Data([1, 2, 3, 4, 5]))
        )
    }

    func testCBORTypedArrayTag64KeepsImageBytesRaw() throws {
        // Tag 64 (uint8 typed array) wrapping a byte string is how a
        // sensor_msgs/Image `data` field arrives under compression: "cbor".
        // Keeping it as bytes is the entire performance win.
        let encoded = Data([0xD8, 0x40, 0x44, 0xDE, 0xAD, 0xBE, 0xEF])
        XCTAssertEqual(try CBORDecoder.decode(encoded), .bytes(Data([0xDE, 0xAD, 0xBE, 0xEF])))
    }

    func testCBORTypedArrayTag69DecodesLittleEndianUInt16() throws {
        // Tag 69: uint16, little endian. 0x0102 -> 513, 0x0304 -> 1027.
        let encoded = Data([0xD8, 0x45, 0x44, 0x01, 0x02, 0x03, 0x04])
        let decoded = try CBORDecoder.decode(encoded)
        XCTAssertEqual(decoded.doubleArrayValue, [513, 1027])
    }

    func testCBORTypedArrayTag65DecodesBigEndianUInt16() throws {
        let encoded = Data([0xD8, 0x41, 0x44, 0x01, 0x02, 0x03, 0x04])
        let decoded = try CBORDecoder.decode(encoded)
        XCTAssertEqual(decoded.doubleArrayValue, [258, 772])
    }

    func testCBORUnknownTagsAreTransparent() throws {
        // Tag 0 is a date string; the app has no use for it but must not fail
        // the whole message over it.
        let encoded = Data([0xC0, 0x63, 0x61, 0x62, 0x63])
        XCTAssertEqual(try CBORDecoder.decode(encoded), .string("abc"))
    }

    func testCBORRejectsTruncatedInput() {
        XCTAssertThrowsError(try CBORDecoder.decode(Data([0x19, 0x03]))) { error in
            XCTAssertEqual(error as? CBORDecoder.Error, .unexpectedEnd)
        }
        XCTAssertThrowsError(try CBORDecoder.decode(Data([0x44, 0x01])))
    }

    func testCBORRejectsTrailingBytes() {
        XCTAssertThrowsError(try CBORDecoder.decode(Data([0x01, 0x02]))) { error in
            XCTAssertEqual(error as? CBORDecoder.Error, .trailingBytes(1))
        }
    }

    func testCBORRejectsRunawayNesting() {
        // 100 nested single-element arrays; the depth guard must stop it well
        // before the stack does.
        let encoded = Data([UInt8](repeating: 0x81, count: 100) + [0x01])
        XCTAssertThrowsError(try CBORDecoder.decode(encoded)) { error in
            XCTAssertEqual(error as? CBORDecoder.Error, .depthLimitExceeded)
        }
    }

    func testCBORDecodesAWholeImagePublishFrame() throws {
        // {"op":"publish","topic":"/t","msg":{"width":2,"height":1,
        //  "encoding":"mono8","step":2,"is_bigendian":0,"data":<tag64 bytes>}}
        var bytes: [UInt8] = [0xA3] // map(3)
        bytes += textKey("op") + text("publish")
        bytes += textKey("topic") + text("/t")
        bytes += textKey("msg")
        bytes += [0xA6] // map(6)
        bytes += textKey("width") + [0x02]
        bytes += textKey("height") + [0x01]
        bytes += textKey("encoding") + text("mono8")
        bytes += textKey("step") + [0x02]
        bytes += textKey("is_bigendian") + [0x00]
        bytes += textKey("data") + [0xD8, 0x40, 0x42, 0x11, 0x22]

        let value = try CBORDecoder.decode(Data(bytes))
        let incoming = try XCTUnwrap(RosbridgeIncoming.parse(value))
        guard case .publish(let topic, let message) = incoming else {
            XCTFail("expected a publish frame")
            return
        }
        XCTAssertEqual(topic, "/t")

        let image = try XCTUnwrap(ROSMessageParser.image(from: message))
        XCTAssertEqual(image.width, 2)
        XCTAssertEqual(image.encoding, "mono8")
        XCTAssertEqual(image.data, Data([0x11, 0x22]))
    }

    private func text(_ value: String) -> [UInt8] {
        let utf8 = [UInt8](value.utf8)
        precondition(utf8.count < 24, "test helper only encodes short strings")
        return [0x60 | UInt8(utf8.count)] + utf8
    }

    private func textKey(_ value: String) -> [UInt8] { text(value) }

    // MARK: - Outbound commands

    func testSubscribeCommandCarriesTheBacklogControls() throws {
        let command = RosbridgeCommand.subscribe(.init(
            topic: "/camera/depth/image_raw",
            messageType: "sensor_msgs/msg/Image",
            id: "sub-1",
            throttleMilliseconds: 66,
            queueLength: 1,
            compression: .cbor
        ))
        let payload = command.payload

        XCTAssertEqual(payload["op"] as? String, "subscribe")
        XCTAssertEqual(payload["topic"] as? String, "/camera/depth/image_raw")
        XCTAssertEqual(payload["type"] as? String, "sensor_msgs/msg/Image")
        XCTAssertEqual(payload["throttle_rate"] as? Int, 66)
        // queue_length 1 is what stops rosbridge itself from queueing frames.
        XCTAssertEqual(payload["queue_length"] as? Int, 1)
        XCTAssertEqual(payload["compression"] as? String, "cbor")
        XCTAssertNil(payload["fragment_size"])

        XCTAssertNoThrow(try command.encoded())
    }

    func testUnsubscribeCommandRoundTripsThroughJSON() throws {
        let command = RosbridgeCommand.unsubscribe(topic: "/odom", id: "sub-2")
        let value = try ROSValue.fromJSON(try command.encoded())
        XCTAssertEqual(value["op"]?.stringValue, "unsubscribe")
        XCTAssertEqual(value["topic"]?.stringValue, "/odom")
        XCTAssertEqual(value["id"]?.stringValue, "sub-2")
    }

    func testTopicDefaultsKeepImagesThrottledAndOdometryFree() {
        XCTAssertGreaterThan(RobotTopic.depthImage.defaultThrottleMilliseconds, 0)
        XCTAssertEqual(RobotTopic.odometry.defaultThrottleMilliseconds, 0)
        XCTAssertEqual(RobotTopic.depthImage.defaultQueueLength, 1)
        XCTAssertGreaterThan(RobotTopic.odometry.defaultQueueLength, 1)
    }

    func testTopicMessageTypesMatchTheRobotContract() {
        XCTAssertEqual(RobotTopic.depthImage.messageType, "sensor_msgs/msg/Image")
        XCTAssertEqual(RobotTopic.depthCameraInfo.messageType, "sensor_msgs/msg/CameraInfo")
        XCTAssertEqual(RobotTopic.odometry.messageType, "nav_msgs/msg/Odometry")
        XCTAssertEqual(RobotTopic.odomCalibrated.messageType, "std_msgs/msg/Bool")
        XCTAssertEqual(RobotTopic.imu.messageType, "sensor_msgs/msg/Imu")
    }

    /// The app is depth-only: subscribing to a thermal topic would cost
    /// bandwidth on the robot for a stream nothing renders.
    ///
    /// The `cropped` topics are barred for the same reason even though their
    /// names say nothing about thermal: they are republished by the thermal
    /// cropper, so subscribing to one makes the app's depth view depend on the
    /// thermal sensor finding a hot region.
    func testNoThermalTopicIsSubscribed() {
        XCTAssertFalse(RobotTopic.allCases.contains { $0.topicName.contains("thermal") })
        XCTAssertFalse(RobotTopic.allCases.contains { $0.topicName.contains("cropped") })
        XCTAssertEqual(RobotTopic.allCases.filter(\.isImageTopic), [.depthImage])
    }

    /// `OdomNodeStatus` reads these and nothing else, so a topic added to the
    /// wrong side of this split would silently change what "running" means.
    func testOdomNodeTopicsAreTheOnesOdomNodePublishes() {
        XCTAssertEqual(
            Set(RobotTopic.allCases.filter(\.isPublishedByOdomNode)),
            [.odometry, .odomCalibrated, .imu]
        )
    }

    // MARK: - Inbound parsing

    func testIncomingPublishParses() throws {
        let value = try ROSValue.fromJSON(Data(#"{"op":"publish","topic":"/a","msg":{"data":true}}"#.utf8))
        guard case .publish(let topic, let message)? = RosbridgeIncoming.parse(value) else {
            XCTFail("expected publish")
            return
        }
        XCTAssertEqual(topic, "/a")
        XCTAssertEqual(ROSMessageParser.boolean(from: message), true)
    }

    func testIncomingStatusParses() throws {
        let value = try FixtureLibrary.value(named: "status_error")
        guard case .status(let level, let message, _)? = RosbridgeIncoming.parse(value) else {
            XCTFail("expected status")
            return
        }
        XCTAssertEqual(level, "error")
        XCTAssertTrue(message.contains("/odom"))
    }

    func testUnknownOpsAreKeptRatherThanDropped() throws {
        let value = try ROSValue.fromJSON(Data(#"{"op":"png","data":"x"}"#.utf8))
        guard case .other(let op, _)? = RosbridgeIncoming.parse(value) else {
            XCTFail("expected other")
            return
        }
        XCTAssertEqual(op, "png")
    }

    func testFramesWithoutAnOpAreRejected() throws {
        let value = try ROSValue.fromJSON(Data(#"{"topic":"/a"}"#.utf8))
        XCTAssertNil(RosbridgeIncoming.parse(value))
    }

    // MARK: - Message parsers

    func testTimeParsingAcceptsBothROS1AndROS2FieldNames() {
        let ros2 = ROSValue.object(["sec": .integer(10), "nanosec": .integer(500_000_000)])
        let ros1 = ROSValue.object(["secs": .integer(10), "nsecs": .integer(500_000_000)])
        XCTAssertEqual(ROSMessageParser.time(from: ros2) ?? 0, 10.5, accuracy: 1e-9)
        XCTAssertEqual(ROSMessageParser.time(from: ros1) ?? 0, 10.5, accuracy: 1e-9)
    }

    func testQuaternionParserRejectsAnAllZeroMessage() {
        // An uninitialised geometry_msgs/Quaternion is all zeros; normalising
        // it would silently produce identity and hide the problem.
        let zero = ROSValue.object([
            "x": .number(0), "y": .number(0), "z": .number(0), "w": .number(0),
        ])
        XCTAssertNil(ROSMessageParser.quaternion(from: zero))
    }

    func testQuaternionParserNormalises() throws {
        let value = ROSValue.object([
            "x": .number(0), "y": .number(0), "z": .number(0), "w": .number(2),
        ])
        let quaternion = try XCTUnwrap(ROSMessageParser.quaternion(from: value))
        XCTAssertEqual(quaternion.length, 1.0, accuracy: 1e-12)
    }

    func testDepthFixtureDecodesToExpectedMetres() throws {
        let message = try FixtureLibrary.message(named: "depth_16uc1")
        let image = try XCTUnwrap(ROSMessageParser.image(from: message))

        XCTAssertEqual(image.width, 4)
        XCTAssertEqual(image.height, 3)
        XCTAssertEqual(image.encoding, "16UC1")
        XCTAssertEqual(image.step, 10)
        XCTAssertFalse(image.isBigEndian)
        XCTAssertEqual(image.stamp, 1717430000.25, accuracy: 1e-6)

        guard case .scalar(let decoded) = try ROSImageDecoder.decode(image, interpretation: .depthMillimetres) else {
            XCTFail("expected a scalar image")
            return
        }
        XCTAssertEqual(Double(decoded.value(x: 0, y: 0)), 1.0, accuracy: 1e-6)
        XCTAssertEqual(Double(decoded.value(x: 2, y: 0)), 3.0, accuracy: 1e-6)
        XCTAssertTrue(decoded.value(x: 3, y: 0).isNaN)
        XCTAssertEqual(Double(decoded.value(x: 3, y: 1)), 4.0, accuracy: 1e-6)
        XCTAssertEqual(Double(decoded.value(x: 0, y: 2)), 0.5, accuracy: 1e-6)
        XCTAssertEqual(Double(decoded.value(x: 2, y: 2)), 0.25, accuracy: 1e-6)
    }

    func testBigEndianDepthFixtureMatchesTheLittleEndianOne() throws {
        let little = try XCTUnwrap(ROSMessageParser.image(from: try FixtureLibrary.message(named: "depth_16uc1")))
        let big = try XCTUnwrap(
            ROSMessageParser.image(from: try FixtureLibrary.message(named: "depth_16uc1_bigendian"))
        )
        XCTAssertTrue(big.isBigEndian)

        guard case .scalar(let a) = try ROSImageDecoder.decode(little, interpretation: .depthMillimetres),
              case .scalar(let b) = try ROSImageDecoder.decode(big, interpretation: .depthMillimetres) else {
            XCTFail("expected scalar images")
            return
        }
        for index in a.values.indices {
            if a.values[index].isNaN {
                XCTAssertTrue(b.values[index].isNaN)
            } else {
                XCTAssertEqual(a.values[index], b.values[index])
            }
        }
    }

    func testColourFixturesAgree() throws {
        let rgb = try XCTUnwrap(ROSMessageParser.image(from: try FixtureLibrary.message(named: "color_rgb8")))
        let bgr = try XCTUnwrap(ROSMessageParser.image(from: try FixtureLibrary.message(named: "color_bgr8")))

        guard case .color(let a) = try ROSImageDecoder.decode(rgb),
              case .color(let b) = try ROSImageDecoder.decode(bgr) else {
            XCTFail("expected colour images")
            return
        }
        XCTAssertEqual(a.rgba, b.rgba)
        XCTAssertEqual(a.rgba, [255, 0, 0, 255, 0, 255, 0, 255])
    }

    func testCameraInfoFixtureYieldsAFieldOfView() throws {
        let info = try XCTUnwrap(ROSMessageParser.cameraInfo(from: try FixtureLibrary.message(named: "camera_info")))
        XCTAssertEqual(info.width, 582)
        XCTAssertEqual(info.height, 360)
        XCTAssertEqual(info.focalLengthX ?? 0, 520.0, accuracy: 1e-9)
        XCTAssertEqual(info.principalPointX ?? 0, 291.0, accuracy: 1e-9)

        let horizontal = try XCTUnwrap(info.horizontalFieldOfView)
        XCTAssertEqual(horizontal, 2 * atan(582.0 / (2 * 520.0)), accuracy: 1e-9)
        // Sanity: a 582 px wide sensor at f=520 is a touch under 60 degrees.
        XCTAssertEqual(horizontal * 180 / .pi, 58.2, accuracy: 0.5)
    }

    func testOdometryFixtureParsesPoseAndTwist() throws {
        let odometry = try XCTUnwrap(ROSMessageParser.odometry(from: try FixtureLibrary.message(named: "odometry")))

        XCTAssertEqual(odometry.frameId, "odom")
        XCTAssertEqual(odometry.childFrameId, "base_link")
        XCTAssertEqual(odometry.stamp, 1717430010.1, accuracy: 1e-6)
        XCTAssertEqual(odometry.pose.position.x, 1.5, accuracy: 1e-9)
        XCTAssertEqual(odometry.pose.position.y, -0.25, accuracy: 1e-9)
        XCTAssertEqual(odometry.pose.orientation.yawAroundZ, .pi / 2, accuracy: 1e-9)
        XCTAssertEqual(odometry.linearVelocity.x, 0.4, accuracy: 1e-9)
        XCTAssertEqual(odometry.angularVelocity.z, 0.1, accuracy: 1e-9)

        XCTAssertEqual(try XCTUnwrap(odometry.positionVariance), 0.02, accuracy: 1e-9)
        XCTAssertTrue(odometry.isPositionObserved)
    }

    /// What `odom_node` actually publishes: a real orientation, a placeholder
    /// position, and a covariance that admits it.
    func testOrientationOnlyOdometryFixtureReportsPositionUnobserved() throws {
        let odometry = try XCTUnwrap(
            ROSMessageParser.odometry(from: try FixtureLibrary.message(named: "odometry_orientation_only"))
        )
        XCTAssertEqual(odometry.pose.orientation.yawAroundZ, .pi / 2, accuracy: 1e-9)
        XCTAssertEqual(odometry.pose.position, .zero)
        XCTAssertEqual(try XCTUnwrap(odometry.positionVariance), 1.0e6, accuracy: 1e-3)
        XCTAssertFalse(odometry.isPositionObserved)
    }

    /// A zeroed covariance is "nobody filled this in", not "perfectly known".
    /// Treating it as absent — and absent as observed — keeps the app working
    /// with publishers that never populate the field.
    func testZeroedCovarianceReadsAsAbsentRatherThanCertain() throws {
        let json = """
        {"header":{"stamp":{"sec":1,"nanosec":0},"frame_id":"odom"},
         "child_frame_id":"base_link",
         "pose":{"pose":{"position":{"x":1,"y":0,"z":0},
                         "orientation":{"x":0,"y":0,"z":0,"w":1}},
                 "covariance":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}}
        """
        let odometry = try XCTUnwrap(ROSMessageParser.odometry(from: try ROSValue.fromJSON(Data(json.utf8))))
        XCTAssertNil(odometry.positionVariance)
        XCTAssertTrue(odometry.isPositionObserved)
    }

    func testMissingCovarianceIsTreatedAsObserved() throws {
        let json = """
        {"header":{"stamp":{"sec":1,"nanosec":0},"frame_id":"odom"},
         "child_frame_id":"base_link",
         "pose":{"pose":{"position":{"x":1,"y":0,"z":0},
                         "orientation":{"x":0,"y":0,"z":0,"w":1}}}}
        """
        let odometry = try XCTUnwrap(ROSMessageParser.odometry(from: try ROSValue.fromJSON(Data(json.utf8))))
        XCTAssertNil(odometry.positionVariance)
        XCTAssertTrue(odometry.isPositionObserved)
    }

    func testOdometryWithoutAPoseIsRejected() throws {
        let value = try ROSValue.fromJSON(Data(#"{"header":{"stamp":{"sec":1,"nanosec":0}}}"#.utf8))
        XCTAssertNil(ROSMessageParser.odometry(from: value))
    }

    func testBoolFixturesParse() throws {
        XCTAssertEqual(
            ROSMessageParser.boolean(from: try FixtureLibrary.message(named: "odom_calibrated_true")),
            true
        )
        XCTAssertEqual(
            ROSMessageParser.boolean(from: try FixtureLibrary.message(named: "odom_calibrated_false")),
            false
        )
    }

    func testImuFixtureParses() throws {
        let imu = try XCTUnwrap(ROSMessageParser.imu(from: try FixtureLibrary.message(named: "imu")))
        XCTAssertEqual(imu.frameId, "base_link")
        XCTAssertEqual(imu.angularVelocity.z, 0.15, accuracy: 1e-9)
        XCTAssertNotNil(imu.orientation)

        // odom_node marks acceleration unavailable with covariance[0] = -1.
        // Reading it as an honest nil rather than as (0, 0, 0) is what keeps
        // the diagnostics screen from showing a 0.00 m/s² gravity check and
        // flagging a perfectly healthy node as broken.
        XCTAssertNil(imu.linearAcceleration)
        XCTAssertNil(imu.accelerationMagnitude)
    }

    func testImuAccelerationIsReadWhenTheCovarianceDoesNotDisclaimIt() throws {
        let json = """
        {"header":{"stamp":{"sec":1,"nanosec":0},"frame_id":"base_link"},
         "orientation":{"x":0,"y":0,"z":0,"w":1},
         "orientation_covariance":[0,0,0,0,0,0,0,0,0],
         "angular_velocity":{"x":0,"y":0,"z":0},
         "angular_velocity_covariance":[0,0,0,0,0,0,0,0,0],
         "linear_acceleration":{"x":0.12,"y":-0.05,"z":9.79},
         "linear_acceleration_covariance":[0,0,0,0,0,0,0,0,0]}
        """
        let imu = try XCTUnwrap(ROSMessageParser.imu(from: try ROSValue.fromJSON(Data(json.utf8))))
        let acceleration = try XCTUnwrap(imu.linearAcceleration)
        XCTAssertEqual(acceleration.z, 9.79, accuracy: 1e-9)
        XCTAssertEqual(try XCTUnwrap(imu.accelerationMagnitude), 9.79082, accuracy: 1e-4)
    }

    func testDashboardStateFixtureParses() throws {
        let state = DashboardState.parse(try FixtureLibrary.value(named: "dashboard_state"))
        XCTAssertTrue(state.isRunning)
        XCTAssertEqual(state.logs.count, 2)
        XCTAssertEqual(state.cpuCores.count, 4)
        XCTAssertEqual(state.cpuCores[1].load, 63)
        XCTAssertEqual(state.cpuTemperature ?? 0, 58, accuracy: 1e-9)
    }

    func testEveryFixtureIsReadable() throws {
        for name in FixtureLibrary.allNames {
            XCTAssertNoThrow(try FixtureLibrary.value(named: name), "fixture \(name)")
        }
    }

    static var allTests: [(String, (RosbridgeTests) -> () throws -> Void)] {
        [
            ("testJSONBooleansSurviveAsBooleansNotNumbers", testJSONBooleansSurviveAsBooleansNotNumbers),
            ("testNumbersKeepIntegerAndFloatingPointApart", testNumbersKeepIntegerAndFloatingPointApart),
            ("testByteArrayAcceptsBase64AndPlainArrays", testByteArrayAcceptsBase64AndPlainArrays),
            ("testDoubleArrayReadsCameraInfoMatrices", testDoubleArrayReadsCameraInfoMatrices),
            ("testNullAndMissingKeysAreDistinguishable", testNullAndMissingKeysAreDistinguishable),
            ("testCBORUnsignedIntegers", testCBORUnsignedIntegers),
            ("testCBORNegativeIntegers", testCBORNegativeIntegers),
            ("testCBORSimpleValues", testCBORSimpleValues),
            ("testCBORFloats", testCBORFloats),
            ("testCBORStringsAndByteStrings", testCBORStringsAndByteStrings),
            ("testCBORArraysAndMaps", testCBORArraysAndMaps),
            ("testCBORIndefiniteLengthContainers", testCBORIndefiniteLengthContainers),
            ("testCBORTypedArrayTag64KeepsImageBytesRaw", testCBORTypedArrayTag64KeepsImageBytesRaw),
            ("testCBORTypedArrayTag69DecodesLittleEndianUInt16", testCBORTypedArrayTag69DecodesLittleEndianUInt16),
            ("testCBORTypedArrayTag65DecodesBigEndianUInt16", testCBORTypedArrayTag65DecodesBigEndianUInt16),
            ("testCBORUnknownTagsAreTransparent", testCBORUnknownTagsAreTransparent),
            ("testCBORRejectsTruncatedInput", testCBORRejectsTruncatedInput),
            ("testCBORRejectsTrailingBytes", testCBORRejectsTrailingBytes),
            ("testCBORRejectsRunawayNesting", testCBORRejectsRunawayNesting),
            ("testCBORDecodesAWholeImagePublishFrame", testCBORDecodesAWholeImagePublishFrame),
            ("testSubscribeCommandCarriesTheBacklogControls", testSubscribeCommandCarriesTheBacklogControls),
            ("testUnsubscribeCommandRoundTripsThroughJSON", testUnsubscribeCommandRoundTripsThroughJSON),
            ("testTopicDefaultsKeepImagesThrottledAndOdometryFree", testTopicDefaultsKeepImagesThrottledAndOdometryFree),
            ("testTopicMessageTypesMatchTheRobotContract", testTopicMessageTypesMatchTheRobotContract),
            ("testNoThermalTopicIsSubscribed", testNoThermalTopicIsSubscribed),
            ("testOdomNodeTopicsAreTheOnesOdomNodePublishes", testOdomNodeTopicsAreTheOnesOdomNodePublishes),
            ("testIncomingPublishParses", testIncomingPublishParses),
            ("testIncomingStatusParses", testIncomingStatusParses),
            ("testUnknownOpsAreKeptRatherThanDropped", testUnknownOpsAreKeptRatherThanDropped),
            ("testFramesWithoutAnOpAreRejected", testFramesWithoutAnOpAreRejected),
            ("testTimeParsingAcceptsBothROS1AndROS2FieldNames", testTimeParsingAcceptsBothROS1AndROS2FieldNames),
            ("testQuaternionParserRejectsAnAllZeroMessage", testQuaternionParserRejectsAnAllZeroMessage),
            ("testQuaternionParserNormalises", testQuaternionParserNormalises),
            ("testDepthFixtureDecodesToExpectedMetres", testDepthFixtureDecodesToExpectedMetres),
            ("testBigEndianDepthFixtureMatchesTheLittleEndianOne", testBigEndianDepthFixtureMatchesTheLittleEndianOne),
            ("testColourFixturesAgree", testColourFixturesAgree),
            ("testCameraInfoFixtureYieldsAFieldOfView", testCameraInfoFixtureYieldsAFieldOfView),
            ("testOdometryFixtureParsesPoseAndTwist", testOdometryFixtureParsesPoseAndTwist),
            ("testOdometryWithoutAPoseIsRejected", testOdometryWithoutAPoseIsRejected),
            ("testOrientationOnlyOdometryFixtureReportsPositionUnobserved", testOrientationOnlyOdometryFixtureReportsPositionUnobserved),
            ("testZeroedCovarianceReadsAsAbsentRatherThanCertain", testZeroedCovarianceReadsAsAbsentRatherThanCertain),
            ("testMissingCovarianceIsTreatedAsObserved", testMissingCovarianceIsTreatedAsObserved),
            ("testBoolFixturesParse", testBoolFixturesParse),
            ("testImuFixtureParses", testImuFixtureParses),
            ("testImuAccelerationIsReadWhenTheCovarianceDoesNotDisclaimIt", testImuAccelerationIsReadWhenTheCovarianceDoesNotDisclaimIt),
            ("testDashboardStateFixtureParses", testDashboardStateFixtureParses),
            ("testEveryFixtureIsReadable", testEveryFixtureIsReadable),
        ]
    }
}
