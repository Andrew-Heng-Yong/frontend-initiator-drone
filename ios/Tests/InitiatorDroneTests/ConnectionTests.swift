#if canImport(XCTest)
import XCTest
// In Xcode the tests compile as their own module; the headless runner in
// Scripts/ compiles them alongside the sources instead, so neither import
// exists there.
@testable import InitiatorDrone
#endif
import Foundation

/// Endpoints, tracking-status derivation, reconnection, and an end-to-end run
/// of the client against the fixture transport.
final class ConnectionTests: XCTestCase {

    // MARK: - Endpoint

    func testHostNormalisationStripsWhateverWasPasted() {
        let cases: [(String, String)] = [
            ("192.168.1.42", "192.168.1.42"),
            ("  192.168.1.42  ", "192.168.1.42"),
            ("http://192.168.1.42", "192.168.1.42"),
            ("http://192.168.1.42:4173", "192.168.1.42"),
            ("http://192.168.1.42:4173/", "192.168.1.42"),
            ("ws://drone.local:9090", "drone.local"),
            ("HTTPS://Drone.local/", "Drone.local"),
        ]
        for (input, expected) in cases {
            XCTAssertEqual(RobotEndpoint(host: input).normalizedHost, expected, "input '\(input)'")
        }
    }

    func testURLsAreBuiltFromTheNormalisedHost() {
        let endpoint = RobotEndpoint(host: "http://10.0.0.7:4173/", dashboardPort: 4173, rosbridgePort: 9090)
        XCTAssertEqual(endpoint.dashboardBaseURL?.absoluteString, "http://10.0.0.7:4173")
        XCTAssertEqual(endpoint.rosbridgeURL?.absoluteString, "ws://10.0.0.7:9090/")
    }

    func testTLSSwitchesBothSchemes() {
        let endpoint = RobotEndpoint(host: "robot.example", useTLS: true)
        XCTAssertEqual(endpoint.dashboardBaseURL?.scheme, "https")
        XCTAssertEqual(endpoint.rosbridgeURL?.scheme, "wss")
    }

    func testValidationRejectsUnusableAddresses() {
        XCTAssertFalse(RobotEndpoint(host: "").isValid)
        XCTAssertFalse(RobotEndpoint(host: "   ").isValid)
        XCTAssertFalse(RobotEndpoint(host: "has space").isValid)
        XCTAssertFalse(RobotEndpoint(host: "10.0.0.1", dashboardPort: 0).isValid)
        XCTAssertFalse(RobotEndpoint(host: "10.0.0.1", rosbridgePort: 70000).isValid)
        XCTAssertTrue(RobotEndpoint(host: "10.0.0.1").isValid)
        XCTAssertTrue(RobotEndpoint(host: "drone-01.local").isValid)
    }

    func testDisplayNameFallsBackToHost() {
        XCTAssertEqual(RobotEndpoint(name: "  ", host: "10.0.0.1").displayName, "10.0.0.1")
        XCTAssertEqual(RobotEndpoint(name: "Rooftop", host: "10.0.0.1").displayName, "Rooftop")
    }

    // MARK: - Tracking status

    func testPublishingOdometryAloneIsNotTracking() {
        // The requirement stated as a test: fresh odometry with visual tracking
        // false must never read as tracking.
        let status = RobotTrackingStatus.evaluate(
            isCalibrated: true,
            isVisualTracking: false,
            odometryAge: 0.05
        )
        XCTAssertEqual(status, .visualTrackingLost)
        XCTAssertFalse(status.isTrustworthy)
        // The pose is still worth drawing, greyed out: it says where the robot
        // was when tracking failed.
        XCTAssertTrue(status.hasUsablePose)
    }

    func testFreshCalibratedAndTrackingIsTrustworthy() {
        let status = RobotTrackingStatus.evaluate(
            isCalibrated: true,
            isVisualTracking: true,
            odometryAge: 0.02
        )
        XCTAssertEqual(status, .tracking)
        XCTAssertTrue(status.isTrustworthy)
    }

    func testStalenessOutranksTheFlags() {
        // A "tracking is fine" flag from thirty seconds ago proves nothing.
        let status = RobotTrackingStatus.evaluate(
            isCalibrated: true,
            isVisualTracking: true,
            odometryAge: 3.0,
            stalenessThreshold: 0.5
        )
        XCTAssertEqual(status, .stale(age: 3.0))
        XCTAssertFalse(status.isTrustworthy)
    }

    func testUncalibratedHasNoUsablePose() {
        let status = RobotTrackingStatus.evaluate(
            isCalibrated: false,
            isVisualTracking: true,
            odometryAge: 0.01
        )
        XCTAssertEqual(status, .notCalibrated)
        XCTAssertFalse(status.hasUsablePose)
    }

    func testMissingFlagsReadAsUnknownRatherThanGood() {
        XCTAssertEqual(
            RobotTrackingStatus.evaluate(isCalibrated: nil, isVisualTracking: nil, odometryAge: 0.01),
            .unknown
        )
        XCTAssertEqual(
            RobotTrackingStatus.evaluate(isCalibrated: true, isVisualTracking: nil, odometryAge: 0.01),
            .unknown
        )
        XCTAssertEqual(
            RobotTrackingStatus.evaluate(isCalibrated: nil, isVisualTracking: nil, odometryAge: nil),
            .unknown
        )
    }

    func testNoOdometryYetButUncalibratedReportsTheRealCause() {
        XCTAssertEqual(
            RobotTrackingStatus.evaluate(isCalibrated: false, isVisualTracking: nil, odometryAge: nil),
            .notCalibrated
        )
    }

    // MARK: - Reconnect policy

    func testBackoffGrowsAndThenLevelsOff() {
        let policy = ReconnectPolicy(initialDelay: 0.5, maximumDelay: 8.0, multiplier: 2.0, jitterFraction: 0)

        XCTAssertEqual(policy.baseDelay(forAttempt: 0), 0.5, accuracy: 1e-9)
        XCTAssertEqual(policy.baseDelay(forAttempt: 1), 1.0, accuracy: 1e-9)
        XCTAssertEqual(policy.baseDelay(forAttempt: 2), 2.0, accuracy: 1e-9)
        XCTAssertEqual(policy.baseDelay(forAttempt: 3), 4.0, accuracy: 1e-9)
        XCTAssertEqual(policy.baseDelay(forAttempt: 4), 8.0, accuracy: 1e-9)
        XCTAssertEqual(policy.baseDelay(forAttempt: 40), 8.0, accuracy: 1e-9)
    }

    func testBackoffIsMonotonic() {
        let policy = ReconnectPolicy()
        var previous = 0.0
        for attempt in 0..<12 {
            let delay = policy.baseDelay(forAttempt: attempt)
            XCTAssertGreaterThanOrEqual(delay, previous)
            previous = delay
        }
    }

    func testJitterStaysWithinItsBandAndNeverGoesNegative() {
        let policy = ReconnectPolicy(initialDelay: 1.0, maximumDelay: 10.0, multiplier: 2.0, jitterFraction: 0.25)
        for attempt in 0..<6 {
            let base = policy.baseDelay(forAttempt: attempt)
            for unit in stride(from: 0.0, through: 1.0, by: 0.1) {
                let delay = policy.delay(forAttempt: attempt, randomUnit: unit)
                XCTAssertGreaterThanOrEqual(delay, 0.05)
                XCTAssertLessThanOrEqual(delay, min(base * 1.25 + 1e-9, 10.0))
            }
        }
    }

    func testZeroJitterIsExactlyTheBaseDelay() {
        let policy = ReconnectPolicy(initialDelay: 2.0, jitterFraction: 0)
        XCTAssertEqual(policy.delay(forAttempt: 0, randomUnit: 0.9), 2.0, accuracy: 1e-12)
    }

    func testPolicyClampsNonsensicalConfiguration() {
        let policy = ReconnectPolicy(initialDelay: -5, maximumDelay: -1, multiplier: 0.1, jitterFraction: 4)
        XCTAssertGreaterThan(policy.initialDelay, 0)
        XCTAssertGreaterThanOrEqual(policy.maximumDelay, policy.initialDelay)
        XCTAssertGreaterThanOrEqual(policy.multiplier, 1.0)
        XCTAssertLessThanOrEqual(policy.jitterFraction, 1.0)
    }

    // MARK: - End-to-end against the fixture transport

    /// Collects client events for assertions, on the main queue.
    private final class EventSink {
        var states: [RosbridgeConnectionState] = []
        var images: [RobotTopic: Int] = [:]
        var odometry: [OdometryMessage] = []
        var flags: [RobotTopic: Bool] = [:]
        var imu: [ImuMessage] = []
        var cameraInfo: [CameraInfoMessage] = []
        var health: [TopicHealth] = []
        var logs: [RosbridgeLogEntry] = []

        func handle(_ event: RosbridgeEvent) {
            switch event {
            case .stateChanged(let state): states.append(state)
            case .image(let topic, _): images[topic, default: 0] += 1
            case .odometry(let message): odometry.append(message)
            case .flag(let topic, let value): flags[topic] = value
            case .imu(let message): imu.append(message)
            case .cameraInfo(let info): cameraInfo.append(info)
            case .health(let value): health = value
            case .log(let entry): logs.append(entry)
            }
        }
    }

    func testClientStreamsEveryTopicFromTheFixtureTransport() {
        let transport = SimulatedRosbridgeTransport()
        let client = RosbridgeClient(transport: transport)
        let sink = EventSink()

        let connected = expectation(description: "connected")
        let streaming = expectation(description: "all topics seen")

        client.onEvent = { event in
            sink.handle(event)
            if case .stateChanged(.connected) = event { connected.fulfill() }
            if sink.images[.depthImage] ?? 0 >= 2,
               sink.images[.thermalImage] ?? 0 >= 2,
               sink.odometry.count >= 5,
               !sink.imu.isEmpty,
               !sink.cameraInfo.isEmpty,
               sink.flags.count == 2 {
                streaming.fulfill()
            }
        }

        client.connect(to: RobotEndpoint(host: "fixtures.local"))
        wait(for: [connected, streaming], timeout: 10)

        XCTAssertEqual(sink.flags[.vioCalibrated], true)
        XCTAssertEqual(sink.flags[.visualTracking], true)

        // Odometry stamps must advance, which is what makes interpolation
        // meaningful downstream.
        if sink.odometry.count >= 2 {
            XCTAssertGreaterThan(sink.odometry.last!.stamp, sink.odometry.first!.stamp)
        }

        client.disconnect()
    }

    func testClientReconnectsAfterAWiFiDropAndResubscribes() {
        // The Wi-Fi-loss path: the socket goes quiet with no clean close, the
        // client must notice, back off, reconnect and re-subscribe.
        let transport = SimulatedRosbridgeTransport()
        let client = RosbridgeClient(
            transport: transport,
            configuration: .init(
                topics: [.odometry, .vioCalibrated],
                reconnectPolicy: ReconnectPolicy(
                    initialDelay: 0.1,
                    maximumDelay: 0.3,
                    multiplier: 1.5,
                    jitterFraction: 0
                )
            )
        )
        let sink = EventSink()

        let firstConnect = expectation(description: "first connect")
        let flowingBeforeDrop = expectation(description: "odometry flowing before the drop")
        let sawReconnecting = expectation(description: "entered reconnecting")
        let secondConnect = expectation(description: "reconnected")
        secondConnect.expectedFulfillmentCount = 2 // the initial one plus the retry

        client.onEvent = { event in
            sink.handle(event)
            if case .stateChanged(.connected) = event {
                firstConnect.fulfill()
                secondConnect.fulfill()
            }
            if case .stateChanged(.reconnecting) = event {
                sawReconnecting.fulfill()
            }
            if sink.odometry.count >= 5 { flowingBeforeDrop.fulfill() }
        }

        client.connect(to: RobotEndpoint(host: "fixtures.local"))
        // Wait for real data, not just for the socket: the point of the test is
        // that the stream resumes, so it has to be running first.
        wait(for: [firstConnect, flowingBeforeDrop], timeout: 5)

        let odometryBeforeDrop = sink.odometry.count
        XCTAssertGreaterThan(odometryBeforeDrop, 0)

        transport.simulateConnectionLoss()
        wait(for: [sawReconnecting, secondConnect], timeout: 10)

        // Data must actually resume, not just the socket reopen.
        let resumed = expectation(description: "odometry resumed after reconnect")
        client.onEvent = { event in
            sink.handle(event)
            if sink.odometry.count > odometryBeforeDrop + 5 { resumed.fulfill() }
        }
        wait(for: [resumed], timeout: 10)

        client.disconnect()
    }

    func testDisconnectStopsTheStreamAndDoesNotReconnect() {
        let transport = SimulatedRosbridgeTransport()
        let client = RosbridgeClient(transport: transport)
        let sink = EventSink()

        let connected = expectation(description: "connected")
        client.onEvent = { event in
            sink.handle(event)
            if case .stateChanged(.connected) = event { connected.fulfill() }
        }
        client.connect(to: RobotEndpoint(host: "fixtures.local"))
        wait(for: [connected], timeout: 5)

        client.disconnect()

        let settled = sink.odometry.count
        let idle = expectation(description: "settle")
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.6) { idle.fulfill() }
        wait(for: [idle], timeout: 3)

        XCTAssertEqual(sink.states.last, .idle)
        XCTAssertLessThanOrEqual(sink.odometry.count - settled, 2, "no new data after disconnect")
    }

    func testHealthSnapshotsReportRatesForSubscribedTopics() {
        let transport = SimulatedRosbridgeTransport()
        let client = RosbridgeClient(
            transport: transport,
            configuration: .init(topics: [.odometry, .depthImage])
        )
        let sink = EventSink()

        let reported = expectation(description: "health with a live rate")
        client.onEvent = { event in
            sink.handle(event)
            if sink.health.contains(where: { $0.topic == .odometry && $0.rateHz > 1 }) {
                reported.fulfill()
            }
        }

        client.connect(to: RobotEndpoint(host: "fixtures.local"))
        wait(for: [reported], timeout: 10)

        let odometryHealth = sink.health.first { $0.topic == .odometry }
        XCTAssertNotNil(odometryHealth)
        XCTAssertTrue(odometryHealth?.isSubscribed ?? false)
        XCTAssertFalse(odometryHealth?.isSilent ?? true)

        client.disconnect()
    }

    func testSimulatorHonoursTheRequestedThrottle() {
        // The client asks rosbridge to hold frames back; the fixture transport
        // implements the same contract, so a slow throttle must show up as a
        // low rate rather than as dropped frames on the phone.
        let transport = SimulatedRosbridgeTransport()
        let client = RosbridgeClient(
            transport: transport,
            configuration: .init(
                topics: [.depthImage],
                throttleOverrides: [.depthImage: 500]
            )
        )
        let sink = EventSink()

        let done = expectation(description: "sampled")
        client.onEvent = { sink.handle($0) }
        client.connect(to: RobotEndpoint(host: "fixtures.local"))
        DispatchQueue.main.asyncAfter(deadline: .now() + 2.2) { done.fulfill() }
        wait(for: [done], timeout: 6)

        let count = sink.images[.depthImage] ?? 0
        XCTAssertGreaterThan(count, 1, "at least a couple of frames should arrive")
        XCTAssertLessThanOrEqual(count, 8, "a 500 ms throttle must not deliver 10 fps")

        client.disconnect()
    }

    func testSimulatedPoseIsContinuous() {
        // The synthesised trajectory backs the offline demo; a discontinuity in
        // it would look like a tracking failure that is not real.
        let transport = SimulatedRosbridgeTransport()
        var previous = transport.simulatedPose(at: 0)
        for step in 1...2000 {
            let time = Double(step) * 0.01
            let pose = transport.simulatedPose(at: time)
            XCTAssertLessThan(
                pose.position.distance(to: previous.position),
                0.1,
                "jump at t=\(time)"
            )
            previous = pose
        }
    }

    static var allTests: [(String, (ConnectionTests) -> () throws -> Void)] {
        [
            ("testHostNormalisationStripsWhateverWasPasted", testHostNormalisationStripsWhateverWasPasted),
            ("testURLsAreBuiltFromTheNormalisedHost", testURLsAreBuiltFromTheNormalisedHost),
            ("testTLSSwitchesBothSchemes", testTLSSwitchesBothSchemes),
            ("testValidationRejectsUnusableAddresses", testValidationRejectsUnusableAddresses),
            ("testDisplayNameFallsBackToHost", testDisplayNameFallsBackToHost),
            ("testPublishingOdometryAloneIsNotTracking", testPublishingOdometryAloneIsNotTracking),
            ("testFreshCalibratedAndTrackingIsTrustworthy", testFreshCalibratedAndTrackingIsTrustworthy),
            ("testStalenessOutranksTheFlags", testStalenessOutranksTheFlags),
            ("testUncalibratedHasNoUsablePose", testUncalibratedHasNoUsablePose),
            ("testMissingFlagsReadAsUnknownRatherThanGood", testMissingFlagsReadAsUnknownRatherThanGood),
            ("testNoOdometryYetButUncalibratedReportsTheRealCause", testNoOdometryYetButUncalibratedReportsTheRealCause),
            ("testBackoffGrowsAndThenLevelsOff", testBackoffGrowsAndThenLevelsOff),
            ("testBackoffIsMonotonic", testBackoffIsMonotonic),
            ("testJitterStaysWithinItsBandAndNeverGoesNegative", testJitterStaysWithinItsBandAndNeverGoesNegative),
            ("testZeroJitterIsExactlyTheBaseDelay", testZeroJitterIsExactlyTheBaseDelay),
            ("testPolicyClampsNonsensicalConfiguration", testPolicyClampsNonsensicalConfiguration),
            ("testClientStreamsEveryTopicFromTheFixtureTransport", testClientStreamsEveryTopicFromTheFixtureTransport),
            ("testClientReconnectsAfterAWiFiDropAndResubscribes", testClientReconnectsAfterAWiFiDropAndResubscribes),
            ("testDisconnectStopsTheStreamAndDoesNotReconnect", testDisconnectStopsTheStreamAndDoesNotReconnect),
            ("testHealthSnapshotsReportRatesForSubscribedTopics", testHealthSnapshotsReportRatesForSubscribedTopics),
            ("testSimulatorHonoursTheRequestedThrottle", testSimulatorHonoursTheRequestedThrottle),
            ("testSimulatedPoseIsContinuous", testSimulatedPoseIsContinuous),
        ]
    }
}
