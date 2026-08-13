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

    func testPublishingOdometryAloneIsNotFullTracking() {
        // The requirement stated as a test: fresh, calibrated odometry whose
        // covariance says position is unobserved must never read as tracking.
        // This is the normal state with odom_node.
        let status = RobotTrackingStatus.evaluate(
            isCalibrated: true,
            isPositionObserved: false,
            odometryAge: 0.05
        )
        XCTAssertEqual(status, .orientationOnly)
        XCTAssertFalse(status.isTrustworthy)
        // The heading is measured even though the position is not, and the pose
        // is still worth drawing: it says which way the robot is facing.
        XCTAssertTrue(status.isOrientationTrustworthy)
        XCTAssertTrue(status.hasUsablePose)
    }

    func testFreshCalibratedAndTrackingIsTrustworthy() {
        let status = RobotTrackingStatus.evaluate(
            isCalibrated: true,
            isPositionObserved: true,
            odometryAge: 0.02
        )
        XCTAssertEqual(status, .tracking)
        XCTAssertTrue(status.isTrustworthy)
    }

    func testStalenessOutranksTheFlags() {
        // A "tracking is fine" flag from thirty seconds ago proves nothing.
        let status = RobotTrackingStatus.evaluate(
            isCalibrated: true,
            isPositionObserved: true,
            odometryAge: 3.0,
            stalenessThreshold: 0.5
        )
        XCTAssertEqual(status, .stale(age: 3.0))
        XCTAssertFalse(status.isTrustworthy)
    }

    func testUncalibratedHasNoUsablePose() {
        let status = RobotTrackingStatus.evaluate(
            isCalibrated: false,
            isPositionObserved: true,
            odometryAge: 0.01
        )
        XCTAssertEqual(status, .notCalibrated)
        XCTAssertFalse(status.hasUsablePose)
    }

    func testMissingFlagsReadAsUnknownRatherThanGood() {
        XCTAssertEqual(
            RobotTrackingStatus.evaluate(isCalibrated: nil, isPositionObserved: nil, odometryAge: 0.01),
            .unknown
        )
        XCTAssertEqual(
            RobotTrackingStatus.evaluate(isCalibrated: true, isPositionObserved: nil, odometryAge: 0.01),
            .unknown
        )
        XCTAssertEqual(
            RobotTrackingStatus.evaluate(isCalibrated: nil, isPositionObserved: nil, odometryAge: nil),
            .unknown
        )
    }

    func testNoOdometryYetButUncalibratedReportsTheRealCause() {
        XCTAssertEqual(
            RobotTrackingStatus.evaluate(isCalibrated: false, isPositionObserved: nil, odometryAge: nil),
            .notCalibrated
        )
    }

    // MARK: - Odom node status

    /// Legacy nodes may publish calibration only on transition, and a status
    /// heartbeat can still be delayed or dropped. Odometry alone has to be
    /// enough to conclude the node is up.
    func testOdometryAloneProvesTheNodeIsRunning() {
        let status = OdomNodeStatus.evaluate(
            isLinkConnected: true,
            isGraphRunning: true,
            isCalibrated: nil,
            nodeMessageAge: 0.02,
            odometryAge: 0.02
        )
        XCTAssertEqual(status, .running)
        XCTAssertTrue(status.isNodePresent)
    }

    func testNothingOnAnyOdomTopicMeansTheNodeIsNotRunning() {
        let status = OdomNodeStatus.evaluate(
            isLinkConnected: true,
            isGraphRunning: true,
            isCalibrated: nil,
            nodeMessageAge: nil,
            odometryAge: nil
        )
        XCTAssertEqual(status, .notRunning)
        XCTAssertFalse(status.isNodePresent)
    }

    /// A stopped launch is a different problem from a crashed node, and saying
    /// so is the difference between "press Start" and "go and debug the Pi".
    func testAStoppedGraphOutranksTopicSilence() {
        XCTAssertEqual(
            OdomNodeStatus.evaluate(
                isLinkConnected: true,
                isGraphRunning: false,
                isCalibrated: nil,
                nodeMessageAge: nil,
                odometryAge: nil
            ),
            .graphStopped
        )
    }

    func testNoLinkMeansUnknownRatherThanNotRunning() {
        XCTAssertEqual(
            OdomNodeStatus.evaluate(
                isLinkConnected: false,
                isGraphRunning: true,
                isCalibrated: true,
                nodeMessageAge: 0.1,
                odometryAge: 0.1
            ),
            .unknown
        )
    }

    /// `calibrated = false` is the node explaining the odometry gap it is
    /// itself causing, so it must not read as a fault.
    func testCalibratingOutranksTheOdometryGapItCauses() {
        let status = OdomNodeStatus.evaluate(
            isLinkConnected: true,
            isGraphRunning: true,
            isCalibrated: false,
            nodeMessageAge: 0.3,
            odometryAge: nil
        )
        XCTAssertEqual(status, .calibrating)
        XCTAssertTrue(status.isNodePresent)
    }

    func testANodeThatStopsPublishingReadsAsSilentWithItsAge() {
        XCTAssertEqual(
            OdomNodeStatus.evaluate(
                isLinkConnected: true,
                isGraphRunning: true,
                isCalibrated: true,
                nodeMessageAge: 9.0,
                odometryAge: 9.0,
                silenceThreshold: 1.0
            ),
            .silent(age: 9.0)
        )

        // Heard from, but no pose has ever arrived.
        guard case .silent(let age) = OdomNodeStatus.evaluate(
            isLinkConnected: true,
            isGraphRunning: true,
            isCalibrated: true,
            nodeMessageAge: 0.2,
            odometryAge: nil
        ) else {
            XCTFail("expected silent")
            return
        }
        XCTAssertTrue(age.isInfinite)
    }

    /// Fixtures mode has no dashboard to ask, and an unknown launch state must
    /// not be read as a stopped one.
    func testUnknownGraphStateFallsThroughToTheTopics() {
        XCTAssertEqual(
            OdomNodeStatus.evaluate(
                isLinkConnected: true,
                isGraphRunning: nil,
                isCalibrated: true,
                nodeMessageAge: 0.02,
                odometryAge: 0.02
            ),
            .running
        )
    }

    // MARK: - AR video format choice

    /// The 4:3 format is the full sensor readout; every 16:9 one is a vertical
    /// crop of it, so the tallest frame is strictly more of the scene.
    func testTheTallestFrameWins() {
        let formats = [
            VideoFormatCandidate(width: 1920, height: 1080, framesPerSecond: 60),
            VideoFormatCandidate(width: 1920, height: 1440, framesPerSecond: 60),
            VideoFormatCandidate(width: 1280, height: 720, framesPerSecond: 60),
        ]
        XCTAssertEqual(VideoFormatSelection.widestFieldOfView(among: formats), 1)
    }

    /// Same shape means same coverage, so the choice falls back to ARKit's own
    /// ordering rather than to resolution or floating-point noise.
    func testEqualAspectRatiosKeepARKitsOrdering() {
        let formats = [
            VideoFormatCandidate(width: 1920, height: 1440, framesPerSecond: 60),
            VideoFormatCandidate(width: 3840, height: 2880, framesPerSecond: 30),
        ]
        XCTAssertEqual(VideoFormatSelection.widestFieldOfView(among: formats), 0)
    }

    func testNoFormatsYieldsNoChoice() {
        XCTAssertNil(VideoFormatSelection.widestFieldOfView(among: []))
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
               sink.odometry.count >= 5,
               !sink.imu.isEmpty,
               !sink.cameraInfo.isEmpty,
               sink.flags.count == 1 {
                streaming.fulfill()
            }
        }

        client.connect(to: RobotEndpoint(host: "fixtures.local"))
        wait(for: [connected, streaming], timeout: 10)

        XCTAssertEqual(sink.flags[.odomCalibrated], true)

        // Position must be reported as unobserved: odom_node integrates the
        // gyro only, and the simulator has to model that rather than inventing
        // a translation the real robot cannot produce.
        if let newest = sink.odometry.last {
            XCTAssertFalse(newest.isPositionObserved)
            XCTAssertEqual(newest.pose.position, .zero)
        }

        // Odometry stamps must advance, which is what makes interpolation
        // meaningful downstream.
        if sink.odometry.count >= 2 {
            XCTAssertGreaterThan(sink.odometry.last!.stamp, sink.odometry.first!.stamp)
        }

        client.disconnect()
    }

    /// Stop has to make the fixture graph genuinely silent, not merely flip a
    /// label — that silence is what the status pills and `OdomNodeStatus` read.
    func testStopSilencesTheFixtureGraphAndStartBringsItBack() {
        let transport = SimulatedRosbridgeTransport()
        transport.calibrationDuration = 0.3
        let client = RosbridgeClient(transport: transport)
        let sink = EventSink()

        let streaming = expectation(description: "streaming")
        client.onEvent = { event in
            sink.handle(event)
            if sink.odometry.count >= 3 { streaming.fulfill() }
        }
        client.connect(to: RobotEndpoint(host: "fixtures.local"))
        wait(for: [streaming], timeout: 10)
        XCTAssertTrue(transport.isGraphRunning)

        transport.stopGraph()
        XCTAssertFalse(transport.isGraphRunning)

        // Let anything already in flight land, then require true silence.
        Thread.sleep(forTimeInterval: 0.3)
        let settled = sink.odometry.count
        let settledImages = sink.images[.depthImage] ?? 0
        Thread.sleep(forTimeInterval: 0.5)
        XCTAssertEqual(sink.odometry.count, settled, "a stopped graph must publish no odometry")
        XCTAssertEqual(sink.images[.depthImage] ?? 0, settledImages, "a stopped graph must publish no depth")

        let resumed = expectation(description: "resumed")
        client.onEvent = { event in
            sink.handle(event)
            if sink.odometry.count > settled + 2 { resumed.fulfill() }
        }
        transport.startGraph()
        wait(for: [resumed], timeout: 10)

        client.disconnect()
    }

    /// Calibrate has to reproduce what `vio_node` does — drop `calibrated` to
    /// false, stop publishing a pose while it collects samples, then come back
    /// — because that is the sequence the VIO node status is built to read.
    func testCalibrateOnFixturesWithholdsOdometryUntilItCompletes() {
        let transport = SimulatedRosbridgeTransport()
        transport.calibrationDuration = 0.6
        let client = RosbridgeClient(transport: transport)
        let sink = EventSink()

        let streaming = expectation(description: "streaming")
        client.onEvent = { event in
            sink.handle(event)
            if sink.odometry.count >= 3, sink.flags[.odomCalibrated] == true { streaming.fulfill() }
        }
        client.connect(to: RobotEndpoint(host: "fixtures.local"))
        wait(for: [streaming], timeout: 10)

        let calibrating = expectation(description: "reports uncalibrated")
        client.onEvent = { event in
            sink.handle(event)
            if sink.flags[.odomCalibrated] == false {
                calibrating.fulfill()
            }
        }
        transport.calibrate()
        wait(for: [calibrating], timeout: 10)

        let duringCalibration = sink.odometry.count
        Thread.sleep(forTimeInterval: 0.3)
        XCTAssertEqual(
            sink.odometry.count,
            duringCalibration,
            "no pose may be published while calibrating"
        )

        let finished = expectation(description: "calibrated again")
        client.onEvent = { event in
            sink.handle(event)
            if sink.flags[.odomCalibrated] == true, sink.odometry.count > duringCalibration {
                finished.fulfill()
            }
        }
        wait(for: [finished], timeout: 10)

        client.disconnect()
    }

    func testClientReconnectsAfterAWiFiDropAndResubscribes() {
        // The Wi-Fi-loss path: the socket goes quiet with no clean close, the
        // client must notice, back off, reconnect and re-subscribe.
        let transport = SimulatedRosbridgeTransport()
        let client = RosbridgeClient(
            transport: transport,
            configuration: .init(
                topics: [.odometry, .odomCalibrated],
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
            ("testPublishingOdometryAloneIsNotFullTracking", testPublishingOdometryAloneIsNotFullTracking),
            ("testFreshCalibratedAndTrackingIsTrustworthy", testFreshCalibratedAndTrackingIsTrustworthy),
            ("testStalenessOutranksTheFlags", testStalenessOutranksTheFlags),
            ("testUncalibratedHasNoUsablePose", testUncalibratedHasNoUsablePose),
            ("testMissingFlagsReadAsUnknownRatherThanGood", testMissingFlagsReadAsUnknownRatherThanGood),
            ("testNoOdometryYetButUncalibratedReportsTheRealCause", testNoOdometryYetButUncalibratedReportsTheRealCause),
            ("testOdometryAloneProvesTheNodeIsRunning", testOdometryAloneProvesTheNodeIsRunning),
            ("testNothingOnAnyOdomTopicMeansTheNodeIsNotRunning", testNothingOnAnyOdomTopicMeansTheNodeIsNotRunning),
            ("testAStoppedGraphOutranksTopicSilence", testAStoppedGraphOutranksTopicSilence),
            ("testNoLinkMeansUnknownRatherThanNotRunning", testNoLinkMeansUnknownRatherThanNotRunning),
            ("testCalibratingOutranksTheOdometryGapItCauses", testCalibratingOutranksTheOdometryGapItCauses),
            ("testANodeThatStopsPublishingReadsAsSilentWithItsAge", testANodeThatStopsPublishingReadsAsSilentWithItsAge),
            ("testUnknownGraphStateFallsThroughToTheTopics", testUnknownGraphStateFallsThroughToTheTopics),
            ("testTheTallestFrameWins", testTheTallestFrameWins),
            ("testEqualAspectRatiosKeepARKitsOrdering", testEqualAspectRatiosKeepARKitsOrdering),
            ("testNoFormatsYieldsNoChoice", testNoFormatsYieldsNoChoice),
            ("testBackoffGrowsAndThenLevelsOff", testBackoffGrowsAndThenLevelsOff),
            ("testBackoffIsMonotonic", testBackoffIsMonotonic),
            ("testJitterStaysWithinItsBandAndNeverGoesNegative", testJitterStaysWithinItsBandAndNeverGoesNegative),
            ("testZeroJitterIsExactlyTheBaseDelay", testZeroJitterIsExactlyTheBaseDelay),
            ("testPolicyClampsNonsensicalConfiguration", testPolicyClampsNonsensicalConfiguration),
            ("testClientStreamsEveryTopicFromTheFixtureTransport", testClientStreamsEveryTopicFromTheFixtureTransport),
            ("testStopSilencesTheFixtureGraphAndStartBringsItBack", testStopSilencesTheFixtureGraphAndStartBringsItBack),
            ("testCalibrateOnFixturesWithholdsOdometryUntilItCompletes", testCalibrateOnFixturesWithholdsOdometryUntilItCompletes),
            ("testClientReconnectsAfterAWiFiDropAndResubscribes", testClientReconnectsAfterAWiFiDropAndResubscribes),
            ("testDisconnectStopsTheStreamAndDoesNotReconnect", testDisconnectStopsTheStreamAndDoesNotReconnect),
            ("testHealthSnapshotsReportRatesForSubscribedTopics", testHealthSnapshotsReportRatesForSubscribedTopics),
            ("testSimulatorHonoursTheRequestedThrottle", testSimulatorHonoursTheRequestedThrottle),
            ("testSimulatedPoseIsContinuous", testSimulatedPoseIsContinuous),
        ]
    }
}
