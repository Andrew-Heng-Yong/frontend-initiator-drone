#if canImport(XCTest)
import XCTest
// In Xcode the tests compile as their own module; the headless runner in
// Scripts/ compiles them alongside the sources instead, so neither import
// exists there.
@testable import InitiatorDrone
#endif
import Foundation

/// The CSV log: schema, formatting, and the file lifecycle.
final class LocalizationRecorderTests: XCTestCase {

    private var directory: URL!

    override func setUp() {
        directory = FileManager.default.temporaryDirectory
            .appendingPathComponent("initiator-recorder-tests-\(UUID().uuidString)")
    }

    override func tearDown() {
        if let directory { try? FileManager.default.removeItem(at: directory) }
    }

    private func sample(
        source: LocalizationSource = .odometry,
        robot: Pose = Pose(position: Vector3(1, 2, 3))
    ) -> LocalizationSample {
        LocalizationSample(source: source, robotPoseInWorld: robot)
    }

    private func rows(of url: URL) throws -> [String] {
        try String(contentsOf: url, encoding: .utf8)
            .split(separator: "\n", omittingEmptySubsequences: true)
            .map(String.init)
    }

    // MARK: - Formatting

    func testHeaderAndRowHaveTheSameNumberOfColumns() {
        let row = LocalizationCSV.row(sample())
        XCTAssertEqual(
            row.split(separator: ",", omittingEmptySubsequences: false).count,
            LocalizationCSV.columns.count
        )
    }

    /// Absent is not zero. A tag row and an odometry row differ in which columns
    /// are filled, and writing `0` for "no tag here" would put a tag at the
    /// origin in every analysis of the file.
    func testAbsentValuesAreBlankRatherThanZero() {
        let row = LocalizationCSV.row(sample(source: .odometry))
        let fields = row.split(separator: ",", omittingEmptySubsequences: false).map(String.init)
        let tagIDIndex = LocalizationCSV.columns.firstIndex(of: "tag_id")!
        let tagRangeIndex = LocalizationCSV.columns.firstIndex(of: "tag_range_m")!
        XCTAssertEqual(fields[tagIDIndex], "")
        XCTAssertEqual(fields[tagRangeIndex], "")
    }

    func testTagRowsCarryTheirTagColumns() {
        var tagSample = sample(source: .aprilTag)
        tagSample.tagID = 12
        tagSample.tagRange = 1.25
        tagSample.tagDecisionMargin = 88
        tagSample.tagReprojectionError = 0.42

        let fields = LocalizationCSV.row(tagSample)
            .split(separator: ",", omittingEmptySubsequences: false).map(String.init)
        func field(_ name: String) -> String {
            fields[LocalizationCSV.columns.firstIndex(of: name)!]
        }
        XCTAssertEqual(field("source"), "apriltag")
        XCTAssertEqual(field("tag_id"), "12")
        XCTAssertEqual(Double(field("tag_range_m")), 1.25)
        XCTAssertEqual(Double(field("tag_reprojection_px")), 0.42)
    }

    /// The separator must be a full stop whatever the phone's region is set to,
    /// or every row silently gains a column when the file is opened.
    func testNumbersUseAFullStopRegardlessOfLocale() {
        var candidate = sample(robot: Pose(position: Vector3(1.25, -0.5, 0)))
        candidate.tagRange = 2.75
        let row = LocalizationCSV.row(candidate)
        XCTAssertTrue(row.contains("1.250000"), row)
        XCTAssertFalse(row.contains("1,250000"), row)
        XCTAssertEqual(
            row.split(separator: ",", omittingEmptySubsequences: false).count,
            LocalizationCSV.columns.count
        )
    }

    /// The two frames have different up axes, so their headings are measured
    /// about different ones. Reporting both as the same angle would make the
    /// odometry column quietly meaningless.
    func testOdometryHeadingIsMeasuredAboutTheROSUpAxis() {
        var candidate = sample()
        candidate.odometryPoseInOdom = Pose(orientation: Quaternion.aroundZ(.pi / 2))
        candidate.robotPoseInWorld = Pose(orientation: Quaternion.aroundY(.pi / 4))

        let fields = LocalizationCSV.row(candidate)
            .split(separator: ",", omittingEmptySubsequences: false).map(String.init)
        func field(_ name: String) -> Double? {
            Double(fields[LocalizationCSV.columns.firstIndex(of: name)!])
        }
        XCTAssertEqual(try XCTUnwrap(field("odom_yaw_deg")), 90, accuracy: 1e-3)
        XCTAssertEqual(try XCTUnwrap(field("robot_yaw_deg")), 45, accuracy: 1e-3)
    }

    // MARK: - File lifecycle

    func testStartWritesAHeaderAndStopReturnsTheFile() throws {
        let recorder = LocalizationRecorder()
        let url = try recorder.start(in: directory, filename: "test.csv")
        XCTAssertTrue(recorder.isRecording)

        recorder.record(sample())
        let finished = try XCTUnwrap(recorder.stop())
        XCTAssertEqual(finished, url)
        XCTAssertFalse(recorder.isRecording)

        let lines = try rows(of: finished)
        XCTAssertEqual(lines.first, LocalizationCSV.header)
        XCTAssertEqual(lines.count, 2)
    }

    func testEveryRecordedSampleReachesTheFile() throws {
        let recorder = LocalizationRecorder(flushThreshold: 8)
        let url = try recorder.start(in: directory, filename: "many.csv")
        for _ in 0..<500 { recorder.record(sample()) }
        recorder.stop()

        let lines = try rows(of: url)
        XCTAssertEqual(lines.count, 501, "header plus every row, including a partial final batch")
    }

    /// Sequence numbers are assigned by the recorder, not the caller, so a gap
    /// in the column is real evidence of a dropped row rather than a caller bug.
    func testSequenceNumbersAreAssignedAndContiguous() throws {
        let recorder = LocalizationRecorder(flushThreshold: 4)
        let url = try recorder.start(in: directory, filename: "seq.csv")
        for _ in 0..<20 { recorder.record(sample()) }
        recorder.stop()

        let lines = try rows(of: url).dropFirst()
        let sequences = lines.compactMap { Int($0.split(separator: ",")[0]) }
        XCTAssertEqual(sequences, Array(0..<20))
    }

    func testRecordingBeforeStartIsIgnoredRatherThanCrashing() {
        let recorder = LocalizationRecorder()
        recorder.record(sample())
        XCTAssertEqual(recorder.recordedRowCount, 0)
        XCTAssertNil(recorder.stop())
    }

    func testStartingTwiceIsRefused() throws {
        let recorder = LocalizationRecorder()
        _ = try recorder.start(in: directory, filename: "one.csv")
        XCTAssertThrowsError(try recorder.start(in: directory, filename: "two.csv"))
        recorder.stop()
    }

    /// A recording that is interrupted rather than stopped must still leave
    /// usable data behind.
    func testFlushMakesRowsReadableWithoutStopping() throws {
        let recorder = LocalizationRecorder(flushThreshold: 1000)
        let url = try recorder.start(in: directory, filename: "flush.csv")
        for _ in 0..<10 { recorder.record(sample()) }
        XCTAssertEqual(try rows(of: url).count, 1, "nothing but the header should have reached disk yet")

        recorder.flush()
        XCTAssertEqual(try rows(of: url).count, 11)
        recorder.stop()
    }

    /// Elapsed time is derived from the start, so the column is monotonic even
    /// though samples arrive from several sources at different rates.
    func testSessionSecondsAreMeasuredFromTheStart() throws {
        let recorder = LocalizationRecorder(flushThreshold: 1)
        let start = Date()
        let url = try recorder.start(in: directory, filename: "time.csv", now: start)
        recorder.record(sample(), now: start.addingTimeInterval(2.5))
        recorder.record(sample(), now: start.addingTimeInterval(4.0))
        recorder.stop()

        let lines = try rows(of: url).dropFirst()
        let index = LocalizationCSV.columns.firstIndex(of: "session_seconds")!
        let seconds = lines.map { Double($0.split(separator: ",", omittingEmptySubsequences: false)[index])! }
        XCTAssertEqual(seconds, [2.5, 4.0])
    }

    /// The memory ceiling must never be able to starve the file.
    ///
    /// A ceiling below the flush threshold would drop rows before a flush could
    /// ever be triggered, leaving an empty file while the recorder cheerfully
    /// reported it was recording. The constructor raises the ceiling to meet
    /// the threshold, so while writes succeed nothing is ever dropped.
    func testAnUndersizedBufferCeilingCannotStarveTheFile() throws {
        let recorder = LocalizationRecorder(flushThreshold: 100, maximumBufferedRows: 10)
        let url = try recorder.start(in: directory, filename: "bounded.csv")
        for _ in 0..<250 { recorder.record(sample()) }
        recorder.stop()

        XCTAssertEqual(recorder.droppedRowCount, 0)
        XCTAssertEqual(try rows(of: url).count, 251)
    }

    func testExistingRecordingsAreListedNewestFirst() throws {
        let recorder = LocalizationRecorder()
        for name in ["a.csv", "b.csv", "c.csv"] {
            _ = try recorder.start(in: directory, filename: name)
            recorder.record(sample())
            recorder.stop()
            Thread.sleep(forTimeInterval: 0.02)
        }
        let listed = LocalizationRecorder.existingRecordings(in: directory).map(\.lastPathComponent)
        XCTAssertEqual(listed, ["c.csv", "b.csv", "a.csv"])
    }

    func testSuggestedFilenamesSortChronologically() {
        let earlier = LocalizationRecorder.suggestedFilename(at: Date(timeIntervalSince1970: 1_700_000_000))
        let later = LocalizationRecorder.suggestedFilename(at: Date(timeIntervalSince1970: 1_700_003_600))
        XCTAssertTrue(earlier.hasSuffix(".csv"))
        XCTAssertLessThan(earlier, later)
    }

    static var allTests: [(String, (LocalizationRecorderTests) -> () throws -> Void)] {
        [
        ("testHeaderAndRowHaveTheSameNumberOfColumns", testHeaderAndRowHaveTheSameNumberOfColumns),
        ("testAbsentValuesAreBlankRatherThanZero", testAbsentValuesAreBlankRatherThanZero),
        ("testTagRowsCarryTheirTagColumns", testTagRowsCarryTheirTagColumns),
        ("testNumbersUseAFullStopRegardlessOfLocale", testNumbersUseAFullStopRegardlessOfLocale),
        ("testOdometryHeadingIsMeasuredAboutTheROSUpAxis", testOdometryHeadingIsMeasuredAboutTheROSUpAxis),
        ("testStartWritesAHeaderAndStopReturnsTheFile", testStartWritesAHeaderAndStopReturnsTheFile),
        ("testEveryRecordedSampleReachesTheFile", testEveryRecordedSampleReachesTheFile),
        ("testSequenceNumbersAreAssignedAndContiguous", testSequenceNumbersAreAssignedAndContiguous),
        ("testRecordingBeforeStartIsIgnoredRatherThanCrashing", testRecordingBeforeStartIsIgnoredRatherThanCrashing),
        ("testStartingTwiceIsRefused", testStartingTwiceIsRefused),
        ("testFlushMakesRowsReadableWithoutStopping", testFlushMakesRowsReadableWithoutStopping),
        ("testSessionSecondsAreMeasuredFromTheStart", testSessionSecondsAreMeasuredFromTheStart),
        ("testAnUndersizedBufferCeilingCannotStarveTheFile", testAnUndersizedBufferCeilingCannotStarveTheFile),
        ("testExistingRecordingsAreListedNewestFirst", testExistingRecordingsAreListedNewestFirst),
        ("testSuggestedFilenamesSortChronologically", testSuggestedFilenamesSortChronologically),
        ]
    }
}
