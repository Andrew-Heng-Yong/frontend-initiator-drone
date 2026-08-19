import Foundation

/// Which estimator produced a row.
public enum LocalizationSource: String, Codable, Sendable, CaseIterable {
    /// A tag sighting, converted to a robot pose in the AR world frame.
    case aprilTag = "apriltag"
    /// The robot's own odometry, put through the alignment in force.
    case odometry = "odom"
}

/// One row of the log: a robot pose in the world, and where it came from.
///
/// Both sources are written in the **AR world frame**, not the phone's. That is
/// the only way the two are comparable, and comparing them is the entire point
/// of recording: the tag rows are the absolute reference, the odometry rows are
/// dead reckoning, and the gap between them at a shared instant is the drift.
public struct LocalizationSample: Equatable, Sendable {
    public var sequence: Int
    /// Seconds since recording started.
    public var sessionSeconds: Double
    /// Wall clock, for lining the log up against a rosbag.
    public var wallTime: Date
    public var source: LocalizationSource

    /// Robot `base_link` in the ARKit world frame.
    public var robotPoseInWorld: Pose

    /// Tag columns, filled on `aprilTag` rows.
    public var tagID: Int?
    public var tagRange: Double?
    public var tagDecisionMargin: Double?
    public var tagReprojectionError: Double?

    /// The raw `/odom` pose in the ROS `odom` frame at this instant, whatever
    /// the source. On a tag row this is what odometry was claiming at the moment
    /// the tag disagreed with it, which is what makes a tag row directly
    /// comparable to the odometry rows around it.
    public var odometryPoseInOdom: Pose?
    public var odometryStamp: Double?

    /// The phone's own pose. A tag result is only "in the world" because this
    /// was composed in; logging it makes that checkable afterwards rather than
    /// something to take on trust.
    public var phonePoseInWorld: Pose?

    /// The alignment in force, which is what maps odometry into the world.
    public var alignment: RobotAlignment?

    public init(
        sequence: Int = 0,
        sessionSeconds: Double = 0,
        wallTime: Date = Date(),
        source: LocalizationSource,
        robotPoseInWorld: Pose,
        tagID: Int? = nil,
        tagRange: Double? = nil,
        tagDecisionMargin: Double? = nil,
        tagReprojectionError: Double? = nil,
        odometryPoseInOdom: Pose? = nil,
        odometryStamp: Double? = nil,
        phonePoseInWorld: Pose? = nil,
        alignment: RobotAlignment? = nil
    ) {
        self.sequence = sequence
        self.sessionSeconds = sessionSeconds
        self.wallTime = wallTime
        self.source = source
        self.robotPoseInWorld = robotPoseInWorld
        self.tagID = tagID
        self.tagRange = tagRange
        self.tagDecisionMargin = tagDecisionMargin
        self.tagReprojectionError = tagReprojectionError
        self.odometryPoseInOdom = odometryPoseInOdom
        self.odometryStamp = odometryStamp
        self.phonePoseInWorld = phonePoseInWorld
        self.alignment = alignment
    }
}

/// Formats samples as CSV.
///
/// Numbers go through `String(format:)` with no locale, which is POSIX, so the
/// decimal separator is always a full stop. Using a locale-aware formatter would
/// write `1,25` on a French phone and silently shift every column one to the
/// right when the file was opened.
public enum LocalizationCSV {

    public static let columns = [
        "sequence", "wall_time_iso", "session_seconds", "source",
        "robot_x", "robot_y", "robot_z",
        "robot_qx", "robot_qy", "robot_qz", "robot_qw", "robot_yaw_deg",
        "tag_id", "tag_range_m", "tag_decision_margin", "tag_reprojection_px",
        "odom_x", "odom_y", "odom_z", "odom_yaw_deg", "odom_stamp",
        "phone_x", "phone_y", "phone_z", "phone_yaw_deg",
        "align_x", "align_y", "align_z", "align_yaw_deg",
    ]

    public static var header: String { columns.joined(separator: ",") }

    private static let timestampFormatter: ISO8601DateFormatter = {
        let formatter = ISO8601DateFormatter()
        formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
        return formatter
    }()

    /// Blank rather than `0` for an absent value, so a reader can tell "no tag
    /// on this row" from "a tag exactly at the origin". Pandas reads an empty
    /// field as `NaN`, which is what it is.
    private static func number(_ value: Double?, _ places: Int = 6) -> String {
        guard let value, value.isFinite else { return "" }
        return String(format: "%.\(places)f", value)
    }

    private static func degrees(_ radians: Double?) -> String {
        guard let radians, radians.isFinite else { return "" }
        return String(format: "%.3f", radians * 180 / .pi)
    }

    public static func row(_ sample: LocalizationSample) -> String {
        let robot = sample.robotPoseInWorld
        var fields: [String] = [
            String(sample.sequence),
            timestampFormatter.string(from: sample.wallTime),
            number(sample.sessionSeconds, 3),
            sample.source.rawValue,
            number(robot.position.x), number(robot.position.y), number(robot.position.z),
            number(robot.orientation.x), number(robot.orientation.y),
            number(robot.orientation.z), number(robot.orientation.w),
            degrees(robot.orientation.yawAroundY),
            sample.tagID.map(String.init) ?? "",
            number(sample.tagRange, 4),
            number(sample.tagDecisionMargin, 2),
            number(sample.tagReprojectionError, 3),
        ]

        // Odometry is in the ROS `odom` frame, so its heading is about ROS +Z,
        // not the AR up axis the other columns use.
        if let odometry = sample.odometryPoseInOdom {
            fields += [
                number(odometry.position.x), number(odometry.position.y), number(odometry.position.z),
                degrees(odometry.orientation.yawAroundZ),
            ]
        } else {
            fields += ["", "", "", ""]
        }
        fields.append(number(sample.odometryStamp, 6))

        if let phone = sample.phonePoseInWorld {
            fields += [
                number(phone.position.x), number(phone.position.y), number(phone.position.z),
                degrees(phone.orientation.yawAroundY),
            ]
        } else {
            fields += ["", "", "", ""]
        }

        if let alignment = sample.alignment {
            fields += [
                number(alignment.originInAR.x), number(alignment.originInAR.y),
                number(alignment.originInAR.z), degrees(alignment.yaw),
            ]
        } else {
            fields += ["", "", "", ""]
        }
        return fields.joined(separator: ",")
    }
}

/// Appends localization samples to a CSV file on disk.
///
/// Thread-safe, and buffered on purpose. A half-hour session at odometry rate is
/// tens of thousands of rows; a write syscall each would put file I/O on the
/// path that also feeds the render loop. Rows accumulate in memory and are
/// flushed in batches, and the buffer is bounded so a disk that stops accepting
/// writes costs a fixed amount of memory rather than growing until the app is
/// killed.
public final class LocalizationRecorder: @unchecked Sendable {

    public enum RecorderError: Error, LocalizedError {
        case alreadyRecording
        case notRecording
        case couldNotCreateFile(String)

        public var errorDescription: String? {
            switch self {
            case .alreadyRecording: return "A recording is already running."
            case .notRecording: return "No recording is running."
            case .couldNotCreateFile(let path): return "Could not create \(path)."
            }
        }
    }

    private let lock = NSLock()
    private var handle: FileHandle?
    private var url: URL?
    private var buffer: [String] = []
    private var sequence = 0
    private var startedAt: Date?
    private var droppedRows = 0

    /// Rows held before a flush.
    private let flushThreshold: Int
    /// Hard ceiling on buffered rows, so a file that stops accepting writes
    /// costs a fixed amount of memory instead of growing without limit.
    ///
    /// Never below `flushThreshold`: a ceiling under the threshold would drop
    /// rows before a flush could ever be triggered, so the file would stay
    /// empty while the recorder reported it was recording. While writes are
    /// succeeding this ceiling is unreachable, because every flush empties the
    /// buffer completely.
    private let maximumBufferedRows: Int

    public init(flushThreshold: Int = 64, maximumBufferedRows: Int = 20_000) {
        let threshold = max(1, flushThreshold)
        self.flushThreshold = threshold
        self.maximumBufferedRows = max(threshold, maximumBufferedRows)
    }

    public var isRecording: Bool { lock.withLock { handle != nil } }
    public var currentURL: URL? { lock.withLock { url } }
    public var recordedRowCount: Int { lock.withLock { sequence } }
    public var droppedRowCount: Int { lock.withLock { droppedRows } }

    public var elapsedSeconds: Double {
        lock.withLock { startedAt.map { Date().timeIntervalSince($0) } ?? 0 }
    }

    /// A filename that sorts chronologically and needs no disambiguation.
    public static func suggestedFilename(at date: Date = Date()) -> String {
        let formatter = DateFormatter()
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.dateFormat = "yyyy-MM-dd_HHmmss"
        return "localization_\(formatter.string(from: date)).csv"
    }

    @discardableResult
    public func start(in directory: URL, filename: String? = nil, now: Date = Date()) throws -> URL {
        lock.lock()
        defer { lock.unlock() }
        guard handle == nil else { throw RecorderError.alreadyRecording }

        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        let target = directory.appendingPathComponent(filename ?? Self.suggestedFilename(at: now))
        let header = Data((LocalizationCSV.header + "\n").utf8)
        guard FileManager.default.createFile(atPath: target.path, contents: header) else {
            throw RecorderError.couldNotCreateFile(target.path)
        }
        guard let opened = try? FileHandle(forWritingTo: target) else {
            throw RecorderError.couldNotCreateFile(target.path)
        }
        opened.seekToEndOfFile()

        handle = opened
        url = target
        buffer.removeAll(keepingCapacity: true)
        sequence = 0
        droppedRows = 0
        startedAt = now
        return target
    }

    /// Records a sample. Fills in the sequence number and elapsed time, so
    /// callers never have to keep a counter.
    public func record(_ sample: LocalizationSample, now: Date = Date()) {
        lock.lock()
        defer { lock.unlock() }
        guard handle != nil, let startedAt else { return }

        var stamped = sample
        stamped.sequence = sequence
        stamped.wallTime = now
        stamped.sessionSeconds = now.timeIntervalSince(startedAt)
        sequence += 1

        guard buffer.count < maximumBufferedRows else {
            droppedRows += 1
            return
        }
        buffer.append(LocalizationCSV.row(stamped))
        if buffer.count >= flushThreshold { flushLocked() }
    }

    /// Writes anything buffered without ending the recording, so a session that
    /// is interrupted — backgrounded, or the app killed — still leaves a file
    /// with everything up to the last flush.
    public func flush() {
        lock.lock()
        defer { lock.unlock() }
        flushLocked()
    }

    private func flushLocked() {
        guard let handle, !buffer.isEmpty else { return }
        let payload = buffer.joined(separator: "\n") + "\n"
        buffer.removeAll(keepingCapacity: true)
        handle.write(Data(payload.utf8))
    }

    /// Ends the recording and returns the finished file.
    @discardableResult
    public func stop() -> URL? {
        lock.lock()
        defer { lock.unlock() }
        guard handle != nil else { return nil }
        flushLocked()
        try? handle?.close()
        handle = nil
        startedAt = nil
        let finished = url
        url = nil
        return finished
    }

    /// Every recording already on disk, newest first.
    public static func existingRecordings(in directory: URL) -> [URL] {
        let contents = (try? FileManager.default.contentsOfDirectory(
            at: directory,
            includingPropertiesForKeys: [.contentModificationDateKey],
            options: [.skipsHiddenFiles]
        )) ?? []
        return contents
            .filter { $0.pathExtension.lowercased() == "csv" }
            .sorted { left, right in
                let leftDate = (try? left.resourceValues(forKeys: [.contentModificationDateKey]))?
                    .contentModificationDate ?? .distantPast
                let rightDate = (try? right.resourceValues(forKeys: [.contentModificationDateKey]))?
                    .contentModificationDate ?? .distantPast
                return leftDate > rightDate
            }
    }
}

private extension NSLock {
    func withLock<T>(_ body: () -> T) -> T {
        lock()
        defer { unlock() }
        return body()
    }
}
