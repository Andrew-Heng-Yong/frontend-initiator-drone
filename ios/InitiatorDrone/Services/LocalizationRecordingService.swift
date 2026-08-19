import Combine
import Foundation

/// Drives the CSV log: starts and stops it, decides what gets a row, and
/// exposes enough state for a button to describe itself.
///
/// ## What it records, and why both
///
/// Every accepted tag fix, and the odometry estimate sampled on a timer. Both
/// are the robot's pose **in the AR world frame**, which is the only way they
/// can be compared — the tag pose starts out relative to the phone, and logging
/// it that way would produce a file measuring how the operator walked around.
///
/// The tag rows are absolute; the odometry rows are dead reckoning through
/// whatever alignment is in force. Differencing them at a shared instant is the
/// drift, which is the number this log exists to produce.
///
/// Tag rows also carry the odometry reading from the same moment, so a tag row
/// can be differenced against odometry without interpolating between the
/// odometry rows on either side of it.
@MainActor
public final class LocalizationRecordingService: ObservableObject {

    @Published public private(set) var isRecording = false
    @Published public private(set) var rowCount = 0
    @Published public private(set) var tagRowCount = 0
    @Published public private(set) var elapsedSeconds: Double = 0
    @Published public private(set) var currentFilename: String?
    @Published public private(set) var lastError: String?
    /// Finished files, newest first.
    @Published public private(set) var recordings: [URL] = []

    /// How often an odometry row is written while recording.
    ///
    /// Not every odometry message: `/odom` runs at IMU rate, which would be
    /// tens of thousands of near-identical rows for a pose that, with a
    /// gyro-only estimator, only rotates. Five a second is plenty to see drift
    /// and keeps a half-hour session comfortably openable in a spreadsheet.
    public var odometrySampleInterval: TimeInterval = 0.2

    private let recorder = LocalizationRecorder()
    private var timer: Timer?
    private var lastOdometrySampleAt: Date?

    /// Where recordings live. `Documents` so they appear in the Files app under
    /// the app's own folder, which is what makes them shareable off the phone
    /// without a cable.
    public static var directory: URL {
        let documents = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        return documents.appendingPathComponent("Localization", isDirectory: true)
    }

    public init() {
        refreshRecordings()
    }

    public func refreshRecordings() {
        recordings = LocalizationRecorder.existingRecordings(in: Self.directory)
    }

    // MARK: - Lifecycle

    public func start() {
        guard !isRecording else { return }
        do {
            let url = try recorder.start(in: Self.directory)
            currentFilename = url.lastPathComponent
            isRecording = true
            rowCount = 0
            tagRowCount = 0
            elapsedSeconds = 0
            lastError = nil
            lastOdometrySampleAt = nil
            startTimer()
        } catch {
            lastError = (error as? LocalizedError)?.errorDescription ?? error.localizedDescription
        }
    }

    public func stop() {
        guard isRecording else { return }
        timer?.invalidate()
        timer = nil
        _ = recorder.stop()
        isRecording = false
        lastOdometrySampleAt = nil
        refreshRecordings()
    }

    /// Writes out whatever is buffered without ending the session, so a file is
    /// usable if the app is backgrounded or killed before Stop is pressed.
    public func flush() {
        guard isRecording else { return }
        recorder.flush()
    }

    private func startTimer() {
        timer?.invalidate()
        let created = Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in
                guard let self, self.isRecording else { return }
                self.elapsedSeconds = self.recorder.elapsedSeconds
                self.rowCount = self.recorder.recordedRowCount
            }
        }
        timer = created
    }

    public func deleteRecording(at url: URL) {
        try? FileManager.default.removeItem(at: url)
        refreshRecordings()
    }

    public func deleteAllRecordings() {
        for url in recordings { try? FileManager.default.removeItem(at: url) }
        refreshRecordings()
    }

    // MARK: - Recording

    /// Records an accepted tag fix. Always written — a fix is rare and is the
    /// reference the whole file is built around, so it is never rate limited.
    public func recordTagFix(
        robotPoseInWorld: Pose,
        tagID: Int,
        range: Double,
        decisionMargin: Double? = nil,
        reprojectionError: Double? = nil,
        odometryPose: Pose?,
        odometryStamp: Double?,
        phonePoseInWorld: Pose,
        alignment: RobotAlignment?
    ) {
        guard isRecording else { return }
        recorder.record(LocalizationSample(
            source: .aprilTag,
            robotPoseInWorld: robotPoseInWorld,
            tagID: tagID,
            tagRange: range,
            tagDecisionMargin: decisionMargin,
            tagReprojectionError: reprojectionError,
            odometryPoseInOdom: odometryPose,
            odometryStamp: odometryStamp,
            phonePoseInWorld: phonePoseInWorld,
            alignment: alignment
        ))
        tagRowCount += 1
        rowCount = recorder.recordedRowCount
    }

    /// Records the odometry estimate, rate limited to `odometrySampleInterval`.
    public func recordOdometry(
        robotPoseInWorld: Pose,
        odometryPose: Pose,
        odometryStamp: Double?,
        phonePoseInWorld: Pose?,
        alignment: RobotAlignment?,
        now: Date = Date()
    ) {
        guard isRecording else { return }
        if let last = lastOdometrySampleAt, now.timeIntervalSince(last) < odometrySampleInterval {
            return
        }
        lastOdometrySampleAt = now
        recorder.record(LocalizationSample(
            source: .odometry,
            robotPoseInWorld: robotPoseInWorld,
            odometryPoseInOdom: odometryPose,
            odometryStamp: odometryStamp,
            phonePoseInWorld: phonePoseInWorld,
            alignment: alignment
        ), now: now)
        rowCount = recorder.recordedRowCount
    }

    public var summary: String {
        guard isRecording else {
            return recordings.isEmpty ? "No recordings yet." : "\(recordings.count) saved."
        }
        return String(
            format: "%@ · %d rows (%d tag) · %.0f s",
            currentFilename ?? "recording", rowCount, tagRowCount, elapsedSeconds
        )
    }
}
