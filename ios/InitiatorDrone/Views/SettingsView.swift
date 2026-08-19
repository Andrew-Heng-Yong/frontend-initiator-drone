import SwiftUI

/// Depth colour map, stream throttling and pose handling.
struct SettingsView: View {
    @EnvironmentObject private var settings: SettingsStore
    @EnvironmentObject private var recording: LocalizationRecordingService

    var body: some View {
        NavigationStack {
            Form {
                depthSection
                streamSection
                poseSection
                sceneSection
                pointCloudSection
                cameraMountSection
                aprilTagSection
                tagLocalizationSection
                recordingSection
                fixturesSection
                aboutSection
            }
            .navigationTitle("Settings")
        }
    }

    // MARK: - Depth

    private var depthSection: some View {
        Section {
            Picker("Colour map", selection: binding(\.depthColorMap.style)) {
                ForEach(ColorRampStyle.allCases) { Text($0.displayName).tag($0) }
            }

            Picker("Range", selection: binding(\.depthColorMap.rangeMode)) {
                Text("Fixed").tag(ScalarRangeMode.fixed)
                Text("Auto").tag(ScalarRangeMode.autoPerFrame)
                Text("Auto (smoothed)").tag(ScalarRangeMode.autoSmoothed)
            }

            if settings.settings.depthColorMap.rangeMode == .fixed {
                rangeRow(
                    title: "Near",
                    value: binding(\.depthColorMap.low),
                    range: 0.05...5.0,
                    step: 0.05,
                    unit: "m"
                )
                rangeRow(
                    title: "Far",
                    value: binding(\.depthColorMap.high),
                    range: 0.5...20.0,
                    step: 0.1,
                    unit: "m"
                )
            }

            Toggle("Near is the warm end", isOn: binding(\.depthColorMap.reversed))

            ColorLegend(
                style: settings.settings.depthColorMap.style,
                low: settings.settings.depthColorMap.low,
                high: settings.settings.depthColorMap.high,
                unit: .metres,
                reversed: settings.settings.depthColorMap.reversed
            )
        } header: {
            Text("Depth colour map")
        }
    }

    // MARK: - Stream

    private var streamSection: some View {
        Section {
            Picker("Wire format", selection: binding(\.compression)) {
                ForEach(RosbridgeCompression.allCases) { Text($0.displayName).tag($0) }
            }

            VStack(alignment: .leading) {
                HStack {
                    Text("Image throttle")
                    Spacer()
                    Text(throttleLabel)
                        .monospacedDigit()
                        .foregroundStyle(.secondary)
                }
                Slider(
                    value: Binding(
                        get: { Double(settings.settings.imageThrottleMilliseconds) },
                        set: { settings.settings.imageThrottleMilliseconds = Int($0) }
                    ),
                    in: 0...500,
                    step: 10
                )
            }
        } header: {
            Text("Image stream")
        }
    }

    // MARK: - Pose

    private var poseSection: some View {
        Section {
            rangeRow(
                title: "Stale after",
                value: binding(\.odometryStalenessThreshold),
                range: 0.1...5.0,
                step: 0.1,
                unit: "s"
            )
            rangeRow(
                title: "Extrapolate up to",
                value: binding(\.odometryExtrapolationLimit),
                range: 0.0...0.5,
                step: 0.01,
                unit: "s",
                format: "%.2f"
            )
        } header: {
            Text("Odometry")
        }
    }

    private var sceneSection: some View {
        Section("AR scene") {
            Toggle("Show camera frustum", isOn: binding(\.showsCameraFrustum))
            Toggle("Show robot trail", isOn: binding(\.showsRobotTrail))
            Text("The frustum uses the depth CameraInfo field of view, drawn from the camera mount set below.")
                .font(.caption)
                .foregroundStyle(.secondary)
        }
    }

    /// Where the depth camera is bolted to the robot.
    ///
    /// Offsets are metres in `base_link` axes and angles are degrees, because
    /// those are the units of the tape measure and protractor someone will
    /// actually use on the airframe. Everything converts to radians behind
    /// `CameraExtrinsics`.
    private var cameraMountSection: some View {
        Section {
            offsetRow(
                title: "Forward",
                value: binding(\.cameraExtrinsics.x),
                help: "+X, ahead of base_link"
            )
            offsetRow(
                title: "Left",
                value: binding(\.cameraExtrinsics.y),
                help: "+Y, to the robot's left"
            )
            offsetRow(
                title: "Up",
                value: binding(\.cameraExtrinsics.z),
                help: "+Z, above base_link"
            )
            angleRow(
                title: "Pitch",
                value: binding(\.cameraExtrinsics.pitchDegrees),
                help: "positive tilts the camera down"
            )
            angleRow(
                title: "Roll",
                value: binding(\.cameraExtrinsics.rollDegrees),
                help: "positive drops the right side"
            )

            if !settings.settings.cameraExtrinsics.isIdentity {
                Button("Reset mount to base_link") {
                    settings.settings.cameraExtrinsics = .identity
                }
            }
        } header: {
            Text("Offsets · Camera mount")
        }
    }

    private var pointCloudSection: some View {
        Section {
            Toggle("Show point cloud", isOn: binding(\.pointCloud.isEnabled))

            if settings.settings.pointCloud.isEnabled {
                Stepper(
                    "Sample every \(settings.settings.pointCloud.pixelStride) px",
                    value: binding(\.pointCloud.pixelStride),
                    in: 1...8
                )
                Stepper(
                    "Max \(settings.settings.pointCloud.maximumPoints / 1000)k points",
                    value: Binding(
                        get: { settings.settings.pointCloud.maximumPoints / 1000 },
                        set: { settings.settings.pointCloud.maximumPoints = $0 * 1000 }
                    ),
                    in: 1...100
                )
                rangeRow(
                    title: "Nearest",
                    value: binding(\.pointCloud.minimumDepth),
                    range: 0.05...3.0,
                    step: 0.05,
                    unit: "m",
                    format: "%.2f"
                )
                rangeRow(
                    title: "Furthest",
                    value: binding(\.pointCloud.maximumDepth),
                    range: 0.5...20.0,
                    step: 0.5,
                    unit: "m",
                    format: "%.1f"
                )
                rangeRow(
                    title: "Point size",
                    value: binding(\.pointCloud.pointSize),
                    range: 1.0...20.0,
                    step: 1.0,
                    unit: "px",
                    format: "%.0f"
                )
            }
        } header: {
            Text("Point cloud")
        }
    }

    private var fixturesSection: some View {
        Section {
            Toggle("Hold the robot still", isOn: binding(\.fixtureRobotIsStatic))
        } header: {
            Text("Fixtures mode")
        }
    }

    private var aboutSection: some View {
        Section {
            LabeledContent("Depth topic", value: RobotTopic.depthImage.topicName)
            LabeledContent("Odometry topic", value: RobotTopic.odometry.topicName)
            Button(role: .destructive) {
                settings.resetToDefaults()
            } label: {
                Label("Reset settings", systemImage: "arrow.counterclockwise")
            }
        } header: {
            Text("About")
        }
        .font(.footnote)
    }

    // MARK: - AprilTags

    private var aprilTagSection: some View {
        Section {
            ForEach($settings.settings.aprilTags) { $mount in
                AprilTagMountRow(mount: $mount)
            }
            .onDelete { offsets in
                settings.settings.aprilTags.remove(atOffsets: offsets)
            }

            Button {
                settings.settings.aprilTags.append(nextTagMount())
            } label: {
                Label("Add tag", systemImage: "plus.circle")
            }

            if settings.settings.aprilTags.isEmpty {
                Text("No tags configured. Tag relocalisation does nothing until at least one is added.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        } header: {
            Text("Offsets · AprilTags")
        }
    }

    /// Picks an ID not already in use, so adding several tags in a row does not
    /// silently create duplicates that shadow one another.
    private func nextTagMount() -> AprilTagMount {
        let used = Set(settings.settings.aprilTags.map(\.tagID))
        let free = AprilTagFamily.validIDs.first { !used.contains($0) } ?? 0
        return AprilTagMount(tagID: free, sizeMetres: 0.10)
    }

    private var tagLocalizationSection: some View {
        Section {
            Toggle("Relocalise from tags", isOn: binding(\.tagLocalization.isEnabled))

            if settings.settings.tagLocalization.isEnabled {
                rangeRow(
                    title: "Maximum range",
                    value: binding(\.tagLocalization.maximumRange),
                    range: 0.5...10.0, step: 0.5, unit: "m", format: "%.1f"
                )
                Stepper(
                    "Confirm over \(settings.settings.tagLocalization.requiredConsecutiveSightings) frames",
                    value: binding(\.tagLocalization.requiredConsecutiveSightings),
                    in: 1...10
                )
                rangeRow(
                    title: "Largest correction",
                    value: binding(\.tagLocalization.maximumCorrection),
                    range: 0.0...10.0, step: 0.5, unit: "m", format: "%.1f"
                )
            }
        } header: {
            Text("Tag relocalisation")
        }
    }

    // MARK: - Recording

    private var recordingSection: some View {
        Section {
            Button {
                if recording.isRecording { recording.stop() } else { recording.start() }
            } label: {
                Label(
                    recording.isRecording ? "Stop recording" : "Record localisation",
                    systemImage: recording.isRecording ? "stop.circle.fill" : "record.circle"
                )
                .foregroundStyle(recording.isRecording ? Color.red : Color.accentColor)
            }

            Text(recording.summary)
                .font(.caption)
                .foregroundStyle(.secondary)
                .monospacedDigit()

            if let error = recording.lastError {
                Text(error)
                    .font(.caption)
                    .foregroundStyle(Color.red)
            }

            ForEach(recording.recordings, id: \.self) { url in
                ShareLink(item: url) {
                    HStack {
                        Image(systemName: "square.and.arrow.up")
                        VStack(alignment: .leading, spacing: 1) {
                            Text(url.lastPathComponent)
                                .font(.caption)
                            Text(fileSizeLabel(url))
                                .font(.caption2)
                                .foregroundStyle(.secondary)
                        }
                    }
                }
            }
            .onDelete { offsets in
                for index in offsets { recording.deleteRecording(at: recording.recordings[index]) }
            }
        } header: {
            Text("Localisation log")
        }
    }

    private func fileSizeLabel(_ url: URL) -> String {
        let attributes = try? FileManager.default.attributesOfItem(atPath: url.path)
        let bytes = (attributes?[.size] as? NSNumber)?.intValue ?? 0
        return ByteCountFormatter.string(fromByteCount: Int64(bytes), countStyle: .file)
    }

    // MARK: - Helpers

    private var throttleLabel: String {
        let value = settings.settings.imageThrottleMilliseconds
        guard value > 0 else { return "unthrottled" }
        return String(format: "%d ms (≤ %.0f fps)", value, 1000.0 / Double(value))
    }

    private func rangeRow(
        title: String,
        value: Binding<Double>,
        range: ClosedRange<Double>,
        step: Double,
        unit: String,
        format: String = "%.2f"
    ) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            HStack {
                Text(title)
                Spacer()
                Text(String(format: "\(format) \(unit)", value.wrappedValue))
                    .monospacedDigit()
                    .foregroundStyle(.secondary)
            }
            Slider(value: value, in: range, step: step)
        }
    }

    private func offsetRow(title: String, value: Binding<Double>, help: String) -> some View {
        measurementRow(title: title, value: value, unit: "m", step: 0.01, help: help)
    }

    private func angleRow(title: String, value: Binding<Double>, help: String) -> some View {
        measurementRow(title: title, value: value, unit: "°", step: 1.0, help: help)
    }

    /// A typed measurement rather than a slider.
    ///
    /// These are numbers someone reads off a tape measure or a protractor, so
    /// the field has to accept `0.085` exactly. A slider cannot, and rounding a
    /// measured offset to the nearest slider step is how a mount ends up a
    /// centimetre out for no reason anyone can see. The stepper is there for
    /// nudging once the measured value is in.
    private func measurementRow(
        title: String,
        value: Binding<Double>,
        unit: String,
        step: Double,
        help: String
    ) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            HStack {
                Text(title)
                Spacer(minLength: 12)
                TextField(title, value: value, format: .number.precision(.fractionLength(0...3)))
                    .keyboardType(.numbersAndPunctuation)  // .decimalPad has no minus sign
                    .multilineTextAlignment(.trailing)
                    .monospacedDigit()
                    .frame(maxWidth: 90)
                Text(unit)
                    .foregroundStyle(.secondary)
                Stepper(title, value: value, step: step)
                    .labelsHidden()
            }
            Text(help)
                .font(.caption2)
                .foregroundStyle(.secondary)
        }
    }

    private func binding<T>(_ keyPath: WritableKeyPath<AppSettings, T>) -> Binding<T> {
        Binding(
            get: { settings.settings[keyPath: keyPath] },
            set: { settings.settings[keyPath: keyPath] = $0 }
        )
    }
}
