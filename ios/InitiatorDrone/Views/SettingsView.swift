import SwiftUI

/// Depth colour map, stream throttling and pose handling.
struct SettingsView: View {
    @EnvironmentObject private var settings: SettingsStore

    var body: some View {
        NavigationStack {
            Form {
                depthSection
                streamSection
                poseSection
                sceneSection
                pointCloudSection
                cameraMountSection
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
        } footer: {
            Text("""
            16UC1 and mono16 depth frames are read as millimetres and 32FC1 as metres, following ROS \
            convention. Samples of zero mean "no return" and are drawn transparent rather than as a \
            surface at the lens.
            """)
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
        } footer: {
            Text("""
            The throttle is enforced on the robot, so frames above the limit are never sent at all — \
            the cheapest possible way to keep a backlog from forming. CBOR sends image bytes raw \
            instead of base64 and needs rosbridge_suite 0.11 or newer; switch to JSON if frames stop \
            arriving after changing it.
            """)
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
        } footer: {
            Text("""
            Poses are interpolated between buffered samples at the exact instant each frame is drawn. \
            Past the newest sample the marker holds still by default; allowing extrapolation \
            integrates the reported twist forward instead, which looks smoother but invents motion \
            that was never measured.
            """)
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
            Text("Camera mount")
        } footer: {
            Text("""
            Measured from base_link to the depth camera. Both the point cloud and the frustum use \
            it, so a wrong value tilts the whole cloud rather than shifting it slightly — a 15° \
            pitch error lifts a wall 2 m away by about half a metre. Yaw is deliberately absent: a \
            camera rotated about the vertical looks exactly like a robot pointing elsewhere, and \
            entering it here would hide real heading errors.
            """)
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
        } footer: {
            Text("""
            The depth frame is deprojected with the CameraInfo intrinsics and drawn at the robot's \
            pose, so it needs both a depth stream and an alignment before anything appears. Where \
            it lands relative to the robot comes from the camera mount below.
            """)
        }
    }

    private var fixturesSection: some View {
        Section {
            Toggle("Hold the robot still", isOn: binding(\.fixtureRobotIsStatic))
        } header: {
            Text("Fixtures mode")
        } footer: {
            Text("""
            The fixture robot turns slowly on the spot; this stops it. Position is always zero \
            either way, because that is all odom_node reports. Odometry keeps publishing at the \
            same rate regardless — only the pose stops changing. Hold it still when you are \
            checking whether the marker and point cloud land in the right place, because a turning \
            robot makes a yaw alignment error look like motion.
            """)
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
        } footer: {
            Text("Visualisation and diagnostics only. This app sends no flight-control commands.")
        }
        .font(.footnote)
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
