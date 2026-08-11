import SwiftUI

/// Depth colour map, stream throttling, pose handling and the AR camera.
struct SettingsView: View {
    @EnvironmentObject private var settings: SettingsStore
    #if canImport(ARKit)
    @EnvironmentObject private var arSession: ARSessionController
    #endif

    var body: some View {
        NavigationStack {
            Form {
                depthSection
                streamSection
                poseSection
                cameraSection
                sceneSection
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

    // MARK: - Camera

    private var cameraSection: some View {
        Section {
            Toggle("Use wide-angle lens", isOn: binding(\.prefersUltraWideCamera))

            #if canImport(ARKit)
            LabeledContent("Active lens", value: arSession.lensLabel)
            if !arSession.videoFormatLabel.isEmpty {
                LabeledContent("Video format", value: arSession.videoFormatLabel)
                    .font(.caption)
            }
            // Said here rather than only in the footer, so the toggle having no
            // visible effect is explained before it is tried.
            if !arSession.supportsUltraWide {
                Label(
                    "This device offers no ultra-wide format to ARKit.",
                    systemImage: "info.circle"
                )
                .font(.caption)
                .foregroundStyle(.secondary)
            }
            #endif
        } header: {
            Text("AR camera")
        } footer: {
            Text("""
            The ultra-wide lens roughly doubles the field of view, so the robot marker stays on \
            screen from much closer and while the phone is being moved around. It is used only on \
            devices whose ARKit configuration offers an ultra-wide video format; everywhere else \
            the app falls back to the standard lens and this toggle has no effect. Changing it \
            restarts AR tracking, which clears the robot alignment.
            """)
        }
    }

    private var sceneSection: some View {
        Section("AR scene") {
            Toggle("Show camera frustum", isOn: binding(\.showsCameraFrustum))
            Toggle("Show robot trail", isOn: binding(\.showsRobotTrail))
            Text("The frustum uses the depth CameraInfo field of view, drawn from base_link. It shows coverage, not a calibrated camera mounting.")
                .font(.caption)
                .foregroundStyle(.secondary)
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

    private func binding<T>(_ keyPath: WritableKeyPath<AppSettings, T>) -> Binding<T> {
        Binding(
            get: { settings.settings[keyPath: keyPath] },
            set: { settings.settings[keyPath: keyPath] = $0 }
        )
    }
}
