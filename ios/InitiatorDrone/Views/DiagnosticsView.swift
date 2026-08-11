import SwiftUI
#if canImport(ARKit)
import ARKit
#endif

/// Topic rates, last message times, the VIO pose, IMU values, and the raw
/// rosbridge log.
struct DiagnosticsView: View {
    @EnvironmentObject private var model: AppModel
    @EnvironmentObject private var connection: RobotConnection
    #if canImport(ARKit)
    @EnvironmentObject private var arSession: ARSessionController
    @EnvironmentObject private var alignment: AlignmentController
    #endif

    @State private var logFilter: LogFilter = .all

    private enum LogFilter: String, CaseIterable, Identifiable {
        case all = "All"
        case problems = "Problems"
        var id: String { rawValue }
    }

    var body: some View {
        NavigationStack {
            List {
                linkSection
                topicSection
                vioNodeSection
                vioSection
                imuSection
                #if canImport(ARKit)
                phoneSection
                #endif
                streamSection
                logSection
            }
            .navigationTitle("Diagnostics")
            .toolbar {
                ToolbarItem(placement: .primaryAction) {
                    Menu {
                        Button {
                            connection.clearLogs()
                        } label: {
                            Label("Clear log", systemImage: "trash")
                        }
                        if connection.isSimulated {
                            Button {
                                model.simulateConnectionLoss()
                            } label: {
                                Label("Simulate Wi-Fi drop", systemImage: "wifi.slash")
                            }
                        }
                    } label: {
                        Image(systemName: "ellipsis.circle")
                    }
                }
            }
        }
    }

    // MARK: - Link

    private var linkSection: some View {
        Section("Link") {
            LabeledContent("rosbridge", value: connection.connectionState.shortDescription)
            LabeledContent("Robot", value: connection.endpoint?.displayName ?? "—")
            LabeledContent("Source", value: connection.isSimulated ? "Fixtures" : "Live")

            if let offset = connection.clockOffset {
                LabeledContent("Clock offset") {
                    Text(String(format: "%+.3f s", offset))
                        .monospacedDigit()
                }
            } else {
                LabeledContent("Clock offset", value: "not estimated")
            }

            if let dashboard = connection.dashboardState {
                LabeledContent("ROS graph", value: dashboard.isRunning ? "Running" : "Stopped")
                if let temperature = dashboard.cpuTemperature {
                    LabeledContent("Robot CPU temp", value: String(format: "%.0f °C", temperature))
                }
                if !dashboard.cpuCores.isEmpty {
                    let peak = dashboard.cpuCores.map(\.load).max() ?? 0
                    LabeledContent("Robot CPU peak", value: "\(peak)%")
                }
            }

            if let error = connection.dashboardError {
                Label(error, systemImage: "exclamationmark.triangle")
                    .font(.footnote)
                    .foregroundStyle(.orange)
            }
        }
    }

    // MARK: - Topics

    private var topicSection: some View {
        Section {
            if connection.topicHealth.isEmpty {
                Text("No subscriptions yet.")
                    .font(.footnote)
                    .foregroundStyle(.secondary)
            }
            ForEach(connection.topicHealth) { health in
                VStack(alignment: .leading, spacing: 4) {
                    HStack {
                        Circle()
                            .fill(health.isSilent ? Color.red : Color.green)
                            .frame(width: 8, height: 8)
                        Text(health.topic.displayName)
                            .font(.subheadline)
                        Spacer()
                        Text(String(format: "%.1f Hz", health.rateHz))
                            .font(.system(.subheadline, design: .monospaced))
                            .foregroundStyle(health.isSilent ? Color.red : Color.primary)
                    }
                    Text(health.topic.topicName)
                        .font(.system(size: 10, design: .monospaced))
                        .foregroundStyle(.secondary)
                    HStack(spacing: 12) {
                        Text(lastMessageText(health))
                        Text("\(health.totalCount) msgs")
                        Text(health.isSubscribed ? "subscribed" : "not subscribed")
                    }
                    .font(.system(size: 10))
                    .foregroundStyle(.secondary)
                }
                .padding(.vertical, 2)
            }
        } header: {
            Text("Topic rates")
        } footer: {
            Text("A rate above zero only proves the publisher is alive. Check the VIO flags below before trusting the pose.")
        }
    }

    private func lastMessageText(_ health: TopicHealth) -> String {
        guard let age = health.lastMessageAge else { return "never" }
        if age < 1 { return String(format: "%.0f ms ago", age * 1000) }
        return String(format: "%.1f s ago", age)
    }

    // MARK: - VIO

    private var vioNodeSection: some View {
        Section {
            LabeledContent("Node", value: connection.vioNodeStatus.shortLabel)
            Text(connection.vioNodeStatus.detailLabel)
                .font(.caption)
                .foregroundStyle(.secondary)

            LabeledContent("/vio/calibrated", value: flagText(connection.isCalibrated))
            LabeledContent("/vio/visual_tracking", value: flagText(connection.isVisualTracking))
        } header: {
            Text("VIO node")
        } footer: {
            Text("""
            Node status is inferred from traffic on the topics vio_node owns, because the robot's \
            rosbridge is launched without rosapi_node and cannot answer /rosapi/nodes. Both flags \
            are published only when they change, so "no message yet" is normal on a phone that \
            connected after calibration had already finished.
            """)
        }
    }

    private var vioSection: some View {
        Section {
            LabeledContent("Status", value: connection.trackingStatus.shortLabel)
            Text(connection.trackingStatus.detailLabel)
                .font(.caption)
                .foregroundStyle(.secondary)

            if let odometry = connection.latestOdometry {
                LabeledContent("Frames", value: "\(odometry.frameId) → \(odometry.childFrameId)")
                LabeledContent("Position") {
                    Text(String(
                        format: "%.3f  %.3f  %.3f",
                        odometry.pose.position.x,
                        odometry.pose.position.y,
                        odometry.pose.position.z
                    ))
                    .font(.system(.caption, design: .monospaced))
                }
                LabeledContent("Quaternion") {
                    Text(String(
                        format: "%.3f  %.3f  %.3f  %.3f",
                        odometry.pose.orientation.x,
                        odometry.pose.orientation.y,
                        odometry.pose.orientation.z,
                        odometry.pose.orientation.w
                    ))
                    .font(.system(.caption, design: .monospaced))
                }
                LabeledContent("RPY") {
                    let angles = odometry.pose.orientation.rollPitchYaw
                    let scale = 180.0 / Double.pi
                    Text(String(
                        format: "%.1f  %.1f  %.1f °",
                        angles.roll * scale,
                        angles.pitch * scale,
                        angles.yaw * scale
                    ))
                    .font(.system(.caption, design: .monospaced))
                }
                LabeledContent("Twist (body)") {
                    Text(String(
                        format: "%.2f  %.2f  %.2f m/s",
                        odometry.linearVelocity.x,
                        odometry.linearVelocity.y,
                        odometry.linearVelocity.z
                    ))
                    .font(.system(.caption, design: .monospaced))
                }
                LabeledContent("Buffered samples", value: "\(connection.sampler.sampleCount)")
            } else {
                Text("No odometry received.")
                    .font(.footnote)
                    .foregroundStyle(.secondary)
            }
        } header: {
            Text("VIO pose")
        }
    }

    private func flagText(_ value: Bool?) -> String {
        guard let value else { return "no message yet" }
        return value ? "true" : "false"
    }

    // MARK: - IMU

    private var imuSection: some View {
        Section("IMU (calibrated)") {
            if let imu = connection.latestIMU {
                LabeledContent("Frame", value: imu.frameId.isEmpty ? "—" : imu.frameId)
                LabeledContent("Linear accel") {
                    Text(String(
                        format: "%.2f  %.2f  %.2f m/s²",
                        imu.linearAcceleration.x,
                        imu.linearAcceleration.y,
                        imu.linearAcceleration.z
                    ))
                    .font(.system(.caption, design: .monospaced))
                }
                LabeledContent("|a|") {
                    Text(String(format: "%.2f m/s²", imu.accelerationMagnitude))
                        .font(.system(.caption, design: .monospaced))
                        // Both branches must be the same type: `.primary` alone
                        // is a HierarchicalShapeStyle, which will not unify
                        // with a Color.
                        .foregroundStyle(
                            abs(imu.accelerationMagnitude - 9.81) < 0.5 ? Color.primary : Color.orange
                        )
                }
                LabeledContent("Angular rate") {
                    Text(String(
                        format: "%.3f  %.3f  %.3f rad/s",
                        imu.angularVelocity.x,
                        imu.angularVelocity.y,
                        imu.angularVelocity.z
                    ))
                    .font(.system(.caption, design: .monospaced))
                }
                if let orientation = imu.orientation {
                    LabeledContent("Orientation RPY") {
                        let angles = orientation.rollPitchYaw
                        let scale = 180.0 / Double.pi
                        Text(String(
                            format: "%.1f  %.1f  %.1f °",
                            angles.roll * scale,
                            angles.pitch * scale,
                            angles.yaw * scale
                        ))
                        .font(.system(.caption, design: .monospaced))
                    }
                }
            } else {
                Text("No IMU messages received.")
                    .font(.footnote)
                    .foregroundStyle(.secondary)
            }
        }
    }

    // MARK: - Phone

    #if canImport(ARKit)
    private var phoneSection: some View {
        Section("Phone (ARKit)") {
            LabeledContent("Tracking", value: arSession.trackingStateLabel)
            LabeledContent("Lens", value: arSession.lensLabel)
            if !arSession.videoFormatLabel.isEmpty {
                LabeledContent("Video format", value: arSession.videoFormatLabel)
                    .font(.caption)
            }
            if !arSession.trackingStateDetail.isEmpty {
                Text(arSession.trackingStateDetail)
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
            LabeledContent("Position (AR)") {
                Text(String(
                    format: "%.3f  %.3f  %.3f",
                    arSession.phonePose.position.x,
                    arSession.phonePose.position.y,
                    arSession.phonePose.position.z
                ))
                .font(.system(.caption, design: .monospaced))
            }
            LabeledContent("Heading") {
                Text(String(format: "%.0f°", arSession.phonePose.orientation.yawAroundY * 180 / .pi))
                    .font(.system(.caption, design: .monospaced))
            }
            LabeledContent("Alignment", value: alignment.isAligned ? alignment.summary : "not set")
                .font(.caption)
        }
    }
    #endif

    // MARK: - Streams

    private var streamSection: some View {
        Section {
            LabeledContent("Depth rendered", value: String(format: "%.1f fps", connection.depthRenderFPS))
            LabeledContent("Depth frames dropped", value: "\(connection.droppedDepthFrames)")

            if let info = connection.depthCameraInfo {
                LabeledContent("Depth camera", value: "\(info.width)x\(info.height)")
                if let horizontal = info.horizontalFieldOfView, let vertical = info.verticalFieldOfView {
                    LabeledContent("FOV") {
                        Text(String(
                            format: "%.0f° x %.0f°",
                            horizontal * 180 / .pi,
                            vertical * 180 / .pi
                        ))
                        .font(.system(.caption, design: .monospaced))
                    }
                }
            }
        } header: {
            Text("Depth stream")
        } footer: {
            Text("""
            Dropped frames are deliberate: the decoder keeps only the newest frame, so a slow phone \
            falls behind in latency rather than in memory. A number that climbs steadily means the \
            image throttle is set faster than this phone and link can keep up.
            """)
        }
    }

    // MARK: - Log

    private var logSection: some View {
        Section {
            Picker("Filter", selection: $logFilter) {
                ForEach(LogFilter.allCases) { Text($0.rawValue).tag($0) }
            }
            .pickerStyle(.segmented)

            if filteredLogs.isEmpty {
                Text("Nothing logged.")
                    .font(.footnote)
                    .foregroundStyle(.secondary)
            }

            ForEach(filteredLogs.reversed()) { entry in
                HStack(alignment: .top, spacing: 8) {
                    Circle()
                        .fill(color(for: entry.level))
                        .frame(width: 6, height: 6)
                        .padding(.top, 5)
                    VStack(alignment: .leading, spacing: 1) {
                        Text(entry.text)
                            .font(.system(size: 11, design: .monospaced))
                            .textSelection(.enabled)
                        Text(entry.date, style: .time)
                            .font(.system(size: 9))
                            .foregroundStyle(.secondary)
                    }
                }
            }
        } header: {
            Text("rosbridge messages")
        }
    }

    private var filteredLogs: [RosbridgeLogEntry] {
        switch logFilter {
        case .all:
            return connection.logs
        case .problems:
            return connection.logs.filter { $0.level != .info }
        }
    }

    private func color(for level: RosbridgeLogEntry.Level) -> Color {
        switch level {
        case .info: return .secondary
        case .warning: return .orange
        case .error: return .red
        }
    }
}
