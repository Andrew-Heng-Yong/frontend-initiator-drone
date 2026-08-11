import SwiftUI
#if canImport(ARKit)
import ARKit
#endif

/// The main screen: the phone camera with the robot drawn into it, the cropped
/// depth stream, and the controls.
///
/// The layout adapts rather than being duplicated: the same overlay pieces are
/// arranged as a column in portrait and as a row in landscape, so there is only
/// one set of behaviour to reason about.
struct LiveView: View {
    @EnvironmentObject private var model: AppModel
    @EnvironmentObject private var connection: RobotConnection
    @EnvironmentObject private var settings: SettingsStore
    #if canImport(ARKit)
    @EnvironmentObject private var arSession: ARSessionController
    @EnvironmentObject private var alignment: AlignmentController
    #endif

    @State private var showsSensorPanel = true
    @State private var showsAlignmentSheet = false

    var body: some View {
        GeometryReader { geometry in
            let isLandscape = geometry.size.width > geometry.size.height

            ZStack {
                cameraLayer
                    .ignoresSafeArea()

                if isLandscape {
                    landscapeOverlay
                } else {
                    portraitOverlay
                }

                #if canImport(ARKit)
                if case .pickingPosition = alignment.phase {
                    crosshair
                }
                #endif
            }
        }
        .sheet(isPresented: $showsAlignmentSheet) {
            #if canImport(ARKit)
            AlignmentSheet()
                .environmentObject(model)
                .environmentObject(alignment)
                .environmentObject(arSession)
                .environmentObject(connection)
            #endif
        }
        .onAppear {
            #if canImport(ARKit)
            model.startARIfNeeded()
            #endif
        }
    }

    // MARK: - Camera layer

    @ViewBuilder
    private var cameraLayer: some View {
        #if canImport(ARKit)
        if arSession.isSupported {
            ARRobotSceneView(
                sampler: connection.sampler,
                trackingStatus: connection.trackingStatus,
                cameraInfo: settings.settings.showsCameraFrustum ? connection.depthCameraInfo : nil,
                showsFrustum: settings.settings.showsCameraFrustum,
                showsTrail: settings.settings.showsRobotTrail,
                placementPhase: alignment.phase,
                previewPosition: alignment.previewPosition,
                pendingYaw: alignment.pendingYaw,
                isAligned: alignment.isAligned,
                session: arSession.session,
                onTapPlacement: { position in
                    alignment.placeOrigin(
                        at: position,
                        initialYaw: arSession.currentPose.orientation.yawAroundY
                    )
                },
                onPreviewUpdate: { position in
                    alignment.updatePreview(position)
                }
            )
        } else {
            unsupportedCameraPlaceholder
        }
        #else
        unsupportedCameraPlaceholder
        #endif
    }

    private var unsupportedCameraPlaceholder: some View {
        ZStack {
            Color.black
            VStack(spacing: 10) {
                Image(systemName: "arkit")
                    .font(.largeTitle)
                Text("ARKit world tracking is not available on this device.")
                    .font(.footnote)
                    .multilineTextAlignment(.center)
                Text("The depth stream and all diagnostics still work.")
                    .font(.caption2)
                    .foregroundStyle(.secondary)
                    .multilineTextAlignment(.center)
            }
            .padding(30)
        }
    }

    // MARK: - Overlays

    private var portraitOverlay: some View {
        VStack(spacing: 10) {
            statusStrip
            Spacer(minLength: 0)
            if showsSensorPanel {
                sensorPanel
                    .frame(maxHeight: 260)
                    .transition(.move(edge: .bottom).combined(with: .opacity))
            }
            metricsStrip
            controlBar
        }
        .padding(.horizontal, 12)
        .padding(.bottom, 6)
    }

    private var landscapeOverlay: some View {
        HStack(alignment: .top, spacing: 10) {
            VStack(alignment: .leading, spacing: 8) {
                statusStrip
                Spacer(minLength: 0)
                metricsStrip
            }
            .frame(maxWidth: .infinity, alignment: .leading)

            if showsSensorPanel {
                VStack(spacing: 8) {
                    sensorPanel
                    controlBar
                }
                .frame(width: 320)
                .transition(.move(edge: .trailing).combined(with: .opacity))
            } else {
                controlBar
                    .frame(width: 320)
            }
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 8)
    }

    // MARK: - Status

    private var statusStrip: some View {
        ScrollView(.horizontal, showsIndicators: false) {
            HStack(spacing: 8) {
                StatusPill(
                    title: "Link",
                    value: connection.connectionState.shortDescription,
                    level: connection.connectionState.pillLevel
                )

                StatusPill(
                    title: "VIO node",
                    value: connection.vioNodeStatus.shortLabel,
                    level: connection.vioNodeStatus.pillLevel
                )

                StatusPill(
                    title: "Robot track",
                    value: connection.trackingStatus.shortLabel,
                    level: connection.trackingStatus.pillLevel
                )

                #if canImport(ARKit)
                StatusPill(
                    title: "Phone AR",
                    value: arSession.trackingStateLabel,
                    level: arSession.isTrackingUsable ? .good : .warning
                )

                StatusPill(
                    title: "Align",
                    value: alignment.isAligned ? "Set" : "Not set",
                    level: alignment.isAligned ? .good : .warning
                )
                #endif

                if connection.isSimulated {
                    StatusPill(title: "Source", value: "Fixtures", level: .warning, systemImage: "testtube.2")
                }
            }
            .padding(.top, 4)
        }
    }

    // MARK: - Metrics

    private var metricsStrip: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack(spacing: 10) {
                MetricTile(
                    label: "Depth",
                    value: String(format: "%.1f fps", connection.depthRenderFPS),
                    caption: depthCaption,
                    isDimmed: connection.depthFrame == nil
                )
                MetricTile(
                    label: "Odom",
                    value: String(format: "%.1f Hz", connection.odometryRateHz),
                    caption: connection.trackingStatus.shortLabel,
                    isDimmed: connection.latestOdometry == nil
                )
                MetricTile(
                    label: "VIO node",
                    value: connection.vioNodeStatus.shortLabel,
                    caption: connection.isCalibrated.map { $0 ? "calibrated" : "not calibrated" }
                        ?? "no flag yet",
                    isDimmed: !connection.vioNodeStatus.isNodePresent
                )
            }

            HStack(spacing: 10) {
                MetricTile(
                    label: "Position (odom)",
                    value: positionText,
                    caption: "x, y, z m",
                    isDimmed: connection.latestOdometry == nil
                )
                MetricTile(
                    label: "Orientation",
                    value: orientationText,
                    caption: "roll, pitch, yaw °",
                    isDimmed: connection.latestOdometry == nil
                )
            }
        }
        .padding(10)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12, style: .continuous))
    }

    private var depthCaption: String? {
        guard let frame = connection.depthFrame else { return nil }
        return "\(frame.width)x\(frame.height) \(frame.encoding)"
    }

    private var positionText: String {
        guard let odometry = connection.latestOdometry else { return "—" }
        return String(
            format: "%.2f  %.2f  %.2f",
            odometry.pose.position.x,
            odometry.pose.position.y,
            odometry.pose.position.z
        )
    }

    private var orientationText: String {
        guard let odometry = connection.latestOdometry else { return "—" }
        let angles = odometry.pose.orientation.rollPitchYaw
        let degrees = 180.0 / Double.pi
        return String(
            format: "%.0f  %.0f  %.0f",
            angles.roll * degrees,
            angles.pitch * degrees,
            angles.yaw * degrees
        )
    }

    // MARK: - Sensor panel

    private var sensorPanel: some View {
        VStack(spacing: 8) {
            SensorImageView(frame: connection.depthFrame, placeholder: depthPlaceholder)
                .frame(maxWidth: .infinity)
                .aspectRatio(4.0 / 3.0, contentMode: .fit)
                .background(Color.black.opacity(0.6))
                .clipShape(RoundedRectangle(cornerRadius: 10, style: .continuous))

            depthLegend
        }
        .padding(10)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12, style: .continuous))
        .accessibilityElement(children: .contain)
        .accessibilityLabel("Depth stream")
    }

    private var depthPlaceholder: String {
        connection.connectionState.isConnected
            ? "Waiting for \(RobotTopic.depthImage.topicName)"
            : "Not connected"
    }

    private var depthLegend: some View {
        ColorLegend(
            style: settings.settings.depthColorMap.style,
            low: connection.depthFrame?.rangeLow ?? settings.settings.depthColorMap.low,
            high: connection.depthFrame?.rangeHigh ?? settings.settings.depthColorMap.high,
            unit: connection.depthFrame?.unit ?? .metres,
            reversed: settings.settings.depthColorMap.reversed
        )
    }

    // MARK: - Controls

    private var controlBar: some View {
        VStack(spacing: 8) {
            HStack(spacing: 8) {
                Button {
                    Task { await connection.startGraph() }
                } label: {
                    Label("Start", systemImage: "play.fill")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .tint(.green)
                .disabled(!canControlGraph || (connection.dashboardState?.isRunning ?? false))

                Button {
                    Task { await connection.stopGraph() }
                } label: {
                    Label("Stop", systemImage: "stop.fill")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.bordered)
                .tint(.red)
                .disabled(!canControlGraph || !(connection.dashboardState?.isRunning ?? false))

                Button {
                    Task { await connection.calibrateVIO() }
                } label: {
                    Label("Calibrate", systemImage: "scope")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.bordered)
                .tint(.blue)
                // Calibration only means something while the graph is up.
                .disabled(!connection.canCalibrate)
            }
            .font(.footnote)

            HStack(spacing: 8) {
                #if canImport(ARKit)
                Button {
                    showsAlignmentSheet = true
                } label: {
                    Label(alignment.isAligned ? "Re-align robot" : "Align robot", systemImage: "scope")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.bordered)
                #endif

                Button {
                    withAnimation(.easeInOut(duration: 0.2)) { showsSensorPanel.toggle() }
                } label: {
                    Label(
                        showsSensorPanel ? "Hide depth" : "Show depth",
                        systemImage: showsSensorPanel ? "eye.slash" : "eye"
                    )
                    .frame(maxWidth: .infinity)
                }
                .buttonStyle(.bordered)
            }
            .font(.footnote)

            if let message = disabledExplanation {
                Text(message)
                    .font(.caption2)
                    .foregroundStyle(.secondary)
                    .frame(maxWidth: .infinity, alignment: .leading)
            }
        }
        .padding(10)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12, style: .continuous))
    }

    private var canControlGraph: Bool {
        !connection.isSimulated && connection.endpoint != nil
    }

    /// Explains a disabled control instead of leaving the operator guessing.
    private var disabledExplanation: String? {
        if connection.isSimulated {
            return "Running on fixtures. Start, Stop and Calibrate are disabled."
        }
        if connection.endpoint == nil {
            return "Add a robot on the Robot tab to enable the controls."
        }
        if let error = connection.dashboardError {
            return "Dashboard: \(error)"
        }
        if connection.dashboardState?.isRunning == false {
            return "ROS graph is stopped. Calibration is unavailable until you start it."
        }
        // The node not being up explains a missing marker better than anything
        // downstream of it can, so it is reported ahead of the pose status.
        if !connection.vioNodeStatus.isNodePresent, connection.connectionState.isConnected {
            return connection.vioNodeStatus.detailLabel
        }
        if case .calibrating = connection.vioNodeStatus {
            return connection.vioNodeStatus.detailLabel
        }
        if case .visualTrackingLost = connection.trackingStatus {
            return connection.trackingStatus.detailLabel
        }
        return nil
    }

    private var crosshair: some View {
        ZStack {
            Circle()
                .strokeBorder(.white.opacity(0.8), lineWidth: 2)
                .frame(width: 34, height: 34)
            Circle()
                .fill(.white.opacity(0.9))
                .frame(width: 4, height: 4)
        }
        .shadow(radius: 3)
        .allowsHitTesting(false)
    }
}
