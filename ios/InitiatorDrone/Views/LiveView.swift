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
    /// Hides every instrument overlay, leaving the camera and the robot marker.
    @State private var isFullScreen = false

    var body: some View {
        GeometryReader { geometry in
            let isLandscape = geometry.size.width > geometry.size.height

            ZStack {
                cameraLayer
                    .ignoresSafeArea()

                if !isFullScreen {
                    if isLandscape {
                        landscapeOverlay
                    } else {
                        portraitOverlay
                    }
                }

                #if canImport(ARKit)
                // Scene content rather than chrome, so it survives full screen —
                // and placement is still confirmed by tapping the camera view.
                if case .pickingPosition = alignment.phase {
                    crosshair
                }
                #endif

                if isFullScreen {
                    exitFullScreenButton
                }
            }
        }
        // The tab bar, the status bar and the home indicator are all chrome too.
        // Leaving them up would make "full screen" mean "slightly fewer panels".
        .toolbar(isFullScreen ? .hidden : .visible, for: .tabBar)
        .statusBarHidden(isFullScreen)
        .persistentSystemOverlays(isFullScreen ? .hidden : .automatic)
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

    private func setFullScreen(_ value: Bool) {
        withAnimation(.easeInOut(duration: 0.25)) { isFullScreen = value }
    }

    /// The only way back.
    ///
    /// Deliberately a button rather than a tap-anywhere gesture: a full-screen
    /// tap catcher would sit on top of `ARRobotSceneView` and swallow the taps
    /// that place the alignment origin. A small persistent control costs a
    /// corner of the view and never fights the AR session for a gesture.
    private var exitFullScreenButton: some View {
        VStack {
            HStack {
                Spacer()
                Button {
                    setFullScreen(false)
                } label: {
                    Image(systemName: "arrow.down.right.and.arrow.up.left")
                        .font(.system(size: 15, weight: .semibold))
                        .padding(11)
                        .background(.ultraThinMaterial, in: Circle())
                        .overlay(
                            Circle().strokeBorder(.white.opacity(0.25), lineWidth: 1)
                        )
                }
                .accessibilityLabel("Exit full screen")
            }
            Spacer()
        }
        .padding(.horizontal, 14)
        .padding(.top, 10)
        .transition(.opacity)
    }

    // MARK: - Camera layer

    @ViewBuilder
    private var cameraLayer: some View {
        #if canImport(ARKit)
        if arSession.isSupported {
            ARRobotSceneView(
                sampler: connection.sampler,
                pointCloudStore: connection.pointCloudStore,
                trackingStatus: connection.trackingStatus,
                cameraInfo: settings.settings.showsCameraFrustum ? connection.depthCameraInfo : nil,
                showsFrustum: settings.settings.showsCameraFrustum,
                showsTrail: settings.settings.showsRobotTrail,
                showsPointCloud: settings.settings.pointCloud.isEnabled,
                pointSize: settings.settings.pointCloud.pointSize,
                cameraExtrinsics: settings.settings.cameraExtrinsics,
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
                    title: "Odom node",
                    value: connection.odomNodeStatus.shortLabel,
                    level: connection.odomNodeStatus.pillLevel
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
                    label: "Odom node",
                    value: connection.odomNodeStatus.shortLabel,
                    caption: connection.isCalibrated.map { $0 ? "calibrated" : "not calibrated" }
                        ?? "no flag yet",
                    isDimmed: !connection.odomNodeStatus.isNodePresent
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
                    Task { await connection.calibrateOdometry() }
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

                // Icon only, and not sharing the flexible width: the two
                // labelled buttons beside it are already tight on a phone.
                Button {
                    setFullScreen(true)
                } label: {
                    Image(systemName: "arrow.up.left.and.arrow.down.right")
                        .frame(height: 20)
                }
                .buttonStyle(.bordered)
                .accessibilityLabel("Full screen")
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

    /// Fixtures included: the simulator models the same launch lifecycle, so
    /// these controls mean the same thing there.
    private var canControlGraph: Bool {
        connection.endpoint != nil
    }

    /// Explains a disabled control instead of leaving the operator guessing.
    private var disabledExplanation: String? {
        if connection.endpoint == nil {
            return "Add a robot on the Robot tab to enable the controls."
        }
        if let error = connection.dashboardError {
            return "Dashboard: \(error)"
        }
        if connection.dashboardState?.isRunning == false {
            return connection.isSimulated
                ? "Simulated launch is stopped. Press Start to bring the fixture streams back."
                : "ROS graph is stopped. Calibration is unavailable until you start it."
        }
        // The node not being up explains a missing marker better than anything
        // downstream of it can, so it is reported ahead of the pose status.
        if !connection.odomNodeStatus.isNodePresent, connection.connectionState.isConnected {
            return connection.odomNodeStatus.detailLabel
        }
        if case .calibrating = connection.odomNodeStatus {
            return connection.odomNodeStatus.detailLabel
        }
        // Deliberately not banners: `.orientationOnly` is the permanent, correct
        // state with odom_node, and a banner that is always up is wallpaper. The
        // "Robot track" pill carries it in amber instead. `.stale` is the
        // transient version of the same problem, which is what banners are for.
        if case .stale = connection.trackingStatus {
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
