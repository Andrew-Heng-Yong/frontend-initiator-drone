import SwiftUI

/// Robot address entry, connection test, and the list of saved robots.
struct ConnectionView: View {
    @EnvironmentObject private var model: AppModel
    @EnvironmentObject private var connection: RobotConnection
    @EnvironmentObject private var robots: RobotStore

    @State private var host: String = ""
    @State private var name: String = ""
    @State private var dashboardPort: String = String(RobotEndpoint.defaultDashboardPort)
    @State private var rosbridgePort: String = String(RobotEndpoint.defaultRosbridgePort)
    @State private var editingID: UUID?

    @State private var isTesting = false
    @State private var testResult: ConnectionTestResult?

    var body: some View {
        NavigationStack {
            Form {
                addressSection
                actionsSection
                if let result = testResult { testResultSection(result) }
                savedSection
                fixturesSection
            }
            .navigationTitle("Robot")
            .onAppear(perform: loadSelectedIntoForm)
        }
    }

    // MARK: - Address

    private var addressSection: some View {
        Section {
            TextField("Name (optional)", text: $name)
                .textInputAutocapitalization(.words)

            TextField("IP address or hostname", text: $host)
                .keyboardType(.URL)
                .textInputAutocapitalization(.never)
                .autocorrectionDisabled()
                .font(.system(.body, design: .monospaced))

            HStack {
                Text("Dashboard port")
                Spacer()
                TextField("4173", text: $dashboardPort)
                    .keyboardType(.numberPad)
                    .multilineTextAlignment(.trailing)
                    .frame(width: 80)
                    .font(.system(.body, design: .monospaced))
            }

            HStack {
                Text("rosbridge port")
                Spacer()
                TextField("9090", text: $rosbridgePort)
                    .keyboardType(.numberPad)
                    .multilineTextAlignment(.trailing)
                    .frame(width: 80)
                    .font(.system(.body, design: .monospaced))
            }
        } header: {
            Text("Robot address")
        } footer: {
            if let endpoint = draftEndpoint, endpoint.isValid {
                VStack(alignment: .leading, spacing: 2) {
                    Text(endpoint.dashboardBaseURL?.absoluteString ?? "")
                    Text(endpoint.rosbridgeURL?.absoluteString ?? "")
                }
                .font(.system(.caption2, design: .monospaced))
            } else {
                Text("Paste anything — a bare IP, or a full http:// URL. The scheme and path are stripped.")
            }
        }
    }

    // MARK: - Actions

    private var actionsSection: some View {
        Section {
            Button {
                Task { await runConnectionTest() }
            } label: {
                HStack {
                    Label("Test connection", systemImage: "checkmark.seal")
                    if isTesting {
                        Spacer()
                        ProgressView()
                    }
                }
            }
            .disabled(!(draftEndpoint?.isValid ?? false) || isTesting)

            Button {
                guard let endpoint = draftEndpoint, endpoint.isValid else { return }
                model.connect(to: endpoint)
            } label: {
                Label("Save and connect", systemImage: "bolt.horizontal.circle")
            }
            .disabled(!(draftEndpoint?.isValid ?? false))

            if connection.endpoint != nil {
                Button(role: .destructive) {
                    model.disconnect()
                } label: {
                    Label("Disconnect", systemImage: "xmark.circle")
                }
            }
        } header: {
            Text("Connection")
        } footer: {
            HStack(spacing: 6) {
                Circle()
                    .fill(connection.connectionState.isConnected ? Color.green : Color.orange)
                    .frame(width: 8, height: 8)
                Text(statusFooter)
            }
            .font(.caption)
        }
    }

    private var statusFooter: String {
        guard let endpoint = connection.endpoint else { return "Not connected." }
        return "\(connection.connectionState.shortDescription) — \(endpoint.displayName)"
    }

    private func testResultSection(_ result: ConnectionTestResult) -> some View {
        Section("Last test") {
            Label(
                result.isReachable ? "Dashboard reachable" : "Not reachable",
                systemImage: result.isReachable ? "checkmark.circle.fill" : "xmark.octagon.fill"
            )
            .foregroundStyle(result.isReachable ? .green : .red)

            LabeledContent("Round trip", value: String(format: "%.0f ms", result.latency * 1000))
            LabeledContent("ROS graph", value: result.graphRunning ? "Running" : "Stopped")
            Text(result.message)
                .font(.caption)
                .foregroundStyle(.secondary)
        }
    }

    // MARK: - Saved robots

    private var savedSection: some View {
        Section("Saved robots") {
            if robots.robots.isEmpty {
                Text("No robots saved yet.")
                    .foregroundStyle(.secondary)
                    .font(.footnote)
            }

            ForEach(robots.robots) { robot in
                Button {
                    loadIntoForm(robot)
                    model.connect(to: robot)
                } label: {
                    HStack {
                        VStack(alignment: .leading, spacing: 2) {
                            Text(robot.displayName)
                                .foregroundStyle(.primary)
                            Text("\(robot.normalizedHost):\(robot.dashboardPort) · ws \(robot.rosbridgePort)")
                                .font(.system(.caption2, design: .monospaced))
                                .foregroundStyle(.secondary)
                        }
                        Spacer()
                        if connection.endpoint?.id == robot.id {
                            Image(systemName: "dot.radiowaves.left.and.right")
                                .foregroundStyle(.green)
                        }
                    }
                }
                .swipeActions {
                    Button(role: .destructive) {
                        robots.remove(robot)
                    } label: {
                        Label("Delete", systemImage: "trash")
                    }
                    Button {
                        loadIntoForm(robot)
                    } label: {
                        Label("Edit", systemImage: "pencil")
                    }
                    .tint(.blue)
                }
            }
            .onDelete { robots.remove(atOffsets: $0) }
        }
    }

    private var fixturesSection: some View {
        Section {
            Button {
                model.connectToFixtures()
            } label: {
                Label("Run on fixtures (no robot)", systemImage: "testtube.2")
            }
        } header: {
            Text("Offline")
        } footer: {
            Text("""
            Replays synthesised depth, thermal, odometry and IMU data through the real rosbridge \
            message path, so the live view, alignment and diagnostics can be exercised with no drone \
            on the network.
            """)
        }
    }

    // MARK: - Form plumbing

    private var draftEndpoint: RobotEndpoint? {
        let trimmed = host.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return nil }
        return RobotEndpoint(
            id: editingID ?? UUID(),
            name: name,
            host: trimmed,
            dashboardPort: Int(dashboardPort) ?? RobotEndpoint.defaultDashboardPort,
            rosbridgePort: Int(rosbridgePort) ?? RobotEndpoint.defaultRosbridgePort
        )
    }

    private func loadSelectedIntoForm() {
        guard host.isEmpty, let robot = connection.endpoint ?? robots.selected else { return }
        loadIntoForm(robot)
    }

    private func loadIntoForm(_ robot: RobotEndpoint) {
        editingID = robot.id
        name = robot.name
        host = robot.host
        dashboardPort = String(robot.dashboardPort)
        rosbridgePort = String(robot.rosbridgePort)
    }

    private func runConnectionTest() async {
        guard let endpoint = draftEndpoint, endpoint.isValid else { return }
        isTesting = true
        defer { isTesting = false }

        let api = DashboardAPI(endpoint: endpoint)
        testResult = await api.testConnection()
    }
}
