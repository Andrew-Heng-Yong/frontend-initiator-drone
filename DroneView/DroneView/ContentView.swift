import SwiftUI
import UniformTypeIdentifiers

@main struct DroneViewApp: App {
    var body: some Scene {
        WindowGroup { ContentView().preferredColorScheme(.dark) }
    }
}

struct ContentView: View {
    @State private var model = DroneModel()
    @State private var arModel = PhoneReconstruction()
    @State private var selectedTab = AppTab.ar
    @State private var showsConnection = false
    @AppStorage("moduleAddress") private var address = "http://192.168.1.6:8080"
    @Environment(\.scenePhase) private var scenePhase
    private enum AppTab: Hashable { case scene, cameras, tracking, ar }

    var body: some View {
        TabView(selection: $selectedTab) {
            Tab("Thermal AR", systemImage: "arkit", value: AppTab.ar) {
                ThermalARView(model: arModel)
            }
            Tab("Scene", systemImage: "cube.transparent", value: AppTab.scene) {
                NavigationStack {
                    SceneView(model: model, connect: { showsConnection = true })
                        .toolbarBackground(.hidden, for: .navigationBar)
                }
            }
            Tab("Cameras", systemImage: "camera", value: AppTab.cameras) {
                NavigationStack { CameraView(model: model).navigationTitle("Cameras") }
            }
            Tab("Tracking", systemImage: "waveform.path.ecg", value: AppTab.tracking) {
                NavigationStack { TrackingView(model: model).navigationTitle("Tracking") }
            }
        }
        .onChange(of: model.endpoint) {_,endpoint in
            arModel.stop()
            if let endpoint,scenePhase == .active {arModel.start(endpoint)}
        }
        .onChange(of: scenePhase) {_,phase in
            if phase == .background {arModel.suspend();UIApplication.shared.isIdleTimerDisabled=false}
            else if phase == .active,let endpoint=model.endpoint {arModel.resume(endpoint);UIApplication.shared.isIdleTimerDisabled=true}
        }
        .onChange(of: arModel.running) {_,running in UIApplication.shared.isIdleTimerDisabled=running}
        .toolbarBackground(.hidden, for: .tabBar)
        .sheet(isPresented: $showsConnection) { ConnectionView(model: model) }
        .task {
            if let url = TrackingAPI.address(address) { model.connect(url) }
        }
        .task(id: "\(model.connectionID)-\(scenePhase == .active)") {
            if scenePhase == .active { await model.run() }
        }
    }
}

private struct ConnectionView: View {
    let model: DroneModel
    @AppStorage("moduleAddress") private var address = "http://192.168.1.6:8080"
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationStack {
            Form {
                Section("Module address") {
                    TextField("http://192.168.1.6:8080", text: $address)
                        .keyboardType(.URL).textInputAutocapitalization(.never).autocorrectionDisabled()
                        .accessibilityLabel("Module server address")
                        .onSubmit(connect)
                    Button("Connect", action: connect).disabled(TrackingAPI.address(address) == nil)
                    if model.endpoint != nil {
                        Button("Disconnect", role: .destructive) { model.disconnect(); dismiss() }
                    }
                }
                Section {
                    Text("Use the same local network as the module. Enter the Camera + Gyro server address, including its port (normally 8080).")
                    Text("Disconnecting stops this app's requests. Tracking and recording on the module continue.")
                }
            }
            .navigationTitle("Connection")
            .toolbar { ToolbarItem(placement: .confirmationAction) { Button("Done") { dismiss() } } }
        }
    }

    private func connect() {
        guard let url = TrackingAPI.address(address) else { return }
        address = url.absoluteString
        model.connect(url)
        dismiss()
    }
}

private struct CameraView: View {
    @Bindable var model: DroneModel
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 18) {
                Picker("Camera feed", selection: $model.feed) {
                    ForEach(CameraFeed.allCases) { feed in Text(feed.title).tag(feed) }
                }.pickerStyle(.segmented)

                TimelineView(.periodic(from: .now, by: 1)) { context in
                    let label = model.feedLabel(at: context.date)
                    VStack(alignment: .leading, spacing: 10) {
                        HStack {
                            Text(model.feed.title).font(.title2.bold())
                            Spacer()
                            Text(label).foregroundStyle(label == "Live" ? .mint : .orange)
                        }
                        ZStack {
                            RoundedRectangle(cornerRadius: 12).fill(.black)
                            if let image = model.image {
                                Image(uiImage: image).resizable().scaledToFit()
                                    .opacity(label == "Live" || label == "Synthetic" ? 1 : 0.4)
                                    .accessibilityLabel("\(model.state?.isDemo == true ? "Synthetic" : "Module") \(model.feed.title) camera image, \(label)")
                            } else {
                                ContentUnavailableView("No image yet", systemImage: "camera", description: Text("\(model.feed.title) frames appear when the module provides them."))
                            }
                        }
                        .aspectRatio(4 / 3, contentMode: .fit)
                        .compositingGroup().clipShape(.rect(cornerRadius: 12))
                    }
                }
                Text(caption).font(.callout).foregroundStyle(.secondary)
                if let error = model.imageError { Text(error).font(.callout).foregroundStyle(.orange) }
            }.padding()
        }
    }

    private var caption: String {
        if model.state?.isDemo == true { return "SIMULATION · Generated test scene, not your camera." }
        switch model.feed {
        case .rgb: return "RGB and registered depth drive camera tracking."
        case .depth: return "Registered depth · colour scale 0–6 m."
        case .thermal:
            let range = model.state?.metrics.thermalRange?.map { $0.formatted(.number.precision(.fractionLength(1))) }.joined(separator: "–") ?? "—"
            return "Thermal · \(range) °C · independent view, not aligned with RGB."
        }
    }
}

private struct TrackingView: View {
    let model: DroneModel
    @State private var confirmsReset = false
    @State private var exportsMap = false
    @State private var document = MapDocument(data: Data())
    @State private var exportMessage: String?

    var body: some View {
        TimelineView(.periodic(from: .now, by: 1)) { context in
            Form {
                Section("Tracking") {
                    LabeledContent("Status", value: model.trackingLabel(at: context.date))
                    Text(model.isConnected(at: context.date) ? model.state?.reason ?? "Waiting for frames" : "Last scene retained. Connect to the module to receive current data.")
                        .font(.callout).foregroundStyle(.secondary)
                    if let state = model.state, state.frame > 0 {
                        LabeledContent("X · right", value: metres(state.position.x))
                        LabeledContent("Y · down", value: metres(state.position.y))
                        LabeledContent("Z · forward", value: metres(state.position.z))
                    }
                    Text("Metres from the starting camera position. The reference is not gravity aligned; local odometry drifts.")
                        .font(.caption).foregroundStyle(.secondary)
                }
                Section("Health") {
                    LabeledContent("Visual inliers", value: "\(model.state?.metrics.inliers ?? 0) / \(model.state?.metrics.matches ?? 0)")
                    LabeledContent("Frame processing", value: measurement(model.state?.metrics.processingMs, unit: "ms"))
                    LabeledContent("RGB/depth timing", value: measurement(model.state?.metrics.syncMs, unit: "ms"))
                    LabeledContent("Valid depth", value: measurement(model.state?.metrics.validDepthPercent, unit: "%"))
                    LabeledContent("Gyroscope", value: model.isConnected(at: context.date) ? model.state?.gyroLabel ?? "Waiting" : "Unavailable")
                    LabeledContent("Mapped points", value: (model.state?.points ?? 0).formatted())
                }
                Section("Map & recording") {
                    Button(model.state?.recording == true ? "Stop recording" : "Record sequence", systemImage: "record.circle") {
                        Task { await model.command("api/record") }
                    }
                    Button("Save 3D map", systemImage: "square.and.arrow.up") {
                        Task {
                            if let data = await model.exportMap() { document = MapDocument(data: data); exportsMap = true }
                        }
                    }
                    Button("Start a new map", systemImage: "arrow.counterclockwise", role: .destructive) { confirmsReset = true }
                    if model.busy { ProgressView("Contacting module…") }
                    if let state = model.state {
                        Text(state.recording ? "Recording · \(state.recordedFrames) / 300 frames" : "\(state.recordedFrames) frames saved on module")
                            .font(.callout).foregroundStyle(.secondary)
                    }
                    Text("Sequences stay on the module. Map export saves a coloured PLY file to Files.")
                        .font(.caption).foregroundStyle(.secondary)
                }
                .disabled(!model.isConnected(at: context.date) || model.busy)
                if let message = model.actionMessage { Section { Text(message) } }
                if let exportMessage { Section { Text(exportMessage) } }
            }
        }
        .alert("Start a new map?", isPresented: $confirmsReset) {
            Button("Start new map", role: .destructive) { Task { await model.command("api/reset") } }
            Button("Cancel", role: .cancel) {}
        } message: {
            Text("Clear the module's scene and use the next camera frame as the new origin? Save the map first if you need it.")
        }
        .fileExporter(isPresented: $exportsMap, document: document, contentType: MapDocument.type, defaultFilename: "scene.ply") { result in
            switch result {
            case .success: exportMessage = "Map saved."
            case .failure(let error): exportMessage = "Could not save map: \(error.localizedDescription)"
            }
        }
    }

    private func metres(_ value: Float) -> String { value.formatted(.number.precision(.fractionLength(3))) + " m" }
    private func measurement(_ value: Double?, unit: String) -> String {
        value.map { $0.formatted(.number.precision(.fractionLength(1))) + " " + unit } ?? "—"
    }
}

struct MapDocument: FileDocument {
    static let type = UTType(filenameExtension: "ply") ?? .data
    static var readableContentTypes: [UTType] { [type] }
    let data: Data
    init(data: Data) { self.data = data }
    init(configuration: ReadConfiguration) throws { data = configuration.file.regularFileContents ?? Data() }
    func fileWrapper(configuration: WriteConfiguration) throws -> FileWrapper { FileWrapper(regularFileWithContents: data) }
}

#Preview { ContentView() }
