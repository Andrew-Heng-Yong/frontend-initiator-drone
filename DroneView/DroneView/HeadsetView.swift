import SwiftUI

struct HeadsetProfile: Codable, Equatable {
    // ponytail: an adjustable radial model, not measured MERGE optics. Replace the
    // defaults with a verified viewer profile after through-the-lens calibration.
    var scale = 1.0
    var lensSpacing = 0.44
    var verticalCenter = 0.5
    var distortion = 0.2

    var validated: Self {
        func bounded(_ value: Double, _ limits: ClosedRange<Double>, _ fallback: Double) -> Double {
            value.isFinite ? min(limits.upperBound, max(limits.lowerBound, value)) : fallback
        }
        return Self(scale: bounded(scale, 0.6...1.4, 1),
                    lensSpacing: bounded(lensSpacing, 0.35...0.6, 0.44),
                    verticalCenter: bounded(verticalCenter, 0.35...0.65, 0.5),
                    distortion: bounded(distortion, 0...0.6, 0.2))
    }
}

@MainActor @Observable final class HeadsetSettings {
    var active = false
    var showHUD = true
    var showGrid = false
    var profile: HeadsetProfile {
        didSet {
            if let data = try? JSONEncoder().encode(profile.validated) {
                defaults.set(data, forKey: "headset-optics-v1")
            }
        }
    }
    @ObservationIgnored private let defaults: UserDefaults

    init(defaults: UserDefaults = .standard) {
        self.defaults = defaults
        profile = defaults.data(forKey: "headset-optics-v1")
            .flatMap { try? JSONDecoder().decode(HeadsetProfile.self, from: $0) }?.validated ?? HeadsetProfile()
    }

    func enter(grid: Bool = false) { showGrid = grid; showHUD = true; active = true }

    // A stale or missing camera must never masquerade as live passthrough.
    static func cameraIsLive(running: Bool, age: Double?) -> Bool {
        running && age.map { $0.isFinite && $0 >= 0 && $0 < 0.3 } == true
    }
}

struct HeadsetSetupView: View {
    @Bindable var settings: HeadsetSettings
    let start: (Bool) -> Void
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationStack {
            Form {
                Section("MERGE phone headset") {
                    Text("Open the camera window and check that neither the rear camera nor LiDAR is covered. Seat the phone in landscape and adjust both lenses until the image is sharp.")
                    Text("Both eyes see the same phone camera image. This is monoscopic passthrough, without stereo depth. Remain seated; remove the headset if the view lags or feels uncomfortable.")
                    Text("For thermal AR, align both cameras in the normal AR view before inserting the phone. Passthrough also works while the Pi is disconnected.")
                    Text("Tap the left half to hide or show the display information. Tap the right half to realign thermal AR. Hold either half for one second to exit. The headset buttons work if they touch the screen.")
                }
                Section {
                    adjustment("Image size", value: $settings.profile.scale, range: 0.6...1.4)
                    adjustment("Lens spacing · fraction of screen width", value: $settings.profile.lensSpacing, range: 0.35...0.6)
                    adjustment("Vertical lens centre", value: $settings.profile.verticalCenter, range: 0.35...0.65)
                    adjustment("Radial correction", value: $settings.profile.distortion, range: 0...0.6)
                    Button("Reset optics") { settings.profile = HeadsetProfile() }
                } header: { Text("Optical calibration") } footer: {
                    Text("These are approximate starting values, not a measured MERGE preset. Preview the grid through the lenses, then remove the phone to adjust. Aim for straight lines, square cells and one comfortable central cross. These controls do not change thermal alignment.")
                }
                Section {
                    Button("Preview calibration grid", systemImage: "grid") { start(true) }
                    Button("Start passthrough", systemImage: "viewfinder") { start(false) }
                }
            }
            .navigationTitle("Headset passthrough")
            .toolbar { ToolbarItem(placement: .confirmationAction) { Button("Done") { dismiss() } } }
        }
    }

    private func adjustment(_ title: String, value: Binding<Double>, range: ClosedRange<Double>) -> some View {
        VStack(alignment: .leading) {
            Text("\(title): \(value.wrappedValue, format: .number.precision(.fractionLength(2)))")
            Slider(value: value, in: range).accessibilityLabel(title)
        }
    }
}

struct HeadsetView: View {
    let model: PhoneReconstruction
    @Bindable var settings: HeadsetSettings

    var body: some View {
        GeometryReader { geometry in
            ARMetalView(model: model, headset: settings)
                .contentShape(.rect)
                .gesture(LongPressGesture(minimumDuration: 1).exclusively(before: SpatialTapGesture()).onEnded { action in
                    switch action {
                    case .first: settings.active = false
                    case .second(let tap):
                        if tap.location.x < geometry.size.width / 2 { settings.showHUD.toggle() }
                        else if !settings.showGrid { model.realign() }
                    }
                })
                .accessibilityElement()
                .accessibilityLabel(settings.showGrid ? "Headset calibration grid" : "Headset passthrough. \(model.status)")
                .accessibilityAction(named: "Exit headset") { settings.active = false }
                .accessibilityAction(named: "Toggle display information") { settings.showHUD.toggle() }
                .accessibilityAction(named: "Realign thermal AR") { if !settings.showGrid { model.realign() } }
        }
        .background(.black)
        .overlay {
            if !model.renderingError.isEmpty {
                ZStack {
                    Color.black
                    HStack {
                        ForEach(0..<2) { _ in
                            Text("Display unavailable · remove headset\n\(model.renderingError)")
                                .font(.callout).multilineTextAlignment(.center).frame(maxWidth: .infinity)
                        }
                    }.padding()
                }.allowsHitTesting(false)
            }
        }
        .ignoresSafeArea()
        .statusBarHidden()
        .persistentSystemOverlays(.hidden)
        .onAppear { DroneOrientation.setHeadset(true) }
        .onDisappear { DroneOrientation.setHeadset(false) }
    }
}

@MainActor final class DroneOrientation: NSObject, UIApplicationDelegate {
    private static var mask: UIInterfaceOrientationMask = .allButUpsideDown
    private static var previous: UIInterfaceOrientationMask = .portrait

    func application(_ application: UIApplication, supportedInterfaceOrientationsFor window: UIWindow?) -> UIInterfaceOrientationMask {
        Self.mask
    }

    static func setHeadset(_ enabled: Bool) {
        guard let scene = UIApplication.shared.connectedScenes.compactMap({ $0 as? UIWindowScene })
            .first(where: { $0.activationState == .foregroundActive }) else { return }
        if enabled {
            switch scene.interfaceOrientation {
            case .landscapeLeft: previous = .landscapeLeft
            case .landscapeRight: previous = .landscapeRight
            default: previous = .portrait
            }
        }
        mask = enabled ? .landscape : .allButUpsideDown
        scene.windows.first(where: \.isKeyWindow)?.rootViewController?.setNeedsUpdateOfSupportedInterfaceOrientations()
        scene.requestGeometryUpdate(.iOS(interfaceOrientations: enabled ? .landscape : previous))
    }
}
