import SwiftUI

@main
struct InitiatorDroneApp: App {
    @StateObject private var model = AppModel()

    var body: some Scene {
        WindowGroup {
            RootView()
                .environmentObject(model)
                .environmentObject(model.connection)
                .environmentObject(model.robots)
                .environmentObject(model.settings)
                .environmentObject(model.arSession)
                .environmentObject(model.alignment)
                .environmentObject(model.tagDetection)
                .environmentObject(model.recording)
                // The live view is a camera view; a dark chrome keeps the
                // instrument overlays readable against it.
                .preferredColorScheme(.dark)
        }
    }
}
