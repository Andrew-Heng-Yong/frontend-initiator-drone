import SwiftUI

struct RootView: View {
    @EnvironmentObject private var model: AppModel

    var body: some View {
        TabView(selection: $model.selectedTab) {
            LiveView()
                .tabItem { Label("Live", systemImage: "camera.viewfinder") }
                .tag(AppModel.Tab.live)

            ConnectionView()
                .tabItem { Label("Robot", systemImage: "antenna.radiowaves.left.and.right") }
                .tag(AppModel.Tab.connection)

            DiagnosticsView()
                .tabItem { Label("Diagnostics", systemImage: "waveform.path.ecg") }
                .tag(AppModel.Tab.diagnostics)

            SettingsView()
                .tabItem { Label("Settings", systemImage: "slider.horizontal.3") }
                .tag(AppModel.Tab.settings)
        }
        .tint(.green)
    }
}
