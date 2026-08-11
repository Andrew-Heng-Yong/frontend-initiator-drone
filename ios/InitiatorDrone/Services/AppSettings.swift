import Foundation
import Combine

/// User-visible settings that survive relaunch.
///
/// Persisted as one JSON blob in `UserDefaults` rather than as a dozen keys, so
/// adding a field never leaves a half-migrated state on someone's phone.
public struct AppSettings: Equatable, Codable, Sendable {
    public var depthColorMap: ScalarColorMapSettings
    public var compression: RosbridgeCompression
    /// Minimum milliseconds between image messages, requested from rosbridge.
    public var imageThrottleMilliseconds: Int
    /// Odometry older than this counts as stale.
    public var odometryStalenessThreshold: Double
    /// How far the twist may be integrated past the newest odometry sample when
    /// rendering, in seconds. Zero holds the last pose instead.
    public var odometryExtrapolationLimit: Double
    /// Show the robot's depth-camera frustum, sized from `CameraInfo`.
    public var showsCameraFrustum: Bool
    /// Draw a trail behind the robot marker.
    public var showsRobotTrail: Bool
    /// Run the AR session on the widest field of view the device offers rather
    /// than on ARKit's default format. Wider framing keeps the robot marker on
    /// screen from much closer, which is the normal way this app is held.
    public var prefersWidestFieldOfView: Bool

    public init(
        depthColorMap: ScalarColorMapSettings = .depthDefault,
        compression: RosbridgeCompression = .none,
        imageThrottleMilliseconds: Int = 66,
        odometryStalenessThreshold: Double = 0.5,
        odometryExtrapolationLimit: Double = 0.0,
        showsCameraFrustum: Bool = true,
        showsRobotTrail: Bool = true,
        prefersWidestFieldOfView: Bool = true
    ) {
        self.depthColorMap = depthColorMap
        self.compression = compression
        self.imageThrottleMilliseconds = imageThrottleMilliseconds
        self.odometryStalenessThreshold = odometryStalenessThreshold
        self.odometryExtrapolationLimit = odometryExtrapolationLimit
        self.showsCameraFrustum = showsCameraFrustum
        self.showsRobotTrail = showsRobotTrail
        self.prefersWidestFieldOfView = prefersWidestFieldOfView
    }

    public func depthInterpretation(for encoding: ROSImageEncoding) -> ScalarInterpretation {
        .depthDefault(for: encoding)
    }
}

/// Observable wrapper that reads and writes `AppSettings` to `UserDefaults`.
@MainActor
public final class SettingsStore: ObservableObject {
    /// Bumped from `v1` when the thermal, blend and picture-in-picture settings
    /// were removed. A `v1` blob cannot decode into the current shape, and
    /// silently falling back to defaults on every launch would look like the
    /// settings screen was broken; a new key resets once and then persists.
    private static let storageKey = "com.initiatordrone.settings.v3"

    @Published public var settings: AppSettings {
        didSet { persist() }
    }

    private let defaults: UserDefaults

    public init(defaults: UserDefaults = .standard) {
        self.defaults = defaults
        if let data = defaults.data(forKey: Self.storageKey),
           let decoded = try? JSONDecoder().decode(AppSettings.self, from: data) {
            self.settings = decoded
        } else {
            self.settings = AppSettings()
        }
    }

    private func persist() {
        guard let data = try? JSONEncoder().encode(settings) else { return }
        defaults.set(data, forKey: Self.storageKey)
    }

    public func resetToDefaults() {
        settings = AppSettings()
    }
}
