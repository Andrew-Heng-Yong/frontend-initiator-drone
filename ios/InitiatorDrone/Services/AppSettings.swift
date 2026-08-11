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

    public init(
        depthColorMap: ScalarColorMapSettings = .depthDefault,
        compression: RosbridgeCompression = .none,
        imageThrottleMilliseconds: Int = 66,
        odometryStalenessThreshold: Double = 0.5,
        odometryExtrapolationLimit: Double = 0.0,
        showsCameraFrustum: Bool = true,
        showsRobotTrail: Bool = true
    ) {
        self.depthColorMap = depthColorMap
        self.compression = compression
        self.imageThrottleMilliseconds = imageThrottleMilliseconds
        self.odometryStalenessThreshold = odometryStalenessThreshold
        self.odometryExtrapolationLimit = odometryExtrapolationLimit
        self.showsCameraFrustum = showsCameraFrustum
        self.showsRobotTrail = showsRobotTrail
    }

    public func depthInterpretation(for encoding: ROSImageEncoding) -> ScalarInterpretation {
        .depthDefault(for: encoding)
    }
}

/// Observable wrapper that reads and writes `AppSettings` to `UserDefaults`.
@MainActor
public final class SettingsStore: ObservableObject {
    /// Schema version for the stored blob. Bumping it abandons the old value
    /// rather than failing to decode it on every launch, which would look like
    /// the settings screen was broken.
    ///
    /// Only *added* properties force a bump: the synthesised decoder demands
    /// every one of them, so an older blob without it throws. Removals are free,
    /// because `JSONDecoder` never asks for keys this struct no longer has.
    ///
    /// `v2` dropped the thermal, blend and picture-in-picture settings and added
    /// an ultra-wide camera preference; `v3` renamed that preference, which is
    /// an add. The preference has since been removed altogether, leaving the
    /// current shape a strict subset of `v1` — so the suffix no longer marks a
    /// real incompatibility, and it stays only because changing it would reset
    /// everyone's settings again for nothing.
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
