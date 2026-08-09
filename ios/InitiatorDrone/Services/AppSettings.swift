import Foundation
import Combine

/// User-visible settings that survive relaunch.
///
/// Persisted as one JSON blob in `UserDefaults` rather than as a dozen keys, so
/// adding a field never leaves a half-migrated state on someone's phone.
public struct AppSettings: Equatable, Codable, Sendable {
    public var depthColorMap: ScalarColorMapSettings
    public var thermalColorMap: ScalarColorMapSettings
    public var imageViewMode: ImageViewMode
    /// Overlay strength in `.blended` mode, `0...1`.
    public var blendOpacity: Double
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
    /// Thermal `mono16` scale factor, in degrees Celsius per count.
    public var thermalScale: Double
    public var thermalOffset: Double

    public init(
        depthColorMap: ScalarColorMapSettings = .depthDefault,
        thermalColorMap: ScalarColorMapSettings = .thermalDefault,
        imageViewMode: ImageViewMode = .depthOnly,
        blendOpacity: Double = 0.55,
        compression: RosbridgeCompression = .none,
        imageThrottleMilliseconds: Int = 66,
        odometryStalenessThreshold: Double = 0.5,
        odometryExtrapolationLimit: Double = 0.0,
        showsCameraFrustum: Bool = true,
        showsRobotTrail: Bool = true,
        thermalScale: Double = 0.01,
        thermalOffset: Double = 0.0
    ) {
        self.depthColorMap = depthColorMap
        self.thermalColorMap = thermalColorMap
        self.imageViewMode = imageViewMode
        self.blendOpacity = blendOpacity
        self.compression = compression
        self.imageThrottleMilliseconds = imageThrottleMilliseconds
        self.odometryStalenessThreshold = odometryStalenessThreshold
        self.odometryExtrapolationLimit = odometryExtrapolationLimit
        self.showsCameraFrustum = showsCameraFrustum
        self.showsRobotTrail = showsRobotTrail
        self.thermalScale = thermalScale
        self.thermalOffset = thermalOffset
    }

    /// The scalar interpretation to use for a thermal frame of a given
    /// encoding, honouring the user's scale and offset.
    public func thermalInterpretation(for encoding: ROSImageEncoding) -> ScalarInterpretation {
        switch encoding {
        case .float32Single:
            return .linear(scale: 1.0, offset: thermalOffset, unit: .celsius)
        case .uint16Single, .mono16:
            return .linear(scale: thermalScale, offset: thermalOffset, unit: .celsius)
        default:
            return .raw
        }
    }

    public func depthInterpretation(for encoding: ROSImageEncoding) -> ScalarInterpretation {
        .depthDefault(for: encoding)
    }
}

/// Observable wrapper that reads and writes `AppSettings` to `UserDefaults`.
@MainActor
public final class SettingsStore: ObservableObject {
    private static let storageKey = "com.initiatordrone.settings.v1"

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
