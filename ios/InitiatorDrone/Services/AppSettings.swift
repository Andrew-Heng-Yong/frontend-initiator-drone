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
    /// Deproject the depth frame into a 3D point cloud in the AR scene.
    public var pointCloud: PointCloudSettings
    /// Where the depth camera is mounted relative to `base_link`. Drives both
    /// the point cloud and the frustum.
    public var cameraExtrinsics: CameraExtrinsics
    /// AprilTags mounted on the robot, with their sizes and offsets. Empty
    /// until the operator measures them, which is why tag relocalisation
    /// reports "no tags configured" rather than silently doing nothing.
    public var aprilTags: [AprilTagMount]
    /// How willing the app is to move the robot on a tag sighting.
    public var tagLocalization: TagLocalizationSettings
    /// In fixtures mode, hold the robot completely still instead of letting it
    /// turn on the spot.
    public var fixtureRobotIsStatic: Bool

    public init(
        depthColorMap: ScalarColorMapSettings = .depthDefault,
        compression: RosbridgeCompression = .none,
        imageThrottleMilliseconds: Int = 66,
        odometryStalenessThreshold: Double = 0.5,
        odometryExtrapolationLimit: Double = 0.0,
        showsCameraFrustum: Bool = true,
        showsRobotTrail: Bool = true,
        pointCloud: PointCloudSettings = .default,
        cameraExtrinsics: CameraExtrinsics = .identity,
        aprilTags: [AprilTagMount] = [],
        tagLocalization: TagLocalizationSettings = .default,
        fixtureRobotIsStatic: Bool = false
    ) {
        self.depthColorMap = depthColorMap
        self.compression = compression
        self.imageThrottleMilliseconds = imageThrottleMilliseconds
        self.odometryStalenessThreshold = odometryStalenessThreshold
        self.odometryExtrapolationLimit = odometryExtrapolationLimit
        self.showsCameraFrustum = showsCameraFrustum
        self.showsRobotTrail = showsRobotTrail
        self.pointCloud = pointCloud
        self.cameraExtrinsics = cameraExtrinsics
        self.aprilTags = aprilTags
        self.tagLocalization = tagLocalization
        self.fixtureRobotIsStatic = fixtureRobotIsStatic
    }

    public func depthInterpretation(for encoding: ROSImageEncoding) -> ScalarInterpretation {
        .depthDefault(for: encoding)
    }

    /// Sizes of every usable tag, keyed by ID — what the detector needs.
    ///
    /// Invalid and switched-off mounts are filtered out here rather than in the
    /// detector, so a tag that is not configured to be looked for is never even
    /// decoded, let alone posed.
    public var tagSizesByID: [Int: Double] {
        var sizes: [Int: Double] = [:]
        for mount in aprilTags where mount.isEnabled && mount.isValid {
            sizes[mount.tagID] = mount.sizeMetres
        }
        return sizes
    }

    /// Whether tag relocalisation can do anything at all right now.
    public var isTagLocalizationUsable: Bool {
        tagLocalization.isEnabled && !tagSizesByID.isEmpty
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
    /// an add. The preference has since been removed altogether. `v4` adds the
    /// point-cloud settings and the fixtures static-robot flag — both genuine
    /// adds, so a `v3` blob would throw on decode and has to be abandoned.
    /// `v5` adds the camera mount extrinsics; `v6` adds the AprilTag mounts and
    /// the tag relocalisation policy.
    private static let storageKey = "com.initiatordrone.settings.v6"

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
