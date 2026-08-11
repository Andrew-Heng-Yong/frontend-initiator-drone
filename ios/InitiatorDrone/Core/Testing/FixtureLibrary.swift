import Foundation

/// Loads the bundled rosbridge JSON fixtures.
///
/// The same files feed the unit tests and the offline mode, so a fixture that
/// the tests prove decodes correctly is also exactly what the app replays when
/// no drone is present.
///
/// Lookup order:
/// 1. `INITIATOR_FIXTURES_DIR`, so the headless test runner can point at the
///    source tree without an app bundle.
/// 2. The bundle that contains this type, which covers both the app and the
///    test target inside Xcode.
public enum FixtureLibrary {

    public enum Error: Swift.Error, CustomStringConvertible {
        case notFound(String)
        case unreadable(String, String)

        public var description: String {
            switch self {
            case .notFound(let name):
                return "Fixture '\(name)' was not found"
            case .unreadable(let name, let reason):
                return "Fixture '\(name)' could not be read: \(reason)"
            }
        }
    }

    /// Directory holding the fixture JSON, when one can be located.
    public static var directoryURL: URL? {
        if let path = ProcessInfo.processInfo.environment["INITIATOR_FIXTURES_DIR"] {
            return URL(fileURLWithPath: path, isDirectory: true)
        }
        #if SWIFT_PACKAGE
        return Bundle.module.url(forResource: "Fixtures", withExtension: nil)
        #else
        let bundle = Bundle(for: BundleToken.self)
        if let url = bundle.url(forResource: "Fixtures", withExtension: nil) {
            return url
        }
        return bundle.resourceURL
        #endif
    }

    public static func data(named name: String) throws -> Data {
        let filename = name.hasSuffix(".json") ? name : name + ".json"

        if let directory = directoryURL {
            let url = directory.appendingPathComponent(filename)
            if FileManager.default.fileExists(atPath: url.path) {
                do {
                    return try Data(contentsOf: url)
                } catch {
                    throw Error.unreadable(filename, error.localizedDescription)
                }
            }
        }

        // Fall back to a flat resource lookup, which is how the files land when
        // Xcode copies them without preserving the folder.
        let base = (filename as NSString).deletingPathExtension
        #if !SWIFT_PACKAGE
        if let url = Bundle(for: BundleToken.self).url(forResource: base, withExtension: "json") {
            return try Data(contentsOf: url)
        }
        #endif
        throw Error.notFound(filename)
    }

    /// A fixture parsed into the rosbridge value tree.
    public static func value(named name: String) throws -> ROSValue {
        try ROSValue.fromJSON(try data(named: name))
    }

    /// A fixture parsed as an inbound rosbridge frame.
    public static func incoming(named name: String) throws -> RosbridgeIncoming {
        guard let parsed = RosbridgeIncoming.parse(try value(named: name)) else {
            throw Error.unreadable(name, "not a rosbridge frame")
        }
        return parsed
    }

    /// The `msg` body of a `publish` fixture.
    public static func message(named name: String) throws -> ROSValue {
        guard case .publish(_, let message) = try incoming(named: name) else {
            throw Error.unreadable(name, "not a publish frame")
        }
        return message
    }

    /// Every fixture filename, for a test that walks all of them.
    public static let allNames = [
        "depth_16uc1",
        "depth_16uc1_bigendian",
        "depth_32fc1",
        "color_rgb8",
        "color_bgr8",
        "camera_info",
        "odometry",
        "vio_calibrated_true",
        "vio_visual_tracking_false",
        "imu",
        "status_error",
        "dashboard_state",
    ]
}

private final class BundleToken {}
