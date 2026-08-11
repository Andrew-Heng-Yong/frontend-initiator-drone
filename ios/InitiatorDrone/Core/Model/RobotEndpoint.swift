import Foundation

/// Where a robot lives on the network.
///
/// Stored rather than derived so the operator can keep several robots, and so a
/// field setup with a non-default port does not need a rebuild.
public struct RobotEndpoint: Equatable, Codable, Identifiable, Sendable {
    public var id: UUID
    /// A friendly name shown in the saved list.
    public var name: String
    /// Hostname, IPv4 address, or `.local` name.
    public var host: String
    /// Port of the Node dashboard's HTTP API.
    public var dashboardPort: Int
    /// Port of the rosbridge WebSocket.
    public var rosbridgePort: Int
    /// Use TLS for both. Off by default: the robot serves plain HTTP on the
    /// local network.
    public var useTLS: Bool
    public var lastUsed: Date

    public init(
        id: UUID = UUID(),
        name: String = "",
        host: String,
        dashboardPort: Int = 4173,
        rosbridgePort: Int = 9090,
        useTLS: Bool = false,
        lastUsed: Date = Date()
    ) {
        self.id = id
        self.name = name
        self.host = host
        self.dashboardPort = dashboardPort
        self.rosbridgePort = rosbridgePort
        self.useTLS = useTLS
        self.lastUsed = lastUsed
    }

    public static let defaultDashboardPort = 4173
    public static let defaultRosbridgePort = 9090

    public var displayName: String {
        name.trimmingCharacters(in: .whitespaces).isEmpty ? host : name
    }

    /// Base URL of the dashboard HTTP server, e.g. `http://192.168.1.42:4173`.
    public var dashboardBaseURL: URL? {
        var components = URLComponents()
        components.scheme = useTLS ? "https" : "http"
        components.host = normalizedHost
        components.port = dashboardPort
        return components.url
    }

    /// rosbridge WebSocket URL, e.g. `ws://192.168.1.42:9090`.
    public var rosbridgeURL: URL? {
        var components = URLComponents()
        components.scheme = useTLS ? "wss" : "ws"
        components.host = normalizedHost
        components.port = rosbridgePort
        components.path = "/"
        return components.url
    }

    /// Strips anything the operator may have pasted around the address — a
    /// scheme, a trailing slash, a port — so `http://10.0.0.5:4173/` and
    /// `10.0.0.5` both work.
    public var normalizedHost: String {
        var value = host.trimmingCharacters(in: .whitespacesAndNewlines)
        for prefix in ["http://", "https://", "ws://", "wss://"] where value.lowercased().hasPrefix(prefix) {
            value = String(value.dropFirst(prefix.count))
        }
        if let slash = value.firstIndex(of: "/") {
            value = String(value[value.startIndex..<slash])
        }
        // A bare IPv6 literal has colons of its own, so only strip a port when
        // there is exactly one colon and what follows is numeric.
        let parts = value.split(separator: ":")
        if parts.count == 2, Int(parts[1]) != nil {
            value = String(parts[0])
        }
        return value
    }

    /// Whether the address looks usable. Deliberately permissive: it rejects
    /// empty and obviously malformed input, and leaves real resolution to the
    /// connection test.
    public var isValid: Bool {
        let value = normalizedHost
        guard !value.isEmpty, value.count <= 253 else { return false }
        guard !value.contains(" ") else { return false }
        guard dashboardPort > 0, dashboardPort <= 65535 else { return false }
        guard rosbridgePort > 0, rosbridgePort <= 65535 else { return false }
        let allowed = CharacterSet.alphanumerics.union(CharacterSet(charactersIn: ".-_:[]"))
        return value.unicodeScalars.allSatisfy { allowed.contains($0) }
    }

    public static let placeholder = RobotEndpoint(name: "Robot", host: "192.168.1.100")
}
