import Foundation
import simd

enum APIError: LocalizedError {
    case invalidAddress, invalidData, http(Int), oversized
    var errorDescription: String? {
        switch self {
        case .invalidAddress: "Enter an HTTP or HTTPS address with a host and optional port."
        case .invalidData: "The module returned invalid data. Check that Camera + Gyro is running."
        case .http(let code): "The module returned HTTP \(code)."
        case .oversized: "The module response exceeds the supported size."
        }
    }
}

enum CameraFeed: String, CaseIterable, Identifiable, Sendable {
    case rgb, depth, thermal
    var id: Self { self }
    var title: String { self == .rgb ? "RGB" : rawValue.capitalized }
}

struct TrackingState: Decodable, Sendable {
    let status: String
    let reason: String
    let mode: String
    let pose: [[Float]]
    let trajectory: [[Float]]
    let frame: Int
    let mapVersion: Int
    let points: Int
    let age: Double?
    let metrics: Metrics
    let images: [String: Double]
    let gyro: Gyro
    let recording: Bool
    let recordedFrames: Int

    struct Metrics: Decodable, Sendable {
        let inliers: Int?
        let matches: Int?
        let processingMs: Double?
        let gyroPrior: Bool?
        let thermalRange: [Double]?
        let syncMs: Double?
        let validDepthPercent: Double?
    }
    struct Gyro: Decodable, Sendable {
        let state: String?
        let calibrated: Bool?
        let fusionReady: Bool?
    }

    var isDemo: Bool { mode == "demo" }
    var position: SIMD3<Float> { SIMD3(pose[0][3], pose[1][3], pose[2][3]) }
    var gyroLabel: String {
        if metrics.gyroPrior == true { return "Assisting" }
        if gyro.fusionReady == true { return "Ready" }
        if gyro.calibrated == true { return "Calibrated · not fused" }
        return gyro.state?.capitalized ?? "Unavailable"
    }

    static func decode(_ data: Data) throws -> Self {
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        let state = try decoder.decode(Self.self, from: data)
        guard state.pose.count == 4, state.pose.allSatisfy({ $0.count == 4 && $0.allSatisfy(\.isFinite) }),
              state.trajectory.count <= 2000,
              state.trajectory.allSatisfy({ $0.count == 3 && $0.allSatisfy(\.isFinite) }),
              state.frame >= 0, state.mapVersion >= 0, (0...200_000).contains(state.points),
              state.age.map({ $0.isFinite && $0 >= 0 }) ?? true,
              state.images.values.allSatisfy({ $0.isFinite && $0 >= 0 }),
              (0...300).contains(state.recordedFrames) else { throw APIError.invalidData }
        return state
    }
}

struct PointCloud: Sendable {
    // Interleaved XYZ/RGB, six float32 values per vertex, ready for Metal.
    let vertices: [Float]
    let minimum: SIMD3<Float>
    let maximum: SIMD3<Float>
    var count: Int { vertices.count / 6 }
    static let empty = PointCloud(vertices: [], minimum: .zero, maximum: .zero)

    static func decode(_ data: Data) throws -> Self {
        guard data.count % 24 == 0, data.count <= 200_000 * 24 else { throw APIError.invalidData }
        if data.isEmpty { return .empty }
        var values = [Float]()
        values.reserveCapacity(data.count / 4)
        var low = SIMD3<Float>(repeating: .infinity)
        var high = SIMD3<Float>(repeating: -.infinity)
        try data.withUnsafeBytes { bytes in
            for offset in stride(from: 0, to: data.count, by: 4) {
                let bits = UInt32(littleEndian: bytes.loadUnaligned(fromByteOffset: offset, as: UInt32.self))
                var value = Float(bitPattern: bits)
                guard value.isFinite else { throw APIError.invalidData }
                let component = (offset / 4) % 6
                if component >= 3 {
                    guard (0...255).contains(value) else { throw APIError.invalidData }
                    value /= 255
                } else {
                    // Optical X right, Y down, Z forward -> display X right, Y up, Z back.
                    if component > 0 { value = -value }
                    low[component] = min(low[component], value)
                    high[component] = max(high[component], value)
                }
                values.append(value)
            }
        }
        return PointCloud(vertices: values, minimum: low, maximum: high)
    }
}

struct TrackingAPI: Sendable {
    let origin: URL
    let session: URLSession

    init(origin: URL, session: URLSession = .shared) {
        self.origin = origin
        self.session = session
    }

    static func address(_ input: String) -> URL? {
        let text = input.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !text.isEmpty, !text.contains(where: \.isWhitespace),
              var parts = URLComponents(string: text.contains("://") ? text : "http://" + text),
              ["http", "https"].contains(parts.scheme?.lowercased() ?? ""),
              let host = parts.host, !host.isEmpty,
              parts.user == nil, parts.password == nil,
              parts.path.isEmpty || parts.path == "/", parts.query == nil, parts.fragment == nil,
              parts.port.map({ (1...65535).contains($0) }) ?? true else { return nil }
        parts.path = "/"
        return parts.url
    }

    func data(_ path: String, post: Bool = false, limit: Int = 8_000_000) async throws -> Data {
        var request = URLRequest(url: origin.appendingPathComponent(path), cachePolicy: .reloadIgnoringLocalCacheData, timeoutInterval: 5)
        if post {
            request.httpMethod = "POST"
            request.setValue("application/json", forHTTPHeaderField: "Content-Type")
            request.httpBody = Data("{}".utf8)
        }
        let (data, response) = try await session.data(for: request)
        try Task.checkCancellation()
        guard let http = response as? HTTPURLResponse else { throw APIError.invalidData }
        guard (200...299).contains(http.statusCode) else { throw APIError.http(http.statusCode) }
        guard data.count <= limit else { throw APIError.oversized }
        return data
    }

    @concurrent func state() async throws -> TrackingState { try TrackingState.decode(await data("api/state", limit: 1_000_000)) }
    @concurrent func cloud() async throws -> PointCloud { try PointCloud.decode(await data("api/scene", limit: 4_800_000)) }
}
