import Foundation
import simd

private func rejects(_ body: () throws -> Void) {
    do { try body(); fatalError("Expected rejection") } catch {}
}

private final class StubProtocol: URLProtocol, @unchecked Sendable {
    override class func canInit(with request: URLRequest) -> Bool { true }
    override class func canonicalRequest(for request: URLRequest) -> URLRequest { request }
    override func startLoading() {
        precondition(request.url?.path == "/api/record" || request.url?.path == "/api/reset")
        precondition(request.httpMethod == "POST")
        precondition(request.value(forHTTPHeaderField: "Content-Type") == "application/json")
        let response = HTTPURLResponse(url: request.url!, statusCode: request.url!.path == "/api/reset" ? 503 : 200, httpVersion: nil, headerFields: nil)!
        client?.urlProtocol(self, didReceive: response, cacheStoragePolicy: .notAllowed)
        client?.urlProtocol(self, didLoad: Data("{}".utf8))
        client?.urlProtocolDidFinishLoading(self)
    }
    override func stopLoading() {}
}

@main struct CoreChecks {
    @MainActor static func main() async throws {
        precondition(TrackingAPI.address(" 192.168.1.6:8080 ")?.absoluteString == "http://192.168.1.6:8080/")
        precondition(TrackingAPI.address("https://camera.local") != nil)
        for invalid in ["", "http://", "ws://camera:9090", "file:///tmp", "http://a b", "http://user:pass@camera", "http://camera/api/state", "http://camera?x=1", "http://camera#x", "http://camera:0", "http://camera:65536"] {
            precondition(TrackingAPI.address(invalid) == nil, invalid)
        }
        let sample = Data(#"{"status":"tracking","reason":"RGB-D tracking","mode":"demo","pose":[[1,0,0,1],[0,1,0,2],[0,0,1,3],[0,0,0,1]],"trajectory":[[1,2,3]],"frame":1,"map_version":2,"points":1,"age":0.1,"metrics":{"inliers":10,"matches":12,"gyro_prior":true,"processing_ms":12.5},"images":{"rgb":0.1},"gyro":{"state":"ready","calibrated":true,"fusion_ready":true},"recording":false,"recorded_frames":0}"#.utf8)
        let state = try TrackingState.decode(sample)
        precondition(state.position == SIMD3(1,2,3) && state.gyroLabel == "Assisting" && state.isDemo)
        precondition(state.metrics.processingMs == 12.5 && state.mapVersion == 2)
        var object = try JSONSerialization.jsonObject(with: sample) as! [String: Any]
        object["pose"] = [[1,2]]
        rejects { _ = try TrackingState.decode(JSONSerialization.data(withJSONObject: object)) }
        object = try JSONSerialization.jsonObject(with: sample) as! [String: Any]
        object["trajectory"] = [[1,2]]
        rejects { _ = try TrackingState.decode(JSONSerialization.data(withJSONObject: object)) }
        var bytes = Data()
        for value: Float in [1,2,3,255,128,0] {
            var bits = value.bitPattern.littleEndian
            withUnsafeBytes(of: &bits) { bytes.append(contentsOf: $0) }
        }
        let cloud = try PointCloud.decode(bytes)
        precondition(cloud.count == 1 && cloud.minimum == SIMD3(1,-2,-3))
        precondition(cloud.vertices[3] == 1 && abs(cloud.vertices[4] - 128.0/255) < 0.00001)
        let empty = try PointCloud.decode(Data())
        precondition(empty.count == 0)
        rejects { _ = try PointCloud.decode(bytes.dropLast()) }
        var bad = bytes
        var nan = Float.nan.bitPattern.littleEndian
        withUnsafeBytes(of: &nan) { bad.replaceSubrange(0..<4, with: $0) }
        rejects { _ = try PointCloud.decode(bad) }
        precondition(SceneCamera.displayPoint(SIMD3(1,2,3)) == SIMD3(1,-2,-3))
        let camera = SceneCamera()
        camera.fit(cloud, position: nil)
        let projected = camera.matrix(aspect: 1) * SIMD4(camera.target, 1)
        precondition(abs(projected.x) < 0.0001 && abs(projected.y) < 0.0001 && projected.z / projected.w > 0 && projected.z / projected.w < 1)
        camera.zoom(1e10); precondition(camera.distance == 0.15)
        camera.drag(x: 0, y: 100000); precondition(camera.pitch == 1.5)
        camera.pans = true; camera.followsCamera = true; camera.drag(x: 10, y: 0)
        precondition(!camera.followsCamera)
        let config = URLSessionConfiguration.ephemeral
        config.protocolClasses = [StubProtocol.self]
        let stub = TrackingAPI(origin: URL(string: "http://test.local:8080")!, session: URLSession(configuration: config))
        _ = try await stub.data("api/record", post: true)
        do { _ = try await stub.data("api/reset", post: true); fatalError("HTTP failure must be reported") }
        catch APIError.http(503) {}
        print("PASS: address and state validation, float32 wire format, axis conversion, camera projection/controls, POST contract and HTTP errors")
        if let input = CommandLine.arguments.dropFirst().first, let url = TrackingAPI.address(input) {
            let api = TrackingAPI(origin: url)
            let live = try await api.state()
            let cloud = try await api.cloud()
            precondition(cloud.count > 0)
            for feed in CameraFeed.allCases {
                let jpeg = try await api.data("api/image/" + feed.rawValue)
                precondition(jpeg.starts(with: [0xff, 0xd8]), "JPEG signature")
            }
            let ply = try await api.data("api/map.ply", limit: 16_000_000)
            precondition(ply.starts(with: Data("ply\n".utf8)))
            print("PASS: live \(live.status), \(cloud.count) points, RGB/depth/thermal JPEGs, PLY export (read only)")
        }
    }
}
