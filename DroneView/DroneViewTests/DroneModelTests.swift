import XCTest
import UIKit
@testable import DroneView

private final class ModuleProtocol: URLProtocol, @unchecked Sendable {
    override class func canInit(with request: URLRequest) -> Bool { true }
    override class func canonicalRequest(for request: URLRequest) -> URLRequest { request }
    override func stopLoading() {}
    override func startLoading() {
        let path = request.url!.path
        let data: Data
        switch path {
        case "/api/state":
            data = Data(#"{"status":"tracking","reason":"RGB-D tracking","mode":"demo","pose":[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],"trajectory":[],"frame":1,"map_version":1,"points":1,"age":0,"metrics":{"gyro_prior":true},"images":{"rgb":0,"depth":0,"thermal":0},"gyro":{"state":"ready"},"recording":false,"recorded_frames":0}"#.utf8)
        case "/api/scene":
            var bytes = Data()
            for value: Float in [0,0,1,255,0,0] {
                var bits = value.bitPattern.littleEndian
                withUnsafeBytes(of: &bits) { bytes.append(contentsOf: $0) }
            }
            data = bytes
        case "/api/map.ply": data = Data("ply\nformat ascii 1.0\nend_header\n".utf8)
        case "/api/reset", "/api/record":
            XCTAssertEqual(request.httpMethod, "POST")
            XCTAssertEqual(request.value(forHTTPHeaderField: "Content-Type"), "application/json")
            data = Data("{}".utf8)
        default:
            // A valid one-pixel PNG; the app accepts standard image payloads.
            data = Data(base64Encoded: "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+jRZkAAAAASUVORK5CYII=")!
        }
        client?.urlProtocol(self, didReceive: HTTPURLResponse(url: request.url!, statusCode: 200, httpVersion: nil, headerFields: nil)!, cacheStoragePolicy: .notAllowed)
        client?.urlProtocol(self, didLoad: data)
        client?.urlProtocolDidFinishLoading(self)
    }
}

final class DroneModelTests: XCTestCase {
    @MainActor func testPollingActionsFreshnessAndCancellation() async throws {
        let config = URLSessionConfiguration.ephemeral
        config.protocolClasses = [ModuleProtocol.self]
        let model = DroneModel(session: URLSession(configuration: config))
        model.connect(URL(string: "http://fixture.local:8080")!)
        let polling = Task { await model.run() }
        defer { polling.cancel() }
        for _ in 0..<60 {
            if model.state != nil && model.cloud.count == 1 && model.image != nil { break }
            try await Task.sleep(for: .milliseconds(100))
        }
        XCTAssertTrue(model.isConnected())
        XCTAssertEqual(model.cloud.vertices, [0,0,-1,1,0,0])
        XCTAssertNotNil(model.image)
        XCTAssertEqual(model.trackingLabel(at: .now), "Tracking")
        XCTAssertEqual(model.feedLabel(at: .now), "Synthetic")
        XCTAssertEqual(model.trackingLabel(at: Date().addingTimeInterval(10)), "Disconnected · retrying")
        model.feed = .thermal
        XCTAssertNil(model.image, "Do not display the previous feed while a new feed loads")
        for _ in 0..<30 {
            if model.image != nil { break }
            try await Task.sleep(for: .milliseconds(100))
        }
        XCTAssertNotNil(model.image)
        await model.command("api/record")
        XCTAssertFalse(model.busy)
        XCTAssertEqual(model.actionMessage, "Recording change requested.")
        await model.command("api/reset")
        XCTAssertTrue(model.actionMessage?.starts(with: "New map requested") == true)
        let ply = await model.exportMap()
        XCTAssertTrue(ply?.starts(with: Data("ply\n".utf8)) == true)
        model.disconnect()
        polling.cancel()
        await polling.value
        XCTAssertFalse(model.isConnected())
        XCTAssertEqual(model.trackingLabel(at: .now), "Disconnected")
        let revision = model.cloudRevision
        try await Task.sleep(for: .milliseconds(300))
        XCTAssertEqual(model.cloudRevision, revision)
        model.connect(URL(string: "http://different.local:8080")!)
        XCTAssertNil(model.state)
        XCTAssertNil(model.image)
        XCTAssertEqual(model.cloud.count, 0)
    }
}
