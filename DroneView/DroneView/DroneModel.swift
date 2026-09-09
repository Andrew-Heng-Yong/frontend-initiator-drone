import Foundation
import Observation
import UIKit

@MainActor @Observable
final class DroneModel {
    private(set) var endpoint: URL?
    private(set) var connectionID = UUID()
    private(set) var state: TrackingState?
    private(set) var lastUpdate: Date?
    private(set) var cloud = PointCloud.empty
    private(set) var cloudRevision = 0
    private(set) var image: UIImage?
    private(set) var imageReceived: Date?
    private(set) var connectionError: String?
    private(set) var sceneError: String?
    private(set) var imageError: String?
    private(set) var actionMessage: String?
    private(set) var busy = false
    var feed: CameraFeed = .rgb {
        didSet { image = nil; imageReceived = nil; imageError = nil }
    }
    @ObservationIgnored private var fetchedMapVersion: Int?
    @ObservationIgnored private let session: URLSession

    init(session: URLSession = .shared) { self.session = session }

    func connect(_ url: URL) {
        endpoint = url
        connectionID = UUID()
        state = nil
        lastUpdate = nil
        cloud = .empty
        cloudRevision += 1
        fetchedMapVersion = nil
        image = nil
        imageReceived = nil
        connectionError = nil
        sceneError = nil
        imageError = nil
        actionMessage = nil
        busy = false
    }

    func disconnect() {
        endpoint = nil
        connectionID = UUID()
        lastUpdate = nil
        connectionError = nil
    }

    func isConnected(at now: Date = .now) -> Bool {
        endpoint != nil && lastUpdate.map { now.timeIntervalSince($0) < 3 } == true
    }

    func trackingLabel(at now: Date) -> String {
        guard endpoint != nil else { return "Disconnected" }
        guard let state else { return connectionError == nil ? "Connecting" : "Unavailable" }
        guard isConnected(at: now) else { return "Disconnected · retrying" }
        if let age = state.age, age + now.timeIntervalSince(lastUpdate ?? now) > 2 { return "Stale" }
        return state.status.capitalized
    }

    func feedLabel(at now: Date) -> String {
        guard isConnected(at: now) else { return "Disconnected" }
        guard let age = state?.images[feed.rawValue] else { return "Waiting" }
        if age + now.timeIntervalSince(lastUpdate ?? now) >= 2 { return "Stale" }
        if imageError != nil { return "Unavailable" }
        guard image != nil, let received = imageReceived else { return "Loading" }
        guard now.timeIntervalSince(received) < 3 else { return "Stale" }
        return state?.isDemo == true ? "Synthetic" : "Live"
    }

    // Three sequential loops: a slow JPEG/map cannot hold up status or queue frames.
    // The view's task owns cancellation, including backgrounding and reconnects.
    func run() async {
        guard let endpoint else { return }
        let api = TrackingAPI(origin: endpoint, session: session)
        let id = connectionID
        await withTaskGroup(of: Void.self) { group in
            group.addTask { await self.pollState(api, id: id) }
            group.addTask { await self.pollCloud(api, id: id) }
            group.addTask { await self.pollImage(api, id: id) }
        }
    }

    private func pollState(_ api: TrackingAPI, id: UUID) async {
        while !Task.isCancelled && id == connectionID {
            do {
                let next = try await api.state()
                guard !Task.isCancelled, id == connectionID else { return }
                // A restarted server can reuse version numbers; frame rollback invalidates the cache.
                if let state, next.frame < state.frame { fetchedMapVersion = nil }
                state = next
                lastUpdate = .now
                connectionError = nil
            } catch {
                guard !Task.isCancelled, id == connectionID else { return }
                connectionError = error.localizedDescription
            }
            do { try await Task.sleep(for: .milliseconds(200)) } catch { return }
        }
    }

    private func pollCloud(_ api: TrackingAPI, id: UUID) async {
        while !Task.isCancelled && id == connectionID {
            if let version = state?.mapVersion, version != fetchedMapVersion, isConnected() {
                do {
                    let next = try await api.cloud()
                    guard !Task.isCancelled, id == connectionID else { return }
                    // Retry if reset/restart happened during the request.
                    if state?.mapVersion == version {
                        cloud = next
                        cloudRevision += 1
                        fetchedMapVersion = version
                        sceneError = nil
                    }
                } catch {
                    guard !Task.isCancelled, id == connectionID else { return }
                    sceneError = error.localizedDescription
                }
            }
            do { try await Task.sleep(for: .seconds(1)) } catch { return }
        }
    }

    private func pollImage(_ api: TrackingAPI, id: UUID) async {
        while !Task.isCancelled && id == connectionID {
            let requestedFeed = feed
            if state?.images[requestedFeed.rawValue] != nil, isConnected() {
                do {
                    let data = try await api.data("api/image/" + requestedFeed.rawValue, limit: 4_000_000)
                    guard let decoded = UIImage(data: data) else { throw APIError.invalidData }
                    guard !Task.isCancelled, id == connectionID else { return }
                    if feed == requestedFeed {
                        image = decoded
                        imageReceived = .now
                        imageError = nil
                    }
                } catch {
                    guard !Task.isCancelled, id == connectionID else { return }
                    if feed == requestedFeed { imageError = error.localizedDescription }
                }
            }
            do { try await Task.sleep(for: .milliseconds(200)) } catch { return }
        }
    }

    func command(_ path: String) async {
        guard let endpoint, isConnected(), !busy else { return }
        let id = connectionID
        busy = true
        actionMessage = nil
        defer { if id == connectionID { busy = false } }
        do {
            _ = try await TrackingAPI(origin: endpoint, session: session).data(path, post: true)
            guard id == connectionID else { return }
            actionMessage = path == "api/reset" ? "New map requested. Waiting for the next camera frame." : "Recording change requested."
            if path == "api/reset" { fetchedMapVersion = nil }
        } catch {
            guard id == connectionID else { return }
            // Record is a toggle: never retry an ambiguous response automatically.
            actionMessage = "\(error.localizedDescription) Check the module's status before trying again."
        }
    }

    func exportMap() async -> Data? {
        guard let endpoint, isConnected(), !busy else { return nil }
        let id = connectionID
        busy = true
        defer { if id == connectionID { busy = false } }
        do {
            let data = try await TrackingAPI(origin: endpoint, session: session).data("api/map.ply", limit: 16_000_000)
            guard data.starts(with: Data("ply\n".utf8)) else { throw APIError.invalidData }
            guard id == connectionID else { return nil }
            return data
        } catch {
            if id == connectionID { actionMessage = "Map export failed: \(error.localizedDescription)" }
            return nil
        }
    }
}
