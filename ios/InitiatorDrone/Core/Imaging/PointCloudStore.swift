import Foundation

/// Thread-safe hand-off of the newest point cloud from the depth worker to the
/// SceneKit render thread.
///
/// The same problem `OdometrySampler` solves: the producer runs on the image
/// pipeline's queue, the consumer is the render callback, and neither can hop
/// to the main actor without adding latency. One cloud is held at a time —
/// older ones have nothing to offer a live view.
///
/// The generation counter is what keeps the render loop cheap. Rebuilding
/// SceneKit geometry allocates, so at 60 fps against a 10 Hz depth stream the
/// renderer would redo the same work six times per frame. Comparing generations
/// makes that a single integer check.
public final class PointCloudStore: @unchecked Sendable {
    private let lock = NSLock()
    private var cloud: PointCloudBuffer = .empty
    private var generation: UInt64 = 0

    public init() {}

    /// Replaces the held cloud and bumps the generation.
    public func store(_ newCloud: PointCloudBuffer) {
        lock.lock()
        defer { lock.unlock() }
        cloud = newCloud
        generation &+= 1
    }

    /// Returns the cloud only when it has changed since `knownGeneration`.
    public func take(ifNewerThan knownGeneration: UInt64) -> (cloud: PointCloudBuffer, generation: UInt64)? {
        lock.lock()
        defer { lock.unlock() }
        guard generation != knownGeneration else { return nil }
        return (cloud, generation)
    }

    public var currentGeneration: UInt64 {
        lock.lock()
        defer { lock.unlock() }
        return generation
    }

    public var pointCount: Int {
        lock.lock()
        defer { lock.unlock() }
        return cloud.count
    }

    /// Clears the cloud on disconnect, so points captured before a reconnect
    /// cannot hang in the air afterwards.
    public func reset() {
        lock.lock()
        defer { lock.unlock() }
        cloud = .empty
        generation &+= 1
    }
}
