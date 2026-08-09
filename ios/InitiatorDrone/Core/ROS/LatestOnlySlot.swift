import Foundation

/// A one-deep mailbox that always holds the newest value and drops the rest.
///
/// This is the app's answer to "never allow image messages to build a backlog".
/// A `DispatchQueue.async` per frame is an unbounded queue: if decoding takes
/// longer than the inter-frame interval — which it will, the moment the phone
/// thermally throttles or Wi-Fi delivers a burst after a stall — the queue
/// grows without limit and each held frame pins hundreds of kilobytes. Here a
/// producer that arrives while a value is already pending simply replaces it,
/// so memory is bounded by one frame no matter how far behind the consumer
/// falls.
///
/// `droppedCount` is surfaced in diagnostics: a steadily climbing number means
/// the throttle rate is set too high for this phone and link.
public final class LatestOnlySlot<Value>: @unchecked Sendable {
    private let lock = NSLock()
    private var pending: Value?
    private var isDraining = false
    private var dropped = 0
    private var accepted = 0

    public init() {}

    /// Stores a value, replacing any value not yet taken.
    /// - Returns: `true` when the caller should schedule a drain, i.e. no
    ///   consumer is already running.
    @discardableResult
    public func offer(_ value: Value) -> Bool {
        lock.lock()
        defer { lock.unlock() }

        if pending != nil { dropped += 1 }
        pending = value
        accepted += 1

        if isDraining { return false }
        isDraining = true
        return true
    }

    /// Takes the pending value, if any. Returns `nil` when the slot is empty,
    /// at which point the drain loop is marked finished so the next `offer`
    /// starts a new one.
    public func take() -> Value? {
        lock.lock()
        defer { lock.unlock() }

        if let value = pending {
            pending = nil
            return value
        }
        isDraining = false
        return nil
    }

    /// Clears the slot and ends any drain loop. Used on disconnect so a frame
    /// captured before a reconnect cannot be rendered afterwards.
    public func reset() {
        lock.lock()
        defer { lock.unlock() }
        pending = nil
        isDraining = false
    }

    public var statistics: (accepted: Int, dropped: Int) {
        lock.lock()
        defer { lock.unlock() }
        return (accepted, dropped)
    }

    public func resetStatistics() {
        lock.lock()
        defer { lock.unlock() }
        dropped = 0
        accepted = 0
    }
}
