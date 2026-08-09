import Foundation

/// Decides how long to wait before the next reconnection attempt.
///
/// Split out as a value type with no timers or sockets in it so the backoff
/// curve can be unit tested directly, which is the part that actually goes
/// wrong: a policy that hammers the robot every 100 ms after a Wi-Fi drop will
/// keep the link saturated exactly when it is weakest.
public struct ReconnectPolicy: Equatable, Sendable {
    /// Wait before the first retry.
    public var initialDelay: Double
    /// Ceiling on the wait.
    public var maximumDelay: Double
    /// Growth factor applied per consecutive failure.
    public var multiplier: Double
    /// Fraction of the delay applied as random jitter, `0...1`. Jitter keeps a
    /// phone and a laptop that dropped together from retrying in lockstep.
    public var jitterFraction: Double

    public init(
        initialDelay: Double = 0.5,
        maximumDelay: Double = 15.0,
        multiplier: Double = 1.8,
        jitterFraction: Double = 0.25
    ) {
        self.initialDelay = max(0.05, initialDelay)
        // Clamp against the already-clamped initial delay, not the raw
        // argument: a negative pair would otherwise leave the ceiling below the
        // floor and produce negative waits.
        self.maximumDelay = max(self.initialDelay, maximumDelay)
        self.multiplier = max(1.0, multiplier)
        self.jitterFraction = min(max(jitterFraction, 0.0), 1.0)
    }

    /// The un-jittered delay for a given attempt. Attempt 0 is the first retry
    /// after a successful connection dropped.
    public func baseDelay(forAttempt attempt: Int) -> Double {
        guard attempt > 0 else { return initialDelay }
        let scaled = initialDelay * pow(multiplier, Double(attempt))
        return min(scaled, maximumDelay)
    }

    /// The delay to actually wait, including jitter.
    ///
    /// - Parameter randomUnit: a value in `0...1`. Injected so tests are
    ///   deterministic; production passes `Double.random(in:)`.
    public func delay(forAttempt attempt: Int, randomUnit: Double = Double.random(in: 0...1)) -> Double {
        let base = baseDelay(forAttempt: attempt)
        guard jitterFraction > 0 else { return base }
        // Symmetric jitter around the base delay, clamped so it never goes
        // negative or exceeds the ceiling.
        let spread = base * jitterFraction
        let offset = (min(max(randomUnit, 0), 1) * 2.0 - 1.0) * spread
        return min(max(base + offset, 0.05), maximumDelay)
    }
}
