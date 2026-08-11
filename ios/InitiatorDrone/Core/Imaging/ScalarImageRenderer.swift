import Foundation

/// How a scalar image's value range is chosen before the colour ramp is applied.
public enum ScalarRangeMode: Equatable, Sendable, Codable {
    /// Use the fixed `low`/`high` bounds from settings.
    case fixed
    /// Recompute bounds from each frame's own percentile range.
    case autoPerFrame
    /// Recompute bounds from the frame, then ease toward them so the display
    /// does not flicker when a hot or near object enters the scene.
    case autoSmoothed
}

/// Colour-map configuration for a scalar (single-channel) image renderer.
public struct ScalarColorMapSettings: Equatable, Sendable, Codable {
    public var style: ColorRampStyle
    /// Value mapped to the bottom of the ramp, in the image's unit.
    public var low: Double
    /// Value mapped to the top of the ramp, in the image's unit.
    public var high: Double
    public var rangeMode: ScalarRangeMode
    /// Invert the ramp, so near/hot reads as the top colour instead.
    public var reversed: Bool
    /// Alpha applied to valid samples, `0...1`.
    public var opacity: Double
    /// Samples outside `low...high` are clamped to the end colours when true,
    /// and drawn transparent when false.
    public var clampOutOfRange: Bool

    public init(
        style: ColorRampStyle,
        low: Double,
        high: Double,
        rangeMode: ScalarRangeMode = .fixed,
        reversed: Bool = false,
        opacity: Double = 1.0,
        clampOutOfRange: Bool = true
    ) {
        self.style = style
        self.low = low
        self.high = high
        self.rangeMode = rangeMode
        self.reversed = reversed
        self.opacity = opacity
        self.clampOutOfRange = clampOutOfRange
    }

    /// Depth defaults: 0.3 m to 6 m covers an indoor Gemini E range, with near
    /// surfaces at the warm end of Turbo.
    public static let depthDefault = ScalarColorMapSettings(
        style: .turbo,
        low: 0.3,
        high: 6.0,
        rangeMode: .fixed,
        reversed: true,
        opacity: 1.0,
        clampOutOfRange: true
    )
}

/// Renders a `ScalarImage` to straight RGBA using a colour ramp.
///
/// This is a class rather than a free function because the auto-range modes
/// carry state between frames, and because it reuses its output buffer instead
/// of allocating one per frame — the difference matters over a 30-minute
/// stream.
public final class ScalarImageRenderer {
    public var settings: ScalarColorMapSettings

    /// Smoothing factor for `.autoSmoothed`, per frame.
    public var smoothing: Double = 0.15

    /// The bounds actually used for the most recent frame, which the UI shows
    /// on the colour legend.
    public private(set) var effectiveRange: (low: Double, high: Double)

    private var rgbaBuffer: [UInt8] = []

    public init(settings: ScalarColorMapSettings) {
        self.settings = settings
        self.effectiveRange = (settings.low, settings.high)
    }

    /// Resets smoothed state, so switching topics does not carry a stale range.
    public func resetRange() {
        effectiveRange = (settings.low, settings.high)
    }

    public func render(_ image: ScalarImage) -> ColorImage {
        let bounds = resolveRange(for: image)
        effectiveRange = bounds

        let ramp = settings.style.ramp
        let alpha = UInt8(max(0, min(255, (settings.opacity * 255).rounded())))
        let span = bounds.high - bounds.low
        // A degenerate range would divide by zero; paint the whole frame at the
        // middle of the ramp instead of producing NaNs.
        let inverseSpan = abs(span) > 1e-9 ? 1.0 / span : 0.0
        let reversed = settings.reversed
        let clamp = settings.clampOutOfRange

        let pixelCount = image.width * image.height
        if rgbaBuffer.count != pixelCount * 4 {
            rgbaBuffer = [UInt8](repeating: 0, count: pixelCount * 4)
        }

        rgbaBuffer.withUnsafeMutableBufferPointer { output in
            ramp.lookupTable.withUnsafeBufferPointer { table in
                for index in 0..<pixelCount {
                    let value = Double(image.values[index])
                    let destination = index * 4

                    guard value.isFinite else {
                        // Invalid sample: fully transparent so the camera or the
                        // other layer shows through instead of a fake reading.
                        output[destination] = 0
                        output[destination + 1] = 0
                        output[destination + 2] = 0
                        output[destination + 3] = 0
                        continue
                    }

                    var normalized = inverseSpan == 0 ? 0.5 : (value - bounds.low) * inverseSpan
                    if !clamp && (normalized < 0 || normalized > 1) {
                        output[destination] = 0
                        output[destination + 1] = 0
                        output[destination + 2] = 0
                        output[destination + 3] = 0
                        continue
                    }
                    normalized = min(max(normalized, 0.0), 1.0)
                    if reversed { normalized = 1.0 - normalized }

                    let color = table[Int(normalized * 255.0)]
                    output[destination] = color.red
                    output[destination + 1] = color.green
                    output[destination + 2] = color.blue
                    output[destination + 3] = alpha
                }
            }
        }

        return ColorImage(width: image.width, height: image.height, rgba: rgbaBuffer)
    }

    private func resolveRange(for image: ScalarImage) -> (low: Double, high: Double) {
        switch settings.rangeMode {
        case .fixed:
            return (settings.low, settings.high)

        case .autoPerFrame:
            guard let range = image.finitePercentileRange() else {
                return (settings.low, settings.high)
            }
            return widen(low: Double(range.min), high: Double(range.max))

        case .autoSmoothed:
            guard let range = image.finitePercentileRange() else {
                return effectiveRange
            }
            let target = widen(low: Double(range.min), high: Double(range.max))
            let factor = min(max(smoothing, 0.0), 1.0)
            return (
                low: effectiveRange.low + (target.low - effectiveRange.low) * factor,
                high: effectiveRange.high + (target.high - effectiveRange.high) * factor
            )
        }
    }

    /// Guards against a frame whose samples are all but identical, which would
    /// otherwise amplify sensor noise into full-scale colour.
    private func widen(low: Double, high: Double) -> (low: Double, high: Double) {
        let minimumSpan = 0.5
        guard high - low < minimumSpan else { return (low, high) }
        let centre = (low + high) * 0.5
        return (centre - minimumSpan / 2, centre + minimumSpan / 2)
    }
}
