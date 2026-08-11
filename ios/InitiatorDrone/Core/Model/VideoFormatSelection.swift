import Foundation

/// An ARKit video format reduced to the properties that decide how much of the
/// world it shows.
///
/// Kept free of ARKit so the choice below is a pure function that runs in the
/// headless test suite; `ARSessionController` maps
/// `ARConfiguration.VideoFormat` onto it.
public struct VideoFormatCandidate: Equatable, Sendable {
    public var width: Int
    public var height: Int
    public var framesPerSecond: Int
    /// `captureDeviceType == .builtInUltraWideCamera`.
    public var isUltraWide: Bool

    public init(width: Int, height: Int, framesPerSecond: Int, isUltraWide: Bool) {
        self.width = width
        self.height = height
        self.framesPerSecond = framesPerSecond
        self.isUltraWide = isUltraWide
    }

    /// Frame height over width. Higher means more of the scene vertically.
    public var aspectRatio: Double {
        width > 0 ? Double(height) / Double(width) : 0
    }
}

/// Picks the ARKit video format that shows the most of the room.
public enum VideoFormatSelection {

    /// Index of the widest-field-of-view format, or `nil` if there are none.
    ///
    /// Two rules, in order:
    ///
    /// 1. **An ultra-wide format wins outright.** Roughly double the field of
    ///    view beats anything a crop of the wide lens can offer. In practice no
    ///    iPhone offers one to `ARWorldTrackingConfiguration` — ARKit uses the
    ///    ultra-wide internally for tracking but does not publish it as a video
    ///    format — so this branch is a no-op on current hardware and exists so
    ///    the app takes it on any device that ever does.
    ///
    /// 2. **Otherwise the tallest frame.** Every format from one lens is read
    ///    from the same sensor: 4:3 is the full readout, and the 16:9 formats
    ///    are that same image with the top and bottom cropped away. Horizontal
    ///    coverage is identical, so the taller aspect ratio is strictly more of
    ///    the scene, at no cost.
    ///
    /// Ties go to whichever ARKit listed first, which is its own recommendation
    /// and therefore the safer resolution and frame rate.
    public static func widestFieldOfView(among candidates: [VideoFormatCandidate]) -> Int? {
        guard !candidates.isEmpty else { return nil }

        let ultraWide = candidates.indices.filter { candidates[$0].isUltraWide }
        let pool = ultraWide.isEmpty ? Array(candidates.indices) : ultraWide

        return pool.min { left, right in
            let difference = candidates[left].aspectRatio - candidates[right].aspectRatio
            // Formats quantise to a handful of exact ratios, so anything inside
            // this tolerance is the same shape and should fall through to
            // ARKit's ordering rather than to floating-point noise.
            if abs(difference) > 1e-6 { return difference > 0 }
            return left < right
        }
    }
}
