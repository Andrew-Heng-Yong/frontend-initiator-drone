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

    public init(width: Int, height: Int, framesPerSecond: Int) {
        self.width = width
        self.height = height
        self.framesPerSecond = framesPerSecond
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
    /// **The tallest frame wins.** Every format ARKit offers world tracking
    /// comes from the same lens and the same sensor: the 4:3 entries are the
    /// full readout, and each 16:9 entry is that image with the top and bottom
    /// cropped away. Horizontal coverage is identical either way, so the taller
    /// aspect ratio is strictly more of the scene at no cost.
    ///
    /// This is the whole of the field of view available to the app. ARKit does
    /// not offer the ultra-wide lens to world tracking on any current iPhone —
    /// it drives that camera itself for tracking but never publishes it as a
    /// selectable format — so there is no wider option to reach for.
    ///
    /// Ties go to whichever ARKit listed first, which is its own recommendation
    /// and therefore the safer resolution and frame rate.
    public static func widestFieldOfView(among candidates: [VideoFormatCandidate]) -> Int? {
        candidates.indices.min { left, right in
            let difference = candidates[left].aspectRatio - candidates[right].aspectRatio
            // Formats quantise to a handful of exact ratios, so anything inside
            // this tolerance is the same shape and should fall through to
            // ARKit's ordering rather than to floating-point noise.
            if abs(difference) > 1e-6 { return difference > 0 }
            return left < right
        }
    }
}
