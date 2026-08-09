import Foundation

/// How the depth and thermal layers are presented in the live view.
///
/// Compositing itself happens at the view layer rather than in pixel buffers:
/// the two sensors publish at different rates and different resolutions, and
/// letting the GPU scale and alpha-blend two independently updating textures is
/// both cheaper and sharper than recombining them on the CPU every frame. The
/// per-layer sampling choice still matters and is set at the view — nearest for
/// depth so no surface is invented between samples, bilinear for the 32x24
/// thermal frame that is magnified twentyfold.
public enum ImageViewMode: String, CaseIterable, Codable, Sendable, Identifiable {
    case depthOnly
    case thermalOnly
    case blended
    case pictureInPicture

    public var id: String { rawValue }

    public var displayName: String {
        switch self {
        case .depthOnly: return "Depth"
        case .thermalOnly: return "Thermal"
        case .blended: return "Blend"
        case .pictureInPicture: return "PiP"
        }
    }

    public var systemImage: String {
        switch self {
        case .depthOnly: return "cube.transparent"
        case .thermalOnly: return "thermometer.medium"
        case .blended: return "circle.lefthalf.filled"
        case .pictureInPicture: return "rectangle.inset.bottomright.filled"
        }
    }

    public var needsDepth: Bool { self != .thermalOnly }
    public var needsThermal: Bool { self != .depthOnly }
}
