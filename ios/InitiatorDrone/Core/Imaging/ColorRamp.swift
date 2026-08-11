import Foundation

public struct RGBColor: Equatable, Sendable, Codable {
    public var red: UInt8
    public var green: UInt8
    public var blue: UInt8

    public init(_ red: UInt8, _ green: UInt8, _ blue: UInt8) {
        self.red = red
        self.green = green
        self.blue = blue
    }
}

/// A colour gradient defined by stops, baked into a 256-entry lookup table.
///
/// Baking matters: a cropped depth frame is a few hundred thousand pixels and
/// the live view redraws it many times a second, so the per-pixel cost has to
/// be one table index, not a gradient search.
public struct ColorRamp: Equatable, Sendable {
    public struct Stop: Equatable, Sendable {
        public var position: Double
        public var color: RGBColor

        public init(_ position: Double, _ color: RGBColor) {
            self.position = position
            self.color = color
        }
    }

    public let name: String
    public let stops: [Stop]
    /// 256 entries of RGB, indexed by the normalised sample value.
    public let lookupTable: [RGBColor]

    public init(name: String, stops: [Stop]) {
        precondition(stops.count >= 2, "A colour ramp needs at least two stops")
        let sorted = stops.sorted { $0.position < $1.position }
        self.name = name
        self.stops = sorted
        self.lookupTable = ColorRamp.bake(sorted)
    }

    private static func bake(_ stops: [Stop]) -> [RGBColor] {
        var table = [RGBColor](repeating: RGBColor(0, 0, 0), count: 256)
        for index in 0..<256 {
            let position = Double(index) / 255.0
            table[index] = sample(stops, at: position)
        }
        return table
    }

    private static func sample(_ stops: [Stop], at position: Double) -> RGBColor {
        guard let first = stops.first, let last = stops.last else { return RGBColor(0, 0, 0) }
        if position <= first.position { return first.color }
        if position >= last.position { return last.color }

        var upperIndex = stops.count - 1
        for (index, stop) in stops.enumerated() where stop.position >= position {
            upperIndex = index
            break
        }
        let upper = stops[upperIndex]
        let lower = stops[max(0, upperIndex - 1)]
        let span = upper.position - lower.position
        let t = span > 0 ? (position - lower.position) / span : 0

        func mix(_ a: UInt8, _ b: UInt8) -> UInt8 {
            let value = Double(a) + (Double(b) - Double(a)) * t
            return UInt8(max(0, min(255, value.rounded())))
        }
        return RGBColor(
            mix(lower.color.red, upper.color.red),
            mix(lower.color.green, upper.color.green),
            mix(lower.color.blue, upper.color.blue)
        )
    }

    /// Looks up a colour for an already-normalised value in `0...1`.
    @inline(__always)
    public func color(normalized value: Double) -> RGBColor {
        let clamped = min(max(value, 0.0), 1.0)
        return lookupTable[Int(clamped * 255.0)]
    }
}

/// The ramps offered in settings. Keeping them as an enum makes the choice
/// codable and survivable across launches.
public enum ColorRampStyle: String, CaseIterable, Codable, Sendable, Identifiable {
    case turbo
    case viridis
    case inferno
    case jet
    case ironbow
    case grayscale

    public var id: String { rawValue }

    public var displayName: String {
        switch self {
        case .turbo: return "Turbo"
        case .viridis: return "Viridis"
        case .inferno: return "Inferno"
        case .jet: return "Jet"
        case .ironbow: return "Ironbow"
        case .grayscale: return "Grayscale"
        }
    }

    public var ramp: ColorRamp { ColorRampStyle.cache[self] ?? ColorRampStyle.build(self) }

    /// Ramps are immutable and small, so they are baked once at first use.
    private static let cache: [ColorRampStyle: ColorRamp] = {
        var table: [ColorRampStyle: ColorRamp] = [:]
        for style in ColorRampStyle.allCases {
            table[style] = build(style)
        }
        return table
    }()

    private static func build(_ style: ColorRampStyle) -> ColorRamp {
        switch style {
        case .turbo:
            return ColorRamp(name: "Turbo", stops: [
                .init(0.000, RGBColor(48, 18, 59)),
                .init(0.125, RGBColor(70, 107, 227)),
                .init(0.250, RGBColor(33, 168, 243)),
                .init(0.375, RGBColor(32, 228, 178)),
                .init(0.500, RGBColor(114, 250, 98)),
                .init(0.625, RGBColor(194, 235, 50)),
                .init(0.750, RGBColor(246, 177, 42)),
                .init(0.875, RGBColor(238, 94, 17)),
                .init(1.000, RGBColor(122, 4, 3)),
            ])

        case .viridis:
            return ColorRamp(name: "Viridis", stops: [
                .init(0.00, RGBColor(68, 1, 84)),
                .init(0.25, RGBColor(59, 82, 139)),
                .init(0.50, RGBColor(33, 145, 140)),
                .init(0.75, RGBColor(94, 201, 98)),
                .init(1.00, RGBColor(253, 231, 37)),
            ])

        case .inferno:
            return ColorRamp(name: "Inferno", stops: [
                .init(0.00, RGBColor(0, 0, 4)),
                .init(0.25, RGBColor(87, 16, 110)),
                .init(0.50, RGBColor(188, 55, 84)),
                .init(0.75, RGBColor(249, 142, 9)),
                .init(1.00, RGBColor(252, 255, 164)),
            ])

        case .jet:
            return ColorRamp(name: "Jet", stops: [
                .init(0.000, RGBColor(0, 0, 131)),
                .init(0.125, RGBColor(0, 60, 170)),
                .init(0.375, RGBColor(5, 255, 255)),
                .init(0.625, RGBColor(255, 255, 0)),
                .init(0.875, RGBColor(250, 0, 0)),
                .init(1.000, RGBColor(128, 0, 0)),
            ])

        case .ironbow:
            return ColorRamp(name: "Ironbow", stops: [
                .init(0.00, RGBColor(0, 0, 0)),
                .init(0.15, RGBColor(26, 0, 64)),
                .init(0.30, RGBColor(86, 0, 120)),
                .init(0.45, RGBColor(150, 10, 110)),
                .init(0.60, RGBColor(208, 50, 60)),
                .init(0.75, RGBColor(245, 120, 10)),
                .init(0.90, RGBColor(255, 215, 60)),
                .init(1.00, RGBColor(255, 255, 255)),
            ])

        case .grayscale:
            return ColorRamp(name: "Grayscale", stops: [
                .init(0.0, RGBColor(0, 0, 0)),
                .init(1.0, RGBColor(255, 255, 255)),
            ])
        }
    }
}
