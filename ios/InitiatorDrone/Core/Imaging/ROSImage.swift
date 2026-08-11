import Foundation

/// A `sensor_msgs/msg/Image` after the transport layer has turned the payload
/// into bytes, but before any pixel interpretation.
public struct ROSImageMessage: Equatable, Sendable {
    /// `header.stamp`, in seconds since the ROS epoch.
    public var stamp: Double
    public var frameId: String
    public var width: Int
    public var height: Int
    /// The raw `encoding` string as published, e.g. `16UC1`.
    public var encoding: String
    public var isBigEndian: Bool
    /// Row stride in bytes. May exceed `width * bytesPerPixel` when the
    /// publisher pads rows, so rows must always be indexed through `step`.
    public var step: Int
    public var data: Data

    public init(
        stamp: Double,
        frameId: String,
        width: Int,
        height: Int,
        encoding: String,
        isBigEndian: Bool,
        step: Int,
        data: Data
    ) {
        self.stamp = stamp
        self.frameId = frameId
        self.width = width
        self.height = height
        self.encoding = encoding
        self.isBigEndian = isBigEndian
        self.step = step
        self.data = data
    }
}

/// The pixel layouts this app understands.
public enum ROSImageEncoding: Equatable, Sendable {
    case mono8
    case mono16
    case uint16Single    // 16UC1
    case float32Single   // 32FC1
    case rgb8
    case bgr8
    case rgba8
    case bgra8

    /// Parses a ROS encoding string, tolerating case and the OpenCV-style
    /// aliases that different drivers emit for the same layout.
    public init?(rosEncoding: String) {
        switch rosEncoding.lowercased().trimmingCharacters(in: .whitespaces) {
        case "mono8", "8uc1":
            self = .mono8
        case "mono16":
            self = .mono16
        case "16uc1":
            self = .uint16Single
        case "32fc1":
            self = .float32Single
        case "rgb8", "8uc3_rgb":
            self = .rgb8
        case "bgr8", "8uc3":
            self = .bgr8
        case "rgba8", "8uc4_rgba":
            self = .rgba8
        case "bgra8", "8uc4":
            self = .bgra8
        default:
            return nil
        }
    }

    public var bytesPerPixel: Int {
        switch self {
        case .mono8: return 1
        case .mono16, .uint16Single: return 2
        case .float32Single: return 4
        case .rgb8, .bgr8: return 3
        case .rgba8, .bgra8: return 4
        }
    }

    /// Single-channel encodings carry a measurement (depth, temperature) rather
    /// than a colour, and are rendered through a colour map.
    public var isSingleChannel: Bool {
        switch self {
        case .mono8, .mono16, .uint16Single, .float32Single: return true
        case .rgb8, .bgr8, .rgba8, .bgra8: return false
        }
    }
}

/// How the numbers in a single-channel image should be read.
public enum ScalarInterpretation: Equatable, Sendable {
    /// 16-bit integers in millimetres, the ROS convention for `16UC1` depth.
    /// Zero means "no return" and becomes `NaN`.
    case depthMillimetres
    /// 32-bit floats already in metres, the ROS convention for `32FC1` depth.
    /// Zero, negative and non-finite samples become `NaN`.
    case depthMetres
    /// Raw counts, passed through untouched.
    case raw
    /// `value * scale + offset`, for sensors that publish counts which map
    /// linearly onto a physical unit — an MLX90640 publishing centi-degrees as
    /// `mono16`, for example.
    case linear(scale: Double, offset: Double, unit: ScalarUnit)

    public var unit: ScalarUnit {
        switch self {
        case .depthMillimetres, .depthMetres: return .metres
        case .raw: return .raw
        case .linear(_, _, let unit): return unit
        }
    }

    /// The interpretation a depth topic should use for a given encoding.
    public static func depthDefault(for encoding: ROSImageEncoding) -> ScalarInterpretation {
        switch encoding {
        case .uint16Single, .mono16: return .depthMillimetres
        case .float32Single: return .depthMetres
        case .mono8: return .raw
        default: return .raw
        }
    }

}

public enum ScalarUnit: String, Equatable, Sendable, Codable {
    case metres
    case celsius
    case raw

    public var shortLabel: String {
        switch self {
        case .metres: return "m"
        case .celsius: return "°C"
        case .raw: return ""
        }
    }
}

/// A decoded single-channel image in physical units.
///
/// Invalid samples are `NaN` rather than a sentinel number, so they propagate
/// through statistics and colour mapping without being mistaken for a real
/// reading at zero metres.
public struct ScalarImage: Equatable, Sendable {
    public var width: Int
    public var height: Int
    /// Row-major, `width * height` samples.
    public var values: [Float]
    public var unit: ScalarUnit

    public init(width: Int, height: Int, values: [Float], unit: ScalarUnit) {
        self.width = width
        self.height = height
        self.values = values
        self.unit = unit
    }

    public var pixelCount: Int { width * height }

    public func value(x: Int, y: Int) -> Float {
        guard x >= 0, y >= 0, x < width, y < height else { return .nan }
        return values[y * width + x]
    }

    /// Smallest and largest finite samples, or `nil` when every sample is
    /// invalid. Used to drive auto-ranged colour maps.
    public var finiteRange: (min: Float, max: Float)? {
        var minimum = Float.greatestFiniteMagnitude
        var maximum = -Float.greatestFiniteMagnitude
        var found = false
        for value in values where value.isFinite {
            if value < minimum { minimum = value }
            if value > maximum { maximum = value }
            found = true
        }
        return found ? (minimum, maximum) : nil
    }

    /// Percentile range over the finite samples, which is far more stable than
    /// min/max for auto-ranging: one hot pixel or one dropout will not swing
    /// the whole colour map.
    public func finitePercentileRange(low: Double = 0.02, high: Double = 0.98) -> (min: Float, max: Float)? {
        var finite = values.filter { $0.isFinite }
        guard !finite.isEmpty else { return nil }
        finite.sort()
        let lowIndex = min(finite.count - 1, max(0, Int(Double(finite.count - 1) * low)))
        let highIndex = min(finite.count - 1, max(0, Int(Double(finite.count - 1) * high)))
        return (finite[lowIndex], finite[max(lowIndex, highIndex)])
    }
}

/// A decoded colour image, always stored as premultiplied-free straight RGBA8.
public struct ColorImage: Equatable, Sendable {
    public var width: Int
    public var height: Int
    /// Row-major RGBA, `width * height * 4` bytes.
    public var rgba: [UInt8]

    public init(width: Int, height: Int, rgba: [UInt8]) {
        self.width = width
        self.height = height
        self.rgba = rgba
    }
}

public enum DecodedROSImage: Equatable, Sendable {
    case scalar(ScalarImage)
    case color(ColorImage)

    public var width: Int {
        switch self {
        case .scalar(let image): return image.width
        case .color(let image): return image.width
        }
    }

    public var height: Int {
        switch self {
        case .scalar(let image): return image.height
        case .color(let image): return image.height
        }
    }
}

public enum ROSImageDecodeError: Error, Equatable, CustomStringConvertible {
    case unsupportedEncoding(String)
    case invalidDimensions(width: Int, height: Int)
    case stepTooSmall(step: Int, required: Int)
    case truncatedData(available: Int, required: Int)

    public var description: String {
        switch self {
        case .unsupportedEncoding(let encoding):
            return "Unsupported image encoding '\(encoding)'"
        case .invalidDimensions(let width, let height):
            return "Invalid image dimensions \(width)x\(height)"
        case .stepTooSmall(let step, let required):
            return "Image step \(step) is smaller than the \(required) bytes one row needs"
        case .truncatedData(let available, let required):
            return "Image data truncated: \(available) bytes available, \(required) required"
        }
    }
}
