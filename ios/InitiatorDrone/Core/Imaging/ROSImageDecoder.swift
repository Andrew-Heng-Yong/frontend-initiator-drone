import Foundation

/// Turns `sensor_msgs/msg/Image` payloads into either physical-unit scalar
/// images or straight RGBA colour images.
///
/// Every read goes through `step` for the row offset and through an explicit
/// byte assembly for multi-byte samples, so padded rows and big-endian
/// publishers are handled rather than assumed away.
public enum ROSImageDecoder {

    /// Decodes a message, choosing the scalar interpretation automatically from
    /// the encoding. Callers that know the topic's physical meaning should pass
    /// an explicit `interpretation` instead.
    public static func decode(_ message: ROSImageMessage) throws -> DecodedROSImage {
        guard let encoding = ROSImageEncoding(rosEncoding: message.encoding) else {
            throw ROSImageDecodeError.unsupportedEncoding(message.encoding)
        }
        return try decode(message, interpretation: .depthDefault(for: encoding))
    }

    /// Decodes a message using a caller-supplied scalar interpretation. The
    /// interpretation is ignored for colour encodings.
    public static func decode(
        _ message: ROSImageMessage,
        interpretation: ScalarInterpretation
    ) throws -> DecodedROSImage {
        guard let encoding = ROSImageEncoding(rosEncoding: message.encoding) else {
            throw ROSImageDecodeError.unsupportedEncoding(message.encoding)
        }
        try validate(message, encoding: encoding)

        if encoding.isSingleChannel {
            return .scalar(try decodeScalar(message, encoding: encoding, interpretation: interpretation))
        }
        return .color(try decodeColor(message, encoding: encoding))
    }

    // MARK: - Validation

    private static func validate(_ message: ROSImageMessage, encoding: ROSImageEncoding) throws {
        guard message.width > 0, message.height > 0,
              message.width <= 16_384, message.height <= 16_384 else {
            throw ROSImageDecodeError.invalidDimensions(width: message.width, height: message.height)
        }

        let rowBytes = message.width * encoding.bytesPerPixel
        // A publisher that omits `step` (rosbridge will send 0) is treated as
        // tightly packed rather than rejected.
        let step = message.step > 0 ? message.step : rowBytes
        guard step >= rowBytes else {
            throw ROSImageDecodeError.stepTooSmall(step: step, required: rowBytes)
        }

        // The final row does not need its trailing padding to be present.
        let required = (message.height - 1) * step + rowBytes
        guard message.data.count >= required else {
            throw ROSImageDecodeError.truncatedData(available: message.data.count, required: required)
        }
    }

    private static func effectiveStep(_ message: ROSImageMessage, encoding: ROSImageEncoding) -> Int {
        message.step > 0 ? message.step : message.width * encoding.bytesPerPixel
    }

    // MARK: - Single channel

    private static func decodeScalar(
        _ message: ROSImageMessage,
        encoding: ROSImageEncoding,
        interpretation: ScalarInterpretation
    ) throws -> ScalarImage {
        let width = message.width
        let height = message.height
        let step = effectiveStep(message, encoding: encoding)
        let bigEndian = message.isBigEndian

        var values = [Float](repeating: .nan, count: width * height)

        message.data.withUnsafeBytes { (raw: UnsafeRawBufferPointer) in
            guard let base = raw.baseAddress else { return }
            let bytes = base.assumingMemoryBound(to: UInt8.self)

            values.withUnsafeMutableBufferPointer { output in
                for y in 0..<height {
                    let rowStart = y * step
                    let outRow = y * width

                    switch encoding {
                    case .mono8:
                        for x in 0..<width {
                            output[outRow + x] = Float(bytes[rowStart + x])
                        }

                    case .mono16, .uint16Single:
                        for x in 0..<width {
                            let offset = rowStart + x * 2
                            let raw16 = readUInt16(bytes, at: offset, bigEndian: bigEndian)
                            output[outRow + x] = Float(raw16)
                        }

                    case .float32Single:
                        for x in 0..<width {
                            let offset = rowStart + x * 4
                            let bits = readUInt32(bytes, at: offset, bigEndian: bigEndian)
                            output[outRow + x] = Float(bitPattern: bits)
                        }

                    default:
                        break
                    }
                }
            }
        }

        applyInterpretation(interpretation, encoding: encoding, to: &values)
        return ScalarImage(width: width, height: height, values: values, unit: interpretation.unit)
    }

    /// Converts raw sample numbers into physical units in place, and maps the
    /// encoding's "no reading" sentinel to `NaN`.
    private static func applyInterpretation(
        _ interpretation: ScalarInterpretation,
        encoding: ROSImageEncoding,
        to values: inout [Float]
    ) {
        switch interpretation {
        case .depthMillimetres:
            for index in values.indices {
                let raw = values[index]
                // A 16-bit depth image uses 0 for "no return"; passing it
                // through as 0 m would paint a wall at the camera.
                values[index] = (raw > 0 && raw.isFinite) ? raw / 1000.0 : .nan
            }

        case .depthMetres:
            for index in values.indices {
                let raw = values[index]
                values[index] = (raw.isFinite && raw > 0) ? raw : .nan
            }

        case .raw:
            if encoding == .float32Single {
                for index in values.indices where !values[index].isFinite {
                    values[index] = .nan
                }
            }

        case .linear(let scale, let offset, _):
            let scaleValue = Float(scale)
            let offsetValue = Float(offset)
            for index in values.indices {
                let raw = values[index]
                if raw.isFinite {
                    values[index] = raw * scaleValue + offsetValue
                } else {
                    values[index] = .nan
                }
            }
        }
    }

    // MARK: - Colour

    private static func decodeColor(
        _ message: ROSImageMessage,
        encoding: ROSImageEncoding
    ) throws -> ColorImage {
        let width = message.width
        let height = message.height
        let step = effectiveStep(message, encoding: encoding)

        var rgba = [UInt8](repeating: 255, count: width * height * 4)

        message.data.withUnsafeBytes { (raw: UnsafeRawBufferPointer) in
            guard let base = raw.baseAddress else { return }
            let bytes = base.assumingMemoryBound(to: UInt8.self)

            rgba.withUnsafeMutableBufferPointer { output in
                for y in 0..<height {
                    let rowStart = y * step
                    let outRow = y * width * 4

                    switch encoding {
                    case .rgb8:
                        for x in 0..<width {
                            let source = rowStart + x * 3
                            let destination = outRow + x * 4
                            output[destination] = bytes[source]
                            output[destination + 1] = bytes[source + 1]
                            output[destination + 2] = bytes[source + 2]
                            output[destination + 3] = 255
                        }

                    case .bgr8:
                        for x in 0..<width {
                            let source = rowStart + x * 3
                            let destination = outRow + x * 4
                            output[destination] = bytes[source + 2]
                            output[destination + 1] = bytes[source + 1]
                            output[destination + 2] = bytes[source]
                            output[destination + 3] = 255
                        }

                    case .rgba8:
                        for x in 0..<width {
                            let source = rowStart + x * 4
                            let destination = outRow + x * 4
                            output[destination] = bytes[source]
                            output[destination + 1] = bytes[source + 1]
                            output[destination + 2] = bytes[source + 2]
                            output[destination + 3] = bytes[source + 3]
                        }

                    case .bgra8:
                        for x in 0..<width {
                            let source = rowStart + x * 4
                            let destination = outRow + x * 4
                            output[destination] = bytes[source + 2]
                            output[destination + 1] = bytes[source + 1]
                            output[destination + 2] = bytes[source]
                            output[destination + 3] = bytes[source + 3]
                        }

                    default:
                        break
                    }
                }
            }
        }

        return ColorImage(width: width, height: height, rgba: rgba)
    }

    // MARK: - Endian-aware readers

    /// Reads two bytes without assuming host order or pointer alignment. ROS
    /// image rows are only byte-aligned, so a direct `load(as: UInt16.self)`
    /// can trap on a padded row start.
    @inline(__always)
    private static func readUInt16(_ bytes: UnsafePointer<UInt8>, at offset: Int, bigEndian: Bool) -> UInt16 {
        let first = UInt16(bytes[offset])
        let second = UInt16(bytes[offset + 1])
        return bigEndian ? (first << 8) | second : (second << 8) | first
    }

    @inline(__always)
    private static func readUInt32(_ bytes: UnsafePointer<UInt8>, at offset: Int, bigEndian: Bool) -> UInt32 {
        let b0 = UInt32(bytes[offset])
        let b1 = UInt32(bytes[offset + 1])
        let b2 = UInt32(bytes[offset + 2])
        let b3 = UInt32(bytes[offset + 3])
        if bigEndian {
            return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3
        }
        return (b3 << 24) | (b2 << 16) | (b1 << 8) | b0
    }
}
