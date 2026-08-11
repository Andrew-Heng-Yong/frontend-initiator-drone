import Foundation

/// A CBOR reader covering the subset rosbridge emits for
/// `compression: "cbor"`, including the RFC 8746 typed-array tags.
///
/// Why bother instead of just using JSON: rosbridge's CBOR encoder sends a
/// `uint8[]` field as tag 64 wrapping a byte string, so a depth frame arrives
/// as raw bytes. The JSON path base64-encodes the same frame, which inflates it
/// by a third on the wire and then costs a second full decode pass on the
/// phone. Over a long session on a crowded 2.4 GHz link that difference is the
/// difference between a smooth stream and a growing backlog.
///
/// Everything here is bounds-checked and throws rather than trapping, because
/// the input is network data.
public enum CBORDecoder {

    public enum Error: Swift.Error, Equatable, CustomStringConvertible {
        case unexpectedEnd
        case unsupportedMajorType(UInt8)
        case unsupportedAdditionalInfo(UInt8)
        case invalidUTF8
        case depthLimitExceeded
        case trailingBytes(Int)

        public var description: String {
            switch self {
            case .unexpectedEnd: return "CBOR input ended unexpectedly"
            case .unsupportedMajorType(let type): return "Unsupported CBOR major type \(type)"
            case .unsupportedAdditionalInfo(let info): return "Unsupported CBOR additional info \(info)"
            case .invalidUTF8: return "CBOR text string was not valid UTF-8"
            case .depthLimitExceeded: return "CBOR nesting is too deep"
            case .trailingBytes(let count): return "\(count) unexpected bytes after the CBOR value"
            }
        }
    }

    /// Decodes one complete CBOR value. Trailing bytes are an error, so a
    /// truncated or concatenated frame is caught rather than silently accepted.
    public static func decode(_ data: Data) throws -> ROSValue {
        var reader = Reader(data: data)
        let value = try reader.readValue(depth: 0)
        let remaining = reader.remainingCount
        guard remaining == 0 else { throw Error.trailingBytes(remaining) }
        return value
    }

    private struct Reader {
        let bytes: [UInt8]
        var index: Int = 0

        init(data: Data) {
            self.bytes = [UInt8](data)
        }

        var remainingCount: Int { bytes.count - index }

        mutating func readByte() throws -> UInt8 {
            guard index < bytes.count else { throw Error.unexpectedEnd }
            defer { index += 1 }
            return bytes[index]
        }

        mutating func readBytes(_ count: Int) throws -> Data {
            guard count >= 0, index + count <= bytes.count else { throw Error.unexpectedEnd }
            defer { index += count }
            return Data(bytes[index..<(index + count)])
        }

        mutating func readUInt(_ byteCount: Int) throws -> UInt64 {
            guard index + byteCount <= bytes.count else { throw Error.unexpectedEnd }
            var value: UInt64 = 0
            for offset in 0..<byteCount {
                value = (value << 8) | UInt64(bytes[index + offset])
            }
            index += byteCount
            return value
        }

        /// Reads the argument that follows a major type's additional info.
        /// Returns `nil` for the indefinite-length marker (31).
        mutating func readArgument(_ additionalInfo: UInt8) throws -> UInt64? {
            switch additionalInfo {
            case 0...23: return UInt64(additionalInfo)
            case 24: return UInt64(try readByte())
            case 25: return try readUInt(2)
            case 26: return try readUInt(4)
            case 27: return try readUInt(8)
            case 31: return nil
            default: throw Error.unsupportedAdditionalInfo(additionalInfo)
            }
        }

        mutating func readValue(depth: Int) throws -> ROSValue {
            guard depth < 64 else { throw Error.depthLimitExceeded }

            let initialByte = try readByte()
            let majorType = initialByte >> 5
            let additionalInfo = initialByte & 0x1F

            switch majorType {
            case 0: // unsigned integer
                guard let argument = try readArgument(additionalInfo) else {
                    throw Error.unsupportedAdditionalInfo(additionalInfo)
                }
                if argument <= UInt64(Int64.max) { return .integer(Int64(argument)) }
                return .number(Double(argument))

            case 1: // negative integer, encoded as -1 - n
                guard let argument = try readArgument(additionalInfo) else {
                    throw Error.unsupportedAdditionalInfo(additionalInfo)
                }
                if argument <= UInt64(Int64.max) { return .integer(-1 - Int64(argument)) }
                return .number(-1.0 - Double(argument))

            case 2: // byte string
                if let length = try readArgument(additionalInfo) {
                    return .bytes(try readBytes(Int(length)))
                }
                var accumulated = Data()
                while try !consumeBreakIfPresent() {
                    guard case .bytes(let chunk) = try readValue(depth: depth + 1) else {
                        throw Error.unsupportedAdditionalInfo(additionalInfo)
                    }
                    accumulated.append(chunk)
                }
                return .bytes(accumulated)

            case 3: // text string
                if let length = try readArgument(additionalInfo) {
                    let raw = try readBytes(Int(length))
                    guard let text = String(data: raw, encoding: .utf8) else { throw Error.invalidUTF8 }
                    return .string(text)
                }
                var accumulated = ""
                while try !consumeBreakIfPresent() {
                    guard case .string(let chunk) = try readValue(depth: depth + 1) else {
                        throw Error.unsupportedAdditionalInfo(additionalInfo)
                    }
                    accumulated += chunk
                }
                return .string(accumulated)

            case 4: // array
                if let count = try readArgument(additionalInfo) {
                    var elements = [ROSValue]()
                    elements.reserveCapacity(min(Int(count), 4096))
                    for _ in 0..<count {
                        elements.append(try readValue(depth: depth + 1))
                    }
                    return .array(elements)
                }
                var elements = [ROSValue]()
                while try !consumeBreakIfPresent() {
                    elements.append(try readValue(depth: depth + 1))
                }
                return .array(elements)

            case 5: // map
                if let count = try readArgument(additionalInfo) {
                    var dictionary = [String: ROSValue](minimumCapacity: Int(min(count, 256)))
                    for _ in 0..<count {
                        let key = try readValue(depth: depth + 1)
                        let value = try readValue(depth: depth + 1)
                        if let name = key.stringValue {
                            dictionary[name] = value
                        } else if let name = key.intValue {
                            dictionary[String(name)] = value
                        }
                    }
                    return .object(dictionary)
                }
                var dictionary = [String: ROSValue]()
                while try !consumeBreakIfPresent() {
                    let key = try readValue(depth: depth + 1)
                    let value = try readValue(depth: depth + 1)
                    if let name = key.stringValue {
                        dictionary[name] = value
                    } else if let name = key.intValue {
                        dictionary[String(name)] = value
                    }
                }
                return .object(dictionary)

            case 6: // tagged value
                guard let tag = try readArgument(additionalInfo) else {
                    throw Error.unsupportedAdditionalInfo(additionalInfo)
                }
                let content = try readValue(depth: depth + 1)
                return try applyTag(tag, to: content)

            case 7: // simple values and floats
                switch additionalInfo {
                case 20: return .bool(false)
                case 21: return .bool(true)
                case 22: return .null
                case 23: return .null // undefined
                case 24: return .integer(Int64(try readByte()))
                case 25: return .number(Double(Float16Bits.value(from: UInt16(try readUInt(2)))))
                case 26: return .number(Double(Float(bitPattern: UInt32(try readUInt(4)))))
                case 27: return .number(Double(bitPattern: try readUInt(8)))
                case 31: throw Error.unsupportedAdditionalInfo(additionalInfo) // stray break
                default: return .integer(Int64(additionalInfo))
                }

            default:
                throw Error.unsupportedMajorType(majorType)
            }
        }

        /// Consumes the indefinite-length break marker (0xFF) if it is next.
        mutating func consumeBreakIfPresent() throws -> Bool {
            guard index < bytes.count else { throw Error.unexpectedEnd }
            if bytes[index] == 0xFF {
                index += 1
                return true
            }
            return false
        }

        /// Interprets the RFC 8746 typed-array tags that rosbridge uses for
        /// numeric arrays. Tag 64 (`uint8`) is the important one: it is how a
        /// `sensor_msgs/Image` `data` field arrives, and keeping it as a byte
        /// string avoids materialising hundreds of thousands of boxed numbers.
        func applyTag(_ tag: UInt64, to content: ROSValue) throws -> ROSValue {
            switch tag {
            case 64, 68: // uint8, uint8 clamped
                return content

            case 65, 69: // uint16 big/little endian
                return decodeTypedArray(content, elementSize: 2, bigEndian: tag == 65) { bits in
                    Double(UInt16(truncatingIfNeeded: bits))
                }

            case 66, 70: // uint32
                return decodeTypedArray(content, elementSize: 4, bigEndian: tag == 66) { bits in
                    Double(UInt32(truncatingIfNeeded: bits))
                }

            case 67, 71: // uint64
                return decodeTypedArray(content, elementSize: 8, bigEndian: tag == 67) { bits in
                    Double(bits)
                }

            case 72: // sint8
                return decodeTypedArray(content, elementSize: 1, bigEndian: true) { bits in
                    Double(Int8(truncatingIfNeeded: bits))
                }

            case 73, 77: // sint16
                return decodeTypedArray(content, elementSize: 2, bigEndian: tag == 73) { bits in
                    Double(Int16(truncatingIfNeeded: bits))
                }

            case 74, 78: // sint32
                return decodeTypedArray(content, elementSize: 4, bigEndian: tag == 74) { bits in
                    Double(Int32(truncatingIfNeeded: bits))
                }

            case 75, 79: // sint64
                return decodeTypedArray(content, elementSize: 8, bigEndian: tag == 75) { bits in
                    Double(Int64(bitPattern: bits))
                }

            case 81, 85: // float32
                return decodeTypedArray(content, elementSize: 4, bigEndian: tag == 81) { bits in
                    Double(Float(bitPattern: UInt32(truncatingIfNeeded: bits)))
                }

            case 82, 86: // float64
                return decodeTypedArray(content, elementSize: 8, bigEndian: tag == 82) { bits in
                    Double(bitPattern: bits)
                }

            default:
                // Unknown tags are transparent: CBOR tags are semantic hints,
                // and dropping one is better than failing the whole message.
                return content
            }
        }

        private func decodeTypedArray(
            _ content: ROSValue,
            elementSize: Int,
            bigEndian: Bool,
            transform: (UInt64) -> Double
        ) -> ROSValue {
            guard case .bytes(let data) = content else { return content }
            let count = data.count / elementSize
            var values = [ROSValue]()
            values.reserveCapacity(count)
            let raw = [UInt8](data)
            for element in 0..<count {
                var bits: UInt64 = 0
                let start = element * elementSize
                if bigEndian {
                    for offset in 0..<elementSize {
                        bits = (bits << 8) | UInt64(raw[start + offset])
                    }
                } else {
                    for offset in stride(from: elementSize - 1, through: 0, by: -1) {
                        bits = (bits << 8) | UInt64(raw[start + offset])
                    }
                }
                values.append(.number(transform(bits)))
            }
            return .array(values)
        }
    }
}

/// Minimal IEEE-754 binary16 support, needed only for CBOR half floats.
/// `Float16` is unavailable on the x86 simulator slice, so this is done by hand.
enum Float16Bits {
    static func value(from bits: UInt16) -> Float {
        let sign = Float((bits & 0x8000) != 0 ? -1 : 1)
        let exponent = Int((bits >> 10) & 0x1F)
        let fraction = Int(bits & 0x03FF)

        if exponent == 0 {
            return sign * Float(pow(2.0, -14.0)) * (Float(fraction) / 1024.0)
        }
        if exponent == 0x1F {
            return fraction == 0 ? sign * Float.infinity : Float.nan
        }
        return sign * Float(pow(2.0, Double(exponent - 15))) * (1.0 + Float(fraction) / 1024.0)
    }
}
