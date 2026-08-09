import Foundation

/// A decoded rosbridge value.
///
/// rosbridge can deliver the same message as JSON or as CBOR depending on the
/// `compression` requested at subscribe time. Both wire formats decode into
/// this one tree, so the message parsers below are written once and are
/// independent of transport.
///
/// The `.bytes` case exists because CBOR carries byte strings natively. That is
/// the whole reason `cbor-raw` is worth supporting: a 640x480 16-bit depth
/// frame arrives as one 600 KB byte string instead of a 800 KB base64 text
/// blob that has to be decoded again.
public enum ROSValue: Equatable, Sendable {
    case null
    case bool(Bool)
    case number(Double)
    case integer(Int64)
    case string(String)
    case bytes(Data)
    case array([ROSValue])
    case object([String: ROSValue])

    // MARK: - Accessors

    public var boolValue: Bool? {
        switch self {
        case .bool(let value): return value
        case .integer(let value): return value != 0
        case .number(let value): return value != 0
        default: return nil
        }
    }

    public var doubleValue: Double? {
        switch self {
        case .number(let value): return value
        case .integer(let value): return Double(value)
        case .bool(let value): return value ? 1 : 0
        case .string(let value): return Double(value)
        default: return nil
        }
    }

    public var intValue: Int? {
        switch self {
        case .integer(let value): return Int(value)
        case .number(let value): return value.isFinite ? Int(value) : nil
        case .bool(let value): return value ? 1 : 0
        case .string(let value): return Int(value)
        default: return nil
        }
    }

    public var stringValue: String? {
        if case .string(let value) = self { return value }
        return nil
    }

    public var arrayValue: [ROSValue]? {
        if case .array(let value) = self { return value }
        return nil
    }

    public var objectValue: [String: ROSValue]? {
        if case .object(let value) = self { return value }
        return nil
    }

    public subscript(key: String) -> ROSValue? {
        guard case .object(let dictionary) = self else { return nil }
        return dictionary[key]
    }

    /// Reads a `uint8[]` field, which rosbridge may send three different ways
    /// depending on the compression setting: a CBOR byte string, a base64
    /// string in JSON, or a plain JSON array of numbers.
    public var byteArrayValue: Data? {
        switch self {
        case .bytes(let data):
            return data
        case .string(let text):
            return Data(base64Encoded: text, options: .ignoreUnknownCharacters)
        case .array(let elements):
            var data = Data(capacity: elements.count)
            for element in elements {
                guard let value = element.intValue else { return nil }
                data.append(UInt8(truncatingIfNeeded: value))
            }
            return data
        default:
            return nil
        }
    }

    /// Reads a numeric array field such as a `CameraInfo` projection matrix.
    public var doubleArrayValue: [Double]? {
        guard case .array(let elements) = self else { return nil }
        var result = [Double]()
        result.reserveCapacity(elements.count)
        for element in elements {
            guard let value = element.doubleValue else { return nil }
            result.append(value)
        }
        return result
    }
}

// MARK: - JSON bridging

public extension ROSValue {
    /// Builds a value tree from `JSONSerialization` output.
    static func fromJSONObject(_ object: Any) -> ROSValue {
        switch object {
        case is NSNull:
            return .null

        case let number as NSNumber:
            // NSNumber erases Bool into a number, so the underlying ObjC type
            // has to be checked to avoid turning `true` into `1.0` and losing
            // the /vio/calibrated semantics.
            if CFGetTypeID(number) == CFBooleanGetTypeID() {
                return .bool(number.boolValue)
            }
            let type = String(cString: number.objCType)
            if type == "c" || type == "C" || type == "s" || type == "i"
                || type == "l" || type == "q" || type == "S" || type == "I"
                || type == "L" || type == "Q" {
                return .integer(number.int64Value)
            }
            return .number(number.doubleValue)

        case let string as String:
            return .string(string)

        case let data as Data:
            return .bytes(data)

        case let array as [Any]:
            return .array(array.map(ROSValue.fromJSONObject))

        case let dictionary as [String: Any]:
            var result = [String: ROSValue](minimumCapacity: dictionary.count)
            for (key, value) in dictionary {
                result[key] = ROSValue.fromJSONObject(value)
            }
            return .object(result)

        default:
            return .null
        }
    }

    /// Parses raw JSON text into a value tree.
    static func fromJSON(_ data: Data) throws -> ROSValue {
        let object = try JSONSerialization.jsonObject(with: data, options: [.fragmentsAllowed])
        return fromJSONObject(object)
    }

    /// Converts back to a `JSONSerialization`-compatible object graph, used
    /// when encoding outbound rosbridge commands.
    var jsonObject: Any {
        switch self {
        case .null: return NSNull()
        case .bool(let value): return value
        case .number(let value): return value
        case .integer(let value): return value
        case .string(let value): return value
        case .bytes(let data): return data.base64EncodedString()
        case .array(let values): return values.map(\.jsonObject)
        case .object(let dictionary): return dictionary.mapValues(\.jsonObject)
        }
    }
}
