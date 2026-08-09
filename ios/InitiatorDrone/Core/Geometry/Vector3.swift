import Foundation

/// A double-precision 3D vector.
///
/// The core geometry layer deliberately avoids `simd` so it can be unit tested
/// on any platform, and so interpolation runs in double precision. Conversion
/// to `simd_float4x4` happens only at the ARKit boundary.
public struct Vector3: Equatable, Hashable, Codable, Sendable {
    public var x: Double
    public var y: Double
    public var z: Double

    public init(_ x: Double, _ y: Double, _ z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }

    public static let zero = Vector3(0, 0, 0)

    public var length: Double { (x * x + y * y + z * z).squareRoot() }
    public var lengthSquared: Double { x * x + y * y + z * z }

    public var normalized: Vector3 {
        let len = length
        guard len > 1e-12 else { return .zero }
        return Vector3(x / len, y / len, z / len)
    }

    public static func + (lhs: Vector3, rhs: Vector3) -> Vector3 {
        Vector3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z)
    }

    public static func - (lhs: Vector3, rhs: Vector3) -> Vector3 {
        Vector3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z)
    }

    public static func * (lhs: Vector3, rhs: Double) -> Vector3 {
        Vector3(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs)
    }

    public static func * (lhs: Double, rhs: Vector3) -> Vector3 { rhs * lhs }

    public static prefix func - (value: Vector3) -> Vector3 {
        Vector3(-value.x, -value.y, -value.z)
    }

    public func dot(_ other: Vector3) -> Double {
        x * other.x + y * other.y + z * other.z
    }

    public func cross(_ other: Vector3) -> Vector3 {
        Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        )
    }

    /// Straight-line distance to `other`.
    public func distance(to other: Vector3) -> Double { (self - other).length }

    /// Component-wise linear interpolation. `t` is not clamped by this method.
    public static func lerp(_ a: Vector3, _ b: Vector3, _ t: Double) -> Vector3 {
        Vector3(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            a.z + (b.z - a.z) * t
        )
    }

    public func isApproximatelyEqual(to other: Vector3, tolerance: Double = 1e-9) -> Bool {
        abs(x - other.x) <= tolerance && abs(y - other.y) <= tolerance && abs(z - other.z) <= tolerance
    }

    public var isFinite: Bool { x.isFinite && y.isFinite && z.isFinite }
}
