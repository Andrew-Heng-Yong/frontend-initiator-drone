import Foundation

/// A double-precision unit quaternion using the Hamilton convention, which is
/// what both ROS (`geometry_msgs/Quaternion`) and ARKit/simd use.
///
/// Component order in this type is `(w, x, y, z)`. Note that ROS JSON messages
/// serialise the fields by name, and `simd_quatf` stores them as `(x, y, z, w)`;
/// both boundaries are handled explicitly at the conversion sites so the order
/// is never assumed.
public struct Quaternion: Equatable, Hashable, Codable, Sendable {
    public var w: Double
    public var x: Double
    public var y: Double
    public var z: Double

    public init(w: Double, x: Double, y: Double, z: Double) {
        self.w = w
        self.x = x
        self.y = y
        self.z = z
    }

    public static let identity = Quaternion(w: 1, x: 0, y: 0, z: 0)

    public var length: Double { (w * w + x * x + y * y + z * z).squareRoot() }

    public var normalized: Quaternion {
        let len = length
        guard len > 1e-12 else { return .identity }
        return Quaternion(w: w / len, x: x / len, y: y / len, z: z / len)
    }

    /// Conjugate. For a unit quaternion this is also the inverse rotation.
    public var conjugate: Quaternion { Quaternion(w: w, x: -x, y: -y, z: -z) }

    public var inverse: Quaternion {
        let normSquared = w * w + x * x + y * y + z * z
        guard normSquared > 1e-24 else { return .identity }
        return Quaternion(w: w / normSquared, x: -x / normSquared, y: -y / normSquared, z: -z / normSquared)
    }

    public var isFinite: Bool { w.isFinite && x.isFinite && y.isFinite && z.isFinite }

    /// Hamilton product. `lhs * rhs` applies `rhs` first, then `lhs`.
    public static func * (lhs: Quaternion, rhs: Quaternion) -> Quaternion {
        Quaternion(
            w: lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
            x: lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
            y: lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
            z: lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w
        )
    }

    public static prefix func - (value: Quaternion) -> Quaternion {
        Quaternion(w: -value.w, x: -value.x, y: -value.y, z: -value.z)
    }

    public func dot(_ other: Quaternion) -> Double {
        w * other.w + x * other.x + y * other.y + z * other.z
    }

    /// Rotates `vector` by this quaternion.
    public func rotate(_ vector: Vector3) -> Vector3 {
        // v' = v + 2 * q_vec x (q_vec x v + w * v)
        let u = Vector3(x, y, z)
        let t = u.cross(vector) + vector * w
        return vector + u.cross(t) * 2.0
    }

    public init(axis: Vector3, angle: Double) {
        let unitAxis = axis.normalized
        let half = angle * 0.5
        let s = sin(half)
        self.init(w: cos(half), x: unitAxis.x * s, y: unitAxis.y * s, z: unitAxis.z * s)
    }

    /// Rotation about the +Y axis, which is "up" in ARKit world space.
    public static func aroundY(_ angle: Double) -> Quaternion {
        Quaternion(axis: Vector3(0, 1, 0), angle: angle)
    }

    /// Rotation about the +Z axis, which is "up" in a ROS REP-103 frame.
    public static func aroundZ(_ angle: Double) -> Quaternion {
        Quaternion(axis: Vector3(0, 0, 1), angle: angle)
    }

    /// Yaw about the frame's +Z axis, in radians, using the ROS convention
    /// (rotation from +X toward +Y). Only meaningful for ROS-frame quaternions.
    public var yawAroundZ: Double {
        let siny = 2.0 * (w * z + x * y)
        let cosy = 1.0 - 2.0 * (y * y + z * z)
        return atan2(siny, cosy)
    }

    /// Yaw about the frame's +Y axis, in radians. Only meaningful for
    /// ARKit-frame quaternions.
    ///
    /// The sign convention is chosen so that this is the exact inverse of
    /// `Quaternion.aroundY(_:)`: `Quaternion.aroundY(t).yawAroundY == t`. In a
    /// right-handed frame a positive rotation about `+Y` swings the forward
    /// axis (`-Z`) toward `-X`, which is counter-clockwise seen from above.
    public var yawAroundY: Double {
        // The node forward axis in ARKit/SceneKit is -Z.
        let forward = rotate(Vector3(0, 0, -1))
        return atan2(-forward.x, -forward.z)
    }

    /// Roll, pitch and yaw in radians for a ROS-frame quaternion, using the
    /// standard aerospace intrinsic Z-Y-X sequence that `tf2` reports.
    public var rollPitchYaw: (roll: Double, pitch: Double, yaw: Double) {
        let sinr = 2.0 * (w * x + y * z)
        let cosr = 1.0 - 2.0 * (x * x + y * y)
        let roll = atan2(sinr, cosr)

        let sinp = 2.0 * (w * y - z * x)
        let pitch: Double
        if abs(sinp) >= 1.0 {
            pitch = (sinp < 0 ? -1.0 : 1.0) * (Double.pi / 2.0)
        } else {
            pitch = asin(sinp)
        }

        return (roll, pitch, yawAroundZ)
    }

    /// Angular difference to `other`, in radians, in the range `0...pi`.
    public func angle(to other: Quaternion) -> Double {
        let d = min(1.0, abs(normalized.dot(other.normalized)))
        return 2.0 * acos(d)
    }

    /// Shortest-path spherical linear interpolation. `t` is clamped to `0...1`.
    public static func slerp(_ a: Quaternion, _ b: Quaternion, _ t: Double) -> Quaternion {
        let clamped = min(max(t, 0.0), 1.0)
        let start = a.normalized
        var end = b.normalized

        var cosTheta = start.dot(end)
        // Take the short way around: q and -q are the same rotation.
        if cosTheta < 0 {
            end = -end
            cosTheta = -cosTheta
        }

        // Nearly parallel: normalised linear interpolation avoids a divide by
        // a vanishing sin(theta).
        if cosTheta > 0.9995 {
            let result = Quaternion(
                w: start.w + (end.w - start.w) * clamped,
                x: start.x + (end.x - start.x) * clamped,
                y: start.y + (end.y - start.y) * clamped,
                z: start.z + (end.z - start.z) * clamped
            )
            return result.normalized
        }

        let theta = acos(min(1.0, max(-1.0, cosTheta)))
        let sinTheta = sin(theta)
        let scaleStart = sin((1.0 - clamped) * theta) / sinTheta
        let scaleEnd = sin(clamped * theta) / sinTheta

        return Quaternion(
            w: start.w * scaleStart + end.w * scaleEnd,
            x: start.x * scaleStart + end.x * scaleEnd,
            y: start.y * scaleStart + end.y * scaleEnd,
            z: start.z * scaleStart + end.z * scaleEnd
        ).normalized
    }

    /// True when both quaternions describe the same rotation, accounting for
    /// the double cover (`q` and `-q` are equivalent).
    public func describesSameRotation(as other: Quaternion, tolerance: Double = 1e-9) -> Bool {
        let a = normalized
        let b = other.normalized
        let same = abs(a.w - b.w) <= tolerance && abs(a.x - b.x) <= tolerance
            && abs(a.y - b.y) <= tolerance && abs(a.z - b.z) <= tolerance
        let flipped = abs(a.w + b.w) <= tolerance && abs(a.x + b.x) <= tolerance
            && abs(a.y + b.y) <= tolerance && abs(a.z + b.z) <= tolerance
        return same || flipped
    }

    /// Column-major 3x3 rotation matrix, as nine doubles ordered
    /// `[m00, m10, m20, m01, m11, m21, m02, m12, m22]`.
    public var rotationMatrixColumnMajor: [Double] {
        let q = normalized
        let xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z
        let xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z
        let wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z
        return [
            1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy),
            2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx),
            2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy),
        ]
    }

    /// Builds a quaternion from a proper rotation matrix given in the same
    /// column-major order produced by `rotationMatrixColumnMajor`.
    public init(rotationMatrixColumnMajor m: [Double]) {
        precondition(m.count == 9, "A 3x3 rotation matrix needs exactly 9 elements")
        func at(_ row: Int, _ column: Int) -> Double { m[column * 3 + row] }

        let trace = at(0, 0) + at(1, 1) + at(2, 2)
        if trace > 0 {
            let s = (trace + 1.0).squareRoot() * 2.0
            self.init(
                w: 0.25 * s,
                x: (at(2, 1) - at(1, 2)) / s,
                y: (at(0, 2) - at(2, 0)) / s,
                z: (at(1, 0) - at(0, 1)) / s
            )
        } else if at(0, 0) > at(1, 1) && at(0, 0) > at(2, 2) {
            let s = (1.0 + at(0, 0) - at(1, 1) - at(2, 2)).squareRoot() * 2.0
            self.init(
                w: (at(2, 1) - at(1, 2)) / s,
                x: 0.25 * s,
                y: (at(0, 1) + at(1, 0)) / s,
                z: (at(0, 2) + at(2, 0)) / s
            )
        } else if at(1, 1) > at(2, 2) {
            let s = (1.0 + at(1, 1) - at(0, 0) - at(2, 2)).squareRoot() * 2.0
            self.init(
                w: (at(0, 2) - at(2, 0)) / s,
                x: (at(0, 1) + at(1, 0)) / s,
                y: 0.25 * s,
                z: (at(1, 2) + at(2, 1)) / s
            )
        } else {
            let s = (1.0 + at(2, 2) - at(0, 0) - at(1, 1)).squareRoot() * 2.0
            self.init(
                w: (at(1, 0) - at(0, 1)) / s,
                x: (at(0, 2) + at(2, 0)) / s,
                y: (at(1, 2) + at(2, 1)) / s,
                z: 0.25 * s
            )
        }
        self = normalized
    }
}
