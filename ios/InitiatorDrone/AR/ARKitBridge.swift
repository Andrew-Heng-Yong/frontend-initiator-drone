#if canImport(ARKit)
import ARKit
import simd

/// The only place `Pose` meets `simd_float4x4`.
///
/// Two things are easy to get wrong at this boundary and are pinned down here:
/// `simd_quatf` stores its components as `(x, y, z, w)` while `Quaternion`
/// stores `(w, x, y, z)`, and `simd_float4x4` is column-major, so the
/// translation lives in `columns.3` and not in the bottom row.
public enum ARKitBridge {

    /// Reads an ARKit transform as a pose in ARKit world coordinates.
    ///
    /// The matrix is assumed rigid — rotation plus translation, no scale —
    /// which is what ARKit produces for camera and anchor transforms.
    public static func pose(from transform: simd_float4x4) -> Pose {
        let translation = Vector3(
            Double(transform.columns.3.x),
            Double(transform.columns.3.y),
            Double(transform.columns.3.z)
        )
        let rotation = simd_quatf(rotationMatrixOf: transform)
        return Pose(
            position: translation,
            orientation: Quaternion(
                w: Double(rotation.vector.w),
                x: Double(rotation.vector.x),
                y: Double(rotation.vector.y),
                z: Double(rotation.vector.z)
            ).normalized
        )
    }

    /// Builds an ARKit transform from a pose in ARKit world coordinates.
    public static func transform(from pose: Pose) -> simd_float4x4 {
        let quaternion = simd_quatf(
            ix: Float(pose.orientation.x),
            iy: Float(pose.orientation.y),
            iz: Float(pose.orientation.z),
            r: Float(pose.orientation.w)
        )
        var matrix = simd_float4x4(quaternion)
        matrix.columns.3 = SIMD4<Float>(
            Float(pose.position.x),
            Float(pose.position.y),
            Float(pose.position.z),
            1
        )
        return matrix
    }

    public static func vector(_ value: Vector3) -> SIMD3<Float> {
        SIMD3<Float>(Float(value.x), Float(value.y), Float(value.z))
    }

    public static func vector3(_ value: SIMD3<Float>) -> Vector3 {
        Vector3(Double(value.x), Double(value.y), Double(value.z))
    }
}

private extension simd_quatf {
    /// Extracts the rotation from a rigid transform, discarding translation.
    init(rotationMatrixOf transform: simd_float4x4) {
        let upperLeft = simd_float3x3(
            SIMD3<Float>(transform.columns.0.x, transform.columns.0.y, transform.columns.0.z),
            SIMD3<Float>(transform.columns.1.x, transform.columns.1.y, transform.columns.1.z),
            SIMD3<Float>(transform.columns.2.x, transform.columns.2.y, transform.columns.2.z)
        )
        self.init(upperLeft)
    }
}

public extension ARCamera.TrackingState {
    var shortLabel: String {
        switch self {
        case .normal: return "Tracking"
        case .notAvailable: return "Not available"
        case .limited(let reason):
            switch reason {
            case .initializing: return "Initialising"
            case .relocalizing: return "Relocalising"
            case .excessiveMotion: return "Too much motion"
            case .insufficientFeatures: return "Not enough detail"
            @unknown default: return "Limited"
            }
        }
    }

    var isUsable: Bool {
        if case .normal = self { return true }
        return false
    }

    var detailLabel: String {
        switch self {
        case .normal:
            return "The phone knows where it is."
        case .notAvailable:
            return "The phone is not tracking yet."
        case .limited(.initializing):
            return "Move the phone slowly to start tracking."
        case .limited(.relocalizing):
            return "Return to where the session was interrupted. Alignment may have shifted."
        case .limited(.excessiveMotion):
            return "Slow down; the phone pose is unreliable."
        case .limited(.insufficientFeatures):
            return "Point at a more textured surface."
        case .limited:
            return "Phone tracking is limited."
        }
    }
}
#endif
