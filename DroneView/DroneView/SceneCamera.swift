import Foundation
import Observation
import simd

@MainActor @Observable
final class SceneCamera {
    var yaw: Float = 0.6
    var pitch: Float = 0.4
    var distance: Float = 6
    var target = SIMD3<Float>(0, 0, -1.5)
    var followsCamera = false
    var showsTrajectory = true
    var pans = false

    func fit(_ cloud: PointCloud, position: SIMD3<Float>?) {
        followsCamera = false
        yaw = 0.6
        pitch = 0.4
        guard cloud.count > 0 else { target = SIMD3(0, 0, -1.5); distance = 6; return }
        let camera = position.map(Self.displayPoint) ?? .zero
        let low = simd_min(simd_min(cloud.minimum, .zero), camera)
        let high = simd_max(simd_max(cloud.maximum, .zero), camera)
        target = (low + high) / 2
        distance = min(150, max(1, simd_length(high - low)) * 1.4)
    }

    nonisolated static func displayPoint(_ optical: SIMD3<Float>) -> SIMD3<Float> {
        SIMD3(optical.x, -optical.y, -optical.z)
    }

    func drag(x: Float, y: Float) {
        if pans {
            followsCamera = false
            let right = SIMD3<Float>(cos(yaw), 0, -sin(yaw))
            let up = SIMD3<Float>(-sin(yaw) * sin(pitch), cos(pitch), -cos(yaw) * sin(pitch))
            target += (-right * x + up * y) * distance * 0.002
        } else {
            yaw -= x * 0.008
            pitch = min(1.5, max(-1.5, pitch + y * 0.008))
        }
    }

    func zoom(_ factor: Float) { distance = min(150, max(0.15, distance / max(0.01, factor))) }

    func matrix(aspect: Float) -> simd_float4x4 {
        let back = SIMD3<Float>(sin(yaw) * cos(pitch), sin(pitch), cos(yaw) * cos(pitch))
        let right = SIMD3<Float>(cos(yaw), 0, -sin(yaw))
        let up = simd_cross(back, right)
        let eye = target + back * distance
        let view = simd_float4x4(columns: (
            SIMD4(right.x, up.x, back.x, 0), SIMD4(right.y, up.y, back.y, 0),
            SIMD4(right.z, up.z, back.z, 0), SIMD4(-simd_dot(right, eye), -simd_dot(up, eye), -simd_dot(back, eye), 1)
        ))
        let scale: Float = 1 / tan(52 * .pi / 360)
        let near: Float = 0.01, far: Float = 500
        let projection = simd_float4x4(columns: (
            SIMD4(scale / max(0.01, aspect), 0, 0, 0), SIMD4(0, scale, 0, 0),
            SIMD4(0, 0, far / (near - far), -1), SIMD4(0, 0, near * far / (near - far), 0)
        ))
        return projection * view
    }
}
