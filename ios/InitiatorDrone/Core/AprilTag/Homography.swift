import Foundation

/// A 3x3 projective transform, row-major.
public struct Homography: Equatable, Sendable {
    public var m: [Double]

    public init(_ m: [Double]) {
        precondition(m.count == 9)
        self.m = m
    }

    public static let identity = Homography([1, 0, 0, 0, 1, 0, 0, 0, 1])

    /// Maps a point, dividing through by the homogeneous coordinate.
    public func apply(_ point: Vector2) -> Vector2 {
        let w = m[6] * point.x + m[7] * point.y + m[8]
        guard abs(w) > 1e-12 else { return Vector2(.nan, .nan) }
        return Vector2(
            (m[0] * point.x + m[1] * point.y + m[2]) / w,
            (m[3] * point.x + m[4] * point.y + m[5]) / w
        )
    }

    /// Inverse transform, from the adjugate. A homography is scale-free, so
    /// dividing by the determinant is cosmetic — it only keeps the numbers near
    /// unity, where `apply`'s guard against a vanishing `w` stays meaningful.
    public var inverse: Homography? {
        let determinant =
            m[0] * (m[4] * m[8] - m[5] * m[7])
            - m[1] * (m[3] * m[8] - m[5] * m[6])
            + m[2] * (m[3] * m[7] - m[4] * m[6])
        guard abs(determinant) > 1e-12 else { return nil }
        let inverseDeterminant = 1.0 / determinant
        return Homography([
            (m[4] * m[8] - m[5] * m[7]) * inverseDeterminant,
            (m[2] * m[7] - m[1] * m[8]) * inverseDeterminant,
            (m[1] * m[5] - m[2] * m[4]) * inverseDeterminant,
            (m[5] * m[6] - m[3] * m[8]) * inverseDeterminant,
            (m[0] * m[8] - m[2] * m[6]) * inverseDeterminant,
            (m[2] * m[3] - m[0] * m[5]) * inverseDeterminant,
            (m[3] * m[7] - m[4] * m[6]) * inverseDeterminant,
            (m[1] * m[6] - m[0] * m[7]) * inverseDeterminant,
            (m[0] * m[4] - m[1] * m[3]) * inverseDeterminant,
        ])
    }

    /// The homography taking the canonical tag square to four image corners.
    ///
    /// The tag square is `[-1, 1]²` with corners listed counter-clockwise in a
    /// y-down frame from the top-left: `(-1,-1), (-1,1), (1,1), (1,-1)`. That
    /// matches `QuadDetector`'s normalised winding, so corner *i* of the quad is
    /// corner *i* of the square and no reordering happens anywhere else.
    ///
    /// Four correspondences give eight equations for the eight free parameters
    /// (the ninth is fixed by scale), so this is an exact solve, not a fit.
    public static func mapping(unitSquareTo corners: [Vector2]) -> Homography? {
        precondition(corners.count == 4)
        let source = [Vector2(-1, -1), Vector2(-1, 1), Vector2(1, 1), Vector2(1, -1)]

        var a = [Double](repeating: 0, count: 8 * 8)
        var b = [Double](repeating: 0, count: 8)
        for index in 0..<4 {
            let s = source[index], d = corners[index]
            let row0 = index * 2, row1 = row0 + 1
            a[row0 * 8 + 0] = s.x; a[row0 * 8 + 1] = s.y; a[row0 * 8 + 2] = 1
            a[row0 * 8 + 6] = -s.x * d.x; a[row0 * 8 + 7] = -s.y * d.x
            b[row0] = d.x
            a[row1 * 8 + 3] = s.x; a[row1 * 8 + 4] = s.y; a[row1 * 8 + 5] = 1
            a[row1 * 8 + 6] = -s.x * d.y; a[row1 * 8 + 7] = -s.y * d.y
            b[row1] = d.y
        }

        guard let h = solve(a: &a, b: &b, size: 8) else { return nil }
        return Homography([h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1])
    }

    /// Gaussian elimination with partial pivoting.
    ///
    /// Pivoting is not optional here: a tag square-on to the camera makes the
    /// two perspective columns vanish, and without a row swap the elimination
    /// divides by something at machine epsilon and returns garbage for the one
    /// viewing angle most likely to be used deliberately.
    static func solve(a: inout [Double], b: inout [Double], size: Int) -> [Double]? {
        for column in 0..<size {
            var pivotRow = column
            var best = abs(a[column * size + column])
            for row in (column + 1)..<size {
                let value = abs(a[row * size + column])
                if value > best {
                    best = value
                    pivotRow = row
                }
            }
            guard best > 1e-12 else { return nil }

            if pivotRow != column {
                for k in 0..<size {
                    a.swapAt(column * size + k, pivotRow * size + k)
                }
                b.swapAt(column, pivotRow)
            }

            let pivot = a[column * size + column]
            for row in (column + 1)..<size {
                let factor = a[row * size + column] / pivot
                guard factor != 0 else { continue }
                for k in column..<size {
                    a[row * size + k] -= factor * a[column * size + k]
                }
                b[row] -= factor * b[column]
            }
        }

        var solution = [Double](repeating: 0, count: size)
        for row in stride(from: size - 1, through: 0, by: -1) {
            var value = b[row]
            for column in (row + 1)..<size {
                value -= a[row * size + column] * solution[column]
            }
            solution[row] = value / a[row * size + row]
        }
        return solution.allSatisfy(\.isFinite) ? solution : nil
    }
}

/// Pinhole intrinsics for the image the detector actually ran on.
///
/// "Actually ran on" is load-bearing. The detector works on a downscaled frame,
/// and intrinsics are in pixels, so they must be scaled with it. Recovering a
/// pose with full-resolution intrinsics from half-resolution corners puts the
/// tag at twice the distance, which looks entirely plausible and is completely
/// wrong.
public struct CameraIntrinsics: Equatable, Sendable {
    public var fx: Double
    public var fy: Double
    public var cx: Double
    public var cy: Double

    public init(fx: Double, fy: Double, cx: Double, cy: Double) {
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
    }

    public var isUsable: Bool { fx > 0 && fy > 0 && fx.isFinite && fy.isFinite }

    public func scaled(by factor: Double) -> CameraIntrinsics {
        CameraIntrinsics(fx: fx * factor, fy: fy * factor, cx: cx * factor, cy: cy * factor)
    }
}

/// Recovers a tag's pose from the homography that maps its square onto the
/// image.
///
/// Because the tag is planar, the homography already contains the pose: with
/// the tag's own Z axis zero, the projection collapses to
/// `H ∝ K · [s·r₁ | s·r₂ | t]`, where `s` is half the tag's edge. Multiplying
/// through by `K⁻¹` leaves the first two rotation columns scaled by a single
/// unknown, and the fact that they must be unit length recovers that scale.
/// The third column is their cross product.
public enum AprilTagPoseEstimator {

    /// - Parameters:
    ///   - homography: canonical tag square to image pixels.
    ///   - intrinsics: for the same image the homography is in.
    ///   - tagSize: the **outer edge of the black border**, in metres.
    /// - Returns: the tag's pose in the camera's optical frame — `+X` right,
    ///   `+Y` down, `+Z` forward, the frame the intrinsics are defined in.
    public static func pose(
        from homography: Homography,
        intrinsics: CameraIntrinsics,
        tagSize: Double
    ) -> Pose? {
        guard intrinsics.isUsable, tagSize > 0 else { return nil }
        let halfSize = tagSize / 2
        let h = homography.m

        // K⁻¹ H, computed directly: K⁻¹ is upper triangular and trivial.
        var m = [Double](repeating: 0, count: 9)
        for column in 0..<3 {
            let x = h[0 + column], y = h[3 + column], z = h[6 + column]
            m[0 + column] = (x - intrinsics.cx * z) / intrinsics.fx
            m[3 + column] = (y - intrinsics.cy * z) / intrinsics.fy
            m[6 + column] = z
        }

        var column1 = Vector3(m[0], m[3], m[6])
        var column2 = Vector3(m[1], m[4], m[7])
        var translationColumn = Vector3(m[2], m[5], m[8])

        let scale1 = column1.length, scale2 = column2.length
        guard scale1 > 1e-12, scale2 > 1e-12 else { return nil }
        // Averaging the two is the standard estimate; they differ only by noise
        // in the corner positions, and using either alone biases the result.
        var scale = (scale1 + scale2) / 2

        // A homography is only defined up to sign, and the wrong sign puts the
        // tag behind the camera — a mirrored pose that reprojects perfectly.
        // The tag being in front is what breaks the tie.
        if translationColumn.z < 0 {
            column1 = -column1
            column2 = -column2
            translationColumn = -translationColumn
        }
        guard scale.isFinite, scale > 0 else { return nil }

        var r1 = column1 * (1.0 / scale)
        var r2 = column2 * (1.0 / scale)
        // Corner noise leaves r₁ and r₂ neither unit nor perpendicular. Symmetric
        // Gram-Schmidt splits the error between them rather than treating the
        // first column as truth, so the recovered rotation does not depend on
        // which axis happened to be listed first.
        let error = r1.dot(r2) / 2
        let corrected1 = (r1 - r2 * error).normalized
        let corrected2 = (r2 - r1 * error).normalized
        r1 = corrected1
        r2 = corrected2
        let r3 = r1.cross(r2).normalized

        scale = scale / halfSize
        let translation = translationColumn * (1.0 / scale)
        guard translation.isFinite, translation.z > 0 else { return nil }

        // Column-major: the columns are the images of the tag's own axes.
        let rotation = Quaternion(rotationMatrixColumnMajor: [
            r1.x, r1.y, r1.z,
            r2.x, r2.y, r2.z,
            r3.x, r3.y, r3.z,
        ])
        guard rotation.isFinite else { return nil }
        return Pose(position: translation, orientation: rotation.normalized)
    }

    /// Mean corner reprojection error in pixels.
    ///
    /// The homography solve is exact, so this cannot check the fit — it checks
    /// that the *rigid pose* extracted from it still explains the corners. A
    /// quad that is not really a square in the world reprojects badly here even
    /// though its homography was perfect, which is what makes this worth
    /// computing.
    public static func reprojectionError(
        pose: Pose,
        intrinsics: CameraIntrinsics,
        tagSize: Double,
        corners: [Vector2]
    ) -> Double {
        let halfSize = tagSize / 2
        let objectPoints = [
            Vector3(-halfSize, -halfSize, 0),
            Vector3(-halfSize, halfSize, 0),
            Vector3(halfSize, halfSize, 0),
            Vector3(halfSize, -halfSize, 0),
        ]
        var total = 0.0
        for (index, objectPoint) in objectPoints.enumerated() {
            let camera = pose.apply(to: objectPoint)
            guard camera.z > 1e-6 else { return .infinity }
            let projected = Vector2(
                intrinsics.fx * camera.x / camera.z + intrinsics.cx,
                intrinsics.fy * camera.y / camera.z + intrinsics.cy
            )
            total += projected.distance(to: corners[index])
        }
        return total / Double(objectPoints.count)
    }
}
