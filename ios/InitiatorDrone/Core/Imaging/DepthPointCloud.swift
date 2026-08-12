import Foundation

/// A packed cloud of coloured points, ready to hand to SceneKit.
///
/// Stored as flat `Float`/`UInt8` arrays rather than an array of structs
/// because that is exactly the layout `SCNGeometrySource` wants, and because a
/// cloud is rebuilt on every depth frame: 20k points as `[Vector3]` would be
/// 480 KB of doubles churned several times a second, against 60 KB here.
public struct PointCloudBuffer: Equatable, Sendable {
    /// `count * 3` floats, x/y/z interleaved.
    public var positions: [Float]
    /// `count * 3` bytes, r/g/b interleaved.
    public var colors: [UInt8]
    public var count: Int
    /// ROS header stamp of the depth frame this came from.
    public var stamp: Double

    public init(positions: [Float], colors: [UInt8], count: Int, stamp: Double) {
        self.positions = positions
        self.colors = colors
        self.count = count
        self.stamp = stamp
    }

    public static let empty = PointCloudBuffer(positions: [], colors: [], count: 0, stamp: 0)

    public var isEmpty: Bool { count == 0 }
}

/// How much of the depth frame to turn into points.
public struct PointCloudSettings: Equatable, Codable, Sendable {
    public var isEnabled: Bool
    /// Sample every Nth pixel in each axis. 2 means a quarter of the pixels.
    public var pixelStride: Int
    /// Hard cap; the stride is widened automatically to stay under it.
    public var maximumPoints: Int
    /// Samples nearer than this are dropped, in metres.
    public var minimumDepth: Double
    /// Samples further than this are dropped, in metres.
    public var maximumDepth: Double
    /// On-screen point size.
    public var pointSize: Double

    public init(
        isEnabled: Bool = true,
        pixelStride: Int = 2,
        maximumPoints: Int = 20_000,
        minimumDepth: Double = 0.2,
        maximumDepth: Double = 8.0,
        pointSize: Double = 6.0
    ) {
        self.isEnabled = isEnabled
        self.pixelStride = max(1, pixelStride)
        self.maximumPoints = max(500, maximumPoints)
        self.minimumDepth = minimumDepth
        self.maximumDepth = maximumDepth
        self.pointSize = pointSize
    }

    public static let `default` = PointCloudSettings()
}

/// Turns a metric depth image into a coloured point cloud in the robot's
/// body frame, expressed in ARKit axes.
///
/// ## The frames involved
///
/// A depth image's pixels live in the **camera optical frame**, which ROS
/// defines (REP-103) as `+X` right, `+Y` down, `+Z` forward — different again
/// from both `base_link` and ARKit. The standard pinhole deprojection gives a
/// point in that frame:
///
/// ```text
/// Z = depth(u, v)
/// X = (u - cx) * Z / fx
/// Y = (v - cy) * Z / fy
/// ```
///
/// Getting from there to something a SceneKit node can draw is two more hops —
/// optical to `base_link`, then `base_link` to ARKit — and composing them by
/// hand collapses to something pleasingly simple:
///
/// ```text
/// optical -> base_link :  x_b =  Z,  y_b = -X,  z_b = -Y
/// base_link -> ARKit   :  x_a = -y_b, y_a = z_b, z_a = -x_b     (FrameConversion)
/// composed             :  x_a =  X,  y_a = -Y,  z_a = -Z
/// ```
///
/// So an optical-frame point maps to ARKit node axes by flipping Y and Z. That
/// is worth stating explicitly because it looks too simple to be a real
/// transform, and `DepthPointCloudTests` pins down each hop separately so a
/// future change cannot quietly break one of them.
///
/// That composition is the *identity-mount* case. A camera bolted somewhere
/// other than `base_link`, or tilted, inserts its own rotation and offset
/// between the first two hops — see `CameraExtrinsics`, which folds all of it
/// into one `OpticalToNodeTransform` so the per-pixel cost stays the same.
///
/// The resulting points are meant to be attached as a **child of the robot
/// marker node**, which already carries the `odom`-to-ARKit pose. They inherit
/// the robot's position and heading for free.
public enum DepthPointCloud {

    /// Builds a cloud from a decoded depth image.
    ///
    /// - Parameters:
    ///   - image: metric depth, with `NaN` for invalid samples.
    ///   - cameraInfo: intrinsics for the same stream.
    ///   - settings: density and range limits.
    ///   - extrinsics: where the camera is mounted on the robot.
    ///   - colorFor: maps a depth in metres to a colour, so the cloud matches
    ///     the 2D view's colour map.
    public static func build(
        from image: ScalarImage,
        cameraInfo: CameraInfoMessage,
        settings: PointCloudSettings,
        extrinsics: CameraExtrinsics = .identity,
        stamp: Double = 0,
        colorFor: (Double) -> RGBColor
    ) -> PointCloudBuffer {
        guard settings.isEnabled,
              image.width > 0, image.height > 0,
              let intrinsics = scaledIntrinsics(for: image, cameraInfo: cameraInfo) else {
            return .empty
        }

        // Hoisted out of the loop: the mount is constant for the whole frame.
        let toNode = extrinsics.opticalToARNode

        // Widen the stride until the worst case fits the cap, so a large frame
        // cannot blow the budget before a single point is emitted.
        var stride = max(1, settings.pixelStride)
        while (image.width / stride) * (image.height / stride) > settings.maximumPoints {
            stride += 1
        }

        let estimated = (image.width / stride) * (image.height / stride)
        var positions = [Float]()
        var colors = [UInt8]()
        positions.reserveCapacity(estimated * 3)
        colors.reserveCapacity(estimated * 3)

        let minimum = Float(settings.minimumDepth)
        let maximum = Float(settings.maximumDepth)
        var count = 0

        for v in Swift.stride(from: 0, to: image.height, by: stride) {
            let row = v * image.width
            for u in Swift.stride(from: 0, to: image.width, by: stride) {
                let depth = image.values[row + u]
                // NaN fails every comparison, so invalid samples drop out here
                // without needing their own branch.
                guard depth >= minimum, depth <= maximum else { continue }

                let z = Double(depth)
                let x = (Double(u) - intrinsics.cx) * z / intrinsics.fx
                let y = (Double(v) - intrinsics.cy) * z / intrinsics.fy

                // optical -> ARKit node axes, mount included.
                let node = toNode.apply(x: x, y: y, z: z)
                positions.append(Float(node.x))
                positions.append(Float(node.y))
                positions.append(Float(node.z))

                let color = colorFor(z)
                colors.append(color.red)
                colors.append(color.green)
                colors.append(color.blue)
                count += 1
            }
        }

        return PointCloudBuffer(positions: positions, colors: colors, count: count, stamp: stamp)
    }

    /// The intrinsics to use for an image of this size.
    ///
    /// When the `CameraInfo` dimensions match the image, its values are used
    /// directly. When they differ the intrinsics are scaled, which is right for
    /// a resized stream — a publisher that *crops* should be shifting its
    /// principal point and republishing, and if it does, the sizes match and
    /// this path is never taken.
    static func scaledIntrinsics(
        for image: ScalarImage,
        cameraInfo: CameraInfoMessage
    ) -> (fx: Double, fy: Double, cx: Double, cy: Double)? {
        guard let fx = cameraInfo.focalLengthX, let fy = cameraInfo.focalLengthY,
              let cx = cameraInfo.principalPointX, let cy = cameraInfo.principalPointY,
              fx > 0, fy > 0 else { return nil }

        guard cameraInfo.width > 0, cameraInfo.height > 0,
              cameraInfo.width != image.width || cameraInfo.height != image.height else {
            return (fx, fy, cx, cy)
        }

        let scaleX = Double(image.width) / Double(cameraInfo.width)
        let scaleY = Double(image.height) / Double(cameraInfo.height)
        return (fx * scaleX, fy * scaleY, cx * scaleX, cy * scaleY)
    }

    /// Deprojects a single pixel to the camera optical frame. Split out so the
    /// pinhole maths can be tested on its own.
    static func opticalPoint(
        u: Double,
        v: Double,
        depth: Double,
        fx: Double,
        fy: Double,
        cx: Double,
        cy: Double
    ) -> Vector3 {
        Vector3((u - cx) * depth / fx, (v - cy) * depth / fy, depth)
    }

    /// Re-expresses an optical-frame point in the robot's `base_link` frame,
    /// ignoring the mount. Kept as the pure axis-convention hop so the tests can
    /// check it independently of `CameraExtrinsics`.
    static func bodyPoint(fromOptical point: Vector3) -> Vector3 {
        Vector3(point.z, -point.x, -point.y)
    }
}
