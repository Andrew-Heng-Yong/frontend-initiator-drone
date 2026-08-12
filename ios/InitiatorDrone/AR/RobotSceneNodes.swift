#if canImport(ARKit)
import ARKit
import SceneKit
import UIKit
import simd

/// Builds the SceneKit content drawn over the camera feed.
///
/// A note that runs through all of it: a node's local forward axis in SceneKit
/// is `-Z`. `FrameConversion` maps ROS `+X` (robot forward) onto exactly that,
/// so a node given the converted orientation already points where the robot
/// points, and the ROS body axes land on node axes as:
///
/// ```text
/// ROS +X (forward) -> node -Z
/// ROS +Y (left)    -> node -X
/// ROS +Z (up)      -> node +Y
/// ```
public enum RobotSceneNodes {

    public enum Palette {
        public static let tracking = UIColor(red: 0.20, green: 0.85, blue: 0.55, alpha: 1.0)
        public static let degraded = UIColor(red: 1.00, green: 0.72, blue: 0.20, alpha: 1.0)
        public static let invalid = UIColor(red: 0.95, green: 0.35, blue: 0.35, alpha: 1.0)
        public static let origin = UIColor(red: 0.45, green: 0.65, blue: 1.00, alpha: 1.0)
        public static let axisX = UIColor(red: 0.95, green: 0.30, blue: 0.30, alpha: 1.0)
        public static let axisY = UIColor(red: 0.35, green: 0.90, blue: 0.40, alpha: 1.0)
        public static let axisZ = UIColor(red: 0.40, green: 0.60, blue: 1.00, alpha: 1.0)
    }

    public static func color(for status: RobotTrackingStatus) -> UIColor {
        switch status {
        case .tracking: return Palette.tracking
        case .orientationOnly: return Palette.degraded
        case .stale: return Palette.invalid
        case .notCalibrated, .unknown: return Palette.invalid
        }
    }

    // MARK: - Robot marker

    /// The robot marker: a body, a forward arrow, and a ROS axis triad.
    ///
    /// Named children let the render loop recolour it without rebuilding
    /// geometry every frame.
    public static func makeRobotNode() -> SCNNode {
        let root = SCNNode()
        root.name = "robot"

        let body = SCNBox(width: 0.26, height: 0.09, length: 0.26, chamferRadius: 0.012)
        let bodyMaterial = SCNMaterial()
        bodyMaterial.diffuse.contents = Palette.tracking
        bodyMaterial.emission.contents = Palette.tracking.withAlphaComponent(0.35)
        bodyMaterial.transparency = 0.85
        bodyMaterial.isDoubleSided = true
        body.materials = [bodyMaterial]

        let bodyNode = SCNNode(geometry: body)
        bodyNode.name = "robotBody"
        root.addChildNode(bodyNode)

        // Forward arrow along the node's -Z, which is the robot's +X.
        let cone = SCNCone(topRadius: 0, bottomRadius: 0.05, height: 0.16)
        let coneMaterial = SCNMaterial()
        coneMaterial.diffuse.contents = Palette.tracking
        coneMaterial.emission.contents = Palette.tracking.withAlphaComponent(0.5)
        cone.materials = [coneMaterial]

        let coneNode = SCNNode(geometry: cone)
        coneNode.name = "robotArrow"
        // A cone points along +Y by default; rotate it onto -Z.
        coneNode.eulerAngles = SCNVector3(-Float.pi / 2, 0, 0)
        coneNode.position = SCNVector3(0, 0, -0.20)
        root.addChildNode(coneNode)

        root.addChildNode(makeAxisTriad(length: 0.22, radius: 0.006))
        return root
    }

    /// A triad drawn in ROS colour convention (X red, Y green, Z blue) but laid
    /// out on the node axes those ROS axes map to.
    public static func makeAxisTriad(length: CGFloat, radius: CGFloat) -> SCNNode {
        let root = SCNNode()
        root.name = "axes"

        func axis(color: UIColor, direction: SCNVector3) -> SCNNode {
            let cylinder = SCNCylinder(radius: radius, height: length)
            let material = SCNMaterial()
            material.diffuse.contents = color
            material.emission.contents = color.withAlphaComponent(0.6)
            material.lightingModel = .constant
            cylinder.materials = [material]

            let node = SCNNode(geometry: cylinder)
            // A cylinder runs along +Y; aim it along `direction`, then push it
            // out by half its length so it starts at the origin.
            node.simdOrientation = simd_quatf(
                from: SIMD3<Float>(0, 1, 0),
                to: simd_normalize(SIMD3<Float>(direction.x, direction.y, direction.z))
            )
            node.simdPosition = simd_normalize(SIMD3<Float>(direction.x, direction.y, direction.z))
                * Float(length / 2)
            return node
        }

        root.addChildNode(axis(color: Palette.axisX, direction: SCNVector3(0, 0, -1))) // ROS +X
        root.addChildNode(axis(color: Palette.axisY, direction: SCNVector3(-1, 0, 0))) // ROS +Y
        root.addChildNode(axis(color: Palette.axisZ, direction: SCNVector3(0, 1, 0)))  // ROS +Z
        return root
    }

    /// A marker for the robot's `odom` origin, so the operator can see whether
    /// the alignment landed where they meant it to.
    public static func makeOriginNode() -> SCNNode {
        let root = SCNNode()
        root.name = "odomOrigin"

        let ring = SCNTorus(ringRadius: 0.18, pipeRadius: 0.006)
        let material = SCNMaterial()
        material.diffuse.contents = Palette.origin
        material.emission.contents = Palette.origin.withAlphaComponent(0.6)
        material.lightingModel = .constant
        ring.materials = [material]

        let ringNode = SCNNode(geometry: ring)
        root.addChildNode(ringNode)
        root.addChildNode(makeAxisTriad(length: 0.3, radius: 0.004))
        return root
    }

    // MARK: - Camera frustum

    /// A wireframe pyramid showing where the robot's depth camera is looking.
    ///
    /// Built with its apex at the origin and its axis along `-Z`, the node's own
    /// forward direction. Where that ends up on the robot is the caller's job:
    /// it applies `CameraExtrinsics.poseInARNode`, so the mount offset and tilt
    /// are a transform on this node rather than something baked into the mesh.
    public static func makeFrustumNode(
        horizontalFOV: Double,
        verticalFOV: Double,
        range: Double
    ) -> SCNNode {
        let halfWidth = Float(range * tan(horizontalFOV / 2))
        let halfHeight = Float(range * tan(verticalFOV / 2))
        let depth = Float(-range) // forward is -Z

        let apex = SCNVector3(0, 0, 0)
        let corners = [
            SCNVector3(-halfWidth, halfHeight, depth),
            SCNVector3(halfWidth, halfHeight, depth),
            SCNVector3(halfWidth, -halfHeight, depth),
            SCNVector3(-halfWidth, -halfHeight, depth),
        ]

        var vertices: [SCNVector3] = []
        for corner in corners {
            vertices.append(apex)
            vertices.append(corner)
        }
        for index in 0..<4 {
            vertices.append(corners[index])
            vertices.append(corners[(index + 1) % 4])
        }

        let node = SCNNode(geometry: makeLineGeometry(vertices: vertices, color: Palette.origin))
        node.name = "frustum"
        return node
    }

    // MARK: - Point cloud

    /// Builds point geometry from a deprojected depth frame.
    ///
    /// SceneKit draws a `.point` primitive straight from the vertex source when
    /// the element carries no index data, which is what makes this cheap — the
    /// two `Data` blobs come out of `PointCloudBuffer` already in the exact
    /// layout the sources want, with no per-point object allocated anywhere.
    ///
    /// The result is meant to be attached under the robot marker node, so the
    /// cloud inherits the robot's `odom`-to-ARKit pose.
    public static func makePointCloudGeometry(
        from cloud: PointCloudBuffer,
        pointSize: CGFloat
    ) -> SCNGeometry? {
        guard cloud.count > 0,
              cloud.positions.count == cloud.count * 3,
              cloud.colors.count == cloud.count * 3 else { return nil }

        let positionData = cloud.positions.withUnsafeBufferPointer { Data(buffer: $0) }
        let colorData = cloud.colors.withUnsafeBufferPointer { Data(buffer: $0) }

        let vertexSource = SCNGeometrySource(
            data: positionData,
            semantic: .vertex,
            vectorCount: cloud.count,
            usesFloatComponents: true,
            componentsPerVector: 3,
            bytesPerComponent: MemoryLayout<Float>.size,
            dataOffset: 0,
            dataStride: MemoryLayout<Float>.size * 3
        )

        let colorSource = SCNGeometrySource(
            data: colorData,
            semantic: .color,
            vectorCount: cloud.count,
            usesFloatComponents: false,
            componentsPerVector: 3,
            bytesPerComponent: MemoryLayout<UInt8>.size,
            dataOffset: 0,
            dataStride: MemoryLayout<UInt8>.size * 3
        )

        // Nil index data means "draw the vertices in order", which is exactly
        // what a point cloud wants and avoids building an index buffer.
        let element = SCNGeometryElement(
            data: nil,
            primitiveType: .point,
            primitiveCount: cloud.count,
            bytesPerIndex: MemoryLayout<Int32>.size
        )
        element.pointSize = pointSize
        element.minimumPointScreenSpaceRadius = 1.0
        element.maximumPointScreenSpaceRadius = max(1.0, pointSize)

        let geometry = SCNGeometry(sources: [vertexSource, colorSource], elements: [element])
        let material = SCNMaterial()
        // Constant lighting with a white diffuse lets the per-vertex colours
        // through untouched; anything else would relight the depth colour map.
        material.lightingModel = .constant
        material.diffuse.contents = UIColor.white
        material.isDoubleSided = true
        material.writesToDepthBuffer = true
        geometry.materials = [material]
        return geometry
    }

    // MARK: - Trail

    /// A polyline through the robot's recent positions.
    public static func makeTrailGeometry(points: [Vector3], color: UIColor) -> SCNGeometry? {
        guard points.count >= 2 else { return nil }
        var vertices: [SCNVector3] = []
        vertices.reserveCapacity((points.count - 1) * 2)
        for index in 0..<(points.count - 1) {
            vertices.append(SCNVector3(Float(points[index].x), Float(points[index].y), Float(points[index].z)))
            vertices.append(SCNVector3(
                Float(points[index + 1].x),
                Float(points[index + 1].y),
                Float(points[index + 1].z)
            ))
        }
        return makeLineGeometry(vertices: vertices, color: color)
    }

    /// Builds a `.line` primitive from vertex pairs.
    public static func makeLineGeometry(vertices: [SCNVector3], color: UIColor) -> SCNGeometry {
        let source = SCNGeometrySource(vertices: vertices)
        let indices = (0..<vertices.count).map { UInt32($0) }
        let element = SCNGeometryElement(
            data: Data(bytes: indices, count: indices.count * MemoryLayout<UInt32>.size),
            primitiveType: .line,
            primitiveCount: vertices.count / 2,
            bytesPerIndex: MemoryLayout<UInt32>.size
        )

        let geometry = SCNGeometry(sources: [source], elements: [element])
        let material = SCNMaterial()
        material.diffuse.contents = color
        material.emission.contents = color
        material.lightingModel = .constant
        material.isDoubleSided = true
        geometry.materials = [material]
        return geometry
    }

    // MARK: - Placement preview

    /// The disc and arrow shown while the operator is placing the origin.
    public static func makePlacementPreviewNode() -> SCNNode {
        let root = SCNNode()
        root.name = "placementPreview"

        let disc = SCNCylinder(radius: 0.2, height: 0.004)
        let discMaterial = SCNMaterial()
        discMaterial.diffuse.contents = Palette.origin.withAlphaComponent(0.4)
        discMaterial.emission.contents = Palette.origin.withAlphaComponent(0.3)
        discMaterial.lightingModel = .constant
        disc.materials = [discMaterial]
        root.addChildNode(SCNNode(geometry: disc))

        let arrow = SCNCone(topRadius: 0, bottomRadius: 0.06, height: 0.22)
        let arrowMaterial = SCNMaterial()
        arrowMaterial.diffuse.contents = Palette.origin
        arrowMaterial.emission.contents = Palette.origin.withAlphaComponent(0.6)
        arrowMaterial.lightingModel = .constant
        arrow.materials = [arrowMaterial]

        let arrowNode = SCNNode(geometry: arrow)
        arrowNode.name = "previewArrow"
        arrowNode.eulerAngles = SCNVector3(-Float.pi / 2, 0, 0)
        arrowNode.position = SCNVector3(0, 0.02, -0.28)
        root.addChildNode(arrowNode)

        return root
    }
}
#endif
