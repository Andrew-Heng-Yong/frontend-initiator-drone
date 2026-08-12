#if canImport(ARKit)
import ARKit
import SceneKit
import SwiftUI
import simd

/// The camera view with the robot drawn into it.
///
/// The robot's pose is sampled inside SceneKit's render callback, at the
/// timestamp of the frame about to be drawn, rather than being pushed in
/// whenever a rosbridge message happens to arrive. That is what keeps the
/// marker steady: the phone renders at 60 Hz, odometry arrives at maybe 30 Hz
/// and jittery, and interpolating to the render instant is the difference
/// between a marker that glides and one that stutters.
public struct ARRobotSceneView: UIViewRepresentable {

    public var sampler: OdometrySampler
    public var pointCloudStore: PointCloudStore
    public var trackingStatus: RobotTrackingStatus
    public var cameraInfo: CameraInfoMessage?
    public var showsFrustum: Bool
    public var showsTrail: Bool
    public var showsPointCloud: Bool
    public var pointSize: Double
    public var cameraExtrinsics: CameraExtrinsics
    public var placementPhase: AlignmentController.Phase
    public var previewPosition: Vector3?
    public var pendingYaw: Double
    public var isAligned: Bool

    /// Called with a raycast hit when the operator taps during placement.
    public var onTapPlacement: ((Vector3) -> Void)?
    /// Called continuously with the crosshair raycast while placing.
    public var onPreviewUpdate: ((Vector3?) -> Void)?

    public init(
        sampler: OdometrySampler,
        pointCloudStore: PointCloudStore,
        trackingStatus: RobotTrackingStatus,
        cameraInfo: CameraInfoMessage?,
        showsFrustum: Bool,
        showsTrail: Bool,
        showsPointCloud: Bool,
        pointSize: Double,
        cameraExtrinsics: CameraExtrinsics,
        placementPhase: AlignmentController.Phase,
        previewPosition: Vector3?,
        pendingYaw: Double,
        isAligned: Bool,
        session: ARSession,
        onTapPlacement: ((Vector3) -> Void)? = nil,
        onPreviewUpdate: ((Vector3?) -> Void)? = nil
    ) {
        self.sampler = sampler
        self.pointCloudStore = pointCloudStore
        self.trackingStatus = trackingStatus
        self.cameraInfo = cameraInfo
        self.showsFrustum = showsFrustum
        self.showsTrail = showsTrail
        self.showsPointCloud = showsPointCloud
        self.pointSize = pointSize
        self.cameraExtrinsics = cameraExtrinsics
        self.placementPhase = placementPhase
        self.previewPosition = previewPosition
        self.pendingYaw = pendingYaw
        self.isAligned = isAligned
        self.session = session
        self.onTapPlacement = onTapPlacement
        self.onPreviewUpdate = onPreviewUpdate
    }

    private let session: ARSession

    public func makeCoordinator() -> Coordinator {
        Coordinator(sampler: sampler, pointCloudStore: pointCloudStore)
    }

    public func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView(frame: .zero)
        view.session = session
        view.scene = SCNScene()
        view.automaticallyUpdatesLighting = true
        view.antialiasingMode = .multisampling2X
        view.rendersContinuously = true
        view.delegate = context.coordinator
        view.preferredFramesPerSecond = 60
        view.debugOptions = []

        context.coordinator.attach(to: view)

        let tap = UITapGestureRecognizer(
            target: context.coordinator,
            action: #selector(Coordinator.handleTap(_:))
        )
        view.addGestureRecognizer(tap)

        return view
    }

    public func updateUIView(_ view: ARSCNView, context: Context) {
        context.coordinator.onTapPlacement = onTapPlacement
        context.coordinator.onPreviewUpdate = onPreviewUpdate
        context.coordinator.update(
            trackingStatus: trackingStatus,
            cameraInfo: cameraInfo,
            showsFrustum: showsFrustum,
            showsTrail: showsTrail,
            showsPointCloud: showsPointCloud,
            pointSize: pointSize,
            cameraExtrinsics: cameraExtrinsics,
            placementPhase: placementPhase,
            previewPosition: previewPosition,
            pendingYaw: pendingYaw,
            isAligned: isAligned
        )
    }

    public static func dismantleUIView(_ view: ARSCNView, coordinator: Coordinator) {
        coordinator.detach()
    }

    // MARK: - Coordinator

    /// `ARSCNView.delegate` is typed `ARSCNViewDelegate`, which inherits from
    /// `SCNSceneRendererDelegate` — conforming to the renderer protocol alone is
    /// not enough to be assignable. Every method of both is optional, so the
    /// conformance costs nothing beyond the declaration.
    public final class Coordinator: NSObject, ARSCNViewDelegate {
        private let sampler: OdometrySampler
        private let pointCloudStore: PointCloudStore
        private weak var view: ARSCNView?

        private let robotNode = RobotSceneNodes.makeRobotNode()
        private let originNode = RobotSceneNodes.makeOriginNode()
        private let previewNode = RobotSceneNodes.makePlacementPreviewNode()
        private let trailNode = SCNNode()
        private var frustumNode: SCNNode?
        private let pointCloudNode = SCNNode()
        private var pointCloudGeneration: UInt64 = 0

        /// Written on the main thread by `updateUIView`, read on the render
        /// thread. Small, but a lock is cheaper than a hard-to-reproduce
        /// glitch.
        private let lock = NSLock()
        private var config = Config()
        private var lastTrailRebuild: TimeInterval = 0
        private var lastFrustumSignature: String = ""
        private var previewTimer: Timer?

        var onTapPlacement: ((Vector3) -> Void)?
        var onPreviewUpdate: ((Vector3?) -> Void)?

        private struct Config {
            var trackingStatus: RobotTrackingStatus = .unknown
            var cameraInfo: CameraInfoMessage?
            var showsFrustum = true
            var showsTrail = true
            var showsPointCloud = true
            var pointSize: Double = 6
            var cameraExtrinsics: CameraExtrinsics = .identity
            var placementPhase: AlignmentController.Phase = .idle
            var previewPosition: Vector3?
            var pendingYaw: Double = 0
            var isAligned = false
        }

        init(sampler: OdometrySampler, pointCloudStore: PointCloudStore) {
            self.sampler = sampler
            self.pointCloudStore = pointCloudStore
            super.init()
            trailNode.name = "trail"
            pointCloudNode.name = "pointCloud"
        }

        func attach(to view: ARSCNView) {
            self.view = view
            let root = view.scene.rootNode
            root.addChildNode(robotNode)
            root.addChildNode(originNode)
            root.addChildNode(trailNode)
            root.addChildNode(previewNode)

            // Parented to the robot marker, so the cloud inherits the
            // odom-to-ARKit pose and alignment without a second transform.
            robotNode.addChildNode(pointCloudNode)

            robotNode.isHidden = true
            originNode.isHidden = true
            previewNode.isHidden = true

            startPreviewTimer()
        }

        func detach() {
            previewTimer?.invalidate()
            previewTimer = nil
            view = nil
        }

        func update(
            trackingStatus: RobotTrackingStatus,
            cameraInfo: CameraInfoMessage?,
            showsFrustum: Bool,
            showsTrail: Bool,
            showsPointCloud: Bool,
            pointSize: Double,
            cameraExtrinsics: CameraExtrinsics,
            placementPhase: AlignmentController.Phase,
            previewPosition: Vector3?,
            pendingYaw: Double,
            isAligned: Bool
        ) {
            lock.lock()
            config.trackingStatus = trackingStatus
            config.cameraInfo = cameraInfo
            config.showsFrustum = showsFrustum
            config.showsTrail = showsTrail
            config.showsPointCloud = showsPointCloud
            config.pointSize = pointSize
            config.cameraExtrinsics = cameraExtrinsics
            config.placementPhase = placementPhase
            config.previewPosition = previewPosition
            config.pendingYaw = pendingYaw
            config.isAligned = isAligned
            lock.unlock()
        }

        // MARK: Placement

        /// Raycasts from the screen centre a few times a second while the
        /// operator is aiming. Kept on the main thread and off the render loop,
        /// because `ARSCNView.raycastQuery` is a view API.
        private func startPreviewTimer() {
            previewTimer?.invalidate()
            let timer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
                self?.updatePreviewRaycast()
            }
            previewTimer = timer
        }

        private func updatePreviewRaycast() {
            lock.lock()
            let phase = config.placementPhase
            lock.unlock()

            guard case .pickingPosition = phase, let view = self.view else {
                onPreviewUpdate?(nil)
                return
            }
            let centre = CGPoint(x: view.bounds.midX, y: view.bounds.midY)
            guard let position = raycast(from: centre, in: view) else {
                onPreviewUpdate?(nil)
                return
            }
            onPreviewUpdate?(position)
        }

        private func raycast(from point: CGPoint, in view: ARSCNView) -> Vector3? {
            guard let query = view.raycastQuery(
                from: point,
                allowing: .estimatedPlane,
                alignment: .horizontal
            ) else { return nil }
            guard let result = view.session.raycast(query).first else { return nil }
            let pose = ARKitBridge.pose(from: result.worldTransform)
            return pose.position
        }

        @objc func handleTap(_ recognizer: UITapGestureRecognizer) {
            lock.lock()
            let phase = config.placementPhase
            lock.unlock()

            guard case .pickingPosition = phase, let view = self.view else { return }
            let location = recognizer.location(in: view)
            // Prefer the tapped point, and fall back to the crosshair so a tap
            // that misses a detected plane still does something sensible.
            let hit = raycast(from: location, in: view)
                ?? raycast(from: CGPoint(x: view.bounds.midX, y: view.bounds.midY), in: view)
            guard let hit else { return }
            onTapPlacement?(hit)
        }

        // MARK: Render loop

        public func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
            lock.lock()
            let current = config
            lock.unlock()

            updatePlacementPreview(current)
            updateOrigin(current)
            updateRobot(current, time: time)
            updateTrail(current, time: time)
            updatePointCloud(current)
        }

        private func updatePlacementPreview(_ current: Config) {
            switch current.placementPhase {
            case .idle:
                previewNode.isHidden = true

            case .pickingPosition:
                // Track the crosshair so the operator can see exactly where the
                // origin would land before committing to it.
                guard let position = current.previewPosition else {
                    previewNode.isHidden = true
                    return
                }
                previewNode.isHidden = false
                previewNode.simdPosition = ARKitBridge.vector(position)
                previewNode.simdOrientation = simd_quatf(
                    angle: Float(current.pendingYaw),
                    axis: SIMD3<Float>(0, 1, 0)
                )

            case .adjustingHeading(let position):
                previewNode.isHidden = false
                previewNode.simdPosition = ARKitBridge.vector(position)
                previewNode.simdOrientation = simd_quatf(
                    angle: Float(current.pendingYaw),
                    axis: SIMD3<Float>(0, 1, 0)
                )
            }
        }

        private func updateOrigin(_ current: Config) {
            guard let alignment = sampler.currentAlignment else {
                originNode.isHidden = true
                return
            }
            originNode.isHidden = false
            originNode.simdTransform = ARKitBridge.transform(from: alignment.arFromOdom)
        }

        private func updateRobot(_ current: Config, time: TimeInterval) {
            // The odometry buffer is keyed to the robot's ROS clock, mapped
            // onto wall time by the clock estimator, so the sample time must be
            // wall time and not SceneKit's monotonic `time`.
            let localTime = Date().timeIntervalSince1970

            guard current.isAligned,
                  current.trackingStatus.hasUsablePose,
                  let result = sampler.poseInAR(atLocalTime: localTime) else {
                robotNode.isHidden = true
                frustumNode?.isHidden = true
                return
            }

            robotNode.isHidden = false
            robotNode.simdTransform = ARKitBridge.transform(from: result.pose)

            let color = RobotSceneNodes.color(for: current.trackingStatus)
            recolourMarker(color)

            // A clamped or extrapolated sample means the pose is being held or
            // guessed; fading it is an honest signal that it is not measured.
            switch result.sample.kind {
            case .clampedToNewest, .extrapolated:
                robotNode.opacity = 0.45
            default:
                robotNode.opacity = 1.0
            }

            updateFrustum(current)
        }

        private func recolourMarker(_ color: UIColor) {
            // Only the body and arrow follow the status colour; the axis triad
            // keeps ROS's red/green/blue so it stays readable.
            for name in ["robotBody", "robotArrow"] {
                guard let node = robotNode.childNode(withName: name, recursively: true),
                      let materials = node.geometry?.materials else { continue }
                for material in materials {
                    material.diffuse.contents = color
                    material.emission.contents = color.withAlphaComponent(0.4)
                }
            }
        }

        private func updateFrustum(_ current: Config) {
            guard current.showsFrustum,
                  let info = current.cameraInfo,
                  let horizontal = info.horizontalFieldOfView,
                  let vertical = info.verticalFieldOfView else {
                frustumNode?.isHidden = true
                return
            }

            let signature = String(format: "%.4f-%.4f", horizontal, vertical)
            if signature != lastFrustumSignature || frustumNode == nil {
                frustumNode?.removeFromParentNode()
                let node = RobotSceneNodes.makeFrustumNode(
                    horizontalFOV: horizontal,
                    verticalFOV: vertical,
                    range: 3.0
                )
                robotNode.addChildNode(node)
                frustumNode = node
                lastFrustumSignature = signature
            }
            // The mount moves the node rather than the geometry, so changing the
            // offset never rebuilds the mesh. The point cloud does the opposite
            // — it folds the mount into the vertices — because it is rebuilt
            // every frame anyway and a second transform node would be one more
            // place for the two to disagree.
            frustumNode?.simdTransform = ARKitBridge.transform(
                from: current.cameraExtrinsics.poseInARNode
            )
            frustumNode?.isHidden = false
        }

        /// Rebuilds the point cloud geometry, but only when the depth worker
        /// has actually produced a new one.
        ///
        /// This runs at the display refresh rate against a depth stream of a
        /// few hertz, so the generation check is doing most of the work: it
        /// turns ~57 of every 60 calls into a single integer comparison rather
        /// than rebuilding tens of thousands of vertices.
        private func updatePointCloud(_ current: Config) {
            guard current.showsPointCloud else {
                pointCloudNode.isHidden = true
                return
            }
            pointCloudNode.isHidden = false

            guard let update = pointCloudStore.take(ifNewerThan: pointCloudGeneration) else {
                return
            }
            pointCloudGeneration = update.generation

            guard update.cloud.count > 0 else {
                pointCloudNode.geometry = nil
                return
            }
            pointCloudNode.geometry = RobotSceneNodes.makePointCloudGeometry(
                from: update.cloud,
                pointSize: CGFloat(current.pointSize)
            )
        }

        private func updateTrail(_ current: Config, time: TimeInterval) {
            guard current.showsTrail, current.isAligned else {
                trailNode.isHidden = true
                return
            }
            trailNode.isHidden = false

            // Rebuilding line geometry allocates; five times a second is plenty
            // for a trail and keeps it off the per-frame budget.
            guard time - lastTrailRebuild > 0.2 else { return }
            lastTrailRebuild = time

            let points = sampler.trailInAR(maximumCount: 90)
            trailNode.geometry = RobotSceneNodes.makeTrailGeometry(
                points: points,
                color: RobotSceneNodes.color(for: current.trackingStatus).withAlphaComponent(0.7)
            )
        }
    }
}
#endif

struct Previews_ARRobotSceneView_Previews: PreviewProvider {
    static var previews: some View {
        /*@START_MENU_TOKEN@*/Text("Hello, World!")/*@END_MENU_TOKEN@*/
    }
}
