import SwiftUI
import MetalKit

struct SceneView: View {
    let model: DroneModel
    let connect: () -> Void
    @State private var camera = SceneCamera()
    @State private var lastDrag = CGSize.zero
    @State private var lastMagnification: CGFloat = 1
    @State private var rendererError: String?
    @State private var fitted = false

    var body: some View {
        ZStack {
            MetalScene(model: model, camera: camera, error: $rendererError)
                .ignoresSafeArea()
                .gesture(DragGesture().onChanged { value in
                    camera.drag(x: Float(value.translation.width - lastDrag.width), y: Float(value.translation.height - lastDrag.height))
                    lastDrag = value.translation
                }.onEnded { _ in lastDrag = .zero })
                .simultaneousGesture(MagnifyGesture().onChanged { value in
                    camera.zoom(Float(value.magnification / lastMagnification))
                    lastMagnification = value.magnification
                }.onEnded { _ in lastMagnification = 1 })
                .accessibilityElement(children: .ignore)
                .accessibilityLabel("3D scene, \(model.cloud.count) coloured points")
                .accessibilityHint("Use the View menu to change scene controls.")
                .accessibilityAction(named: "Orbit left") { camera.yaw -= 0.2 }
                .accessibilityAction(named: "Orbit right") { camera.yaw += 0.2 }
                .accessibilityAction(named: "Look up") { camera.pitch = min(1.5, camera.pitch + 0.2) }
                .accessibilityAction(named: "Look down") { camera.pitch = max(-1.5, camera.pitch - 0.2) }

            if let error = rendererError ?? model.sceneError ?? model.connectionError {
                ContentUnavailableView("Scene unavailable", systemImage: "exclamationmark.triangle", description: Text(error))
                    .allowsHitTesting(false)
            } else if model.cloud.count == 0 {
                ContentUnavailableView("Waiting for camera", systemImage: "cube.transparent", description: Text(model.state?.reason ?? "Connecting to your module…"))
                    .allowsHitTesting(false)
            }
        }
        .toolbar {
            ToolbarItem(placement: .topBarLeading) {
                Button("Connection", systemImage: "network", action: connect)
            }
            ToolbarItem(placement: .principal) { SceneStatus(model: model) }
            ToolbarItem(placement: .topBarTrailing) {
                Button("Fit", systemImage: "arrow.up.left.and.arrow.down.right") { camera.fit(model.cloud, position: model.state?.position) }
            }
            ToolbarItem(placement: .topBarTrailing) {
                Menu("View", systemImage: "slider.horizontal.3") {
                    Toggle("Follow camera", isOn: $camera.followsCamera)
                    Toggle("Show trajectory", isOn: $camera.showsTrajectory)
                    Toggle("Drag to pan", isOn: $camera.pans)
                    Button("Zoom in") { camera.zoom(1.3) }
                    Button("Zoom out") { camera.zoom(1 / 1.3) }
                }
            }
        }
        .onChange(of: model.cloudRevision) {
            if !fitted && model.cloud.count > 0 {
                camera.fit(model.cloud, position: model.state?.position)
                fitted = true
            }
        }
        .onChange(of: model.connectionID) { fitted = false; camera.fit(.empty, position: nil) }
        .onChange(of: model.state?.frame) {
            if camera.followsCamera, let position = model.state?.position { camera.target = SceneCamera.displayPoint(position) }
        }
        .onChange(of: camera.followsCamera) {
            if camera.followsCamera, let position = model.state?.position { camera.target = SceneCamera.displayPoint(position) }
        }
    }
}

private struct SceneStatus: View {
    let model: DroneModel
    var body: some View {
        TimelineView(.periodic(from: .now, by: 1)) { context in
            VStack(spacing: 2) {
                Text(model.state?.isDemo == true ? "SIMULATION" : model.trackingLabel(at: context.date))
                    .font(.subheadline.weight(.semibold))
                Text(model.state?.recording == true ? "Recording · \(model.cloud.count.formatted()) points" : "\(model.cloud.count.formatted()) points")
                    .font(.caption).foregroundStyle(.secondary).monospacedDigit()
            }
        }
        .allowsHitTesting(false)
    }
}

private struct MetalScene: UIViewRepresentable {
    let model: DroneModel
    let camera: SceneCamera
    @Binding var error: String?

    final class Coordinator { var renderer: SceneRenderer? }
    func makeCoordinator() -> Coordinator { Coordinator() }

    func makeUIView(context: Context) -> MTKView {
        let view = MTKView(frame: .zero, device: MTLCreateSystemDefaultDevice())
        view.clearColor = MTLClearColor(red: 0.045, green: 0.075, blue: 0.095, alpha: 1)
        view.depthStencilPixelFormat = .depth32Float
        view.isPaused = true
        view.enableSetNeedsDisplay = true
        do {
            let renderer = try SceneRenderer(view: view)
            context.coordinator.renderer = renderer
            view.delegate = renderer
        } catch {
            Task { @MainActor in self.error = error.localizedDescription }
        }
        return view
    }

    func updateUIView(_ view: MTKView, context: Context) {
        // Read camera properties here so Observation invalidates this view for gestures.
        let matrix = camera.matrix(aspect: Float(max(1, view.bounds.width) / max(1, view.bounds.height)))
        context.coordinator.renderer?.update(model: model, camera: camera, matrix: matrix)
        view.setNeedsDisplay()
    }
}

@MainActor private final class SceneRenderer: NSObject, MTKViewDelegate {
    private let device: MTLDevice
    private let queue: MTLCommandQueue
    private let pointPipeline: MTLRenderPipelineState
    private let linePipeline: MTLRenderPipelineState
    private let depth: MTLDepthStencilState
    private var cloudBuffer: MTLBuffer?
    private var cloudCount = 0
    private var revision = -1
    private var lines: MTLBuffer?
    private var lineCount = 0
    private var matrix = matrix_identity_float4x4
    private var camera: SceneCamera?

    init(view: MTKView) throws {
        guard let device = view.device, let queue = device.makeCommandQueue(),
              let library = device.makeDefaultLibrary() else { throw APIError.invalidData }
        self.device = device
        self.queue = queue
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "sceneVertex")
        descriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.fragmentFunction = library.makeFunction(name: "scenePoint")
        pointPipeline = try device.makeRenderPipelineState(descriptor: descriptor)
        descriptor.fragmentFunction = library.makeFunction(name: "sceneLine")
        linePipeline = try device.makeRenderPipelineState(descriptor: descriptor)
        let depthDescriptor = MTLDepthStencilDescriptor()
        depthDescriptor.depthCompareFunction = .lessEqual
        depthDescriptor.isDepthWriteEnabled = true
        guard let depth = device.makeDepthStencilState(descriptor: depthDescriptor) else { throw APIError.invalidData }
        self.depth = depth
    }

    func update(model: DroneModel, camera: SceneCamera, matrix: simd_float4x4) {
        self.matrix = matrix
        self.camera = camera
        if revision != model.cloudRevision {
            let vertices = model.cloud.vertices
            cloudBuffer = vertices.isEmpty ? nil : device.makeBuffer(bytes: vertices, length: vertices.count * 4)
            cloudCount = model.cloud.count
            revision = model.cloudRevision
        }
        var vertices = [Float]()
        func line(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ color: SIMD3<Float>) {
            vertices += [a.x, a.y, a.z, color.x, color.y, color.z, b.x, b.y, b.z, color.x, color.y, color.z]
        }
        for i in -10...10 {
            let n = Float(i)
            line(SIMD3(n, -1, -10), SIMD3(n, -1, 10), SIMD3(0.15, 0.23, 0.27))
            line(SIMD3(-10, -1, n), SIMD3(10, -1, n), SIMD3(0.15, 0.23, 0.27))
        }
        line(.zero, SIMD3(0.45, 0, 0), SIMD3(1, 0.3, 0.3))
        line(.zero, SIMD3(0, -0.45, 0), SIMD3(0.3, 1, 0.3))
        line(.zero, SIMD3(0, 0, -0.45), SIMD3(0.3, 0.5, 1))
        if let state = model.state {
            if camera.showsTrajectory {
                for (a, b) in zip(state.trajectory, state.trajectory.dropFirst()) {
                    line(SceneCamera.displayPoint(SIMD3(a[0], a[1], a[2])), SceneCamera.displayPoint(SIMD3(b[0], b[1], b[2])), SIMD3(0.93, 0.72, 0.42))
                }
            }
            func world(_ p: SIMD3<Float>) -> SIMD3<Float> {
                let rows = state.pose
                return SceneCamera.displayPoint(SIMD3(
                    rows[0][0]*p.x + rows[0][1]*p.y + rows[0][2]*p.z + rows[0][3],
                    rows[1][0]*p.x + rows[1][1]*p.y + rows[1][2]*p.z + rows[1][3],
                    rows[2][0]*p.x + rows[2][1]*p.y + rows[2][2]*p.z + rows[2][3]))
            }
            let corners: [SIMD3<Float>] = [SIMD3(-0.16, -0.12, 0.25), SIMD3(0.16, -0.12, 0.25), SIMD3(0.16, 0.12, 0.25), SIMD3(-0.16, 0.12, 0.25)]
            let color: SIMD3<Float> = model.isConnected() && state.status == "tracking" ? SIMD3(0.4, 0.88, 0.75) : SIMD3(0.95, 0.65, 0.3)
            for i in 0..<4 {
                line(world(.zero), world(corners[i]), color)
                line(world(corners[i]), world(corners[(i + 1) % 4]), color)
            }
        }
        lines = device.makeBuffer(bytes: vertices, length: vertices.count * 4)
        lineCount = vertices.count / 6
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) { view.setNeedsDisplay() }

    func draw(in view: MTKView) {
        guard let pass = view.currentRenderPassDescriptor, let drawable = view.currentDrawable,
              let command = queue.makeCommandBuffer(), let encoder = command.makeRenderCommandEncoder(descriptor: pass) else { return }
        if let camera {
            matrix = camera.matrix(aspect: Float(max(1, view.drawableSize.width) / max(1, view.drawableSize.height)))
        }
        encoder.setDepthStencilState(depth)
        encoder.setVertexBytes(&matrix, length: MemoryLayout<simd_float4x4>.stride, index: 1)
        if let cloudBuffer, cloudCount > 0 {
            encoder.setRenderPipelineState(pointPipeline)
            encoder.setVertexBuffer(cloudBuffer, offset: 0, index: 0)
            encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: cloudCount)
        }
        encoder.setRenderPipelineState(linePipeline)
        encoder.setVertexBuffer(lines, offset: 0, index: 0)
        encoder.drawPrimitives(type: .line, vertexStart: 0, vertexCount: lineCount)
        encoder.endEncoding()
        command.present(drawable)
        command.commit()
    }
}
