import SwiftUI
import ARKit
import MetalKit

struct ThermalARView: View {
    @Bindable var model:PhoneReconstruction
    @State private var settings=false
    var body: some View {
        ZStack(alignment:.bottom) {
            ARMetalView(model:model).ignoresSafeArea()
            VStack(spacing:10) {
                Text(!model.renderingError.isEmpty ? model.renderingError : (model.cameraWarning.isEmpty ? model.status : model.cameraWarning)).font(.callout).multilineTextAlignment(.center)
                if model.running {
                    if !model.aligned,!model.alignmentDiagnostics.isEmpty {
                        Text("Shared features: \(model.alignmentDiagnostics["inliers",default:0])/30 verified · \(model.alignmentDiagnostics["confirmations",default:0])/3 consistent views")
                            .font(.caption.monospacedDigit())
                    }
                    Text(String(format:"%.1f processed · %.0f rendered fps · %.0f ms · %d skipped · %d inliers",model.fps,model.renderFPS,model.latencyMS,model.dropped,model.inliers)).font(.caption.monospacedDigit())
                    Text("Camera: \(model.cameraFPS,format:.number.precision(.fractionLength(0))) fps · Phone thermal state: \(model.thermalState)").font(.caption2)
                }
                HStack {
                    Button("Realign") {model.realign()}.disabled(!model.running)
                    Button("Thermal",systemImage:"slider.horizontal.3") {settings=true}
                }.buttonStyle(.borderedProminent)
            }.padding().background(.regularMaterial,in:.rect(cornerRadius:16)).padding()
        }
        .overlay(alignment:.topTrailing) {
            if model.running,!model.aligned,let preview=model.rigPreview {
                VStack(spacing:4) {
                    Image(uiImage:preview).resizable().scaledToFit().frame(width:160)
                        .accessibilityLabel("Rig camera preview. Aim both cameras at the same scene.")
                    Text("Rig camera").font(.caption)
                }.padding(6).background(.regularMaterial,in:.rect(cornerRadius:10)).padding()
            }
        }
        .sheet(isPresented:$settings) {
            NavigationStack {
                Form {
                    Section("Approximate legacy alignment") {
                        adjustment("Horizontal offset",value:$model.alignment.offsetX,range:-320...320)
                        adjustment("Vertical offset",value:$model.alignment.offsetY,range:-240...240)
                        adjustment("Scale",value:$model.alignment.scale,range:0.5...1.5)
                        adjustment("Barrel",value:$model.alignment.barrel,range:-1...1)
                        adjustment("Horizontal stretch",value:$model.alignment.stretchX,range:0.5...1.5)
                        adjustment("Vertical stretch",value:$model.alignment.stretchY,range:0.5...1.5)
                        Toggle("Flip horizontal",isOn:$model.alignment.flipX)
                        Toggle("Flip vertical",isOn:$model.alignment.flipY)
                        Text("Offsets use the old 640 × 480 reference. Changes refresh thermal points and keep camera alignment. Driver image flips are accounted for separately.").font(.caption)
                        Button("Save for this camera profile") {model.saveAlignment()}
                    }
                    Section("Thermal display") {
                        Toggle("Show dots and highlight through walls",isOn:$model.showHeatThroughWalls)
                        Toggle("Filled heat region",isOn:$model.heatHighlight)
                        adjustment("Highlight above °C",value:$model.heatThreshold,range:15...45)
                        Text("Shows warm surfaces currently observed by the rig, including people and warm objects. It does not identify people or sense through walls.").font(.caption)
                    }
                    Section("Diagnostics") {
                        ShareLink("Export scan log",item:model.logURL)
                        Text("Timestamped tracking, timing, frame rates, camera stalls and errors. No camera images.").font(.caption)
                    }
                    Section("Temperature scale · °C") {
                        Toggle("Automatic contrast (relative)",isOn:$model.automaticTemperatureScale)
                        Text("Fixed limits keep room-temperature surfaces cool when a person leaves. Automatic contrast recolours each scene.").font(.caption)
                        if model.automaticTemperatureScale {
                            Text(String(format:"%.1f–%.1f °C",model.lowerTemperature,model.upperTemperature))
                        } else {
                            adjustment("Minimum",value:$model.lowerTemperature,range:0...50)
                            adjustment("Maximum",value:$model.upperTemperature,range:20...100)
                        }
                    }
                }.navigationTitle("Thermal alignment")
                    .toolbar {ToolbarItem(placement:.confirmationAction){Button("Done"){settings=false}}}
            }
        }
    }
    private func adjustment(_ title:String,value:Binding<Double>,range:ClosedRange<Double>)->some View {
        VStack(alignment:.leading){Text("\(title): \(value.wrappedValue,format:.number.precision(.fractionLength(2)))");Slider(value:value,in:range).accessibilityLabel(title)}
    }
}

struct ARMetalView: UIViewRepresentable {
    let model:PhoneReconstruction
    func makeCoordinator()->Renderer {Renderer(model:model)}
    func makeUIView(context:Context)->MTKView {
        let view=MTKView(frame:.zero,device:MTLCreateSystemDefaultDevice())
        view.colorPixelFormat = .bgra8Unorm;view.depthStencilPixelFormat = .depth32Float;view.preferredFramesPerSecond=30
        view.delegate=context.coordinator;context.coordinator.configure(view)
        return view
    }
    func updateUIView(_ view:MTKView,context:Context) {}
    static func dismantleUIView(_ view:MTKView,coordinator:Renderer) {view.delegate=nil}
    @MainActor final class Renderer:NSObject,MTKViewDelegate {
        let model:PhoneReconstruction
        var queue:MTLCommandQueue?,cameraPipeline:MTLRenderPipelineState?,pointPipeline:MTLRenderPipelineState?,heatPipeline:MTLRenderPipelineState?,depthState:MTLDepthStencilState?,heatDepthState:MTLDepthStencilState?
        var cache:CVMetalTextureCache?
        var pointBuffer:MTLBuffer?,heatBuffer:MTLBuffer?
        var emptyDepth:MTLTexture?
        var bufferRevision = -1
        var renderedFrames=0
        var renderStart=CACurrentMediaTime()
        init(model:PhoneReconstruction){self.model=model}
        func configure(_ view:MTKView) {
            guard let device=view.device else {model.status="Metal is unavailable";return}
            do {
                guard let library=device.makeDefaultLibrary() else {throw APIError.invalidData}
                queue=device.makeCommandQueue()
                let descriptor=MTLRenderPipelineDescriptor();descriptor.colorAttachments[0].pixelFormat=view.colorPixelFormat
                descriptor.depthAttachmentPixelFormat=view.depthStencilPixelFormat
                descriptor.vertexFunction=library.makeFunction(name:"arCameraVertex");descriptor.fragmentFunction=library.makeFunction(name:"arCameraFragment")
                cameraPipeline=try device.makeRenderPipelineState(descriptor:descriptor)
                descriptor.vertexFunction=library.makeFunction(name:"thermalVertex");descriptor.fragmentFunction=library.makeFunction(name:"thermalFragment")
                pointPipeline=try device.makeRenderPipelineState(descriptor:descriptor)
                descriptor.fragmentFunction=library.makeFunction(name:"thermalHeatFragment")
                let blend=descriptor.colorAttachments[0]!
                blend.isBlendingEnabled=true;blend.sourceRGBBlendFactor = .sourceAlpha;blend.destinationRGBBlendFactor = .oneMinusSourceAlpha
                heatPipeline=try device.makeRenderPipelineState(descriptor:descriptor)
                let depth=MTLDepthStencilDescriptor();depth.depthCompareFunction = .lessEqual;depth.isDepthWriteEnabled=true
                depthState=device.makeDepthStencilState(descriptor:depth)
                let unoccluded=MTLDepthStencilDescriptor();unoccluded.depthCompareFunction = .always;unoccluded.isDepthWriteEnabled=false
                heatDepthState=device.makeDepthStencilState(descriptor:unoccluded)
                let empty=MTLTextureDescriptor.texture2DDescriptor(pixelFormat:.r32Float,width:1,height:1,mipmapped:false)
                empty.storageMode = .shared
                emptyDepth=device.makeTexture(descriptor:empty)
                var zero=Float(0);emptyDepth?.replace(region:MTLRegionMake2D(0,0,1,1),mipmapLevel:0,withBytes:&zero,bytesPerRow:4)
                CVMetalTextureCacheCreate(nil,nil,device,nil,&cache)
            }catch {model.renderingError="AR renderer: \(error.localizedDescription)";model.cameraError(model.renderingError)}
        }
        func mtkView(_ view:MTKView,drawableSizeWillChange size:CGSize) {}
        func texture(_ buffer:CVPixelBuffer,_ format:MTLPixelFormat,_ plane:Int)->(CVMetalTexture,MTLTexture)? {
            guard let cache else{return nil};var output:CVMetalTexture?
            let planar=CVPixelBufferIsPlanar(buffer)
            let w=planar ? CVPixelBufferGetWidthOfPlane(buffer,plane):CVPixelBufferGetWidth(buffer)
            let h=planar ? CVPixelBufferGetHeightOfPlane(buffer,plane):CVPixelBufferGetHeight(buffer)
            guard CVMetalTextureCacheCreateTextureFromImage(nil,cache,buffer,nil,format,w,h,plane,&output)==kCVReturnSuccess,
                  let output,let texture=CVMetalTextureGetTexture(output) else{return nil}
            return (output,texture)
        }
        func draw(in view:MTKView) {
            guard model.running,let frame=model.arSession.currentFrame,let descriptor=view.currentRenderPassDescriptor,
                  let drawable=view.currentDrawable,let command=queue?.makeCommandBuffer(),let cameraPipeline,
                  let encoder=command.makeRenderCommandEncoder(descriptor:descriptor),
                  let y=texture(frame.capturedImage,.r8Unorm,0),let uv=texture(frame.capturedImage,.rg8Unorm,1) else{return}
            model.cameraHealth(age:ProcessInfo.processInfo.systemUptime-frame.timestamp)
            let orientation=view.window?.windowScene?.interfaceOrientation ?? .portrait
            let transform=frame.displayTransform(for:orientation,viewportSize:view.bounds.size).inverted()
            var uvTransform=simd_float3x3(columns:(SIMD3(Float(transform.a),Float(transform.b),0),SIMD3(Float(transform.c),Float(transform.d),0),SIMD3(Float(transform.tx),Float(transform.ty),1)))
            encoder.setRenderPipelineState(cameraPipeline)
            encoder.setVertexBytes(&uvTransform,length:MemoryLayout<simd_float3x3>.stride,index:0)
            encoder.setFragmentTexture(y.1,index:0);encoder.setFragmentTexture(uv.1,index:1)
            encoder.drawPrimitives(type:.triangleStrip,vertexStart:0,vertexCount:4)
            var retained:[CVMetalTexture]=[y.0,uv.0]
            let sceneDepth=frame.sceneDepth.flatMap{texture($0.depthMap,.r32Float,0)}
            if model.aligned,(!model.points.isEmpty || !model.heatSurface.isEmpty),let pointPipeline,
               model.showHeatThroughWalls || sceneDepth != nil,let depthTexture=sceneDepth?.1 ?? emptyDepth,let device=view.device {
                if let sceneDepth {retained.append(sceneDepth.0)}
                let cameraFromMap=frame.camera.viewMatrix(for:orientation)*model.worldFromMap
                var matrix=frame.camera.projectionMatrix(for:orientation,viewportSize:view.bounds.size,zNear:0.05,zFar:20)*cameraFromMap
                var opticalFromMap=opticalToAR*frame.camera.transform.inverse*model.worldFromMap
                var k=frame.camera.intrinsics
                var sizes=SIMD4(Float(CVPixelBufferGetWidth(frame.capturedImage)),Float(CVPixelBufferGetHeight(frame.capturedImage)),Float(view.drawableSize.width),Float(view.drawableSize.height))
                var range=SIMD2(Float(model.lowerTemperature),Float(max(model.lowerTemperature+0.1,model.upperTemperature)))
                if bufferRevision != model.pointRevision {
                    pointBuffer=model.points.isEmpty ? nil : device.makeBuffer(bytes:model.points,length:model.points.count*MemoryLayout<SIMD4<Float>>.stride,options:.storageModeShared)
                    heatBuffer=model.heatSurface.isEmpty ? nil : device.makeBuffer(bytes:model.heatSurface,length:model.heatSurface.count*MemoryLayout<SIMD4<Float>>.stride,options:.storageModeShared)
                    bufferRevision=model.pointRevision
                }
                var options=SIMD2(Float(model.heatThreshold),model.showHeatThroughWalls ? Float(1):Float(0))
                encoder.setVertexBytes(&matrix,length:64,index:1)
                encoder.setVertexBytes(&opticalFromMap,length:64,index:2);encoder.setVertexBytes(&k,length:MemoryLayout<simd_float3x3>.stride,index:3)
                encoder.setVertexBytes(&sizes,length:16,index:4);encoder.setFragmentBytes(&range,length:8,index:0)
                encoder.setFragmentBytes(&options,length:8,index:1);encoder.setFragmentTexture(depthTexture,index:0)
                if let buffer=pointBuffer {
                    encoder.setRenderPipelineState(pointPipeline)
                    encoder.setDepthStencilState(model.showHeatThroughWalls ? heatDepthState : depthState)
                    encoder.setVertexBuffer(buffer,offset:0,index:0)
                    encoder.drawPrimitives(type:.point,vertexStart:0,vertexCount:model.points.count)
                }
                if model.heatHighlight,Date().timeIntervalSince1970-model.heatObservationTime<0.5,let heatBuffer,let heatPipeline {
                    encoder.setRenderPipelineState(heatPipeline);encoder.setDepthStencilState(heatDepthState)
                    encoder.setVertexBuffer(heatBuffer,offset:0,index:0)
                    encoder.drawPrimitives(type:.triangle,vertexStart:0,vertexCount:model.heatSurface.count)
                }
            }
            encoder.endEncoding();command.present(drawable)
            let textures=retained
            command.addCompletedHandler { _ in _ = textures }
            command.commit()
            renderedFrames += 1
            let elapsed=CACurrentMediaTime()-renderStart
            if elapsed>=1 {model.renderFPS=Double(renderedFrames)/elapsed;renderedFrames=0;renderStart=CACurrentMediaTime()}
        }
    }
}
