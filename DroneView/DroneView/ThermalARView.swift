import SwiftUI
import ARKit
import MetalKit

struct ThermalARView: View {
    @Bindable var model:PhoneReconstruction
    var openHeadset: () -> Void = {}
    @State private var settings=false
    var body: some View {
        ZStack(alignment:.bottom) {
            ARMetalView(model:model).ignoresSafeArea()
            VStack(spacing:10) {
                Text(!model.renderingError.isEmpty ? model.renderingError : (model.cameraWarning.isEmpty ? model.status : model.cameraWarning)).font(.callout).multilineTextAlignment(.center)
                if model.running {
                    if model.alignmentEstablished,!model.aligned {
                        Text("Alignment saved · overlay paused until tracking recovers").font(.caption)
                    } else if !model.alignmentEstablished,!model.alignmentDiagnostics.isEmpty {
                        Text("Shared features: \(model.alignmentDiagnostics["inliers",default:0])/30 verified · \(model.alignmentDiagnostics["confirmations",default:0])/3 consistent views")
                            .font(.caption.monospacedDigit())
                    }
                    Text(String(format:"%.1f processed · %.0f rendered fps · %.0f ms · %d skipped · %d inliers",model.fps,model.renderFPS,model.latencyMS,model.dropped,model.inliers)).font(.caption.monospacedDigit())
                    Text("Camera: \(model.cameraFPS,format:.number.precision(.fractionLength(0))) fps · Phone thermal state: \(model.thermalState)").font(.caption2)
                }
                HStack {
                    Button("Realign") {model.realign()}.disabled(!model.running)
                    Button("Thermal",systemImage:"slider.horizontal.3") {settings=true}
                    Button("Headset",systemImage:"viewfinder",action:openHeadset)
                }.buttonStyle(.borderedProminent).labelStyle(.titleOnly)
            }.padding().background(.regularMaterial,in:.rect(cornerRadius:16)).padding()
        }
        .overlay(alignment:.topTrailing) {
            if model.running,!model.alignmentEstablished,let preview=model.rigPreview {
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
                        Toggle("Show surrounding point cloud",isOn:$model.showPointCloud)
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
    var headset:HeadsetSettings? = nil
    func makeCoordinator()->Renderer {Renderer(model:model,headset:headset)}
    func makeUIView(context:Context)->MTKView {
        let view=MTKView(frame:.zero,device:MTLCreateSystemDefaultDevice())
        view.colorPixelFormat = .bgra8Unorm;view.depthStencilPixelFormat = .depth32Float
        view.preferredFramesPerSecond=headset == nil ? 30:60
        view.clearColor=MTLClearColorMake(0,0,0,1)
        view.delegate=context.coordinator;context.coordinator.configure(view)
        return view
    }
    func updateUIView(_ view:MTKView,context:Context) {}
    static func dismantleUIView(_ view:MTKView,coordinator:Renderer) {view.isPaused=true;view.delegate=nil}
    @MainActor final class Renderer:NSObject,MTKViewDelegate {
        let model:PhoneReconstruction
        let headset:HeadsetSettings?
        var queue:MTLCommandQueue?,cameraPipeline:MTLRenderPipelineState?,pointPipeline:MTLRenderPipelineState?,heatPipeline:MTLRenderPipelineState?,depthState:MTLDepthStencilState?,heatDepthState:MTLDepthStencilState?
        var cache:CVMetalTextureCache?
        var pointBuffer:MTLBuffer?,heatBuffer:MTLBuffer?
        var emptyDepth:MTLTexture?
        var headsetPipeline:MTLRenderPipelineState?
        var composite:MTLTexture?,compositeDepth:MTLTexture?,hudTexture:MTLTexture?
        private var hudKey="",hudTime=0.0
        private let inFlight=DispatchSemaphore(value:2)
        var bufferRevision = -1
        var renderedFrames=0
        var renderStart=CACurrentMediaTime()
        init(model:PhoneReconstruction,headset:HeadsetSettings?=nil){self.model=model;self.headset=headset}
        func configure(_ view:MTKView) {
            guard let device=view.device else {model.renderingError="Metal is unavailable";return}
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
                descriptor.vertexFunction=library.makeFunction(name:"arCameraVertex");descriptor.fragmentFunction=library.makeFunction(name:"headsetFragment")
                descriptor.depthAttachmentPixelFormat = .invalid;blend.isBlendingEnabled=false
                headsetPipeline=try device.makeRenderPipelineState(descriptor:descriptor)
                let depth=MTLDepthStencilDescriptor();depth.depthCompareFunction = .lessEqual;depth.isDepthWriteEnabled=true
                depthState=device.makeDepthStencilState(descriptor:depth)
                let unoccluded=MTLDepthStencilDescriptor();unoccluded.depthCompareFunction = .always;unoccluded.isDepthWriteEnabled=false
                heatDepthState=device.makeDepthStencilState(descriptor:unoccluded)
                let empty=MTLTextureDescriptor.texture2DDescriptor(pixelFormat:.r32Float,width:1,height:1,mipmapped:false)
                empty.storageMode = .shared
                emptyDepth=device.makeTexture(descriptor:empty)
                var zero=Float(0);emptyDepth?.replace(region:MTLRegionMake2D(0,0,1,1),mipmapLevel:0,withBytes:&zero,bytesPerRow:4)
                CVMetalTextureCacheCreate(nil,nil,device,nil,&cache)
                model.renderingError=""
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
            guard let descriptor=view.currentRenderPassDescriptor,let drawable=view.currentDrawable,
                  let command=queue?.makeCommandBuffer(),let cameraPipeline,let device=view.device else{return}
            guard inFlight.wait(timeout:.now()) == .success else{return}
            var submitted=false
            defer {if !submitted {inFlight.signal()}}
            let frame=model.running ? model.arSession.currentFrame:nil
            let age=frame.map{ProcessInfo.processInfo.systemUptime-$0.timestamp}
            if let age {model.cameraHealth(age:age)}
            let orientation=view.window?.windowScene?.interfaceOrientation ?? .portrait
            var viewport=view.bounds.size,renderSize=view.drawableSize,pass=descriptor
            if headset != nil {
                // Match the source camera's aspect ratio. Both camera and thermal
                // projection use this viewport, before the same per-eye lens warp.
                var aspect=Double(4)/3
                if let frame {
                    aspect=Double(CVPixelBufferGetWidth(frame.capturedImage))/Double(CVPixelBufferGetHeight(frame.capturedImage))
                    if !orientation.isLandscape {aspect=1/aspect}
                }
                let width=max(1,Int(view.drawableSize.width/2))
                renderSize=CGSize(width:width,height:max(1,Int(Double(width)/aspect)))
                viewport=renderSize
                guard let offscreen=makeCompositePass(size:renderSize,device:device) else {
                    model.renderingError="Could not allocate headset display";return
                }
                pass=offscreen
            }
            guard let encoder=command.makeRenderCommandEncoder(descriptor:pass) else{return}
            var retained:[CVMetalTexture]=[]
            let live=HeadsetSettings.cameraIsLive(running:model.running,age:age)
            var drewCamera=false
            if let frame,(headset == nil || (live && orientation.isLandscape)),
               let y=texture(frame.capturedImage,.r8Unorm,0),let uv=texture(frame.capturedImage,.rg8Unorm,1) {
                let transform=frame.displayTransform(for:orientation,viewportSize:viewport).inverted()
                var uvTransform=simd_float3x3(columns:(SIMD3(Float(transform.a),Float(transform.b),0),SIMD3(Float(transform.c),Float(transform.d),0),SIMD3(Float(transform.tx),Float(transform.ty),1)))
                encoder.setRenderPipelineState(cameraPipeline)
                encoder.setVertexBytes(&uvTransform,length:MemoryLayout<simd_float3x3>.stride,index:0)
                encoder.setFragmentTexture(y.1,index:0);encoder.setFragmentTexture(uv.1,index:1)
                encoder.drawPrimitives(type:.triangleStrip,vertexStart:0,vertexCount:4)
                retained=[y.0,uv.0];drewCamera=true
            let sceneDepth=frame.sceneDepth.flatMap{texture($0.depthMap,.r32Float,0)}
            if model.aligned,case .normal=frame.camera.trackingState,model.cameraAgeMS<300,
               (!model.points.isEmpty || !model.heatSurface.isEmpty),let pointPipeline,
               model.showHeatThroughWalls || sceneDepth != nil,let depthTexture=sceneDepth?.1 ?? emptyDepth,let device=view.device {
                if let sceneDepth {retained.append(sceneDepth.0)}
                let cameraFromMap=frame.camera.viewMatrix(for:orientation)*model.worldFromMap
                var matrix=frame.camera.projectionMatrix(for:orientation,viewportSize:viewport,zNear:0.05,zFar:20)*cameraFromMap
                var opticalFromMap=opticalToAR*frame.camera.transform.inverse*model.worldFromMap
                var k=frame.camera.intrinsics
                var sizes=SIMD4(Float(CVPixelBufferGetWidth(frame.capturedImage)),Float(CVPixelBufferGetHeight(frame.capturedImage)),Float(renderSize.width),Float(renderSize.height))
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
                if model.showPointCloud,let buffer=pointBuffer {
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
            }
            encoder.endEncoding()
            if let headset,let composite {
                let warning=headset.showGrid ? "CALIBRATION GRID · NOT LIVE" :
                    (!orientation.isLandscape ? "ROTATE PHONE TO LANDSCAPE" :
                     (!drewCamera ? "CAMERA UNAVAILABLE · REMOVE HEADSET\n\(model.status)" :
                      (!model.aligned ? "THERMAL PAUSED\n\(model.status)" : "")))
                updateHUD(warning:warning,aspect:Double(composite.width)/Double(composite.height),device:device)
                descriptor.depthAttachment.texture=nil
                guard let hudTexture,encodeHeadset(command:command,pass:descriptor,source:composite,hud:hudTexture,
                                                  size:view.drawableSize,profile:headset.profile,grid:headset.showGrid) else {
                    model.renderingError="Headset compositor unavailable";return
                }
            }
            command.present(drawable)
            // These immutable CoreVideo wrappers are only retained/released by
            // the completion handler, never read or mutated across threads.
            nonisolated(unsafe) let textures=retained
            let semaphore=inFlight
            let model=self.model
            command.addCompletedHandler { completed in
                withExtendedLifetime(textures) {_ = semaphore.signal()}
                if completed.status == .error {
                    let message=completed.error?.localizedDescription ?? "GPU command failed"
                    Task { @MainActor in model.renderingError=message;model.cameraError(message) }
                }
            }
            submitted=true
            command.commit()
            renderedFrames += 1
            let elapsed=CACurrentMediaTime()-renderStart
            if elapsed>=1 {model.renderFPS=Double(renderedFrames)/elapsed;renderedFrames=0;renderStart=CACurrentMediaTime()}
        }

        func makeCompositePass(size:CGSize,device:MTLDevice)->MTLRenderPassDescriptor? {
            let width=max(1,Int(size.width)),height=max(1,Int(size.height))
            if composite?.width != width || composite?.height != height {
                let color=MTLTextureDescriptor.texture2DDescriptor(pixelFormat:.bgra8Unorm,width:width,height:height,mipmapped:false)
                color.storageMode = .private;color.usage=[.renderTarget,.shaderRead]
                composite=device.makeTexture(descriptor:color)
                let depth=MTLTextureDescriptor.texture2DDescriptor(pixelFormat:.depth32Float,width:width,height:height,mipmapped:false)
                depth.storageMode = .private;depth.usage = .renderTarget
                compositeDepth=device.makeTexture(descriptor:depth)
            }
            guard let composite,let compositeDepth else{return nil}
            let pass=MTLRenderPassDescriptor()
            pass.colorAttachments[0].texture=composite;pass.colorAttachments[0].loadAction = .clear
            pass.colorAttachments[0].storeAction = .store;pass.colorAttachments[0].clearColor=MTLClearColorMake(0,0,0,1)
            pass.depthAttachment.texture=compositeDepth;pass.depthAttachment.loadAction = .clear;pass.depthAttachment.clearDepth=1
            return pass
        }

        // Also exercised with a synthetic image by the GPU tests; no AR session required.
        func encodeHeadset(command:MTLCommandBuffer,pass:MTLRenderPassDescriptor,source:MTLTexture,hud:MTLTexture,
                           size:CGSize,profile:HeadsetProfile,grid:Bool)->Bool {
            guard size.width>=2,size.height>0,let headsetPipeline,
                  let encoder=command.makeRenderCommandEncoder(descriptor:pass) else{return false}
            let p=profile.validated,eyeWidth=floor(size.width/2)
            var identity=matrix_identity_float3x3
            var optics=SIMD4(Float(p.scale),Float(p.lensSpacing),Float(p.verticalCenter),Float(p.distortion))
            var layout=SIMD4(Float(eyeWidth/size.height),Float(source.width)/Float(source.height),Float(0),grid ? Float(1):Float(0))
            encoder.setRenderPipelineState(headsetPipeline)
            encoder.setVertexBytes(&identity,length:MemoryLayout<simd_float3x3>.stride,index:0)
            encoder.setFragmentBytes(&optics,length:16,index:0)
            encoder.setFragmentTexture(source,index:0);encoder.setFragmentTexture(hud,index:1)
            for eye in 0..<2 {
                layout.z=Float(eye)
                encoder.setViewport(MTLViewport(originX:Double(eye)*eyeWidth,originY:0,width:eyeWidth,height:size.height,znear:0,zfar:1))
                encoder.setFragmentBytes(&layout,length:16,index:1)
                encoder.drawPrimitives(type:.triangleStrip,vertexStart:0,vertexCount:4)
            }
            encoder.endEncoding();return true
        }

        func updateHUD(warning:String,aspect:Double,device:MTLDevice) {
            guard let headset else{return}
            let key="\(warning)|\(headset.showHUD)|\(aspect)"
            let now=CACurrentMediaTime()
            guard key != hudKey || now-hudTime>=1 else{return}
            hudKey=key;hudTime=now
            let size=CGSize(width:960,height:960/aspect),format=UIGraphicsImageRendererFormat()
            // Extended-range UIKit bitmaps are not accepted by this texture loader.
            format.scale=1;format.opaque=false;format.preferredRange = .standard
            let image=UIGraphicsImageRenderer(size:size,format:format).image { context in
                guard headset.showHUD || !warning.isEmpty else{return}
                let style=NSMutableParagraphStyle();style.alignment = .center
                func label(_ text:String,y:Double,height:Double,font:Double) {
                    let rect=CGRect(x:size.width*0.15,y:size.height*y,width:size.width*0.7,height:size.height*height)
                    UIColor.black.withAlphaComponent(0.75).setFill()
                    UIBezierPath(roundedRect:rect,cornerRadius:12).fill()
                    (text as NSString).draw(in:rect.insetBy(dx:12,dy:8),withAttributes:[.font:UIFont.systemFont(ofSize:font,weight:.medium),.foregroundColor:UIColor.white,.paragraphStyle:style])
                }
                if headset.showHUD || headset.showGrid {
                    label(headset.showGrid ? "CALIBRATION · NOT LIVE":"MONO PASSTHROUGH",y:0.13,height:0.08,font:26)
                }
                if headset.showHUD,!headset.showGrid {
                    label(String(format:"Camera %.0f · Display %.0f fps · Frame %.0f ms\n%@",model.cameraFPS,model.renderFPS,model.cameraAgeMS,model.thermalState),y:0.74,height:0.12,font:21)
                }
                if !warning.isEmpty,!headset.showGrid {label(warning,y:0.48,height:0.23,font:25)}
                if headset.showHUD || !warning.isEmpty {
                    label(headset.showGrid ? "Hold to exit · Adjust optics in setup":"Left: HUD · Right: realign · Hold: exit",y:0.86,height:0.07,font:20)
                }
                if headset.showHUD,!headset.showGrid {
                    let cg=context.cgContext;cg.setStrokeColor(UIColor.white.withAlphaComponent(0.6).cgColor);cg.setLineWidth(2)
                    cg.move(to:CGPoint(x:size.width/2-8,y:size.height/2));cg.addLine(to:CGPoint(x:size.width/2+8,y:size.height/2))
                    cg.move(to:CGPoint(x:size.width/2,y:size.height/2-8));cg.addLine(to:CGPoint(x:size.width/2,y:size.height/2+8));cg.strokePath()
                }
            }
            if let cgImage=image.cgImage {
                do {hudTexture=try MTKTextureLoader(device:device).newTexture(cgImage:cgImage,options:[.SRGB:false])}
                catch {model.renderingError="Headset status display: \(error.localizedDescription)"}
            }
        }
    }
}
