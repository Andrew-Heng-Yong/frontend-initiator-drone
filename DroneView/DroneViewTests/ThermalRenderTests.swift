import XCTest
import MetalKit
import simd
@testable import DroneView

final class ThermalRenderTests:XCTestCase {
    @MainActor func testDotsAndFilledHeatThroughForegroundDepth() throws {
        let device=try XCTUnwrap(MTLCreateSystemDefaultDevice(),"A Metal device is required for this rendering check")
        let view=MTKView(frame:.zero,device:device)
        view.colorPixelFormat = .bgra8Unorm;view.depthStencilPixelFormat = .depth32Float
        let renderer=ARMetalView.Renderer(model:PhoneReconstruction())
        renderer.configure(view)
        let dots=try XCTUnwrap(renderer.pointPipeline)
        let filled=try XCTUnwrap(renderer.heatPipeline,"The filled highlight must have a configured pipeline")
        let queue=try XCTUnwrap(device.makeCommandQueue())
        func texture(_ format:MTLPixelFormat)->MTLTexture {
            let descriptor=MTLTextureDescriptor.texture2DDescriptor(pixelFormat:format,width:32,height:32,mipmapped:false)
            descriptor.usage=[.renderTarget,.shaderRead];descriptor.storageMode = format == .depth32Float ? .private:.shared
            return device.makeTexture(descriptor:descriptor)!
        }
        let output=texture(.bgra8Unorm),zbuffer=texture(.depth32Float),foreground=texture(.r32Float)
        let depths=[Float](repeating:0.2,count:32*32)
        depths.withUnsafeBytes{foreground.replace(region:MTLRegionMake2D(0,0,32,32),mipmapLevel:0,withBytes:$0.baseAddress!,bytesPerRow:32*4)}
        func draw(filledHeat:Bool,throughWall:Bool,temperature:Float=28)throws->Int {
            let pass=MTLRenderPassDescriptor()
            pass.colorAttachments[0].texture=output;pass.colorAttachments[0].loadAction = .clear;pass.colorAttachments[0].storeAction = .store
            pass.depthAttachment.texture=zbuffer;pass.depthAttachment.loadAction = .clear;pass.depthAttachment.clearDepth=1
            let command=try XCTUnwrap(queue.makeCommandBuffer()),encoder=try XCTUnwrap(command.makeRenderCommandEncoder(descriptor:pass))
            let points: [SIMD4<Float>]=filledHeat ? [SIMD4(-0.8,-0.8,0.8,temperature),SIMD4(0.8,-0.8,0.8,temperature),SIMD4(0,0.8,0.8,temperature)] : [SIMD4(0,0,0.8,temperature)]
            let buffer=try XCTUnwrap(device.makeBuffer(bytes:points,length:points.count*16))
            var matrix=matrix_identity_float4x4,k=matrix_identity_float3x3,sizes=SIMD4<Float>(1,1,32,32)
            var range=SIMD2<Float>(19,28),options=SIMD2<Float>(24,throughWall ? 1:0)
            encoder.setRenderPipelineState(filledHeat ? filled:dots)
            encoder.setDepthStencilState(throughWall ? renderer.heatDepthState:renderer.depthState)
            encoder.setVertexBuffer(buffer,offset:0,index:0)
            encoder.setVertexBytes(&matrix,length:64,index:1);encoder.setVertexBytes(&matrix,length:64,index:2)
            encoder.setVertexBytes(&k,length:MemoryLayout<simd_float3x3>.stride,index:3);encoder.setVertexBytes(&sizes,length:16,index:4)
            encoder.setFragmentBytes(&range,length:8,index:0);encoder.setFragmentBytes(&options,length:8,index:1)
            encoder.setFragmentTexture(foreground,index:0)
            encoder.drawPrimitives(type:filledHeat ? .triangle:.point,vertexStart:0,vertexCount:points.count)
            encoder.endEncoding();command.commit();command.waitUntilCompleted()
            XCTAssertEqual(command.status,.completed,command.error?.localizedDescription ?? "GPU command failed")
            var pixels=[UInt8](repeating:0,count:32*32*4)
            pixels.withUnsafeMutableBytes{output.getBytes($0.baseAddress!,bytesPerRow:32*4,from:MTLRegionMake2D(0,0,32,32),mipmapLevel:0)}
            return stride(from:0,to:pixels.count,by:4).filter{pixels[$0]>0 || pixels[$0+1]>0 || pixels[$0+2]>0}.count
        }
        for filledHeat in [false,true] {
            XCTAssertEqual(try draw(filledHeat:filledHeat,throughWall:false),0)
            XCTAssertGreaterThan(try draw(filledHeat:filledHeat,throughWall:true),20)
        }
        XCTAssertEqual(try draw(filledHeat:true,throughWall:true,temperature:20),0)
    }
}
