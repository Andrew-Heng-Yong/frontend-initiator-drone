import XCTest
import simd
import SwiftUI
import ARKit
@testable import DroneView

final class SensorFrameTests:XCTestCase {
    @MainActor func testThermalDisplaySettingsSurviveModelRestart() {
        let keys=["thermal-heat-highlight","thermal-show-through-walls","thermal-heat-threshold","thermal-automatic-scale","thermal-scale-lower","thermal-scale-upper"]
        let saved=keys.map{UserDefaults.standard.object(forKey:$0)}
        defer {for (key,value) in zip(keys,saved) {UserDefaults.standard.set(value,forKey:key)}}
        let model=PhoneReconstruction()
        model.heatHighlight=false;model.showHeatThroughWalls=false;model.heatThreshold=19.92
        model.automaticTemperatureScale=false;model.lowerTemperature=18;model.upperTemperature=29
        model.automaticTemperatureScale=true;model.lowerTemperature=10;model.upperTemperature=40
        let restored=PhoneReconstruction()
        XCTAssertEqual(restored.heatThreshold,19.92);XCTAssertFalse(restored.heatHighlight);XCTAssertFalse(restored.showHeatThroughWalls)
        XCTAssertTrue(restored.automaticTemperatureScale)
        XCTAssertEqual(restored.lowerTemperature,18);XCTAssertEqual(restored.upperTemperature,29)
    }
    func testImportedThermalProfileMatchesCameraAndRejectsInvalidValues() throws {
        var a=ThermalAlignment();a.offsetX=4.05;a.offsetY = -8.09;a.stretchX=0.915;a.stretchY=0.9731
        let data=try JSONEncoder().encode(["rig-640x360-K":a])
        XCTAssertEqual(ThermalAlignment.importedProfile(data,key:"rig-640x360-K"),a)
        XCTAssertNil(ThermalAlignment.importedProfile(data,key:"other-camera"))
        XCTAssertNil(ThermalAlignment.importedProfile(Data("invalid".utf8),key:"rig-640x360-K"))
        XCTAssertNil(ThermalAlignment.importedProfile(Data(repeating:0,count:64_001),key:"rig-640x360-K"))
        a.scale=0
        XCTAssertNil(ThermalAlignment.importedProfile(try JSONEncoder().encode(["rig-640x360-K":a]),key:"rig-640x360-K"))
    }
    func testAlignmentConfirmationSurvivesPendingFrames() async {
        let worker=ReconstructionWorker(), pose=matrix_identity_float4x4
        for time in 1...2 {
            await worker.confirmAlignment(pose,at:Double(time))
            let pending=await worker.currentAlignment()
            XCTAssertNil(pending)
        }
        await worker.confirmAlignment(pose,at:3)
        let aligned=await worker.currentAlignment()
        XCTAssertEqual(aligned,pose)
        await worker.invalidate()
        let invalidated=await worker.currentAlignment()
        XCTAssertNil(invalidated)
        await worker.confirmAlignment(pose,at:4)
        let pendingAgain=await worker.currentAlignment()
        XCTAssertNil(pendingAgain)
    }
    func testAlignmentKeepsGoodFitsAcrossMissesAndRejectsAnOutlier() async {
        let worker=ReconstructionWorker()
        var pose=matrix_identity_float4x4
        await worker.confirmAlignment(pose,at:1)
        await worker.confirmAlignment(nil,at:1.5)
        pose.columns.3.x=1
        await worker.confirmAlignment(pose,at:2)
        pose=simd_float4x4(simd_quatf(angle:0.02,axis:SIMD3(0,1,0)))
        pose.columns.3.x=0.02
        await worker.confirmAlignment(pose,at:2.5)
        var aligned=await worker.currentAlignment()
        XCTAssertNil(aligned)
        pose=simd_float4x4(simd_quatf(angle:-0.02,axis:SIMD3(0,1,0)));pose.columns.3.x = -0.02
        await worker.confirmAlignment(pose,at:3)
        aligned=await worker.currentAlignment()
        XCTAssertNotNil(aligned)
        XCTAssertEqual(aligned!.columns.3.x,0,accuracy:0.0001)
        XCTAssertEqual(simd_quatf(aligned!).angle,0,accuracy:0.0001)
        XCTAssertEqual(simd_determinant(aligned!),1,accuracy:0.0001)
        await worker.confirmAlignment(nil,at:10)
        let retained=await worker.currentAlignment()
        XCTAssertEqual(retained,aligned)
    }
    @MainActor func testStartupGuidanceRendering() throws {
        for saved in [false,true] {
        let model=PhoneReconstruction()
        model.running=true;model.status="Aligning · 2/3 consistent views · keep the same scene visible"
        model.alignmentEstablished=saved
        if saved {model.status="Rig tracking lost · alignment saved · return to a previously seen area"}
        model.alignmentDiagnostics=["inliers":84,"confirmations":2]
        model.rigPreview=testImage()
        let scene=try XCTUnwrap(UIApplication.shared.connectedScenes.first as? UIWindowScene)
        let window=UIWindow(windowScene:scene)
        window.frame=CGRect(x:0,y:0,width:393,height:852)
        let controller=UIHostingController(rootView:ThermalARView(model:model))
        window.rootViewController=controller;window.makeKeyAndVisible()
        defer {window.isHidden=true}
        controller.view.layoutIfNeeded()
        let image=UIGraphicsImageRenderer(size:window.bounds.size).image {_ in
            XCTAssertTrue(window.drawHierarchy(in:window.bounds,afterScreenUpdates:true))
        }
        let attachment=XCTAttachment(image:image);attachment.name=saved ? "Tracking recovery guidance" : "Marker-free startup guidance";attachment.lifetime = .keepAlways
        add(attachment)
        }
    }
    func testAlignmentExpiresOldFitsAndDoesNotCountDuplicateFrames() async {
        let worker=ReconstructionWorker(),pose=matrix_identity_float4x4
        await worker.confirmAlignment(pose,at:1)
        await worker.confirmAlignment(pose,at:1)
        await worker.confirmAlignment(pose,at:2)
        var aligned=await worker.currentAlignment()
        XCTAssertNil(aligned)
        await worker.confirmAlignment(pose,at:7)
        await worker.confirmAlignment(pose,at:8)
        aligned=await worker.currentAlignment()
        XCTAssertNil(aligned)
        await worker.confirmAlignment(pose,at:9)
        aligned=await worker.currentAlignment()
        XCTAssertEqual(aligned,pose)
    }
    private func testImage(width:Int=24,height:Int=16) -> UIImage {
        let context=CGContext(data:nil,width:width,height:height,bitsPerComponent:8,bytesPerRow:width,space:CGColorSpaceCreateDeviceGray(),bitmapInfo:0)!
        context.setFillColor(gray:0.6,alpha:1);context.fill(CGRect(x:0,y:0,width:CGFloat(width),height:CGFloat(height)))
        return UIImage(cgImage:context.makeImage()!)
    }
    private func nextFrame(_ wire:Data,sequence:Int,stamp:Double?=nil) throws -> SensorFrame {
        var metadata=try SensorFrame(wire).metadata;metadata["sequence"]=sequence
        if let stamp {
            let shift=stamp-(metadata["timestamp"] as! Double)
            for key in ["timestamp","depth_timestamp","thermal_timestamp"] {
                if let value=metadata[key] as? Double {metadata[key]=value+shift}
            }
        }
        let oldJPEG=metadata["jpeg_bytes"] as! Int
        let jpeg=testImage(width:metadata["width"] as! Int,height:metadata["height"] as! Int).jpegData(compressionQuality:0.9)!
        metadata["jpeg_bytes"]=jpeg.count
        let header=try JSONSerialization.data(withJSONObject:metadata)
        let oldSize=Int(wire.withUnsafeBytes{$0.loadUnaligned(fromByteOffset:4,as:UInt32.self).littleEndian})
        var size=UInt32(header.count).littleEndian
        var next=Data("DVS1".utf8);next.append(withUnsafeBytes(of:&size){Data($0)});next.append(header);next.append(jpeg);next.append(wire.dropFirst(8+oldSize+oldJPEG))
        return try SensorFrame(next)
    }
    func testStartupRecoversAndThermalChangesPreserveAlignment() async throws {
        let url=try XCTUnwrap(Bundle(for:Self.self).url(forResource:"sensor-v1",withExtension:"bin"))
        let wire=try Data(contentsOf:url),frame=try nextFrame(wire,sequence:1),worker=ReconstructionWorker()
        _=await worker.process(frame,phone:nil,alignment:ThermalAlignment())
        let restarted=await worker.process(try nextFrame(wire,sequence:frame.sequence+1,stamp:frame.stamp+2),phone:nil,alignment:ThermalAlignment())
        XCTAssertTrue(restarted.status.hasPrefix("Restarting rig tracking"),restarted.status)
        _=await worker.process(try nextFrame(wire,sequence:frame.sequence+2,stamp:frame.stamp+3),phone:nil,alignment:ThermalAlignment())
        for time in 1...3 {await worker.confirmAlignment(matrix_identity_float4x4,at:Double(time))}
        var thermal=ThermalAlignment();thermal.offsetX += 3
        _=await worker.process(try nextFrame(wire,sequence:frame.sequence+3,stamp:frame.stamp+4),phone:nil,alignment:thermal)
        let retained=await worker.currentAlignment()
        XCTAssertEqual(retained,matrix_identity_float4x4)
        await worker.reset()
        let reset=await worker.currentAlignment()
        XCTAssertNil(reset)
    }
    func testRejectedRigFramePausesWithoutDiscardingAlignment() async throws {
        let url=try XCTUnwrap(Bundle(for:Self.self).url(forResource:"sensor-v1",withExtension:"bin"))
        let wire=try Data(contentsOf:url), frame=try nextFrame(wire,sequence:1), worker=ReconstructionWorker()
        _=await worker.process(frame,phone:nil,alignment:ThermalAlignment())
        for time in 1...3 {await worker.confirmAlignment(matrix_identity_float4x4,at:Double(time))}
        let rejected=await worker.process(try nextFrame(wire,sequence:frame.sequence+1),phone:nil,alignment:ThermalAlignment())
        XCTAssertFalse(rejected.valid)
        XCTAssertTrue(rejected.status.hasPrefix("Rig tracking lost · alignment saved"),rejected.status)
        XCTAssertNotNil(rejected.worldFromMap)
        let retained=await worker.currentAlignment()
        XCTAssertEqual(retained,matrix_identity_float4x4)
    }
    private func trackingFrame(sequence:Int,session:String="00000000-0000-4000-8000-000000000001") throws -> SensorFrame {
        let w=320,h=240
        let context=CGContext(data:nil,width:w,height:h,bitsPerComponent:8,bytesPerRow:w,space:CGColorSpaceCreateDeviceGray(),bitmapInfo:0)!
        context.setFillColor(gray:0.8,alpha:1);context.fill(CGRect(x:0,y:0,width:w,height:h))
        for y in stride(from:16,to:h-16,by:12) {for x in stride(from:16,to:w-16,by:12) {
            let seed=(x*73+y*31)%251
            context.setFillColor(gray:CGFloat(seed)/500,alpha:1)
            context.fillEllipse(in:CGRect(x:x,y:y,width:3+seed%7,height:3+seed%9))
        }}
        let jpeg=UIImage(cgImage:context.makeImage()!).jpegData(compressionQuality:0.95)!
        let metadata:[String:Any]=["version":1,"session":session,"sequence":sequence,
            "timestamp":Double(sequence),"depth_timestamp":Double(sequence),"thermal_timestamp":Double(sequence),
            "width":w,"height":h,"thermal_width":8,"thermal_height":6,
            "K":[250.0,0,160,0,250,120,0,0,1],"jpeg_bytes":jpeg.count,"depth_bytes":w*h*4,
            "thermal_bytes":8*6*4,"depth_encoding":"float32_metres","thermal_encoding":"float32_celsius"]
        let header=try JSONSerialization.data(withJSONObject:metadata)
        var size=UInt32(header.count).littleEndian,wire=Data("DVS1".utf8)
        wire.append(withUnsafeBytes(of:&size){Data($0)});wire.append(header);wire.append(jpeg)
        wire.append([Float](repeating:2,count:w*h).withUnsafeBytes{Data($0)})
        wire.append([Float](repeating:22,count:48).withUnsafeBytes{Data($0)})
        return try SensorFrame(wire)
    }
    func testPhoneTrackingDipKeepsAlignmentAndResumesWithoutSharedView() async throws {
        let worker=ReconstructionWorker()
        _=await worker.process(try trackingFrame(sequence:1),phone:nil,alignment:ThermalAlignment())
        var saved=matrix_identity_float4x4;saved.columns.3.x=1.25
        for time in 1...3 {await worker.confirmAlignment(saved,at:Double(time))}
        func phone(normal:Bool)->PhoneObservation {
            // No shared-view images: recovery must use the saved alignment.
            PhoneObservation(unixTime:2,pose:matrix_identity_float4x4,normal:normal,gray:Data(),depth:Data(),w:0,h:0,k:[])
        }
        let limited=await worker.process(try trackingFrame(sequence:2),phone:phone(normal:false),alignment:ThermalAlignment())
        XCTAssertEqual(limited.rigTrackingState,"tracking")
        XCTAssertFalse(limited.valid);XCTAssertEqual(limited.worldFromMap,saved)
        XCTAssertTrue(limited.status.hasPrefix("Phone tracking paused"),limited.status)
        let missing=await worker.process(try trackingFrame(sequence:3),phone:nil,alignment:ThermalAlignment())
        XCTAssertFalse(missing.valid);XCTAssertEqual(missing.worldFromMap,saved)
        let recovered=await worker.process(try trackingFrame(sequence:4),phone:phone(normal:true),alignment:ThermalAlignment())
        XCTAssertTrue(recovered.valid,recovered.status);XCTAssertEqual(recovered.worldFromMap,saved)
        XCTAssertEqual(recovered.rigPose.columns.3.x,1.25,accuracy:0.01)
        let newMap=await worker.process(try trackingFrame(sequence:5,session:UUID().uuidString),phone:phone(normal:true),alignment:ThermalAlignment())
        XCTAssertFalse(newMap.valid);XCTAssertNil(newMap.worldFromMap)
    }
    @MainActor func testLivePhoneQualityAndStallsGateLateResultsWithoutForgettingAlignment() {
        let model=PhoneReconstruction()
        let result=ReconstructionResult(points:[],worldFromMap:matrix_identity_float4x4,rigPose:matrix_identity_float4x4,valid:true,status:"Aligned",inliers:100)
        model.phoneTrackingChanged(.normal);model.apply(result)
        XCTAssertTrue(model.aligned);XCTAssertTrue(model.alignmentEstablished)
        for state:ARCamera.TrackingState in [.limited(.excessiveMotion),.limited(.insufficientFeatures),.limited(.relocalizing),.notAvailable] {
            model.phoneTrackingChanged(state);model.apply(result)
            XCTAssertFalse(model.aligned);XCTAssertTrue(model.alignmentEstablished)
        }
        model.phoneTrackingChanged(.normal);model.cameraHealth(age:2);model.apply(result)
        XCTAssertFalse(model.aligned);XCTAssertTrue(model.alignmentEstablished)
        model.cameraHealth(age:0.05);model.apply(result)
        XCTAssertTrue(model.aligned);XCTAssertTrue(model.alignmentEstablished)
        model.sessionWasInterrupted(model.arSession);model.apply(result)
        XCTAssertFalse(model.aligned);XCTAssertTrue(model.alignmentEstablished)
        XCTAssertTrue(model.sessionShouldAttemptRelocalization(model.arSession))
        model.realign();XCTAssertFalse(model.alignmentEstablished)
    }
    func testThermalRangeAndCaptureTimeAssociation() async throws {
        let range=try XCTUnwrap(SensorFrame.temperatureRange((0...100).map(Float.init)+[.nan,.infinity]))
        XCTAssertEqual(range.0,2,accuracy:0.0001);XCTAssertEqual(range.1,98,accuracy:0.0001)
        let flat=try XCTUnwrap(SensorFrame.temperatureRange([22,22]))
        XCTAssertEqual(flat.1-flat.0,1)
        XCTAssertNil(SensorFrame.temperatureRange([.nan]))
        let worker=ReconstructionWorker()
        _=await worker.thermalSample(stamp:10,values:[25],at:10)
        let nearest=await worker.thermalSample(stamp:10.2,values:[30],at:10.04)
        XCTAssertEqual(nearest?.values,[25])
        // Stationary capture times still receive newer temperatures.
        let newer=await worker.thermalSample(stamp:10.3,values:[35],at:10.3)
        XCTAssertEqual(newer?.values,[35])
        await worker.reset()
        let cleared=await worker.thermalSample(stamp:nil,values:[],at:10.3)
        XCTAssertNil(cleared)
    }
    func testLiveHeatSurfaceAndVacatedDepth() throws {
        let url=try XCTUnwrap(Bundle(for:Self.self).url(forResource:"sensor-v1",withExtension:"bin"))
        let frame=try SensorFrame(Data(contentsOf:url))
        let hot=ReconstructionWorker.makeHeatSurface(frame:frame,temperatures:frame.thermal,alignment:ThermalAlignment(),pose:matrix_identity_float4x4,threshold:24)
        XCTAssertFalse(hot.isEmpty);XCTAssertEqual(hot.count%3,0)
        XCTAssertTrue(hot.allSatisfy{$0.w==32.5 && abs($0.z-1.25)<0.001})
        XCTAssertTrue(ReconstructionWorker.makeHeatSurface(frame:frame,temperatures:frame.thermal,alignment:ThermalAlignment(),pose:matrix_identity_float4x4,threshold:40).isEmpty)
        XCTAssertTrue(ReconstructionWorker.isObservedFreeSpace(pointDepth:1,measuredDepth:3))
        XCTAssertFalse(ReconstructionWorker.isObservedFreeSpace(pointDepth:3,measuredDepth:1))
        XCTAssertFalse(ReconstructionWorker.isObservedFreeSpace(pointDepth:1,measuredDepth:.nan))
        XCTAssertFalse(ReconstructionWorker.isObservedFreeSpace(pointDepth:1,measuredDepth:0))
    }
    func testDiagnosticLogPersists() async throws {
        let log=ScanLog();log.write("check",["fps":12])
        for _ in 0..<20 {
            if let text=try? String(contentsOf:log.url,encoding:.utf8),text.contains("check") {
                let object=try JSONSerialization.jsonObject(with:Data(text.utf8)) as? [String:Any]
                XCTAssertEqual(object?["fps"] as? Int,12)
                return
            }
            try await Task.sleep(for:.milliseconds(25))
        }
        XCTFail("Diagnostic event was not persisted")
    }
    func testWireGeometryAndTrackingFailures() throws {
        let url=try XCTUnwrap(Bundle(for:Self.self).url(forResource:"sensor-v1",withExtension:"bin"))
        let compressed=try SensorFrame(Data(contentsOf:url))
        XCTAssertEqual(compressed.depth,[Float](repeating:1.25,count:384))
        XCTAssertEqual(compressed.thermal,[Float](repeating:32.5,count:48))
        let metadata:[String:Any]=["version":1,"session":UUID().uuidString,"sequence":1,
            "timestamp":10.0,"depth_timestamp":10.0,"thermal_timestamp":10.0,
            "width":24,"height":16,"thermal_width":8,"thermal_height":6,
            "K":[20.0,0,12,0,20,8,0,0,1],"jpeg_bytes":1,"depth_bytes":24*16*4,
            "thermal_bytes":8*6*4,"depth_encoding":"float32_metres","thermal_encoding":"float32_celsius",
            "thermal_flipped_y":true]
        let header=try JSONSerialization.data(withJSONObject:metadata)
        var size=UInt32(header.count).littleEndian
        var wire=Data("DVS1".utf8);wire.append(withUnsafeBytes(of:&size){Data($0)});wire.append(header);wire.append(0)
        let depth=[Float](repeating:2,count:24*16),thermal=(0..<48).map(Float.init)
        wire.append(depth.withUnsafeBytes{Data($0)});wire.append(thermal.withUnsafeBytes{Data($0)})
        let f=try SensorFrame(wire)
        XCTAssertEqual(f.depth[0],2);XCTAssertEqual(f.thermal[47],47)
        XCTAssertThrowsError(try SensorFrame(wire.dropLast()))
        var a=ThermalAlignment();a.scale=1;a.stretchX=1;a.stretchY=1;a.barrel=0
        a.flipX=false;a.flipY=false
        let index=try XCTUnwrap(a.index(x:12,y:8,frame:f))
        XCTAssertEqual(index,28) // Center maps to the already-oriented stream pixel (4,3).
        XCTAssertEqual(a.index(x:12,y:0,frame:f),12) // Upper thermal samples remain above the center.
        a.flipY=true;XCTAssertEqual(a.index(x:12,y:8,frame:f),20)
        XCTAssertEqual(a.index(x:12,y:0,frame:f),36)
        XCTAssertNil(a.index(x:-100,y:-100,frame:f))
        let identity=poseArray(matrix_identity_float4x4)
        XCTAssertEqual(poseMatrix(identity.withUnsafeBytes{Data($0)}),matrix_identity_float4x4)
        XCTAssertNil(poseMatrix(Data()))
        let bridge=TrackingBridge()
        XCTAssertEqual(bridge.processJPEG(Data([0]),depth:f.depthData,metadata:metadata)["status"] as? String,"invalid")
    }
}
