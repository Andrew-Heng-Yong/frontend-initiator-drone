import XCTest
import simd
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
        for _ in 0..<2 {
            await worker.confirmAlignment(pose)
            let pending=await worker.currentAlignment()
            XCTAssertNil(pending)
        }
        await worker.confirmAlignment(pose)
        let aligned=await worker.currentAlignment()
        XCTAssertEqual(aligned,pose)
        await worker.invalidate()
        let invalidated=await worker.currentAlignment()
        XCTAssertNil(invalidated)
        await worker.confirmAlignment(pose)
        let pendingAgain=await worker.currentAlignment()
        XCTAssertNil(pendingAgain)
    }
    func testRejectedRigFramePausesWithoutDiscardingAlignment() async throws {
        let url=try XCTUnwrap(Bundle(for:Self.self).url(forResource:"sensor-v1",withExtension:"bin"))
        let wire=try Data(contentsOf:url), frame=try SensorFrame(wire), worker=ReconstructionWorker()
        _=await worker.process(frame,phone:nil,alignment:ThermalAlignment())
        for _ in 0..<3 {await worker.confirmAlignment(matrix_identity_float4x4)}
        var metadata=frame.metadata;metadata["sequence"]=frame.sequence+1
        let header=try JSONSerialization.data(withJSONObject:metadata)
        let oldSize=Int(wire.withUnsafeBytes{$0.loadUnaligned(fromByteOffset:4,as:UInt32.self).littleEndian})
        var size=UInt32(header.count).littleEndian
        var next=Data("DVS1".utf8);next.append(withUnsafeBytes(of:&size){Data($0)});next.append(header);next.append(wire.dropFirst(8+oldSize))
        let rejected=await worker.process(try SensorFrame(next),phone:nil,alignment:ThermalAlignment())
        XCTAssertFalse(rejected.valid)
        let retained=await worker.currentAlignment()
        XCTAssertEqual(retained,matrix_identity_float4x4)
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
