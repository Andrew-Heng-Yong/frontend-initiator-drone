import ARKit
import Accelerate
import Foundation
import Observation
import simd
import UIKit

struct PhoneObservation: Sendable {
    let unixTime: Double
    let pose: simd_float4x4
    let normal: Bool
    let gray: Data, depth: Data
    let w: Int, h: Int
    let k: [Double]
}
struct ReconstructionResult: Sendable {
    var points: [SIMD4<Float>]
    var heatSurface: [SIMD4<Float>]=[]
    var worldFromMap: simd_float4x4?
    var rigPose: simd_float4x4
    var valid: Bool
    var status: String
    var inliers: Int
    var rigTrackingState = "unavailable"
    var keyframes = 0
    var alignmentDiagnostics: [String:Int] = [:]
    var thermalDeltaMS = -1.0
}

actor ReconstructionWorker {
    private let bridge=TrackingBridge()
    private var session="", calibration="", thermalCalibration=""
    private var sequence=0
    private var worldFromMap: simd_float4x4?
    private var alignmentSamples: [(time:Double,pose:simd_float4x4)]=[]
    private var confirmations=0
    private var attemptTime=0.0
    private var trackingLostSince: Double?
    private var voxels: [SIMD3<Int32>: SIMD4<Float>]=[:]
    private var order: [SIMD3<Int32>]=[]
    private var oldest=0
    private var thermalHistory: [(stamp:Double,values:[Float])]=[]
    func reset() { bridge.reset();session="";sequence=0;thermalCalibration="";trackingLostSince=nil;invalidate();voxels.removeAll();order.removeAll();oldest=0;thermalHistory=[] }
    func invalidate() { worldFromMap=nil;alignmentSamples=[];confirmations=0;attemptTime=0 }
    func process(_ f: SensorFrame, phone: PhoneObservation?, alignment: ThermalAlignment, heatThreshold:Float=24) -> ReconstructionResult {
        let signature=f.k.description+"\(f.width)x\(f.height)"
        if session != f.session || calibration != signature { reset();session=f.session;calibration=signature }
        let thermalSignature="\(f.thermalWidth)x\(f.thermalHeight)"+String(describing:alignment)
        if thermalCalibration != thermalSignature {
            voxels.removeAll();order.removeAll();oldest=0;thermalHistory=[];thermalCalibration=thermalSignature
        }
        let thermal=thermalSample(stamp:f.thermalStamp,values:f.thermal,at:f.stamp)
        var result=ReconstructionResult(points: [],worldFromMap: worldFromMap,rigPose: matrix_identity_float4x4,valid:false,status:"Waiting for a new sensor frame",inliers:0)
        result.thermalDeltaMS=thermal.map{abs($0.stamp-f.stamp)*1000} ?? -1
        guard f.metadata["mode"] as? String != "demo" else { result.status="Simulation stream · live AR alignment disabled";return result }
        guard f.sequence>sequence else { return result };sequence=f.sequence
        let output=bridge.processJPEG(f.jpeg,depth:f.depthData,metadata:f.metadata)
        guard let data=output["pose"] as? Data, let pose=poseMatrix(data) else { result.status="Invalid sensor frame";return result }
        result.inliers=output["inliers"] as? Int ?? 0
        result.rigTrackingState=output["status"] as? String ?? "unavailable"
        result.keyframes=output["keyframes"] as? Int ?? 0
        let tracking=output["status"] as? String == "tracking"
        // A rejected frame does not reset the rig map. Pause placement until the
        // existing odometry checks accept recovery in that same map.
        guard tracking else {
            if trackingLostSince == nil {trackingLostSince=f.stamp}
            if worldFromMap == nil,let since=trackingLostSince,f.stamp-since>=1 {
                reset();result.status="Restarting rig tracking · hold a textured view"
            } else {result.status=worldFromMap == nil ? "Rig tracking paused · hold a textured view" : "Rig tracking lost · alignment saved · return to a previously seen area"}
            return result
        }
        trackingLostSince=nil
        guard let phone else { result.status="Waiting for a timestamp-matched phone frame";return result }
        guard phone.normal else { result.status="Phone tracking paused · recovering · scan a textured area";return result }
        if f.stamp-attemptTime > (worldFromMap == nil ? 0.5 : 1.0) {
            attemptTime=f.stamp
            if let data=bridge.alignGray(phone.gray,width:phone.w,height:phone.h,depth:phone.depth,intrinsics:phone.k.map(NSNumber.init)),let phoneToRig=poseMatrix(data) {
                let transform=phone.pose * opticalToAR * phoneToRig.inverse * pose.inverse
                confirmAlignment(transform,at:f.stamp)
            } else { confirmAlignment(nil,at:f.stamp) }
        }
        for (key,value) in bridge.alignmentDiagnostics {
            if let key=key as? String,let value=value as? NSNumber {result.alignmentDiagnostics[key]=value.intValue}
        }
        result.alignmentDiagnostics["confirmations"]=confirmations
        guard let transform=currentAlignment() else {
            let reason=result.alignmentDiagnostics["rejection"] ?? 0
            switch reason {
            case 6: result.status="Alignment · insufficient shared depth; aim 1–2 m away"
            case 7: result.status="Alignment · camera depths disagree; hold a shared view"
            case 1: result.status="Alignment · aim both cameras at a well-lit, textured area"
            case 2: result.status="Alignment · rig depth missing at shared features; aim 1–2 m away"
            case 3: result.status="Alignment · include more of the shared scene"
            case 4,5,10: result.status="Alignment · show both cameras the same textured area"
            default: result.status="Alignment · hold both cameras steady on the same scene"
            }
            if confirmations>0 {result.status="Aligning · \(confirmations)/3 consistent views · keep the same scene visible"}
            return result
        }
        result.worldFromMap=transform;result.rigPose=transform*pose*opticalToAR;result.valid=true
        result.status="Aligned · approximate thermal mapping"
        // Hot observations are live, not permanent landmarks.
        let cameraFromMap=pose.inverse
        voxels=voxels.filter {_,v in
            if v.w>=heatThreshold {return false}
            let p=cameraFromMap*SIMD4(v.x,v.y,v.z,1)
            guard p.z>0 else{return true}
            let u=Int((Double(p.x/p.z)*f.k[0]+f.k[2]).rounded()),y=Int((Double(p.y/p.z)*f.k[4]+f.k[5]).rounded())
            guard u>=0,y>=0,u<f.width,y<f.height else{return true}
            let depth=f.depth[y*f.width+u]
            return !Self.isObservedFreeSpace(pointDepth:p.z,measuredDepth:depth)
        }
        order=Array(order.dropFirst(oldest)).filter{voxels[$0] != nil};oldest=0
        if let thermal, abs(thermal.stamp-f.stamp)<0.15 {
            for y in stride(from:0,to:f.height,by:4) { for x in stride(from:0,to:f.width,by:4) {
                let z=f.depth[y*f.width+x]
                guard z.isFinite,z>=0.2,z<=6,let ti=alignment.index(x:x,y:y,frame:f),ti<thermal.values.count else { continue }
                let temperature=thermal.values[ti];guard temperature.isFinite else { continue }
                let p=pose*SIMD4(Float((Double(x)-f.k[2])/f.k[0])*z,Float((Double(y)-f.k[5])/f.k[4])*z,z,1)
                let key=SIMD3<Int32>(Int32(floor(p.x/0.04)),Int32(floor(p.y/0.04)),Int32(floor(p.z/0.04)))
                if voxels[key] == nil {
                    if voxels.count>=60_000 { voxels.removeValue(forKey:order[oldest]);oldest += 1 }
                    order.append(key)
                }
                voxels[key]=SIMD4(p.x,p.y,p.z,temperature)
            }}
            result.heatSurface=Self.makeHeatSurface(frame:f,temperatures:thermal.values,alignment:alignment,pose:pose,threshold:heatThreshold)
            if oldest>60_000 { order.removeFirst(oldest);oldest=0 }
        } else { result.status="Aligned · thermal frame stale" }
        result.points=Array(voxels.values);return result
    }
    static func isObservedFreeSpace(pointDepth:Float,measuredDepth:Float) -> Bool {
        measuredDepth.isFinite && measuredDepth>=0.2 && measuredDepth<=6 && pointDepth<measuredDepth-0.12
    }
    static func makeHeatSurface(frame f:SensorFrame,temperatures:[Float],alignment:ThermalAlignment,pose:simd_float4x4,threshold:Float) -> [SIMD4<Float>] {
        let step=4,w=(f.width+step-1)/step,h=(f.height+step-1)/step
        var grid=[SIMD4<Float>?](repeating:nil,count:w*h),surface=[SIMD4<Float>]()
        for y in 0..<h {for x in 0..<w {
            let u=x*step,v=y*step,z=f.depth[v*f.width+u]
            guard z.isFinite,z>=0.2,z<=6,let i=alignment.index(x:u,y:v,frame:f),i<temperatures.count,temperatures[i].isFinite else{continue}
            grid[y*w+x]=SIMD4(Float((Double(u)-f.k[2])/f.k[0])*z,Float((Double(v)-f.k[5])/f.k[4])*z,z,temperatures[i])
        }}
        guard w>1,h>1 else{return []}
        for y in 0..<h-1 {for x in 0..<w-1 {
            let a=y*w+x,b=a+1,c=a+w,d=c+1
            for ids in [[a,b,c],[b,d,c]] {
                guard let p=grid[ids[0]],let q=grid[ids[1]],let r=grid[ids[2]],max(p.w,max(q.w,r.w))>=threshold else{continue}
                // Do not join a foreground body to a distant wall across a depth edge.
                guard max(p.z,max(q.z,r.z))-min(p.z,min(q.z,r.z))<0.12 else{continue}
                for v in [p,q,r] {let position=pose*SIMD4(v.x,v.y,v.z,1);surface.append(SIMD4(position.x,position.y,position.z,v.w))}
            }
        }}
        return surface
    }
    func thermalSample(stamp:Double?,values:[Float],at capture:Double) -> (stamp:Double,values:[Float])? {
        if let stamp,stamp.isFinite,thermalHistory.last?.stamp != stamp {
            thermalHistory.append((stamp,values))
            if thermalHistory.count>8 {thermalHistory.removeFirst()}
        }
        return thermalHistory.min{abs($0.stamp-capture)<abs($1.stamp-capture)}
    }
    func confirmAlignment(_ transform:simd_float4x4?,at time:Double) {
        guard time.isFinite else {return}
        alignmentSamples.removeAll{time-$0.time>4 || time<$0.time}
        if let transform,alignmentSamples.last?.time != time {alignmentSamples.append((time,transform))}
        if alignmentSamples.count>8 {alignmentSamples.removeFirst(alignmentSamples.count-8)}
        // ponytail: bounded eight-pose consensus; revisit only if longer histories are needed.
        let groups=alignmentSamples.map {seed in alignmentSamples.filter {
            distance(seed.pose,$0.pose)<0.08 && rotationDistance(seed.pose,$0.pose)<0.08
        }}
        guard let group=groups.max(by:{$0.count<$1.count}) else {confirmations=0;return}
        confirmations=group.count
        guard confirmations>=3 else {return}
        let reference=simd_quatf(group[0].pose).vector
        var orientation=SIMD4<Float>(repeating:0),position=SIMD4<Float>(repeating:0)
        for sample in group {
            let q=simd_quatf(sample.pose).vector
            orientation += simd_dot(reference,q)<0 ? -q:q
            position += sample.pose.columns.3
        }
        var averaged=simd_float4x4(simd_quatf(vector:simd_normalize(orientation)))
        averaged.columns.3=position/Float(group.count)
        worldFromMap=averaged
    }
    // Shared-view checks refine alignment when available; loss of overlap is
    // not loss of tracking. Only explicit resets or new map coordinates invalidate it.
    func currentAlignment() -> simd_float4x4? { worldFromMap }
    private func distance(_ a: simd_float4x4,_ b: simd_float4x4)->Float { simd_length(a.columns.3-b.columns.3) }
    private func rotationDistance(_ a: simd_float4x4,_ b: simd_float4x4)->Float {
        let d=a.inverse*b;return acos(min(1,max(-1,(d[0][0]+d[1][1]+d[2][2]-1)/2)))
    }

}

extension PhoneObservation {
    @concurrent static func capture(_ f: ARFrame, unixTime:Double) async -> PhoneObservation? {
        guard let scene=f.sceneDepth else { return nil }
        let image=f.capturedImage, depth=scene.depthMap,confidence=scene.confidenceMap
        CVPixelBufferLockBaseAddress(image,.readOnly);CVPixelBufferLockBaseAddress(depth,.readOnly)
        if let confidence {CVPixelBufferLockBaseAddress(confidence,.readOnly)}
        defer {
            CVPixelBufferUnlockBaseAddress(image,.readOnly);CVPixelBufferUnlockBaseAddress(depth,.readOnly)
            if let confidence {CVPixelBufferUnlockBaseAddress(confidence,.readOnly)}
        }
        let iw=CVPixelBufferGetWidthOfPlane(image,0),ih=CVPixelBufferGetHeightOfPlane(image,0)
        let step=max(1,iw/640),w=iw/step,h=ih/step
        guard let ybase=CVPixelBufferGetBaseAddressOfPlane(image,0),let dbase=CVPixelBufferGetBaseAddress(depth) else { return nil }
        let ys=CVPixelBufferGetBytesPerRowOfPlane(image,0),ds=CVPixelBufferGetBytesPerRow(depth)/4
        let dw=CVPixelBufferGetWidth(depth),dh=CVPixelBufferGetHeight(depth)
        let cbase=confidence.flatMap{CVPixelBufferGetBaseAddress($0)?.assumingMemoryBound(to:UInt8.self)}
        let cw=confidence.map{CVPixelBufferGetWidth($0)} ?? 0,ch=confidence.map{CVPixelBufferGetHeight($0)} ?? 0,cs=confidence.map{CVPixelBufferGetBytesPerRow($0)} ?? 0
        var gray=Data(count:w*h),values=[Float](repeating:0,count:w*h)
        let scaled=gray.withUnsafeMutableBytes {bytes in
            var source=vImage_Buffer(data:ybase,height:vImagePixelCount(h*step),width:vImagePixelCount(w*step),rowBytes:ys)
            var target=vImage_Buffer(data:bytes.baseAddress!,height:vImagePixelCount(h),width:vImagePixelCount(w),rowBytes:w)
            return vImageScale_Planar8(&source,&target,nil,vImage_Flags(kvImageHighQualityResampling))
        }
        guard scaled==kvImageNoError else{return nil}
        for y in 0..<h { for x in 0..<w {
            values[y*w+x]=dbase.assumingMemoryBound(to:Float.self)[min(dh-1,y*dh/h)*ds+min(dw-1,x*dw/w)]
            if let cbase {
                if cbase[(y*ch/h)*cs+x*cw/w]==UInt8(ARConfidenceLevel.low.rawValue) {values[y*w+x] = .nan}
            }
        }}
        let k=f.camera.intrinsics,s=Double(step),center=Double(step-1)/2
        let normal:Bool
        if case .normal=f.camera.trackingState {normal=true} else {normal=false}
        return PhoneObservation(unixTime:unixTime,pose:f.camera.transform,normal:normal,
                                gray:gray,depth:values.withUnsafeBytes{Data($0)},w:w,h:h,
                                k:[Double(k[0][0])/s,0,(Double(k[2][0])-center)/s,0,Double(k[1][1])/s,(Double(k[2][1])-center)/s,0,0,1])
    }
}

@MainActor @Observable final class PhoneReconstruction: NSObject, ARSessionDelegate {
    var status="Start a phone scan on the same Wi-Fi as the Pi"
    var running=false
    var points: [SIMD4<Float>]=[] { didSet { pointRevision += 1 } }
    var heatSurface:[SIMD4<Float>]=[]
    var heatObservationTime=0.0
    var heatHighlight=UserDefaults.standard.object(forKey:"thermal-heat-highlight") as? Bool ?? true {
        didSet {UserDefaults.standard.set(heatHighlight,forKey:"thermal-heat-highlight")}
    }
    var showHeatThroughWalls=UserDefaults.standard.object(forKey:"thermal-show-through-walls") as? Bool ?? true {
        didSet {UserDefaults.standard.set(showHeatThroughWalls,forKey:"thermal-show-through-walls")}
    }
    var heatThreshold=UserDefaults.standard.object(forKey:"thermal-heat-threshold") as? Double ?? 20.0 {
        didSet {UserDefaults.standard.set(heatThreshold,forKey:"thermal-heat-threshold")}
    }
    var pointRevision=0
    var worldFromMap=matrix_identity_float4x4
    var aligned=false
    var alignmentEstablished=false
    var phoneTrackingReason="initializing"
    @ObservationIgnored private var phoneTrackingNormal=false
    var alignmentDiagnostics: [String:Int]=[:]
    var rigPreview: UIImage?
    var alignment=ThermalAlignment()
    var automaticTemperatureScale=UserDefaults.standard.object(forKey:"thermal-automatic-scale") as? Bool ?? false {
        didSet {UserDefaults.standard.set(automaticTemperatureScale,forKey:"thermal-automatic-scale")}
    }
    var lowerTemperature=UserDefaults.standard.object(forKey:"thermal-scale-lower") as? Double ?? 19.0 {
        didSet {if !automaticTemperatureScale {UserDefaults.standard.set(lowerTemperature,forKey:"thermal-scale-lower")}}
    }
    var upperTemperature=UserDefaults.standard.object(forKey:"thermal-scale-upper") as? Double ?? 28.0 {
        didSet {if !automaticTemperatureScale {UserDefaults.standard.set(upperTemperature,forKey:"thermal-scale-upper")}}
    }
    var renderingError=""
    var cameraWarning=""
    var cameraFPS=0.0
    var cameraAgeMS=0.0
    @ObservationIgnored private let log=ScanLog()
    var logURL:URL {log.url}
    @ObservationIgnored private var lastDiagnostic=0.0
    @ObservationIgnored private var lastDiagnosticStatus=""
    @ObservationIgnored private var cameraFrames=0
    @ObservationIgnored private var cameraStart=ProcessInfo.processInfo.systemUptime
    var renderFPS=0.0
    var fps=0.0
    var latencyMS=0.0
    var dropped=0
    var inliers=0
    var thermalState="Nominal"
    @ObservationIgnored let arSession=ARSession()
    @ObservationIgnored private let worker=ReconstructionWorker()
    @ObservationIgnored private var observations: [PhoneObservation]=[]
    @ObservationIgnored private var task: Task<Void,Never>?
    @ObservationIgnored private var pendingReset: Task<Void,Never>?
    @ObservationIgnored private var alignmentRevision=0
    @ObservationIgnored private var savedKey=""
    @ObservationIgnored private var generation=UUID()
    @ObservationIgnored private var observationBusy=false
    @ObservationIgnored private var lastObservation=0.0
    func observe(_ frame: ARFrame) {
        phoneTrackingChanged(frame.camera.trackingState)
        cameraFrames += 1
        let now=ProcessInfo.processInfo.systemUptime
        if now-cameraStart>=1 {cameraFPS=Double(cameraFrames)/(now-cameraStart);cameraFrames=0;cameraStart=now}
        guard !observationBusy,frame.timestamp-lastObservation>=0.045 else {return}
        observationBusy=true;lastObservation=frame.timestamp
        let time=Date().timeIntervalSince1970-ProcessInfo.processInfo.systemUptime+frame.timestamp
        let id=generation
        Task {
            let observation=await PhoneObservation.capture(frame,unixTime:time)
            observationBusy=false
            guard id==generation,let observation else{return}
            observations.append(observation)
            if observations.count>30 {observations.removeFirst(observations.count-30)}
        }
    }
    func start(_ endpoint: URL) {
        stop();let id=UUID();generation=id
        guard ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) else {status="A LiDAR iPhone or iPad is required";return}
        log.write("scan_start",["app_version":Bundle.main.infoDictionary?["CFBundleShortVersionString"] ?? "unknown",
                                "system":ProcessInfo.processInfo.operatingSystemVersionString])
        arSession.delegate=self;arSession.delegateQueue = .main
        cameraWarning="";running=true
        status="Connecting to Pi · show both cameras the same scene"
        let config=ARWorldTrackingConfiguration();config.frameSemantics=[.sceneDepth]
        arSession.run(config,options:[.resetTracking,.removeExistingAnchors])
        task=Task { await worker.reset();await loop(endpoint,id:id) }
    }
    func session(_ session:ARSession,didUpdate frame:ARFrame){observe(frame)}
    func session(_ session:ARSession,cameraDidChangeTrackingState camera:ARCamera) {
        phoneTrackingChanged(camera.trackingState)
    }
    func phoneTrackingChanged(_ state:ARCamera.TrackingState) {
        let reason:String
        switch state {
        case .normal: reason="normal"
        case .notAvailable: reason="unavailable"
        case .limited(.excessiveMotion): reason="move more slowly"
        case .limited(.insufficientFeatures): reason="scan a textured area"
        case .limited(.relocalizing): reason="return to a previously seen area"
        default: reason="initializing"
        }
        phoneTrackingNormal=reason=="normal"
        if reason != phoneTrackingReason {log.write("phone_tracking",["reason":reason,"alignment_saved":alignmentEstablished]);phoneTrackingReason=reason}
        if !phoneTrackingNormal {aligned=false;status="Phone tracking paused · \(reason)"}
    }
    func session(_ session:ARSession,didFailWithError error:Error){cameraError(error.localizedDescription);stop();status=error.localizedDescription}
    func sessionWasInterrupted(_ session:ARSession){cameraError("AR session interrupted");observations=[];phoneTrackingChanged(.notAvailable)}
    func sessionShouldAttemptRelocalization(_ session:ARSession)->Bool {true}
    func suspend() {
        guard running else{return}
        log.write("app_background");generation=UUID();task?.cancel();task=nil
        arSession.pause();running=false;aligned=false;phoneTrackingNormal=false;observations=[];heatSurface=[]
    }
    func resume(_ endpoint:URL) {
        guard !running else{return}
        if task == nil,arSession.delegate == nil {start(endpoint);return}
        let id=UUID();generation=id;running=true
        let config=ARWorldTrackingConfiguration();config.frameSemantics=[.sceneDepth]
        arSession.run(config)
        task=Task {await loop(endpoint,id:id)}
    }
    func stop() { if running {log.write("scan_stop")};generation=UUID();task?.cancel();task=nil;arSession.pause();running=false;aligned=false;alignmentEstablished=false;phoneTrackingNormal=false;alignmentDiagnostics=[:];rigPreview=nil;observations=[];points=[];heatSurface=[];arSession.delegate=nil;status="Scan paused" }
    func realign() { log.write("realign_requested");alignmentRevision += 1;aligned=false;alignmentEstablished=false;alignmentDiagnostics=[:];points=[];heatSurface=[];pendingReset=Task { await worker.reset() } }
    func cameraHealth(age:Double) {
        cameraAgeMS=max(0,age*1000)
        if age>1,cameraWarning.isEmpty {
            cameraWarning="Phone camera stalled · waiting to recover"
            aligned=false;log.write("camera_stalled",["age_ms":cameraAgeMS])
        } else if age<0.3,!cameraWarning.isEmpty {cameraWarning="";log.write("camera_resumed")}
    }
    func cameraError(_ message:String) {log.write("ar_error",["message":message])}
    func apply(_ result:ReconstructionResult) {
        alignmentEstablished=result.worldFromMap != nil
        // A capture-time result can arrive after the live phone tracking has degraded.
        aligned=result.valid && phoneTrackingNormal && cameraWarning.isEmpty
        worldFromMap=result.worldFromMap ?? matrix_identity_float4x4
        points=result.points;heatSurface=result.heatSurface;inliers=result.inliers
        status=phoneTrackingNormal ? result.status : "Phone tracking paused · \(phoneTrackingReason)"
        alignmentDiagnostics=result.alignmentDiagnostics
    }
    func saveAlignment() { guard !savedKey.isEmpty,let data=try? JSONEncoder().encode(alignment) else{return};UserDefaults.standard.set(data,forKey:savedKey);log.write("thermal_alignment_saved",["profile":savedKey,"parameters":String(describing:alignment)]) }
    private func loop(_ origin: URL,id:UUID) async {
        var offset=0.0,bestRTT=Double.infinity,lastClock=0.0,lastSequence=0,lastSession="",lastTime=Date(),count=0,lastPreview=0.0
        let config=URLSessionConfiguration.ephemeral;config.timeoutIntervalForRequest=3
        let network=URLSession(configuration:config)
        defer { network.invalidateAndCancel() }
        while !Task.isCancelled && generation==id {
            do {
                if Date().timeIntervalSince1970-lastClock>15 {
                    // Min-RTT NTP-style midpoint estimate; refresh to follow clock drift.
                    bestRTT=Double.infinity
                    for _ in 0..<3 {
                        let before=Date().timeIntervalSince1970
                        let (bytes,response)=try await network.data(from:origin.appendingPathComponent("api/clock"))
                        let after=Date().timeIntervalSince1970
                        guard (response as? HTTPURLResponse)?.statusCode==200,
                              let json=try JSONSerialization.jsonObject(with:bytes) as? [String:Double],let stamp=json["timestamp"],stamp.isFinite else {throw APIError.invalidData}
                        if after-before<bestRTT {bestRTT=after-before;offset=stamp-(before+after)/2}
                    }
                    log.write("clock_sync",["offset_ms":offset*1000,"rtt_ms":bestRTT*1000])
                    lastClock=Date().timeIntervalSince1970
                }
                let (bytes,response)=try await network.data(from:origin.appendingPathComponent("api/sensors"))
                guard (response as? HTTPURLResponse)?.statusCode==200 else {
                    throw NSError(domain:"Sensor stream",code:(response as? HTTPURLResponse)?.statusCode ?? 0,
                                  userInfo:[NSLocalizedDescriptionKey:(response as? HTTPURLResponse)?.statusCode == 503 ? "Pi is waiting for RGB and depth frames" : "Pi sensor endpoint is unavailable"])
                }
                let frame=try await SensorFrame.decode(bytes)
                guard !Task.isCancelled,generation==id else {return}
                if frame.session != lastSession {
                    log.write("sensor_session",["session":frame.session,"width":frame.width,"height":frame.height]);lastSession=frame.session;lastSequence=0;points=[];aligned=false;alignmentEstablished=false;dropped=0
                    savedKey="thermal-display-v2-\(origin.absoluteString)-\(frame.width)x\(frame.height)-\(frame.k)"
                    if let data=UserDefaults.standard.data(forKey:savedKey),let saved=try? JSONDecoder().decode(ThermalAlignment.self,from:data){alignment=saved}
                    else if let data=UserDefaults.standard.data(forKey:savedKey.replacingOccurrences(of:"thermal-display-v2-",with:"thermal-v1-")),var saved=try? JSONDecoder().decode(ThermalAlignment.self,from:data) {
                        saved.flipX=false;saved.flipY=false;alignment=saved;saveAlignment()
                    } else {alignment=ThermalAlignment(frame.metadata["alignment"] as? [String:Any] ?? [:])}
                    // Consume a camera-specific adjustment once; later slider saves remain authoritative.
                    let importURL=FileManager.default.urls(for:.documentDirectory,in:.userDomainMask)[0].appendingPathComponent("ThermalAlignment.json")
                    if FileManager.default.fileExists(atPath:importURL.path) {
                        if let data=try? Data(contentsOf:importURL,options:.mappedIfSafe),let imported=ThermalAlignment.importedProfile(data,key:savedKey) {
                            alignment=imported;saveAlignment()
                            do {try FileManager.default.removeItem(at:importURL)}
                            catch {log.write("thermal_alignment_import_cleanup_failed",["error":error.localizedDescription])}
                        } else {log.write("thermal_alignment_import_rejected",["profile":savedKey])}
                    }
                    log.write("thermal_alignment_loaded",["profile":savedKey,"parameters":String(describing:alignment)])
                }
                guard frame.sequence>lastSequence else {try await Task.sleep(for:.milliseconds(15));continue}
                if lastSequence>0 {dropped += max(0,frame.sequence-lastSequence-1)};lastSequence=frame.sequence
                latencyMS=(Date().timeIntervalSince1970+offset-frame.stamp)*1000
                guard latencyMS >= -100,latencyMS<500 else {aligned=false;rigPreview=nil;status="Sensor stream stale";try await Task.sleep(for:.milliseconds(100));continue}
                let phone=observations.min {abs($0.unixTime+offset-frame.stamp)<abs($1.unixTime+offset-frame.stamp)}
                let paired=phone.flatMap {phoneTrackingNormal && abs($0.unixTime+offset-frame.stamp)+bestRTT/2<0.1 ? $0:nil}
                let revision=alignmentRevision
                await pendingReset?.value
                let result=await worker.process(frame,phone:paired,alignment:alignment,heatThreshold:Float(heatThreshold))
                if automaticTemperatureScale,let range=SensorFrame.temperatureRange(frame.thermal) {
                    lowerTemperature=range.0;upperTemperature=range.1
                }
                guard !Task.isCancelled,generation==id else {return}
                guard revision==alignmentRevision else {continue}
                apply(result);heatObservationTime=frame.stamp-offset
                if alignmentEstablished {rigPreview=nil}
                else if frame.stamp-lastPreview>=0.5 {rigPreview=UIImage(data:frame.jpeg);lastPreview=frame.stamp}
                let depthPercent=100*Double(frame.depth.lazy.filter{$0.isFinite && $0>=0.2 && $0<=6}.count)/Double(frame.depth.count)
                if !aligned,depthPercent<5 {status=String(format:"Rig depth mostly missing · %.1f%% usable",depthPercent)}
                count += 1;let elapsed=Date().timeIntervalSince(lastTime)
                if elapsed>=1 {fps=Double(count)/elapsed;count=0;lastTime=Date()}
                thermalState=String(describing:ProcessInfo.processInfo.thermalState)
                let body:[String:Any]=["session":frame.session,"timestamp":frame.stamp,"valid":aligned,
                                       "alignment_saved":alignmentEstablished,"phone_tracking":phoneTrackingReason,"rig_tracking":result.rigTrackingState,"rig_keyframes":result.keyframes,
                                       "heat_triangles":heatSurface.count/3,"heat_threshold_c":heatThreshold,"show_through_walls":showHeatThroughWalls,"thermal_delta_ms":result.thermalDeltaMS,"temperature_range":[lowerTemperature,upperTemperature],"rig_depth_valid_percent":depthPercent,"alignment":result.alignmentDiagnostics,"camera_fps":cameraFPS,"camera_age_ms":cameraAgeMS,"clock_rtt_ms":bestRTT*1000,"phone_delta_ms":phone.map{abs($0.unixTime+offset-frame.stamp)*1000} ?? -1,
                                       "fps":fps,"render_fps":renderFPS,"latency_ms":latencyMS,"dropped":dropped,"inliers":inliers,"points":points.count,"status":status,
                                       "phone_pose":poseArray(paired?.pose ?? matrix_identity_float4x4),"rig_pose":poseArray(result.rigPose)]
                if Date().timeIntervalSince1970-lastDiagnostic>=1 || status != lastDiagnosticStatus {
                    var diagnostic=body;diagnostic.removeValue(forKey:"phone_pose");diagnostic.removeValue(forKey:"rig_pose")
                    log.write("frame",diagnostic);lastDiagnostic=Date().timeIntervalSince1970;lastDiagnosticStatus=status
                }
                var request=URLRequest(url:origin.appendingPathComponent("api/poses"));request.httpMethod="POST"
                request.setValue("application/json",forHTTPHeaderField:"Content-Type");request.httpBody=try JSONSerialization.data(withJSONObject:body)
                let (_,poseResponse)=try await network.data(for:request)
                guard (poseResponse as? HTTPURLResponse)?.statusCode==200 else {throw APIError.invalidData}
            } catch {
                guard !Task.isCancelled,generation==id else{return}
                // A network gap hides placement; fresh frames can resume in the same maps.
                log.write("stream_error",["message":error.localizedDescription]);aligned=false;rigPreview=nil;status="Connection interrupted · \(error.localizedDescription)"
                do {try await Task.sleep(for:.milliseconds(500))}catch{return}
            }
        }
    }
}
