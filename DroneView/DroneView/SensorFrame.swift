import Foundation
import Compression
import simd

struct SensorFrame: @unchecked Sendable {
    let metadata: [String: Any]
    let session: String
    let sequence: Int
    let stamp: Double
    let width: Int, height: Int
    let k: [Double]
    let jpeg: Data, depthData: Data
    let depth: [Float], thermal: [Float]
    let thermalWidth: Int, thermalHeight: Int
    let thermalStamp: Double?

    @concurrent static func decode(_ data:Data) async throws -> SensorFrame {try SensorFrame(data)}

    init(_ data: Data) throws {
        func bad() -> NSError { NSError(domain: "Sensor stream", code: 1, userInfo: [NSLocalizedDescriptionKey: "Invalid DVS1 sensor frame"]) }
        guard data.count >= 8, data.prefix(4) == Data("DVS1".utf8), data.count <= 12_000_000 else { throw bad() }
        let length = Int(data.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: 4, as: UInt32.self).littleEndian })
        guard length > 0, length <= 512_000, 8 + length <= data.count,
              let m = try JSONSerialization.jsonObject(with: data.subdata(in: 8..<8+length)) as? [String: Any],
              m["version"] as? Int == 1, let session = m["session"] as? String, UUID(uuidString: session) != nil,
              let sequence = m["sequence"] as? Int, sequence > 0,
              let stamp = m["timestamp"] as? Double, stamp.isFinite,
              let ds = m["depth_timestamp"] as? Double, ds.isFinite, abs(ds-stamp) <= 0.0351,
              let w = m["width"] as? Int, let h = m["height"] as? Int, (1...1920).contains(w), (1...1080).contains(h),
              let tw = m["thermal_width"] as? Int, let th = m["thermal_height"] as? Int, (0...640).contains(tw), (0...480).contains(th),
              let k = m["K"] as? [Double], k.count == 9, k.allSatisfy(\.isFinite), k[0] > 0, k[4] > 0,
              let nj = m["jpeg_bytes"] as? Int, (1...4_000_000).contains(nj),
              let nd=m["depth_bytes"] as? Int, let nt=m["thermal_bytes"] as? Int,
              nd>0,nd<=w*h*4+4096,nt>=0,nt<=tw*th*4+4096,
              m["numeric_compression"] == nil || m["numeric_compression"] as? String == "deflate",
              m["depth_encoding"] as? String == "float32_metres", m["thermal_encoding"] as? String == "float32_celsius",
              8+length+nj+nd+nt == data.count else { throw bad() }
        metadata=m;self.session=session;self.sequence=sequence;self.stamp=stamp;width=w;height=h;self.k=k
        thermalWidth=tw;thermalHeight=th;thermalStamp=m["thermal_timestamp"] as? Double
        var offset=8+length
        jpeg=data.subdata(in: offset..<offset+nj);offset += nj
        func numeric(_ input:Data,count:Int)throws->Data {
            if m["numeric_compression"] == nil { guard input.count==count else {throw bad()};return input }
            if count==0 {return Data()}
            var output=Data(count:count)
            let size=output.withUnsafeMutableBytes { destination in input.withUnsafeBytes { source in
                compression_decode_buffer(destination.bindMemory(to:UInt8.self).baseAddress!,count,source.bindMemory(to:UInt8.self).baseAddress!,input.count,nil,COMPRESSION_ZLIB)
            }}
            guard size==count else {throw bad()};return output
        }
        depthData=try numeric(data.subdata(in:offset..<offset+nd),count:w*h*4);offset += nd
        depth=Self.floats(depthData);thermal=Self.floats(try numeric(data.subdata(in:offset..<data.count),count:tw*th*4))
    }
    static func temperatureRange(_ values:[Float]) -> (Double,Double)? {
        let sorted=values.filter(\.isFinite).sorted()
        guard !sorted.isEmpty else{return nil}
        func percentile(_ q:Double)->Double {
            let x=Double(sorted.count-1)*q,i=Int(x)
            return Double(sorted[i])+(Double(sorted[min(i+1,sorted.count-1)])-Double(sorted[i]))*(x-Double(i))
        }
        let low=percentile(0.02),high=percentile(0.98)
        return (low,max(low+1,high)) // Same percentile range and minimum span as the Pi preview.
    }
    static func floats(_ data: Data) -> [Float] {
        data.withUnsafeBytes { raw in stride(from: 0, to: data.count, by: 4).map { Float(bitPattern: UInt32(littleEndian: raw.loadUnaligned(fromByteOffset: $0, as: UInt32.self))) } }
    }
}

struct ThermalAlignment: Codable, Equatable, Sendable {
    var offsetX=0.0, offsetY=0.0, scale=0.85, barrel = -0.2, stretchX=0.9, stretchY=1.0
    var hfov=90.0, vfov=68.0, referenceWidth=640.0, referenceHeight=480.0
    var flipX=false, flipY=false
    init() {}
    init(_ values: [String: Any]) {
        self.init()
        offsetX=values["offset_x"] as? Double ?? offsetX;offsetY=values["offset_y"] as? Double ?? offsetY
        scale=values["scale"] as? Double ?? scale;barrel=values["barrel_distortion"] as? Double ?? barrel
        stretchX=values["stretch_x"] as? Double ?? stretchX;stretchY=values["stretch_y"] as? Double ?? stretchY
        hfov=values["thermal_hfov"] as? Double ?? hfov;vfov=values["thermal_vfov"] as? Double ?? vfov
        referenceWidth=values["offset_reference_width"] as? Double ?? referenceWidth
        referenceHeight=values["offset_reference_height"] as? Double ?? referenceHeight
        // The Pi stream is already oriented for its RGB view. Legacy browser
        // flips described a different input orientation and are not applied again.
    }
    static func importedProfile(_ data:Data,key:String) -> Self? {
        guard data.count<=64_000,let a=(try? JSONDecoder().decode([String:Self].self,from:data))?[key],
              [a.offsetX,a.offsetY,a.scale,a.barrel,a.stretchX,a.stretchY,a.hfov,a.vfov,a.referenceWidth,a.referenceHeight].allSatisfy(\.isFinite),
              a.scale>0,a.stretchX>0,a.stretchY>0,a.referenceWidth>0,a.referenceHeight>0,
              (1..<179).contains(a.hfov),(1..<179).contains(a.vfov) else{return nil}
        return a
    }
    // Legacy FOV/offset/barrel lookup, evaluated against the current registered RGB intrinsics.
    func index(x: Int, y: Int, frame f: SensorFrame) -> Int? {
        guard scale.isFinite, scale > 0, stretchX > 0, stretchY > 0, referenceWidth > 0, referenceHeight > 0,
              (1..<179).contains(hfov), (1..<179).contains(vfov), f.thermalWidth > 0, f.thermalHeight > 0 else { return nil }
        let w=max(Double(f.thermalWidth), 2*f.k[0]*tan(hfov * .pi/360)*scale*stretchX)
        let h=max(Double(f.thermalHeight), 2*f.k[4]*tan(vfov * .pi/360)*scale*stretchY)
        var u=(Double(x)-(f.k[2]-w/2+offsetX*Double(f.width)/referenceWidth))/w
        var v=(Double(y)-(f.k[5]-h/2+offsetY*Double(f.height)/referenceHeight))/h
        let nx=u*2-1, ny=v*2-1, radial=1+barrel*(nx*nx+ny*ny)/2
        u=(nx*radial+1)/2;v=(ny*radial+1)/2
        guard u.isFinite, v.isFinite, u>=0, u<1, v>=0, v<1 else { return nil }
        let tx=Int(u*Double(f.thermalWidth)), ty=Int(v*Double(f.thermalHeight))
        // Numeric values and the Pi preview share the same driver orientation.
        // User flips are additional corrections relative to that displayed image.
        return (flipY ? f.thermalHeight-1-ty : ty)*f.thermalWidth + (flipX ? f.thermalWidth-1-tx : tx)
    }
}

func poseMatrix(_ data: Data) -> simd_float4x4? {
    guard data.count == 64 else { return nil }
    let a=SensorFrame.floats(data)
    guard a.allSatisfy(\.isFinite) else { return nil }
    return simd_float4x4(columns: (SIMD4(a[0],a[1],a[2],a[3]),SIMD4(a[4],a[5],a[6],a[7]),SIMD4(a[8],a[9],a[10],a[11]),SIMD4(a[12],a[13],a[14],a[15])))
}
let opticalToAR = simd_float4x4(diagonal: SIMD4<Float>(1,-1,-1,1))
func poseArray(_ m: simd_float4x4) -> [Float] { (0..<4).flatMap { c in (0..<4).map { m[c][$0] } } }
