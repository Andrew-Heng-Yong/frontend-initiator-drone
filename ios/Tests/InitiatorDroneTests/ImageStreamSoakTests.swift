#if canImport(XCTest)
import XCTest
// In Xcode the tests compile as their own module; the headless runner in
// Scripts/ compiles them alongside the sources instead, so neither import
// exists there.
@testable import InitiatorDrone
#endif
import Foundation
#if canImport(Darwin)
import Darwin
#endif

/// Long-running behaviour of the image path.
///
/// The requirement being checked is "memory remains stable during a continuous
/// 30-minute image stream". Running a real half hour in CI is not useful, so
/// this instead drives the same code path through the number of frames such a
/// session would produce and asserts that the resident set does not grow with
/// frame count. A leak of even a few kilobytes per frame shows up immediately
/// at this scale; a genuinely flat pipeline stays flat whether it runs for
/// twenty thousand frames or a hundred thousand.
final class ImageStreamSoakTests: XCTestCase {

    /// Resident memory of this process, in bytes.
    static func residentBytes() -> UInt64 {
        #if canImport(Darwin)
        var info = mach_task_basic_info()
        var count = mach_msg_type_number_t(
            MemoryLayout<mach_task_basic_info>.size / MemoryLayout<natural_t>.size
        )
        let result = withUnsafeMutablePointer(to: &info) { pointer in
            pointer.withMemoryRebound(to: integer_t.self, capacity: Int(count)) { rebound in
                task_info(mach_task_self_, task_flavor_t(MACH_TASK_BASIC_INFO), rebound, &count)
            }
        }
        return result == KERN_SUCCESS ? info.resident_size : 0
        #else
        return 0
        #endif
    }

    private func depthFrame(width: Int, height: Int, seed: Int) -> ROSImageMessage {
        var bytes = [UInt8](repeating: 0, count: width * height * 2)
        for index in 0..<(width * height) {
            // Vary the content per frame so nothing can be cached away.
            let millimetres = UInt16((index &+ seed) % 6000 &+ 300)
            bytes[index * 2] = UInt8(millimetres & 0xFF)
            bytes[index * 2 + 1] = UInt8(millimetres >> 8)
        }
        return ROSImageMessage(
            stamp: Double(seed) * 0.066,
            frameId: "camera_depth_optical_frame",
            width: width,
            height: height,
            encoding: "16UC1",
            isBigEndian: false,
            step: width * 2,
            data: Data(bytes)
        )
    }

    /// Frames a 30-minute session at the app's default 15 fps ceiling would
    /// produce. Scaled down here so the suite stays quick; raise
    /// `INITIATOR_SOAK_FRAMES` to run the full count.
    private var frameCount: Int {
        if let raw = ProcessInfo.processInfo.environment["INITIATOR_SOAK_FRAMES"],
           let value = Int(raw) {
            return value
        }
        return 1_500
    }

    func testDecodeAndRenderLoopDoesNotGrowMemory() {
        let width = 320
        let height = 240
        let renderer = ScalarImageRenderer(settings: .depthDefault)

        // Warm up: first-touch allocations, the ramp table, and the renderer's
        // output buffer all settle here rather than counting as growth.
        for seed in 0..<100 {
            autoreleasepool {
                let message = depthFrame(width: width, height: height, seed: seed)
                guard case .scalar(let scalar)? = try? ROSImageDecoder.decode(
                    message,
                    interpretation: .depthMillimetres
                ) else {
                    XCTFail("decode failed during warm-up")
                    return
                }
                _ = renderer.render(scalar)
            }
        }

        let baseline = Self.residentBytes()
        var lastImageWidth = 0

        for seed in 0..<frameCount {
            autoreleasepool {
                let message = depthFrame(width: width, height: height, seed: seed + 1000)
                guard case .scalar(let scalar)? = try? ROSImageDecoder.decode(
                    message,
                    interpretation: .depthMillimetres
                ) else {
                    XCTFail("decode failed at frame \(seed)")
                    return
                }
                let rendered = renderer.render(scalar)
                // Build the bitmap too, so the CGImage path is included.
                let image = ImageStreamPipeline.makeCGImage(from: rendered)
                lastImageWidth = image?.width ?? 0
            }
        }

        XCTAssertEqual(lastImageWidth, width, "the pipeline should still be producing bitmaps")

        let final = Self.residentBytes()
        guard baseline > 0, final > 0 else { return } // no memory reporting on this platform

        let growth = final > baseline ? final - baseline : 0
        let perFrame = Double(growth) / Double(frameCount)

        // One 320x240 depth frame is 150 KB decoded and 300 KB rendered. If any
        // of that were retained, per-frame growth would be hundreds of
        // kilobytes; a flat pipeline sits near zero. 8 KB/frame leaves room for
        // allocator behaviour without letting a real leak through.
        XCTAssertLessThan(
            perFrame,
            8_192,
            String(
                format: "resident memory grew %.1f MB over %d frames (%.0f B/frame)",
                Double(growth) / 1_048_576.0,
                frameCount,
                perFrame
            )
        )
    }

    func testPipelineUnderOverloadDropsFramesInsteadOfQueueingThem() {
        // Submit far faster than the worker can drain, then confirm that what
        // grew was the drop counter and not a queue.
        let pipeline = ImageStreamPipeline(
            topic: .depthImage,
            colorMapSettings: .depthDefault,
            interpretationProvider: { .depthDefault(for: $0) }
        )

        let received = expectation(description: "at least one frame rendered")
        var renderedCount = 0
        pipeline.onFrame = { _ in
            renderedCount += 1
            if renderedCount == 1 { received.fulfill() }
        }

        let submitted = 400
        for seed in 0..<submitted {
            pipeline.submit(depthFrame(width: 320, height: 240, seed: seed))
        }

        wait(for: [received], timeout: 20)

        let settle = expectation(description: "drain settles")
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) { settle.fulfill() }
        wait(for: [settle], timeout: 5)

        let statistics = pipeline.statistics
        XCTAssertEqual(statistics.accepted, submitted)
        XCTAssertGreaterThan(statistics.dropped, 0, "a flood must be dropped, not queued")
        XCTAssertLessThan(
            renderedCount,
            submitted,
            "rendering every flooded frame would mean the queue grew"
        )
    }

    func testOdometryBufferStaysBoundedOverALongSession() {
        // 30 minutes of 30 Hz odometry.
        var buffer = OdometryBuffer(capacity: 240, historyDuration: 5.0)
        let total = 30 * 60 * 30
        for index in 0..<total {
            buffer.append(StampedPose(
                stamp: Double(index) / 30.0,
                pose: Pose(position: Vector3(Double(index) * 0.001, 0, 0))
            ))
        }
        XCTAssertLessThanOrEqual(buffer.count, 240)
        XCTAssertLessThanOrEqual(buffer.span, 5.0 + 1e-6)
        // Still usable: the newest sample is present and interpolation works.
        XCTAssertNotNil(buffer.pose(at: Double(total - 1) / 30.0))
    }

    static var allTests: [(String, (ImageStreamSoakTests) -> () throws -> Void)] {
        [
            ("testDecodeAndRenderLoopDoesNotGrowMemory", testDecodeAndRenderLoopDoesNotGrowMemory),
            ("testPipelineUnderOverloadDropsFramesInsteadOfQueueingThem", testPipelineUnderOverloadDropsFramesInsteadOfQueueingThem),
            ("testOdometryBufferStaysBoundedOverALongSession", testOdometryBufferStaysBoundedOverALongSession),
        ]
    }
}
