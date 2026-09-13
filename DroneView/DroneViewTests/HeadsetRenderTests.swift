import XCTest
import MetalKit
@testable import DroneView

final class HeadsetRenderTests: XCTestCase {
    @MainActor func testFullSizeGridAndHiddenHUDCameraWarning() throws {
        let device = try XCTUnwrap(MTLCreateSystemDefaultDevice())
        let suite = "HeadsetRender-\(UUID().uuidString)"
        let defaults = try XCTUnwrap(UserDefaults(suiteName: suite))
        defer { defaults.removePersistentDomain(forName: suite) }
        let settings = HeadsetSettings(defaults: defaults)
        let renderer = ARMetalView.Renderer(model: PhoneReconstruction(), headset: settings)
        let view = MTKView(frame: .zero, device: device)
        view.depthStencilPixelFormat = .depth32Float
        renderer.configure(view)
        let queue = try XCTUnwrap(renderer.queue)
        let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm, width: 2622, height: 1206, mipmapped: false)
        descriptor.storageMode = .shared; descriptor.usage = [.renderTarget, .shaderRead]
        let output = try XCTUnwrap(device.makeTexture(descriptor: descriptor))
        for grid in [true, false] {
            settings.showGrid = grid; settings.showHUD = false
            renderer.updateHUD(warning: grid ? "CALIBRATION GRID · NOT LIVE" : "CAMERA UNAVAILABLE · REMOVE HEADSET\nWaiting for camera recovery",
                               aspect: 4.0 / 3, device: device)
            let hud = try XCTUnwrap(renderer.hudTexture, renderer.model.renderingError)
            let command = try XCTUnwrap(queue.makeCommandBuffer())
            let source = try XCTUnwrap(renderer.makeCompositePass(size: CGSize(width: 1311, height: 983), device: device))
            let encoder = try XCTUnwrap(command.makeRenderCommandEncoder(descriptor: source))
            encoder.endEncoding()
            let pass = MTLRenderPassDescriptor()
            pass.colorAttachments[0].texture = output; pass.colorAttachments[0].loadAction = .clear; pass.colorAttachments[0].storeAction = .store
            XCTAssertTrue(renderer.encodeHeadset(command: command, pass: pass, source: try XCTUnwrap(renderer.composite),
                                                  hud: hud, size: CGSize(width: 2622, height: 1206),
                                                  profile: settings.profile, grid: grid))
            command.commit(); command.waitUntilCompleted()
            XCTAssertEqual(command.status, .completed)
            var pixels = [UInt8](repeating: 0, count: 2622 * 1206 * 4)
            pixels.withUnsafeMutableBytes { output.getBytes($0.baseAddress!, bytesPerRow: 2622 * 4, from: MTLRegionMake2D(0, 0, 2622, 1206), mipmapLevel: 0) }
            XCTAssertTrue(stride(from: 0, to: pixels.count, by: 4).contains { pixels[$0] > 150 }, "Warnings/grid must remain visible when display information is hidden")
            let provider = try XCTUnwrap(CGDataProvider(data: Data(pixels) as CFData))
            let bitmapInfo = CGBitmapInfo.byteOrder32Little.union(CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedFirst.rawValue))
            let image = try XCTUnwrap(CGImage(width: 2622, height: 1206, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: 2622 * 4,
                                             space: CGColorSpaceCreateDeviceRGB(), bitmapInfo: bitmapInfo, provider: provider, decode: nil,
                                             shouldInterpolate: false, intent: .defaultIntent))
            let attachment = XCTAttachment(image: UIImage(cgImage: image))
            attachment.name = grid ? "Headset calibration grid" : "Camera unavailable with HUD hidden"
            attachment.lifetime = .keepAlways
            add(attachment)
        }
    }

    @MainActor func testProfilePersistenceAndCameraFreshness() throws {
        let suite = "HeadsetTests-\(UUID().uuidString)"
        let defaults = try XCTUnwrap(UserDefaults(suiteName: suite))
        defer { defaults.removePersistentDomain(forName: suite) }
        let settings = HeadsetSettings(defaults: defaults)
        settings.profile = HeadsetProfile(scale: 1.2, lensSpacing: 0.48, verticalCenter: 0.52, distortion: 0.3)
        XCTAssertEqual(HeadsetSettings(defaults: defaults).profile, settings.profile)
        settings.profile = HeadsetProfile(scale: .infinity, lensSpacing: -2, verticalCenter: 5, distortion: .nan)
        XCTAssertEqual(HeadsetSettings(defaults: defaults).profile,
                       HeadsetProfile(scale: 1, lensSpacing: 0.35, verticalCenter: 0.65, distortion: 0.2))
        defaults.set(Data("invalid profile".utf8), forKey: "headset-optics-v1")
        XCTAssertEqual(HeadsetSettings(defaults: defaults).profile, HeadsetProfile())
        XCTAssertTrue(HeadsetSettings.cameraIsLive(running: true, age: 0.02))
        for age: Double? in [nil, -0.1, .nan, .infinity, 0.3, 2] {
            XCTAssertFalse(HeadsetSettings.cameraIsLive(running: true, age: age))
        }
        XCTAssertFalse(HeadsetSettings.cameraIsLive(running: false, age: 0.02))
        settings.enter(grid: true)
        XCTAssertTrue(settings.active && settings.showGrid && settings.showHUD)
        settings.active = false
        settings.enter()
        XCTAssertFalse(settings.showGrid)
    }

    @MainActor func testActualGPUHeadsetComposition() throws {
        let device = try XCTUnwrap(MTLCreateSystemDefaultDevice())
        let view = MTKView(frame: .zero, device: device)
        view.colorPixelFormat = .bgra8Unorm
        view.depthStencilPixelFormat = .depth32Float
        let renderer = ARMetalView.Renderer(model: PhoneReconstruction())
        renderer.configure(view)
        let queue = try XCTUnwrap(renderer.queue)
        func texture(_ width: Int, _ height: Int) throws -> MTLTexture {
            let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm, width: width, height: height, mipmapped: false)
            descriptor.storageMode = .shared
            descriptor.usage = [.shaderRead, .renderTarget]
            return try XCTUnwrap(device.makeTexture(descriptor: descriptor))
        }
        let source = try texture(128, 96), hud = try texture(1, 1), output = try texture(256, 128)
        var clear: UInt32 = 0
        hud.replace(region: MTLRegionMake2D(0, 0, 1, 1), mipmapLevel: 0, withBytes: &clear, bytesPerRow: 4)
        var input = [UInt8](repeating: 0, count: 128 * 96 * 4)
        for y in 0..<96 { for x in 0..<128 {
            let i = (y * 128 + x) * 4
            input[i + 3] = 255
            if hypot(Double(x) - 63.5, Double(y) - 47.5) < 16 {
                input[i] = 255; input[i + 1] = 255; input[i + 2] = 255
            }
            if x < 24 && y < 24 { input[i + 2] = 255 }
            if x >= 104 && y >= 72 { input[i] = 255 }
        } }
        input.withUnsafeBytes { source.replace(region: MTLRegionMake2D(0, 0, 128, 96), mipmapLevel: 0, withBytes: $0.baseAddress!, bytesPerRow: 128 * 4) }
        let identity = HeadsetProfile(scale: 1, lensSpacing: 0.5, verticalCenter: 0.5, distortion: 0)
        func draw(_ profile: HeadsetProfile, grid: Bool = false) throws -> [UInt8] {
            let command = try XCTUnwrap(queue.makeCommandBuffer())
            let pass = MTLRenderPassDescriptor()
            pass.colorAttachments[0].texture = output
            pass.colorAttachments[0].loadAction = .clear
            pass.colorAttachments[0].storeAction = .store
            XCTAssertTrue(renderer.encodeHeadset(command: command, pass: pass, source: source, hud: hud,
                                                  size: CGSize(width: 256, height: 128), profile: profile, grid: grid))
            command.commit(); command.waitUntilCompleted()
            XCTAssertEqual(command.status, .completed, command.error?.localizedDescription ?? "GPU failed")
            var pixels = [UInt8](repeating: 0, count: 256 * 128 * 4)
            pixels.withUnsafeMutableBytes { output.getBytes($0.baseAddress!, bytesPerRow: 256 * 4, from: MTLRegionMake2D(0, 0, 256, 128), mipmapLevel: 0) }
            return pixels
        }
        let pixels = try draw(identity)
        func rgb(_ pixels: [UInt8], _ x: Int, _ y: Int) -> [UInt8] {
            let i = (y * 256 + x) * 4
            return Array(pixels[i..<i + 3])
        }
        let left = (0..<128).flatMap { y in Array(pixels[(y * 256 * 4)..<(y * 256 + 128) * 4]) }
        let right = (0..<128).flatMap { y in Array(pixels[((y * 256 + 128) * 4)..<((y + 1) * 256 * 4)]) }
        XCTAssertTrue(left == right, "Both eyes must sample the identical monoscopic scene")
        XCTAssertEqual(rgb(pixels, 12, 28), [0, 0, 255], "Top-left camera pixels must not flip")
        XCTAssertEqual(rgb(pixels, 116, 100), [255, 0, 0], "Bottom-right camera pixels must not flip")
        XCTAssertEqual(rgb(pixels, 64, 4), [0, 0, 0], "Aspect-fit letterbox must be black")
        let horizontal = (0..<128).filter { rgb(pixels, $0, 64).allSatisfy { $0 > 200 } }.count
        let vertical = (0..<128).filter { rgb(pixels, 64, $0).allSatisfy { $0 > 200 } }.count
        XCTAssertGreaterThan(horizontal, 28)
        XCTAssertLessThanOrEqual(abs(horizontal - vertical), 1, "A circle must stay round in a square eye viewport")
        let warped = try draw(HeadsetProfile(scale: 0.6, lensSpacing: 0.5, verticalCenter: 0.5, distortion: 0.4))
        XCTAssertEqual(rgb(warped, 12, 28), [0, 0, 0], "Out-of-bounds warp must not smear camera edges")
        XCTAssertEqual(rgb(warped, 64, 64), [255, 255, 255])
        let grid = try draw(identity, grid: true)
        XCTAssertGreaterThan(rgb(grid, 64, 64)[1], 200, "The real shader must draw the calibration cross")
        // Camera loss clears the same intermediate surface used by the live path.
        let first = try XCTUnwrap(renderer.makeCompositePass(size: CGSize(width: 128, height: 96), device: device))
        let retained = renderer.composite
        _ = renderer.makeCompositePass(size: CGSize(width: 128, height: 96), device: device)
        XCTAssertTrue(retained === renderer.composite, "Do not allocate a new scene texture every frame")
        let command = try XCTUnwrap(queue.makeCommandBuffer())
        let encoder = try XCTUnwrap(command.makeRenderCommandEncoder(descriptor: first))
        encoder.endEncoding()
        let pass = MTLRenderPassDescriptor()
        pass.colorAttachments[0].texture = output; pass.colorAttachments[0].loadAction = .clear; pass.colorAttachments[0].storeAction = .store
        XCTAssertTrue(renderer.encodeHeadset(command: command, pass: pass, source: try XCTUnwrap(renderer.composite), hud: hud,
                                              size: CGSize(width: 256, height: 128), profile: identity, grid: false))
        command.commit(); command.waitUntilCompleted()
        var cleared = [UInt8](repeating: 255, count: 256 * 128 * 4)
        cleared.withUnsafeMutableBytes { output.getBytes($0.baseAddress!, bytesPerRow: 256 * 4, from: MTLRegionMake2D(0, 0, 256, 128), mipmapLevel: 0) }
        XCTAssertEqual(rgb(cleared, 64, 64), [0, 0, 0], "A missing camera must clear the previous view")
    }
}
