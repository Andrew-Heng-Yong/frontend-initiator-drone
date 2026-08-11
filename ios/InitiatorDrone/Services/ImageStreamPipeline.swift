import Foundation
import CoreGraphics

/// A frame that has been decoded, colour-mapped and turned into something the
/// UI can draw.
public struct RenderedFrame: Equatable {
    public var topic: RobotTopic
    /// ROS header stamp of the source message.
    public var stamp: Double
    /// When the phone finished rendering it.
    public var renderedAt: Date
    public var image: CGImage
    public var width: Int
    public var height: Int
    public var encoding: String
    /// Colour-map bounds actually used, for the legend.
    public var rangeLow: Double
    public var rangeHigh: Double
    public var unit: ScalarUnit
    /// Value under the centre pixel, in `unit`, when the frame is scalar.
    public var centreValue: Double?

    public static func == (lhs: RenderedFrame, rhs: RenderedFrame) -> Bool {
        lhs.topic == rhs.topic && lhs.stamp == rhs.stamp && lhs.renderedAt == rhs.renderedAt
    }
}

/// Decodes and colour-maps one image topic, never falling behind.
///
/// The pipeline is deliberately one-deep. Frames arrive on the rosbridge
/// client's queue and are handed to a `LatestOnlySlot`; a single worker drains
/// it. If decoding a frame takes longer than the gap to the next one, the
/// intermediate frames are dropped rather than queued. That is the correct
/// trade for a live view — an old depth frame has no value — and it is what
/// keeps memory flat over a long session.
public final class ImageStreamPipeline: @unchecked Sendable {
    public let topic: RobotTopic

    /// Called on the main queue with each rendered frame.
    public var onFrame: ((RenderedFrame) -> Void)?
    /// Called on the main queue when a frame could not be decoded.
    public var onError: ((String) -> Void)?

    /// Colour-map settings. Safe to set from the main thread at any time.
    public var colorMapSettings: ScalarColorMapSettings {
        get { lock.synchronized { renderer.settings } }
        set { lock.synchronized { renderer.settings = newValue } }
    }

    /// How to read the numbers in an incoming single-channel frame.
    public var interpretationProvider: (@Sendable (ROSImageEncoding) -> ScalarInterpretation)

    private let slot = LatestOnlySlot<ROSImageMessage>()
    private let workQueue: DispatchQueue
    private let lock = NSLock()
    private let renderer: ScalarImageRenderer
    private var lastErrorText: String?

    public init(
        topic: RobotTopic,
        colorMapSettings: ScalarColorMapSettings,
        interpretationProvider: @escaping @Sendable (ROSImageEncoding) -> ScalarInterpretation
    ) {
        self.topic = topic
        self.renderer = ScalarImageRenderer(settings: colorMapSettings)
        self.interpretationProvider = interpretationProvider
        self.workQueue = DispatchQueue(
            label: "com.initiatordrone.image.\(topic.rawValue)",
            qos: .userInitiated
        )
    }

    /// Statistics for the diagnostics screen.
    public var statistics: (accepted: Int, dropped: Int) { slot.statistics }

    public func reset() {
        slot.reset()
        slot.resetStatistics()
        lock.synchronized { renderer.resetRange() }
    }

    /// Hands a freshly parsed message to the pipeline.
    public func submit(_ message: ROSImageMessage) {
        guard slot.offer(message) else { return }
        workQueue.async { [weak self] in
            self?.drain()
        }
    }

    private func drain() {
        while let message = slot.take() {
            process(message)
        }
    }

    private func process(_ message: ROSImageMessage) {
        guard let encoding = ROSImageEncoding(rosEncoding: message.encoding) else {
            report(error: "Unsupported encoding '\(message.encoding)' on \(topic.topicName)")
            return
        }

        let interpretation = interpretationProvider(encoding)
        let decoded: DecodedROSImage
        do {
            decoded = try ROSImageDecoder.decode(message, interpretation: interpretation)
        } catch {
            report(error: "\(topic.displayName): \(error)")
            return
        }

        var colorImage: ColorImage
        var rangeLow: Double = 0
        var rangeHigh: Double = 0
        var unit: ScalarUnit = .raw
        var centreValue: Double?

        switch decoded {
        case .scalar(let scalar):
            // The renderer carries smoothed auto-range state, so it is shared
            // and must be entered under the lock along with the range readback.
            lock.lock()
            colorImage = renderer.render(scalar)
            rangeLow = renderer.effectiveRange.low
            rangeHigh = renderer.effectiveRange.high
            lock.unlock()

            unit = scalar.unit
            let centre = scalar.value(x: scalar.width / 2, y: scalar.height / 2)
            centreValue = centre.isFinite ? Double(centre) : nil

        case .color(let color):
            colorImage = color
            rangeLow = 0
            rangeHigh = 255
            unit = .raw
        }

        guard let cgImage = ImageStreamPipeline.makeCGImage(from: colorImage) else {
            report(error: "\(topic.displayName): could not build a bitmap")
            return
        }

        let frame = RenderedFrame(
            topic: topic,
            stamp: message.stamp,
            renderedAt: Date(),
            image: cgImage,
            width: colorImage.width,
            height: colorImage.height,
            encoding: message.encoding,
            rangeLow: rangeLow,
            rangeHigh: rangeHigh,
            unit: unit,
            centreValue: centreValue
        )

        lastErrorText = nil
        DispatchQueue.main.async { [weak self] in
            self?.onFrame?(frame)
        }
    }

    private func report(error text: String) {
        // A malformed stream would otherwise produce one log line per frame.
        guard text != lastErrorText else { return }
        lastErrorText = text
        DispatchQueue.main.async { [weak self] in
            self?.onError?(text)
        }
    }

    /// Wraps an RGBA buffer in a `CGImage`.
    ///
    /// `CGDataProvider(data:)` copies, which is what we want here: the source
    /// buffer is the renderer's reusable scratch and will be overwritten by the
    /// next frame while this image is still on screen.
    static func makeCGImage(from image: ColorImage) -> CGImage? {
        guard image.width > 0, image.height > 0,
              image.rgba.count == image.width * image.height * 4 else { return nil }

        let data = Data(image.rgba)
        guard let provider = CGDataProvider(data: data as CFData) else { return nil }

        return CGImage(
            width: image.width,
            height: image.height,
            bitsPerComponent: 8,
            bitsPerPixel: 32,
            bytesPerRow: image.width * 4,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.last.rawValue),
            provider: provider,
            decode: nil,
            shouldInterpolate: false,
            intent: .defaultIntent
        )
    }
}

extension NSLock {
    /// Named `synchronized` rather than `withLock` so it never collides with
    /// the `NSLocking.withLock` that newer SDKs provide.
    @inline(__always)
    func synchronized<T>(_ body: () throws -> T) rethrows -> T {
        lock()
        defer { unlock() }
        return try body()
    }
}
