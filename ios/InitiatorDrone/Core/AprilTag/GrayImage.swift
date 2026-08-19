import Foundation

/// An 8-bit single-channel image, the currency of the tag detector.
///
/// Deliberately a plain `[UInt8]` rather than anything from Core Video or
/// Accelerate: the whole detection pipeline then compiles and runs on the
/// headless test runner, where tags can be synthesised, warped and decoded
/// without a device or a camera. The one place that touches `CVPixelBuffer` is
/// the thin bridge in the AR layer.
public struct GrayImage: Equatable, Sendable {
    public let width: Int
    public let height: Int
    public var pixels: [UInt8]

    public init(width: Int, height: Int, pixels: [UInt8]) {
        precondition(pixels.count == width * height, "pixel count must match the dimensions")
        self.width = width
        self.height = height
        self.pixels = pixels
    }

    public init(width: Int, height: Int, fill: UInt8 = 0) {
        self.init(width: width, height: height, pixels: [UInt8](repeating: fill, count: width * height))
    }

    public var isEmpty: Bool { width <= 0 || height <= 0 }

    @inline(__always)
    public subscript(x: Int, y: Int) -> UInt8 {
        get { pixels[y * width + x] }
        set { pixels[y * width + x] = newValue }
    }

    @inline(__always)
    public func contains(x: Int, y: Int) -> Bool {
        x >= 0 && y >= 0 && x < width && y < height
    }

    /// Bilinear sample in pixel coordinates, clamped at the edges.
    ///
    /// The decoder reads cell centres that almost never land on a pixel centre,
    /// and rounding to the nearest pixel is the difference between reading a
    /// tag at 40 px and failing on it: at that size one cell is under 7 px, so a
    /// half-pixel error is a tenth of a cell.
    public func sampleBilinear(x: Double, y: Double) -> Double {
        guard !isEmpty else { return 0 }
        let clampedX = min(max(x, 0), Double(width - 1))
        let clampedY = min(max(y, 0), Double(height - 1))
        let x0 = Int(clampedX), y0 = Int(clampedY)
        let x1 = min(x0 + 1, width - 1), y1 = min(y0 + 1, height - 1)
        let fx = clampedX - Double(x0), fy = clampedY - Double(y0)

        let top = Double(self[x0, y0]) * (1 - fx) + Double(self[x1, y0]) * fx
        let bottom = Double(self[x0, y1]) * (1 - fx) + Double(self[x1, y1]) * fx
        return top * (1 - fy) + bottom * fy
    }

    /// Box-averaged downscale by an integer factor.
    ///
    /// Detection cost is quadratic in the frame size and a 1920x1440 ARKit frame
    /// is far more resolution than a tag decode needs. Averaging rather than
    /// dropping pixels matters: point sampling aliases the tag's own cell grid,
    /// which is exactly the spatial frequency the decoder cares about.
    public func downscaled(by factor: Int) -> GrayImage {
        guard factor > 1 else { return self }
        let newWidth = width / factor
        let newHeight = height / factor
        guard newWidth > 0, newHeight > 0 else { return self }

        var output = [UInt8](repeating: 0, count: newWidth * newHeight)
        let area = factor * factor
        for y in 0..<newHeight {
            for x in 0..<newWidth {
                var total = 0
                for dy in 0..<factor {
                    let row = (y * factor + dy) * width + x * factor
                    for dx in 0..<factor {
                        total += Int(pixels[row + dx])
                    }
                }
                output[y * newWidth + x] = UInt8(total / area)
            }
        }
        return GrayImage(width: newWidth, height: newHeight, pixels: output)
    }
}

/// A binary image: `true` means "dark", because the tag's ink is what the
/// detector chases.
public struct BinaryImage: Equatable, Sendable {
    public let width: Int
    public let height: Int
    public var dark: [Bool]

    public init(width: Int, height: Int, dark: [Bool]) {
        precondition(dark.count == width * height)
        self.width = width
        self.height = height
        self.dark = dark
    }

    @inline(__always)
    public subscript(x: Int, y: Int) -> Bool {
        get {
            guard x >= 0, y >= 0, x < width, y < height else { return false }
            return dark[y * width + x]
        }
        set { dark[y * width + x] = newValue }
    }
}

/// Local-mean thresholding.
///
/// A global threshold is no use here. The robot is photographed indoors under
/// mixed light and outdoors in sun, and a tag half in shadow has no single
/// grey level that separates its ink from its paper. Comparing each pixel to
/// the mean of its neighbourhood makes the decision local, so the shadowed half
/// and the lit half each threshold correctly.
public enum AdaptiveThreshold {

    /// - Parameters:
    ///   - radius: half-width of the averaging window in pixels. Should comfort-
    ///     ably exceed one tag cell, or the window sits entirely inside a cell
    ///     and the mean tracks the cell instead of the local illumination.
    ///   - offset: how far below the local mean a pixel must fall to count as
    ///     dark. Suppresses speckle in flat regions, where the mean is the pixel.
    public static func apply(to image: GrayImage, radius: Int = 6, offset: Int = 6) -> BinaryImage {
        guard !image.isEmpty else { return BinaryImage(width: 0, height: 0, dark: []) }
        let integral = integralImage(of: image)
        let width = image.width, height = image.height
        var dark = [Bool](repeating: false, count: width * height)

        for y in 0..<height {
            let y0 = max(0, y - radius), y1 = min(height - 1, y + radius)
            for x in 0..<width {
                let x0 = max(0, x - radius), x1 = min(width - 1, x + radius)
                let count = (x1 - x0 + 1) * (y1 - y0 + 1)
                let sum = boxSum(integral, width: width, x0: x0, y0: y0, x1: x1, y1: y1)
                let mean = sum / count
                dark[y * width + x] = Int(image.pixels[y * width + x]) < mean - offset
            }
        }
        return BinaryImage(width: width, height: height, dark: dark)
    }

    /// Summed-area table with a zero row and column, so a box sum is four
    /// lookups regardless of the radius. Without it the cost would be
    /// `O(width * height * radius²)`, which at radius 6 is 169 adds per pixel.
    static func integralImage(of image: GrayImage) -> [Int] {
        let width = image.width, height = image.height
        var integral = [Int](repeating: 0, count: (width + 1) * (height + 1))
        for y in 0..<height {
            var rowSum = 0
            for x in 0..<width {
                rowSum += Int(image.pixels[y * width + x])
                integral[(y + 1) * (width + 1) + (x + 1)] = integral[y * (width + 1) + (x + 1)] + rowSum
            }
        }
        return integral
    }

    @inline(__always)
    static func boxSum(_ integral: [Int], width: Int, x0: Int, y0: Int, x1: Int, y1: Int) -> Int {
        let stride = width + 1
        return integral[(y1 + 1) * stride + (x1 + 1)]
            - integral[y0 * stride + (x1 + 1)]
            - integral[(y1 + 1) * stride + x0]
            + integral[y0 * stride + x0]
    }
}
