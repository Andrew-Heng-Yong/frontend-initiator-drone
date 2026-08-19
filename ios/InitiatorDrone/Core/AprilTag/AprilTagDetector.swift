import Foundation

/// One decoded tag.
public struct AprilTagDetection: Equatable, Sendable {
    public let id: Int
    /// Corners in the detection image, already rotated so corner 0 is the tag's
    /// own top-left.
    public let corners: [Vector2]
    /// Bits corrected during decoding. Zero unless correction was enabled.
    public let hammingDistance: Int
    /// Gap between the darkest payload cell read as white and the brightest read
    /// as black, in grey levels. Small means the decode was a coin toss.
    public let decisionMargin: Double
    /// Tag pose in the camera's **optical** frame: `+X` right, `+Y` down,
    /// `+Z` forward.
    public let poseInCameraOptical: Pose
    /// Mean corner reprojection error, in detection-image pixels.
    public let reprojectionError: Double

    public init(
        id: Int,
        corners: [Vector2],
        hammingDistance: Int,
        decisionMargin: Double,
        poseInCameraOptical: Pose,
        reprojectionError: Double
    ) {
        self.id = id
        self.corners = corners
        self.hammingDistance = hammingDistance
        self.decisionMargin = decisionMargin
        self.poseInCameraOptical = poseInCameraOptical
        self.reprojectionError = reprojectionError
    }

    /// The same pose written in ARKit camera-node axes (`+X` right, `+Y` up,
    /// `-Z` forward), ready to compose with `ARCamera.transform`.
    ///
    /// Optical to node axes is a half turn about X — the same flip the depth
    /// point cloud performs, for the same reason.
    public var poseInCameraNode: Pose {
        let flip = Quaternion(axis: Vector3(1, 0, 0), angle: .pi)
        return Pose(
            position: Vector3(
                poseInCameraOptical.position.x,
                -poseInCameraOptical.position.y,
                -poseInCameraOptical.position.z
            ),
            orientation: (flip * poseInCameraOptical.orientation).normalized
        )
    }

    /// Straight-line distance from the camera, in metres.
    public var range: Double { poseInCameraOptical.position.length }
}

/// Finds and decodes `tag16h5` markers in a greyscale frame.
///
/// The pipeline, and what each stage exists to survive:
///
/// 1. **Downscale** — detection cost is quadratic in frame size, and a 1920x1440
///    ARKit frame is far more than a decode needs.
/// 2. **Adaptive threshold** — a tag half in shadow has no single grey level
///    separating ink from paper.
/// 3. **Connected components and boundary tracing** — finds the black border
///    ring as one blob with one outer boundary.
/// 4. **Polygon simplification** — keeps the boundaries that are really quads.
/// 5. **Border check** — every one of the twenty border cells must read black.
///    This runs *before* decoding and throws away almost every non-tag quad in
///    a scene for the cost of twenty samples.
/// 6. **Decode** — sample the 4x4 payload, match against the family.
/// 7. **Pose** — homography to rigid transform, then reprojection as a check.
///
/// Stages 5 and 7 are both there because of what `tag16h5` is: with only 30
/// codes, roughly one random 16-bit pattern in 550 decodes to a valid ID, and
/// this app moves a robot marker on the strength of a detection.
public enum AprilTagDetector {

    public struct Options: Equatable, Sendable {
        /// Integer downscale applied before detection.
        public var downscale: Int
        public var quad: QuadDetector.Options
        public var thresholdRadius: Int
        public var thresholdOffset: Int
        /// Bits the decoder may correct. Zero by default; see
        /// `AprilTagFamily.decode`.
        public var maximumHammingCorrection: Int
        /// Minimum grey-level separation between the black and white models
        /// before a decode is trusted.
        public var minimumContrast: Double
        /// Minimum decision margin, in grey levels.
        public var minimumDecisionMargin: Double
        /// Largest tolerated mean corner reprojection error, in pixels.
        public var maximumReprojectionError: Double

        public init(
            downscale: Int = 2,
            quad: QuadDetector.Options = .default,
            thresholdRadius: Int = 6,
            thresholdOffset: Int = 6,
            maximumHammingCorrection: Int = 0,
            minimumContrast: Double = 25,
            minimumDecisionMargin: Double = 12,
            maximumReprojectionError: Double = 3.0
        ) {
            self.downscale = max(1, downscale)
            self.quad = quad
            self.thresholdRadius = thresholdRadius
            self.thresholdOffset = thresholdOffset
            self.maximumHammingCorrection = maximumHammingCorrection
            self.minimumContrast = minimumContrast
            self.minimumDecisionMargin = minimumDecisionMargin
            self.maximumReprojectionError = maximumReprojectionError
        }

        public static let `default` = Options()
    }

    /// - Parameters:
    ///   - image: full-resolution greyscale frame.
    ///   - intrinsics: for that full-resolution frame. Scaled internally to
    ///     match the downscale, so callers pass what the camera reports.
    ///   - tagSizes: outer black-border edge length in metres, by tag ID. A tag
    ///     whose ID is absent is detected but not posed, because its size is
    ///     the only thing that sets the scale and there is nothing to guess it
    ///     from.
    public static func detect(
        in image: GrayImage,
        intrinsics: CameraIntrinsics,
        tagSizes: [Int: Double],
        options: Options = .default
    ) -> [AprilTagDetection] {
        guard !image.isEmpty, intrinsics.isUsable else { return [] }

        let working = image.downscaled(by: options.downscale)
        let scaledIntrinsics = intrinsics.scaled(by: 1.0 / Double(options.downscale))
        let binary = AdaptiveThreshold.apply(
            to: working,
            radius: options.thresholdRadius,
            offset: options.thresholdOffset
        )
        let quads = QuadDetector.detect(in: binary, options: options.quad)

        var detections: [AprilTagDetection] = []
        for quad in quads {
            guard let homography = Homography.mapping(unitSquareTo: quad.corners) else { continue }
            guard let reading = readPayload(from: working, homography: homography, options: options)
            else { continue }
            guard let match = AprilTagFamily.decode(
                payload: reading.payload,
                maximumHammingCorrection: options.maximumHammingCorrection
            ) else { continue }
            guard let tagSize = tagSizes[match.id], tagSize > 0 else { continue }

            // The decode reports how many quarter turns the canonical code was
            // rotated by to match what was sampled, so the corners must be
            // turned back by the same amount — note the sign — to make corner 0
            // the tag's own top-left, which is what the pose is defined
            // against. Getting this backwards is silent for half-turn
            // symmetries and 180 degrees wrong for the other two, which is why
            // `testDecodesTagsAtEveryQuarterTurn` checks all four.
            let orientedQuad = quad.rotated(by: -match.rotation)
            guard let orientedHomography = Homography.mapping(unitSquareTo: orientedQuad.corners),
                  let pose = AprilTagPoseEstimator.pose(
                      from: orientedHomography,
                      intrinsics: scaledIntrinsics,
                      tagSize: tagSize
                  )
            else { continue }

            let error = AprilTagPoseEstimator.reprojectionError(
                pose: pose,
                intrinsics: scaledIntrinsics,
                tagSize: tagSize,
                corners: orientedQuad.corners
            )
            guard error <= options.maximumReprojectionError else { continue }

            detections.append(AprilTagDetection(
                id: match.id,
                corners: orientedQuad.corners,
                hammingDistance: match.hammingDistance,
                decisionMargin: reading.decisionMargin,
                poseInCameraOptical: pose,
                reprojectionError: error
            ))
        }
        // Nearest first: if two tags are visible, the closer one carries the
        // better pose, because corner-position noise translates into range
        // error in proportion to distance.
        return detections.sorted { $0.range < $1.range }
    }

    struct PayloadReading {
        var payload: UInt16
        var decisionMargin: Double
    }

    /// Samples the 6x6 grid and returns the 4x4 payload.
    ///
    /// Black and white levels are measured from the tag itself rather than
    /// assumed: the border ring is known black, and a ring half a cell outside
    /// the tag is known white (the quiet zone). Thresholding halfway between
    /// the two makes the decode independent of exposure, which changes by
    /// several stops as the phone pans across a room.
    static func readPayload(
        from image: GrayImage,
        homography: Homography,
        options: Options
    ) -> PayloadReading? {
        let grid = Double(AprilTagFamily.gridSize)

        /// Cell centre in tag-normalised coordinates. Cell (0,0) is top-left of
        /// the 6x6 grid; fractional indices reach outside it.
        func cellPoint(row: Double, column: Double) -> Vector2 {
            Vector2(-1 + 2 * (column + 0.5) / grid, -1 + 2 * (row + 0.5) / grid)
        }

        func sample(row: Double, column: Double) -> Double {
            let point = homography.apply(cellPoint(row: row, column: column))
            guard point.x.isFinite, point.y.isFinite else { return .nan }
            return image.sampleBilinear(x: point.x, y: point.y)
        }

        var blackSamples: [Double] = []
        var whiteSamples: [Double] = []
        let last = AprilTagFamily.gridSize - 1
        for index in 0..<AprilTagFamily.gridSize {
            let position = Double(index)
            for value in [
                sample(row: 0, column: position),
                sample(row: Double(last), column: position),
                sample(row: position, column: 0),
                sample(row: position, column: Double(last)),
            ] {
                guard value.isFinite else { return nil }
                blackSamples.append(value)
            }
            for value in [
                sample(row: -1, column: position),
                sample(row: grid, column: position),
                sample(row: position, column: -1),
                sample(row: position, column: grid),
            ] where value.isFinite {
                whiteSamples.append(value)
            }
        }

        guard !blackSamples.isEmpty, whiteSamples.count >= 8 else { return nil }
        let blackLevel = blackSamples.reduce(0, +) / Double(blackSamples.count)
        let whiteLevel = whiteSamples.reduce(0, +) / Double(whiteSamples.count)
        guard whiteLevel - blackLevel >= options.minimumContrast else { return nil }
        let threshold = (blackLevel + whiteLevel) / 2

        // The border must actually be black. Cheap, and it is what stops a
        // doorway or a monitor bezel from ever reaching the decoder.
        let borderMargin = threshold - (blackSamples.max() ?? .infinity)
        guard borderMargin > 0 else { return nil }

        var payload: UInt16 = 0
        var darkestWhite = Double.greatestFiniteMagnitude
        var brightestBlack = -Double.greatestFiniteMagnitude
        for row in 0..<AprilTagFamily.payloadSize {
            for column in 0..<AprilTagFamily.payloadSize {
                let value = sample(row: Double(row + 1), column: Double(column + 1))
                guard value.isFinite else { return nil }
                let index = row * AprilTagFamily.payloadSize + column
                if value > threshold {
                    payload |= 1 << UInt16(AprilTagFamily.bitCount - 1 - index)
                    darkestWhite = min(darkestWhite, value)
                } else {
                    brightestBlack = max(brightestBlack, value)
                }
            }
        }

        // An all-black or all-white payload has no margin to measure; both are
        // decoded as some tag by the family, so they must be rejected here.
        guard darkestWhite < .greatestFiniteMagnitude,
              brightestBlack > -.greatestFiniteMagnitude else { return nil }
        let margin = min(darkestWhite - brightestBlack, borderMargin)
        guard margin >= options.minimumDecisionMargin else { return nil }

        return PayloadReading(payload: payload, decisionMargin: margin)
    }
}
