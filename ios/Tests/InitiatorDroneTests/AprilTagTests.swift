#if canImport(XCTest)
import XCTest
// In Xcode the tests compile as their own module; the headless runner in
// Scripts/ compiles them alongside the sources instead, so neither import
// exists there.
@testable import InitiatorDrone
#endif
import Foundation

/// The tag detector, end to end on synthesised frames.
///
/// Synthesising is what makes this testable at all: a tag is rendered at a
/// known pose with known intrinsics, put through the real pipeline, and the
/// recovered pose is compared against the one it was drawn from. No device, no
/// camera, no fixture images — and every viewing angle and distance is reachable.
final class AprilTagTests: XCTestCase {

    // MARK: - Rendering helpers

    /// Projects a point on the tag plane, in tag-normalised coordinates where
    /// the black border's outer edge is at ±1, into the image.
    private func project(
        normalised point: Vector2,
        pose: Pose,
        intrinsics: CameraIntrinsics,
        tagSize: Double
    ) -> Vector2 {
        let halfSize = tagSize / 2
        let onTag = Vector3(point.x * halfSize, point.y * halfSize, 0)
        let inCamera = pose.apply(to: onTag)
        return Vector2(
            intrinsics.fx * inCamera.x / inCamera.z + intrinsics.cx,
            intrinsics.fy * inCamera.y / inCamera.z + intrinsics.cy
        )
    }

    /// Renders a tag onto a white background at a given pose.
    ///
    /// Works backwards from each pixel through the inverse homography, so
    /// perspective is exact rather than approximated by splatting, and a tag
    /// seen at 45° has correctly foreshortened cells.
    private func renderTag(
        id: Int,
        pose: Pose,
        intrinsics: CameraIntrinsics,
        tagSize: Double,
        width: Int,
        height: Int,
        black: UInt8 = 30,
        white: UInt8 = 225,
        background: UInt8 = 210
    ) -> GrayImage {
        let corners = [
            project(normalised: Vector2(-1, -1), pose: pose, intrinsics: intrinsics, tagSize: tagSize),
            project(normalised: Vector2(-1, 1), pose: pose, intrinsics: intrinsics, tagSize: tagSize),
            project(normalised: Vector2(1, 1), pose: pose, intrinsics: intrinsics, tagSize: tagSize),
            project(normalised: Vector2(1, -1), pose: pose, intrinsics: intrinsics, tagSize: tagSize),
        ]
        guard let forward = Homography.mapping(unitSquareTo: corners),
              let inverse = forward.inverse else {
            return GrayImage(width: width, height: height, fill: background)
        }

        let cells = AprilTagFamily.cells(forID: id)
        let grid = Double(AprilTagFamily.gridSize)
        var image = GrayImage(width: width, height: height, fill: background)

        for y in 0..<height {
            for x in 0..<width {
                // Supersample: a cell edge crossing a pixel otherwise aliases,
                // and at small tag sizes that noise is the whole signal.
                var total = 0.0
                var samples = 0
                for (offsetX, offsetY) in [(0.25, 0.25), (0.75, 0.25), (0.25, 0.75), (0.75, 0.75)] {
                    let point = inverse.apply(Vector2(Double(x) + offsetX, Double(y) + offsetY))
                    guard point.x.isFinite, point.y.isFinite else { continue }
                    // Quiet zone: half a cell of white all the way round.
                    let quiet = 1 + 2 * 0.5 / grid
                    guard abs(point.x) <= quiet, abs(point.y) <= quiet else { continue }
                    samples += 1
                    if abs(point.x) > 1 || abs(point.y) > 1 {
                        total += Double(white)
                        continue
                    }
                    let column = min(AprilTagFamily.gridSize - 1, max(0, Int((point.x + 1) / 2 * grid)))
                    let row = min(AprilTagFamily.gridSize - 1, max(0, Int((point.y + 1) / 2 * grid)))
                    total += Double(cells[row][column] ? white : black)
                }
                guard samples > 0 else { continue }
                let covered = Double(samples) / 4
                let tagValue = total / Double(samples)
                image[x, y] = UInt8(tagValue * covered + Double(background) * (1 - covered))
            }
        }
        return image
    }

    private var intrinsics: CameraIntrinsics {
        CameraIntrinsics(fx: 600, fy: 600, cx: 320, cy: 240)
    }

    // MARK: - Family

    /// A checksum on the transcribed code table. `tag16h5` promises a minimum
    /// Hamming distance of 5 between different tags in any rotation; a single
    /// mistyped hex digit would almost certainly break that.
    func testFamilyMinimumHammingDistanceIsFive() {
        var table: [(id: Int, code: UInt16)] = []
        for (id, code) in AprilTagFamily.codes.enumerated() {
            var current = code
            for _ in 0..<4 {
                table.append((id, current))
                current = AprilTagFamily.rotate90(current)
            }
        }

        var minimum = Int.max
        for (index, first) in table.enumerated() {
            for second in table[(index + 1)...] where first.id != second.id {
                minimum = min(minimum, (first.code ^ second.code).nonzeroBitCount)
            }
        }
        XCTAssertEqual(minimum, 5, "the family's distance guarantee is what the name means")
    }

    func testFamilyHasThirtyUniqueCodes() {
        XCTAssertEqual(AprilTagFamily.codes.count, 30)
        XCTAssertEqual(Set(AprilTagFamily.codes).count, 30)
        XCTAssertEqual(AprilTagFamily.validIDs, 0...29)
    }

    /// Four quarter turns is the identity — the property every rotation bug
    /// violates.
    func testRotatingFourTimesReturnsTheOriginalCode() {
        for code in AprilTagFamily.codes {
            var rotated = code
            for _ in 0..<4 { rotated = AprilTagFamily.rotate90(rotated) }
            XCTAssertEqual(rotated, code)
        }
    }

    func testEveryCodeDecodesToItsOwnIDInEveryRotation() {
        for (id, code) in AprilTagFamily.codes.enumerated() {
            var rotated = code
            for rotation in 0..<4 {
                let match = AprilTagFamily.decode(payload: rotated)
                XCTAssertEqual(match?.id, id)
                XCTAssertEqual(match?.rotation, rotation)
                XCTAssertEqual(match?.hammingDistance, 0)
                rotated = AprilTagFamily.rotate90(rotated)
            }
        }
    }

    /// Exact matching by default is the single most important false-positive
    /// guard the detector has, so it is pinned by a test rather than left to a
    /// default argument someone might "helpfully" relax.
    func testDecodingRejectsCorruptedPayloadsUnlessCorrectionIsAsked() {
        let corrupted = AprilTagFamily.codes[7] ^ 0b1
        XCTAssertNil(AprilTagFamily.decode(payload: corrupted))

        let corrected = AprilTagFamily.decode(payload: corrupted, maximumHammingCorrection: 1)
        XCTAssertEqual(corrected?.id, 7)
        XCTAssertEqual(corrected?.hammingDistance, 1)
    }

    func testRenderedCellsHaveABlackBorder() {
        let cells = AprilTagFamily.cells(forID: 3)
        XCTAssertEqual(cells.count, 6)
        for index in 0..<6 {
            XCTAssertFalse(cells[0][index])
            XCTAssertFalse(cells[5][index])
            XCTAssertFalse(cells[index][0])
            XCTAssertFalse(cells[index][5])
        }
    }

    // MARK: - Homography

    func testHomographyMapsTheUnitSquareOntoTheGivenCorners() throws {
        let corners = [Vector2(100, 120), Vector2(90, 300), Vector2(280, 320), Vector2(260, 100)]
        let homography = try XCTUnwrap(Homography.mapping(unitSquareTo: corners))
        let source = [Vector2(-1, -1), Vector2(-1, 1), Vector2(1, 1), Vector2(1, -1)]
        for (index, point) in source.enumerated() {
            let mapped = homography.apply(point)
            XCTAssertEqual(mapped.x, corners[index].x, accuracy: 1e-6)
            XCTAssertEqual(mapped.y, corners[index].y, accuracy: 1e-6)
        }
    }

    /// A tag square-on to the camera makes the perspective terms vanish. Without
    /// partial pivoting the solve divides by ~0 exactly here, at the one view
    /// most likely to be set up on purpose.
    func testHomographySolvesTheDegenerateFrontOnCase() throws {
        let corners = [Vector2(200, 200), Vector2(200, 300), Vector2(300, 300), Vector2(300, 200)]
        let homography = try XCTUnwrap(Homography.mapping(unitSquareTo: corners))
        let centre = homography.apply(Vector2(0, 0))
        XCTAssertEqual(centre.x, 250, accuracy: 1e-6)
        XCTAssertEqual(centre.y, 250, accuracy: 1e-6)
    }

    func testHomographyInverseRoundTrips() throws {
        let corners = [Vector2(100, 120), Vector2(90, 300), Vector2(280, 320), Vector2(260, 100)]
        let homography = try XCTUnwrap(Homography.mapping(unitSquareTo: corners))
        let inverse = try XCTUnwrap(homography.inverse)
        for point in [Vector2(0, 0), Vector2(0.4, -0.7), Vector2(-0.9, 0.2)] {
            let round = inverse.apply(homography.apply(point))
            XCTAssertEqual(round.x, point.x, accuracy: 1e-6)
            XCTAssertEqual(round.y, point.y, accuracy: 1e-6)
        }
    }

    // MARK: - Detection

    func testDetectsAndDecodesAFrontOnTag() throws {
        let pose = Pose(position: Vector3(0, 0, 1.0))
        let image = renderTag(id: 5, pose: pose, intrinsics: intrinsics, tagSize: 0.16, width: 640, height: 480)

        let detections = AprilTagDetector.detect(
            in: image, intrinsics: intrinsics, tagSizes: [5: 0.16]
        )
        XCTAssertEqual(detections.count, 1)
        XCTAssertEqual(detections.first?.id, 5)
    }

    func testRecoversTheRangeItWasRenderedAt() throws {
        for distance in [0.6, 1.0, 1.8] {
            let pose = Pose(position: Vector3(0, 0, distance))
            let image = renderTag(id: 11, pose: pose, intrinsics: intrinsics, tagSize: 0.16, width: 640, height: 480)
            let detection = try XCTUnwrap(
                AprilTagDetector.detect(in: image, intrinsics: intrinsics, tagSizes: [11: 0.16]).first,
                "no detection at \(distance) m"
            )
            XCTAssertEqual(detection.poseInCameraOptical.position.z, distance, accuracy: distance * 0.05)
        }
    }

    func testRecoversTranslationOffTheOpticalAxis() throws {
        let pose = Pose(position: Vector3(0.12, -0.08, 1.1))
        let image = renderTag(id: 2, pose: pose, intrinsics: intrinsics, tagSize: 0.16, width: 640, height: 480)
        let detection = try XCTUnwrap(
            AprilTagDetector.detect(in: image, intrinsics: intrinsics, tagSizes: [2: 0.16]).first
        )
        XCTAssertEqual(detection.poseInCameraOptical.position.x, 0.12, accuracy: 0.02)
        XCTAssertEqual(detection.poseInCameraOptical.position.y, -0.08, accuracy: 0.02)
        XCTAssertEqual(detection.poseInCameraOptical.position.z, 1.1, accuracy: 0.06)
    }

    /// A tilted tag is the case that separates a real pose estimate from a
    /// scale guess: front-on, any rotation error is invisible in the corners.
    func testRecoversRotationFromATiltedTag() throws {
        let tilt = Quaternion(axis: Vector3(0, 1, 0), angle: 0.5)
        let pose = Pose(position: Vector3(0, 0, 1.0), orientation: tilt)
        let image = renderTag(id: 8, pose: pose, intrinsics: intrinsics, tagSize: 0.18, width: 640, height: 480)

        let detection = try XCTUnwrap(
            AprilTagDetector.detect(in: image, intrinsics: intrinsics, tagSizes: [8: 0.18]).first
        )
        XCTAssertEqual(detection.id, 8)
        let recovered = detection.poseInCameraOptical.orientation
        let delta = (tilt.conjugate * recovered).normalized
        let angle = 2 * acos(min(1, abs(delta.w)))
        XCTAssertLessThan(angle, 0.12, "rotation off by \(angle * 180 / .pi)°")
    }

    /// Each of the four physical orientations must decode to the same ID, and
    /// the recovered pose must carry the roll that was rendered — that is the
    /// whole point of folding the decoded rotation back into the corners.
    func testDecodesTagsAtEveryQuarterTurn() throws {
        for turn in 0..<4 {
            let roll = Double(turn) * .pi / 2
            let pose = Pose(
                position: Vector3(0, 0, 1.0),
                orientation: Quaternion(axis: Vector3(0, 0, 1), angle: roll)
            )
            let image = renderTag(id: 14, pose: pose, intrinsics: intrinsics, tagSize: 0.18, width: 640, height: 480)
            let detection = try XCTUnwrap(
                AprilTagDetector.detect(in: image, intrinsics: intrinsics, tagSizes: [14: 0.18]).first,
                "no detection at quarter turn \(turn)"
            )
            XCTAssertEqual(detection.id, 14, "quarter turn \(turn) decoded as the wrong tag")

            let delta = (pose.orientation.conjugate * detection.poseInCameraOptical.orientation).normalized
            let angle = 2 * acos(min(1, abs(delta.w)))
            XCTAssertLessThan(angle, 0.15, "quarter turn \(turn) recovered \(angle * 180 / .pi)° off")
        }
    }

    func testDetectsSeveralTagsInOneFrameNearestFirst() throws {
        var image = renderTag(id: 1, pose: Pose(position: Vector3(-0.25, 0, 1.4)), intrinsics: intrinsics, tagSize: 0.16, width: 640, height: 480)
        let second = renderTag(id: 9, pose: Pose(position: Vector3(0.18, 0, 0.9)), intrinsics: intrinsics, tagSize: 0.16, width: 640, height: 480, background: 255)
        // Composite by taking the darker pixel, which lays the second tag over
        // the first frame's white background without disturbing it.
        for index in image.pixels.indices {
            image.pixels[index] = min(image.pixels[index], second.pixels[index])
        }

        let detections = AprilTagDetector.detect(
            in: image, intrinsics: intrinsics, tagSizes: [1: 0.16, 9: 0.16]
        )
        XCTAssertEqual(detections.map(\.id), [9, 1], "nearest tag must come first")
    }

    // MARK: - Rejection

    func testAnEmptySceneProducesNoDetections() {
        let image = GrayImage(width: 320, height: 240, fill: 200)
        XCTAssertTrue(AprilTagDetector.detect(in: image, intrinsics: intrinsics, tagSizes: [0: 0.16]).isEmpty)
    }

    /// A plain dark rectangle is the commonest thing in an indoor scene that
    /// looks like a tag: a monitor, a doorway, a case. Its interior is uniform,
    /// so it has no decision margin, and it must never reach the family.
    func testAPlainDarkRectangleIsNotDecodedAsATag() {
        var image = GrayImage(width: 320, height: 240, fill: 220)
        for y in 70..<170 {
            for x in 90..<220 {
                image[x, y] = 25
            }
        }
        let detections = AprilTagDetector.detect(
            in: image,
            intrinsics: intrinsics,
            tagSizes: Dictionary(uniqueKeysWithValues: (0...29).map { ($0, 0.16) })
        )
        XCTAssertTrue(detections.isEmpty, "decoded \(detections.map(\.id)) from a blank rectangle")
    }

    /// Without a configured size there is nothing to set the scale, and a pose
    /// would be an invention. Detecting but not reporting is the honest result.
    func testATagWithNoConfiguredSizeIsNotReported() {
        let pose = Pose(position: Vector3(0, 0, 1.0))
        let image = renderTag(id: 5, pose: pose, intrinsics: intrinsics, tagSize: 0.16, width: 640, height: 480)
        XCTAssertTrue(AprilTagDetector.detect(in: image, intrinsics: intrinsics, tagSizes: [:]).isEmpty)
        XCTAssertTrue(AprilTagDetector.detect(in: image, intrinsics: intrinsics, tagSizes: [7: 0.16]).isEmpty)
    }

    /// Size scales range linearly, which is why the settings screen makes such
    /// a point of it: telling the app a tag is twice its real size puts the
    /// robot twice as far away, with everything else looking perfect.
    func testAWrongConfiguredSizeScalesTheRangeProportionally() throws {
        let pose = Pose(position: Vector3(0, 0, 1.0))
        let image = renderTag(id: 5, pose: pose, intrinsics: intrinsics, tagSize: 0.16, width: 640, height: 480)
        let detection = try XCTUnwrap(
            AprilTagDetector.detect(in: image, intrinsics: intrinsics, tagSizes: [5: 0.32]).first
        )
        XCTAssertEqual(detection.poseInCameraOptical.position.z, 2.0, accuracy: 0.12)
    }

    func testUnusableIntrinsicsProduceNothingRatherThanNonsense() {
        let image = renderTag(id: 5, pose: Pose(position: Vector3(0, 0, 1)), intrinsics: intrinsics, tagSize: 0.16, width: 640, height: 480)
        let broken = CameraIntrinsics(fx: 0, fy: 0, cx: 0, cy: 0)
        XCTAssertTrue(AprilTagDetector.detect(in: image, intrinsics: broken, tagSizes: [5: 0.16]).isEmpty)
    }

    // MARK: - Camera frame conversion

    /// The optical frame is `+Y` down and `+Z` forward; an ARKit camera node is
    /// `+Y` up and `-Z` forward. A tag straight ahead must therefore be at
    /// negative Z in node axes, not positive.
    func testOpticalToCameraNodeAxesIsAHalfTurnAboutX() {
        let detection = AprilTagDetection(
            id: 0,
            corners: [.zero, .zero, .zero, .zero],
            hammingDistance: 0,
            decisionMargin: 100,
            poseInCameraOptical: Pose(position: Vector3(0.2, 0.3, 1.5)),
            reprojectionError: 0
        )
        let node = detection.poseInCameraNode
        XCTAssertEqual(node.position.x, 0.2, accuracy: 1e-9)
        XCTAssertEqual(node.position.y, -0.3, accuracy: 1e-9)
        XCTAssertEqual(node.position.z, -1.5, accuracy: 1e-9)
    }

    static var allTests: [(String, (AprilTagTests) -> () throws -> Void)] {
        [
        ("testFamilyMinimumHammingDistanceIsFive", testFamilyMinimumHammingDistanceIsFive),
        ("testFamilyHasThirtyUniqueCodes", testFamilyHasThirtyUniqueCodes),
        ("testRotatingFourTimesReturnsTheOriginalCode", testRotatingFourTimesReturnsTheOriginalCode),
        ("testEveryCodeDecodesToItsOwnIDInEveryRotation", testEveryCodeDecodesToItsOwnIDInEveryRotation),
        ("testDecodingRejectsCorruptedPayloadsUnlessCorrectionIsAsked", testDecodingRejectsCorruptedPayloadsUnlessCorrectionIsAsked),
        ("testRenderedCellsHaveABlackBorder", testRenderedCellsHaveABlackBorder),
        ("testHomographyMapsTheUnitSquareOntoTheGivenCorners", testHomographyMapsTheUnitSquareOntoTheGivenCorners),
        ("testHomographySolvesTheDegenerateFrontOnCase", testHomographySolvesTheDegenerateFrontOnCase),
        ("testHomographyInverseRoundTrips", testHomographyInverseRoundTrips),
        ("testDetectsAndDecodesAFrontOnTag", testDetectsAndDecodesAFrontOnTag),
        ("testRecoversTheRangeItWasRenderedAt", testRecoversTheRangeItWasRenderedAt),
        ("testRecoversTranslationOffTheOpticalAxis", testRecoversTranslationOffTheOpticalAxis),
        ("testRecoversRotationFromATiltedTag", testRecoversRotationFromATiltedTag),
        ("testDecodesTagsAtEveryQuarterTurn", testDecodesTagsAtEveryQuarterTurn),
        ("testDetectsSeveralTagsInOneFrameNearestFirst", testDetectsSeveralTagsInOneFrameNearestFirst),
        ("testAnEmptySceneProducesNoDetections", testAnEmptySceneProducesNoDetections),
        ("testAPlainDarkRectangleIsNotDecodedAsATag", testAPlainDarkRectangleIsNotDecodedAsATag),
        ("testATagWithNoConfiguredSizeIsNotReported", testATagWithNoConfiguredSizeIsNotReported),
        ("testAWrongConfiguredSizeScalesTheRangeProportionally", testAWrongConfiguredSizeScalesTheRangeProportionally),
        ("testUnusableIntrinsicsProduceNothingRatherThanNonsense", testUnusableIntrinsicsProduceNothingRatherThanNonsense),
        ("testOpticalToCameraNodeAxesIsAHalfTurnAboutX", testOpticalToCameraNodeAxesIsAHalfTurnAboutX),
        ]
    }
}
