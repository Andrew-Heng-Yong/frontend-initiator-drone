import Foundation

/// A candidate tag boundary: four corners in image pixel coordinates.
public struct Quad: Equatable, Sendable {
    /// Corners in order round the perimeter, normalised by
    /// `QuadDetector.normaliseWinding`: index 0 is the corner nearest the image
    /// origin, and the sequence has a negative shoelace area in y-down pixel
    /// coordinates.
    public var corners: [Vector2]

    public init(corners: [Vector2]) {
        precondition(corners.count == 4, "a quad has four corners")
        self.corners = corners
    }

    /// Shoelace area. Always positive once the winding is normalised.
    public var area: Double {
        var total = 0.0
        for index in 0..<4 {
            let a = corners[index], b = corners[(index + 1) % 4]
            total += a.x * b.y - b.x * a.y
        }
        return abs(total) / 2
    }

    public var centre: Vector2 {
        corners.reduce(Vector2.zero, +) * 0.25
    }

    /// Shortest edge length, used to reject quads too small to carry six
    /// readable cells.
    public var shortestEdge: Double {
        (0..<4).map { corners[$0].distance(to: corners[($0 + 1) % 4]) }.min() ?? 0
    }

    /// Pushes every edge outward by `distance` pixels and re-intersects them.
    ///
    /// This exists to remove a systematic bias, not to be generous. The boundary
    /// tracer returns the centres of the outermost *dark* pixels, but the tag's
    /// real edge is where dark meets light — half a pixel further out on every
    /// side. Uncorrected, the measured tag is one pixel narrower than it is, and
    /// since range scales inversely with apparent size, a 27-pixel tag at 1.8 m
    /// is reported at about 1.93 m. The error is invisible up close and grows
    /// with distance, which is the worst shape an error can have.
    ///
    /// Offsetting the edges and intersecting them is exact under perspective,
    /// unlike pushing each corner away from the centroid, which would move
    /// corners of an oblique quad by the wrong amount.
    public func expanded(by distance: Double) -> Quad {
        guard distance != 0 else { return self }
        let middle = centre

        var offsetOrigins = [Vector2](repeating: .zero, count: 4)
        var directions = [Vector2](repeating: .zero, count: 4)
        for index in 0..<4 {
            let a = corners[index], b = corners[(index + 1) % 4]
            let edge = b - a
            let length = (edge.x * edge.x + edge.y * edge.y).squareRoot()
            guard length > 1e-9 else { return self }
            let unit = Vector2(edge.x / length, edge.y / length)
            var normal = Vector2(unit.y, -unit.x)
            let midpoint = Vector2((a.x + b.x) / 2, (a.y + b.y) / 2)
            if (midpoint - middle).x * normal.x + (midpoint - middle).y * normal.y < 0 {
                normal = Vector2(-normal.x, -normal.y)
            }
            offsetOrigins[index] = a + normal * distance
            directions[index] = unit
        }

        var expandedCorners = [Vector2](repeating: .zero, count: 4)
        for index in 0..<4 {
            let previous = (index + 3) % 4
            guard let point = QuadDetector.intersectLines(
                originA: offsetOrigins[previous], directionA: directions[previous],
                originB: offsetOrigins[index], directionB: directions[index]
            ) else { return self }
            expandedCorners[index] = point
        }
        return Quad(corners: expandedCorners)
    }

    /// Rotates the corner order by `steps` quarter turns, which is how a
    /// decoded rotation is folded back into the geometry.
    public func rotated(by steps: Int) -> Quad {
        let shift = ((steps % 4) + 4) % 4
        guard shift != 0 else { return self }
        return Quad(corners: (0..<4).map { corners[($0 + shift) % 4] })
    }
}

/// A 2D point in image space. Kept separate from `Vector3` so a pixel
/// coordinate can never be handed to something expecting metres.
public struct Vector2: Equatable, Sendable {
    public var x: Double
    public var y: Double

    public init(_ x: Double, _ y: Double) {
        self.x = x
        self.y = y
    }

    public static let zero = Vector2(0, 0)

    public static func + (lhs: Vector2, rhs: Vector2) -> Vector2 { Vector2(lhs.x + rhs.x, lhs.y + rhs.y) }
    public static func - (lhs: Vector2, rhs: Vector2) -> Vector2 { Vector2(lhs.x - rhs.x, lhs.y - rhs.y) }
    public static func * (lhs: Vector2, rhs: Double) -> Vector2 { Vector2(lhs.x * rhs, lhs.y * rhs) }

    public func distance(to other: Vector2) -> Double {
        let dx = x - other.x, dy = y - other.y
        return (dx * dx + dy * dy).squareRoot()
    }

    public func cross(_ other: Vector2) -> Double { x * other.y - y * other.x }
}

/// Finds quadrilateral candidates in a thresholded frame.
///
/// ## Why connected components rather than edge clustering
///
/// The reference AprilTag detector clusters gradient pixels into line segments
/// and assembles quads from them, which handles a tag against a dark background
/// and partial occlusion. This takes the simpler route: label the dark blobs,
/// trace each one's outer boundary, and keep the boundaries that simplify to
/// four corners.
///
/// The cost of that choice, stated plainly: **the tag needs a white quiet zone
/// around it.** With none, the black border merges into a dark background and
/// becomes one blob with it, and no quad is found. Printing tags with a white
/// margin is the standard recommendation anyway, so this trades a failure mode
/// nobody should be hitting for roughly a tenth of the code.
///
/// The other consequence is that only the black border's **outer** boundary is
/// traced. The ring's inner boundary is a quad too, and a naive border-follower
/// would report both — one tag detected twice at two different sizes. Labelling
/// components first makes that impossible: the ring is a single component with
/// a single outer boundary.
public enum QuadDetector {

    public struct Options: Equatable, Sendable {
        /// Smallest blob to consider, in pixels. Below roughly this the six
        /// cells across a `tag16h5` cannot be separated at all.
        public var minimumArea: Int
        /// Largest blob, as a fraction of the frame. A blob covering most of the
        /// frame is a wall or a shadow, not a tag.
        public var maximumAreaFraction: Double
        /// Shortest permitted quad edge, in pixels. Six cells across an edge
        /// means an edge under ~24 px has under 4 px per cell.
        public var minimumEdgeLength: Double
        /// Douglas-Peucker tolerance, as a fraction of the contour perimeter.
        public var simplificationTolerance: Double

        public init(
            minimumArea: Int = 180,
            maximumAreaFraction: Double = 0.35,
            minimumEdgeLength: Double = 20,
            simplificationTolerance: Double = 0.035
        ) {
            self.minimumArea = minimumArea
            self.maximumAreaFraction = maximumAreaFraction
            self.minimumEdgeLength = minimumEdgeLength
            self.simplificationTolerance = simplificationTolerance
        }

        public static let `default` = Options()
    }

    public static func detect(in image: BinaryImage, options: Options = .default) -> [Quad] {
        let components = labelComponents(in: image, minimumArea: options.minimumArea)
        let maximumArea = Int(Double(image.width * image.height) * options.maximumAreaFraction)

        var quads: [Quad] = []
        for component in components where component.area <= maximumArea {
            guard let contour = traceOuterContour(of: component, in: image) else { continue }
            guard contour.count >= 8 else { continue }

            guard let simplified = simplifyToQuad(contour, options: options) else { continue }
            let approximate = Quad(corners: simplified)
            let refined = refineCorners(approximate, contour: contour) ?? approximate

            let quad = normaliseWinding(refined)
            guard isConvex(quad), quad.shortestEdge >= options.minimumEdgeLength else { continue }
            // The traced boundary sits half a pixel inside the real tag edge.
            quads.append(quad.expanded(by: 0.5))
        }
        return quads
    }

    // MARK: - Connected components

    struct Component {
        var label: Int
        var area: Int
        /// Topmost, then leftmost pixel. The boundary trace must start here so
        /// that the pixel above and to the left are both background, which is
        /// what makes the initial backtrack direction well defined.
        var start: (x: Int, y: Int)
        var labels: [Int]
        var width: Int
    }

    /// Eight-connected labelling of dark pixels.
    ///
    /// Eight rather than four on purpose: a tag seen at a steep angle has a
    /// border only a pixel or two thick, and under four-connectivity a
    /// near-diagonal run of such a border breaks into a string of separate
    /// blobs, none of which is a quad.
    static func labelComponents(in image: BinaryImage, minimumArea: Int) -> [Component] {
        let width = image.width, height = image.height
        guard width > 0, height > 0 else { return [] }

        var labels = [Int](repeating: 0, count: width * height)
        var components: [Component] = []
        var nextLabel = 1
        var queue: [Int] = []

        let neighbourOffsets = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]

        for startY in 0..<height {
            for startX in 0..<width {
                let startIndex = startY * width + startX
                guard image.dark[startIndex], labels[startIndex] == 0 else { continue }

                let label = nextLabel
                nextLabel += 1
                labels[startIndex] = label
                queue.removeAll(keepingCapacity: true)
                queue.append(startIndex)
                var area = 0
                var head = 0

                while head < queue.count {
                    let index = queue[head]
                    head += 1
                    area += 1
                    let x = index % width, y = index / width
                    for (dx, dy) in neighbourOffsets {
                        let nx = x + dx, ny = y + dy
                        guard nx >= 0, ny >= 0, nx < width, ny < height else { continue }
                        let neighbourIndex = ny * width + nx
                        guard image.dark[neighbourIndex], labels[neighbourIndex] == 0 else { continue }
                        labels[neighbourIndex] = label
                        queue.append(neighbourIndex)
                    }
                }

                if area >= minimumArea {
                    // The scan reaches a component at its topmost row, and at
                    // the leftmost pixel of that row, which is exactly the
                    // start the tracer needs.
                    components.append(Component(
                        label: label, area: area, start: (startX, startY), labels: [], width: width
                    ))
                }
            }
        }

        // The label map is shared, so it is attached once rather than copied
        // into every component during the scan.
        for index in components.indices {
            components[index].labels = labels
        }
        return components
    }

    // MARK: - Boundary following

    /// Clockwise neighbour offsets starting at West.
    static let traceDirections: [(dx: Int, dy: Int)] = [
        (-1, 0), (-1, -1), (0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1),
    ]

    /// Moore-neighbourhood boundary following.
    ///
    /// Walks the component's outer edge one pixel at a time, always turning as
    /// tightly as possible, which traces the boundary rather than wandering
    /// into the interior. The trace starts having "arrived from the west",
    /// which is guaranteed to be background at the topmost-leftmost pixel.
    static func traceOuterContour(of component: Component, in image: BinaryImage) -> [Vector2]? {
        let width = image.width
        let labels = component.labels
        let label = component.label

        @inline(__always)
        func belongs(_ x: Int, _ y: Int) -> Bool {
            guard x >= 0, y >= 0, x < image.width, y < image.height else { return false }
            return labels[y * width + x] == label
        }

        let start = component.start
        var contour: [Vector2] = [Vector2(Double(start.x), Double(start.y))]
        var current = start
        var backtrack = 0  // arrived from the west
        // A boundary cannot be longer than the component's bounding perimeter;
        // the cap only exists so a pathological shape cannot spin forever.
        let iterationLimit = 4 * (image.width + image.height) + component.area

        for _ in 0..<iterationLimit {
            var moved = false
            for step in 1...8 {
                let direction = (backtrack + step) % 8
                let offset = traceDirections[direction]
                let nx = current.x + offset.dx, ny = current.y + offset.dy
                guard belongs(nx, ny) else { continue }
                // Next search resumes just past the way we came in, so the walk
                // keeps hugging the boundary instead of cutting a corner.
                backtrack = (direction + 4 + 1) % 8
                current = (nx, ny)
                contour.append(Vector2(Double(nx), Double(ny)))
                moved = true
                break
            }
            guard moved else { return nil }
            if current == start, contour.count > 2 { return contour }
        }
        return nil
    }

    /// Simplifies a contour to exactly four corners, widening the tolerance
    /// until it does.
    ///
    /// A single fixed tolerance cannot work across the range of shapes a real
    /// frame produces. Too tight and a perspective-skewed edge keeps a spurious
    /// fifth vertex — measured on a tag at a 30-degree tilt, which is an
    /// entirely ordinary viewing angle. Too loose and a genuine corner is
    /// smoothed away. Sweeping from tight to loose and taking the first
    /// tolerance that yields four keeps the tightest fit that actually works,
    /// rather than committing to one guess.
    static func simplifyToQuad(_ contour: [Vector2], options: Options) -> [Vector2]? {
        let perimeter = contourPerimeter(contour)
        for multiplier in [0.5, 0.75, 1.0, 1.5, 2.0, 3.0] {
            let tolerance = max(1.0, perimeter * options.simplificationTolerance * multiplier)
            let simplified = douglasPeucker(contour, tolerance: tolerance)
            if simplified.count == 4 { return simplified }
            // Once it has collapsed below four corners, looser can only be worse.
            if simplified.count < 4 { return nil }
        }
        return nil
    }

    static func contourPerimeter(_ contour: [Vector2]) -> Double {
        guard contour.count > 1 else { return 0 }
        var total = 0.0
        for index in 0..<(contour.count - 1) {
            total += contour[index].distance(to: contour[index + 1])
        }
        return total
    }

    // MARK: - Polygon simplification

    /// Douglas-Peucker on a closed contour.
    ///
    /// The contour is split at the two mutually most distant points before
    /// simplifying. Running the algorithm on a closed loop as if it were open
    /// pins the arbitrary start pixel as a vertex, and on a square that start
    /// sits mid-edge, so a square reduces to five points and never matches.
    static func douglasPeucker(_ contour: [Vector2], tolerance: Double) -> [Vector2] {
        guard contour.count > 4 else { return contour }
        var points = contour
        if points.first == points.last { points.removeLast() }
        guard points.count > 4 else { return points }

        let first = points[0]
        var farthestIndex = 0
        var farthestDistance = 0.0
        for (index, point) in points.enumerated() {
            let distance = first.distance(to: point)
            if distance > farthestDistance {
                farthestDistance = distance
                farthestIndex = index
            }
        }

        let firstHalf = Array(points[0...farthestIndex])
        let secondHalf = Array(points[farthestIndex...]) + [points[0]]

        var result = simplifyOpen(firstHalf, tolerance: tolerance)
        let tail = simplifyOpen(secondHalf, tolerance: tolerance)
        // Both halves carry the shared endpoints; drop the duplicates.
        result.removeLast()
        result += tail.dropLast()
        return result
    }

    static func simplifyOpen(_ points: [Vector2], tolerance: Double) -> [Vector2] {
        guard points.count > 2 else { return points }
        let first = points[0], last = points[points.count - 1]

        var maximumDistance = 0.0
        var index = 0
        for candidate in 1..<(points.count - 1) {
            let distance = perpendicularDistance(points[candidate], from: first, to: last)
            if distance > maximumDistance {
                maximumDistance = distance
                index = candidate
            }
        }

        guard maximumDistance > tolerance else { return [first, last] }
        let left = simplifyOpen(Array(points[0...index]), tolerance: tolerance)
        let right = simplifyOpen(Array(points[index...]), tolerance: tolerance)
        return left.dropLast() + right
    }

    static func perpendicularDistance(_ point: Vector2, from a: Vector2, to b: Vector2) -> Double {
        let edge = b - a
        let lengthSquared = edge.x * edge.x + edge.y * edge.y
        guard lengthSquared > 1e-12 else { return point.distance(to: a) }
        let t = ((point.x - a.x) * edge.x + (point.y - a.y) * edge.y) / lengthSquared
        let clamped = min(max(t, 0), 1)
        let projection = Vector2(a.x + edge.x * clamped, a.y + edge.y * clamped)
        return point.distance(to: projection)
    }

    // MARK: - Sub-pixel refinement

    /// Replaces pixel-quantised corners with the intersections of lines fitted
    /// to the traced edges.
    ///
    /// Douglas-Peucker can only ever return points that are on the contour, so
    /// its corners are quantised to whole pixels and biased by however the
    /// boundary happened to step around the corner. On a tag filling 50 pixels
    /// that is a two-to-three pixel error, and a border cell centre sits only
    /// four pixels inside the edge — so the decoder ends up sampling a payload
    /// cell where it expects black border, and the tag is rejected. Measured on
    /// a tag at a 20 to 30 degree tilt, which is an entirely ordinary way to
    /// hold a phone.
    ///
    /// Each edge has dozens of contour points along it, so fitting a line
    /// averages the quantisation away and the intersections land well inside a
    /// pixel. Pose accuracy improves for the same reason.
    static func refineCorners(_ quad: Quad, contour: [Vector2]) -> Quad? {
        var edgePoints = [[Vector2]](repeating: [], count: 4)
        var edgeLengths = [Double](repeating: 0, count: 4)
        for index in 0..<4 {
            edgeLengths[index] = quad.corners[index].distance(to: quad.corners[(index + 1) % 4])
        }

        for point in contour {
            var nearestEdge = -1
            var nearestDistance = Double.greatestFiniteMagnitude
            for index in 0..<4 {
                let distance = perpendicularDistance(
                    point, from: quad.corners[index], to: quad.corners[(index + 1) % 4]
                )
                if distance < nearestDistance {
                    nearestDistance = distance
                    nearestEdge = index
                }
            }
            guard nearestEdge >= 0 else { continue }

            // Points near a corner belong to both edges and are rounded by the
            // tracer, so they would drag both fits inward. Dropping the outer
            // fifth of each edge costs nothing: the middle carries the same
            // line, measured more cleanly.
            let cornerDistance = min(
                point.distance(to: quad.corners[nearestEdge]),
                point.distance(to: quad.corners[(nearestEdge + 1) % 4])
            )
            let length = edgeLengths[nearestEdge]
            guard length > 1e-9, cornerDistance > length * 0.2 else { continue }
            guard nearestDistance < max(2.0, length * 0.1) else { continue }
            edgePoints[nearestEdge].append(point)
        }

        var origins = [Vector2](repeating: .zero, count: 4)
        var directions = [Vector2](repeating: .zero, count: 4)
        for index in 0..<4 {
            guard edgePoints[index].count >= 4,
                  let line = fitLine(edgePoints[index]) else { return nil }
            origins[index] = line.origin
            directions[index] = line.direction
        }

        var corners = [Vector2](repeating: .zero, count: 4)
        for index in 0..<4 {
            let previous = (index + 3) % 4
            guard let point = intersectLines(
                originA: origins[previous], directionA: directions[previous],
                originB: origins[index], directionB: directions[index]
            ) else { return nil }
            guard point.x.isFinite, point.y.isFinite else { return nil }
            corners[index] = point
        }
        return Quad(corners: corners)
    }

    /// Total-least-squares line fit: the centroid, and the principal axis of the
    /// scatter matrix.
    ///
    /// Total least squares rather than the usual `y = mx + c`: a tag edge can be
    /// exactly vertical, where the ordinary fit divides by zero. Minimising
    /// perpendicular distance has no privileged axis and no such case.
    static func fitLine(_ points: [Vector2]) -> (origin: Vector2, direction: Vector2)? {
        guard points.count >= 2 else { return nil }
        let count = Double(points.count)
        var meanX = 0.0, meanY = 0.0
        for point in points { meanX += point.x; meanY += point.y }
        meanX /= count
        meanY /= count

        var sxx = 0.0, syy = 0.0, sxy = 0.0
        for point in points {
            let dx = point.x - meanX, dy = point.y - meanY
            sxx += dx * dx
            syy += dy * dy
            sxy += dx * dy
        }
        guard sxx + syy > 1e-12 else { return nil }

        let angle = 0.5 * atan2(2 * sxy, sxx - syy)
        return (Vector2(meanX, meanY), Vector2(cos(angle), sin(angle)))
    }

    static func intersectLines(
        originA: Vector2, directionA: Vector2,
        originB: Vector2, directionB: Vector2
    ) -> Vector2? {
        let denominator = directionA.cross(directionB)
        // Parallel edges mean a degenerate quad; callers keep what they had.
        guard abs(denominator) > 1e-9 else { return nil }
        let delta = originB - originA
        let t = delta.cross(directionB) / denominator
        return originA + directionA * t
    }

    // MARK: - Validation

    /// Rejects self-intersecting and near-degenerate quads by requiring every
    /// turn to go the same way. A bow-tie has the same four corners as a valid
    /// quad and would otherwise decode as noise.
    static func isConvex(_ quad: Quad) -> Bool {
        var sign = 0
        for index in 0..<4 {
            let a = quad.corners[index]
            let b = quad.corners[(index + 1) % 4]
            let c = quad.corners[(index + 2) % 4]
            let cross = (b - a).cross(c - b)
            guard abs(cross) > 1e-9 else { return false }
            let currentSign = cross > 0 ? 1 : -1
            if sign == 0 { sign = currentSign } else if sign != currentSign { return false }
        }
        return true
    }

    /// Puts the corners in the one order the rest of the pipeline assumes.
    ///
    /// The convention, stated exactly because two places have to agree on it:
    /// corner 0 is the corner nearest the image origin, and the sequence has a
    /// **negative shoelace area in y-down pixel coordinates**. That is the
    /// order `Homography.mapping(unitSquareTo:)` pairs with its source list
    /// `(-1,-1), (-1,1), (1,1), (1,-1)` — down the tag's left edge first, then
    /// along the bottom.
    ///
    /// Getting the sign backwards does not fail loudly. The homography still
    /// solves, the quad still maps, and the tag is simply sampled mirrored, so
    /// every payload decodes to nothing at all. `QuadDetectorTests` pins the
    /// sign for that reason.
    ///
    /// Without the canonical *start*, the sampled payload would also depend on
    /// which pixel the boundary tracer happened to begin at, and the rotation
    /// the decoder reports would mean nothing.
    static func normaliseWinding(_ quad: Quad) -> Quad {
        var corners = quad.corners
        var signedArea = 0.0
        for index in 0..<4 {
            let a = corners[index], b = corners[(index + 1) % 4]
            signedArea += a.x * b.y - b.x * a.y
        }
        if signedArea > 0 { corners.reverse() }

        var startIndex = 0
        var best = Double.greatestFiniteMagnitude
        for (index, corner) in corners.enumerated() {
            let score = corner.x + corner.y
            if score < best {
                best = score
                startIndex = index
            }
        }
        return Quad(corners: (0..<4).map { corners[($0 + startIndex) % 4] })
    }
}
