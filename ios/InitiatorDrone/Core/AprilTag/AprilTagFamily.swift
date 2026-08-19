import Foundation

/// The `tag16h5` family: 30 codes over a 4x4 payload, minimum Hamming
/// distance 5.
///
/// The app supports this family and only this family, because that is what the
/// robot carries. Hard-coding it keeps the settings screen from offering a
/// choice nobody should be making in the field, and lets the decoder specialise
/// on `d = 4`.
///
/// ## Layout
///
/// A `tag16h5` marker is a 6x6 grid of cells: a one-cell black border all the
/// way round, with the 4x4 payload inside it. (A white quiet zone outside the
/// black border is required for detection but is not part of the tag.)
///
/// Payload bits are numbered row-major from the top-left, most significant
/// first: bit 15 is row 0 column 0, bit 0 is row 3 column 3. A set bit is a
/// **white** cell, which is the convention the published code tables use.
///
/// ## Why 16h5 is the risky family
///
/// 30 codes in a 65,536-value space is sparse coverage, and every code has four
/// rotations, so 120 of 65,536 random 16-bit patterns decode to *something*.
/// That is roughly one in 550 — and a cluttered scene offers a detector many
/// quadrilaterals per frame. `tag16h5` is well known for false positives, and
/// the mitigations live outside this type: exact-match decoding by default
/// (`maximumHammingCorrection = 0`), a border check that rejects a quad before
/// it is ever decoded, and `TagLocalization` requiring the same tag on several
/// consecutive frames before it will move the robot.
public enum AprilTagFamily {

    /// Payload edge length in cells.
    public static let payloadSize = 4
    /// Total edge length in cells, including the black border.
    public static let gridSize = 6
    public static let bitCount = payloadSize * payloadSize

    /// The published `tag16h5` code table, in ID order.
    ///
    /// `AprilTagFamilyTests` recomputes the minimum Hamming distance over every
    /// code and every rotation and requires it to be 5, which is what the family
    /// name promises. That test is really a checksum on this table: a single
    /// mistyped hex digit almost certainly drops the minimum below 5.
    public static let codes: [UInt16] = [
        0x231b, 0x2ea5, 0x346a, 0x45b9, 0x79a6, 0x7f6b, 0xb358, 0xe745, 0xfe59, 0x156d,
        0x380b, 0xf0ab, 0x0d84, 0x4736, 0x8c72, 0xaf10, 0x093c, 0x93b4, 0xa503, 0x468f,
        0xe137, 0x5795, 0xdf42, 0x1c1d, 0xe9dc, 0x73ad, 0xad5f, 0xd530, 0x07ca, 0xaf2e,
    ]

    public static var validIDs: ClosedRange<Int> { 0...(codes.count - 1) }

    public static func isValidID(_ id: Int) -> Bool { validIDs.contains(id) }

    /// Rotates a payload 90° clockwise.
    ///
    /// The destination cell `(row, column)` takes the value of the source cell
    /// `(size - 1 - column, row)`.
    public static func rotate90(_ code: UInt16) -> UInt16 {
        var rotated: UInt16 = 0
        for row in 0..<payloadSize {
            for column in 0..<payloadSize {
                let sourceIndex = (payloadSize - 1 - column) * payloadSize + row
                let bit = (code >> UInt16(bitCount - 1 - sourceIndex)) & 1
                let destinationIndex = row * payloadSize + column
                rotated |= bit << UInt16(bitCount - 1 - destinationIndex)
            }
        }
        return rotated
    }

    /// Every rotation of every code, precomputed. 120 entries.
    ///
    /// Built once because decoding runs on every candidate quad in every frame,
    /// and a scene can offer dozens of candidates.
    static let rotatedCodes: [(id: Int, rotation: Int, code: UInt16)] = {
        var table: [(id: Int, rotation: Int, code: UInt16)] = []
        table.reserveCapacity(codes.count * 4)
        for (id, code) in codes.enumerated() {
            var current = code
            for rotation in 0..<4 {
                table.append((id, rotation, current))
                current = rotate90(current)
            }
        }
        return table
    }()

    public struct Match: Equatable, Sendable {
        public let id: Int
        /// Quarter turns clockwise between the sampled orientation and the
        /// canonical one. The quad's corners are rotated by this to recover the
        /// tag's true orientation.
        public let rotation: Int
        /// How many bits had to be corrected. Zero unless correction is enabled.
        public let hammingDistance: Int

        public init(id: Int, rotation: Int, hammingDistance: Int) {
            self.id = id
            self.rotation = rotation
            self.hammingDistance = hammingDistance
        }
    }

    /// Decodes a sampled 16-bit payload.
    ///
    /// - Parameter maximumHammingCorrection: how many wrong bits to forgive.
    ///   **Zero by default, deliberately.** The family's distance of 5 makes
    ///   correcting two bits sound safe, but every forgiven bit multiplies the
    ///   space of random patterns that decode to a valid ID — at distance 1 that
    ///   is 17x more, and a false tag does not degrade the pose, it teleports
    ///   the robot somewhere else entirely.
    public static func decode(
        payload: UInt16,
        maximumHammingCorrection: Int = 0
    ) -> Match? {
        var best: Match?
        for entry in rotatedCodes {
            let distance = (payload ^ entry.code).nonzeroBitCount
            guard distance <= maximumHammingCorrection else { continue }
            if distance == 0 {
                return Match(id: entry.id, rotation: entry.rotation, hammingDistance: 0)
            }
            if best == nil || distance < best!.hammingDistance {
                best = Match(id: entry.id, rotation: entry.rotation, hammingDistance: distance)
            }
        }
        return best
    }

    /// Renders a tag as a `gridSize` x `gridSize` array of cell colours, `true`
    /// meaning white. Used to synthesise tags for the tests and to draw the
    /// preview in the settings screen.
    public static func cells(forID id: Int) -> [[Bool]] {
        precondition(isValidID(id), "tag16h5 has ids 0...\(codes.count - 1)")
        let code = codes[id]
        var grid = [[Bool]](repeating: [Bool](repeating: false, count: gridSize), count: gridSize)
        for row in 0..<payloadSize {
            for column in 0..<payloadSize {
                let index = row * payloadSize + column
                let bit = (code >> UInt16(bitCount - 1 - index)) & 1
                grid[row + 1][column + 1] = bit == 1
            }
        }
        return grid
    }
}
