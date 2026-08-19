import SwiftUI

/// One configured AprilTag in the settings list: which tag, how big, where.
///
/// Collapsed to a single summary line until tapped, because the interesting
/// state is usually "which tags are set up", not the nine numbers behind each
/// one. Expanding is what makes a list of four tags readable at all.
struct AprilTagMountRow: View {
    @Binding var mount: AprilTagMount
    @State private var isExpanded = false

    var body: some View {
        DisclosureGroup(isExpanded: $isExpanded) {
            VStack(alignment: .leading, spacing: 10) {
                identityRow
                Divider()
                offsetRows
                Divider()
                angleRows
                Toggle("Look for this tag", isOn: $mount.isEnabled)
                    .font(.footnote)
            }
            .padding(.vertical, 4)
        } label: {
            summaryLabel
        }
    }

    // MARK: - Summary

    private var summaryLabel: some View {
        HStack(spacing: 10) {
            TagPreview(tagID: mount.tagID)
                .frame(width: 30, height: 30)
                .opacity(mount.isEnabled ? 1 : 0.35)

            VStack(alignment: .leading, spacing: 2) {
                HStack(spacing: 6) {
                    Text("Tag \(mount.tagID)")
                        .font(.subheadline.weight(.semibold))
                    if !mount.isEnabled {
                        Text("off")
                            .font(.caption2)
                            .foregroundStyle(.secondary)
                    }
                    if !mount.isValid {
                        Image(systemName: "exclamationmark.triangle.fill")
                            .font(.caption2)
                            .foregroundStyle(Color.orange)
                    }
                }
                Text(mount.summary)
                    .font(.caption2)
                    .foregroundStyle(.secondary)
                    .monospacedDigit()
            }
        }
    }

    // MARK: - Fields

    private var identityRow: some View {
        VStack(alignment: .leading, spacing: 6) {
            Stepper(value: $mount.tagID, in: AprilTagFamily.validIDs) {
                HStack {
                    Text("Tag ID")
                    Spacer()
                    Text("\(mount.tagID)")
                        .monospacedDigit()
                        .foregroundStyle(.secondary)
                }
            }
            // Size is entered in millimetres because that is how a printed tag
            // is measured and how a ruler is marked. Storing metres and
            // converting at the edge keeps the geometry in SI throughout.
            HStack {
                Text("Size (black border)")
                Spacer(minLength: 12)
                TextField(
                    "size",
                    value: Binding(
                        get: { mount.sizeMetres * 1000 },
                        set: { mount.sizeMetres = $0 / 1000 }
                    ),
                    format: .number.precision(.fractionLength(0...1))
                )
                .keyboardType(.decimalPad)
                .multilineTextAlignment(.trailing)
                .monospacedDigit()
                .frame(maxWidth: 80)
                Text("mm")
                    .foregroundStyle(.secondary)
            }
            Text("Family is tag16h5, so IDs run 0–29. Measure the outer black square.")
                .font(.caption2)
                .foregroundStyle(.secondary)
        }
    }

    private var offsetRows: some View {
        VStack(alignment: .leading, spacing: 4) {
            field(title: "Forward", value: $mount.x, unit: "m", step: 0.01, help: "+X, ahead of base_link")
            field(title: "Left", value: $mount.y, unit: "m", step: 0.01, help: "+Y, to the robot's left")
            field(title: "Up", value: $mount.z, unit: "m", step: 0.01, help: "+Z, above base_link")
        }
    }

    private var angleRows: some View {
        VStack(alignment: .leading, spacing: 4) {
            field(title: "Roll", value: $mount.rollDegrees, unit: "°", step: 5, help: "positive drops the tag's right side")
            field(title: "Pitch", value: $mount.pitchDegrees, unit: "°", step: 5, help: "positive tips the face down; −90° faces the sky")
            field(title: "Yaw", value: $mount.yawDegrees, unit: "°", step: 5, help: "positive turns the face to the robot's left; 180° is the tail")
        }
    }

    private func field(
        title: String,
        value: Binding<Double>,
        unit: String,
        step: Double,
        help: String
    ) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            HStack {
                Text(title)
                    .font(.footnote)
                Spacer(minLength: 12)
                TextField(title, value: value, format: .number.precision(.fractionLength(0...3)))
                    .keyboardType(.numbersAndPunctuation)  // .decimalPad has no minus sign
                    .multilineTextAlignment(.trailing)
                    .monospacedDigit()
                    .font(.footnote)
                    .frame(maxWidth: 76)
                Text(unit)
                    .font(.footnote)
                    .foregroundStyle(.secondary)
                Stepper(title, value: value, step: step)
                    .labelsHidden()
            }
            Text(help)
                .font(.caption2)
                .foregroundStyle(.secondary)
        }
    }
}

/// Draws the actual tag, so the ID in the settings can be checked against the
/// marker on the robot without printing a chart or trusting a number.
///
/// Cheap enough to draw inline: `tag16h5` is a 6x6 grid, so this is 36
/// rectangles from the same table the decoder matches against — which means the
/// preview cannot drift out of step with what the app will actually recognise.
struct TagPreview: View {
    let tagID: Int

    var body: some View {
        GeometryReader { geometry in
            let size = min(geometry.size.width, geometry.size.height)
            let cell = size / CGFloat(AprilTagFamily.gridSize)
            if AprilTagFamily.isValidID(tagID) {
                let cells = AprilTagFamily.cells(forID: tagID)
                ZStack(alignment: .topLeading) {
                    Rectangle().fill(.white)
                    ForEach(0..<AprilTagFamily.gridSize, id: \.self) { row in
                        ForEach(0..<AprilTagFamily.gridSize, id: \.self) { column in
                            if !cells[row][column] {
                                Rectangle()
                                    .fill(.black)
                                    .frame(width: cell, height: cell)
                                    .offset(x: CGFloat(column) * cell, y: CGFloat(row) * cell)
                            }
                        }
                    }
                }
                .frame(width: size, height: size)
                .overlay(Rectangle().strokeBorder(.gray.opacity(0.4), lineWidth: 0.5))
            } else {
                Image(systemName: "questionmark.square.dashed")
                    .foregroundStyle(.orange)
                    .frame(width: size, height: size)
            }
        }
        .aspectRatio(1, contentMode: .fit)
    }
}
