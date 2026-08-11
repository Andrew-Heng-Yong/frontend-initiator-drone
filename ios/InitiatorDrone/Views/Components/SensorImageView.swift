import SwiftUI

/// Draws a decoded sensor frame with nearest-neighbour sampling.
///
/// Depth and thermal frames are small — a 32x24 thermal frame blown up to fill
/// a phone screen is a 20x magnification — and SwiftUI's default smoothing
/// would invent detail that the sensor never measured. `.interpolation(.none)`
/// keeps every drawn pixel traceable to a reading.
struct SensorImageView: View {
    var frame: RenderedFrame?
    var placeholder: String
    var interpolate: Bool = false

    var body: some View {
        GeometryReader { geometry in
            ZStack {
                if let frame {
                    Image(decorative: frame.image, scale: 1.0)
                        .resizable()
                        .interpolation(interpolate ? .medium : .none)
                        .aspectRatio(contentMode: .fit)
                        .frame(width: geometry.size.width, height: geometry.size.height)
                } else {
                    VStack(spacing: 8) {
                        Image(systemName: "square.dashed")
                            .font(.title2)
                            .foregroundStyle(.tertiary)
                        Text(placeholder)
                            .font(.caption)
                            .foregroundStyle(.secondary)
                            .multilineTextAlignment(.center)
                    }
                    .frame(width: geometry.size.width, height: geometry.size.height)
                }
            }
        }
    }
}

/// The colour-map legend: a gradient bar with the range end points.
struct ColorLegend: View {
    var style: ColorRampStyle
    var low: Double
    var high: Double
    var unit: ScalarUnit
    var reversed: Bool

    private var gradient: LinearGradient {
        let ramp = style.ramp
        // Sample the baked table rather than the stops, so the legend and the
        // pixels come from the same source of truth.
        let colors = stride(from: 0, through: 16, by: 1).map { index -> Color in
            let position = Double(index) / 16.0
            let value = reversed ? 1.0 - position : position
            let rgb = ramp.color(normalized: value)
            return Color(
                red: Double(rgb.red) / 255.0,
                green: Double(rgb.green) / 255.0,
                blue: Double(rgb.blue) / 255.0
            )
        }
        return LinearGradient(colors: colors, startPoint: .leading, endPoint: .trailing)
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 3) {
            RoundedRectangle(cornerRadius: 3)
                .fill(gradient)
                .frame(height: 8)
            HStack {
                Text(format(low))
                Spacer()
                Text(format(high))
            }
            .font(.system(size: 9, weight: .medium, design: .monospaced))
            .foregroundStyle(.secondary)
        }
    }

    private func format(_ value: Double) -> String {
        let suffix = unit.shortLabel
        return suffix.isEmpty
            ? String(format: "%.0f", value)
            : String(format: "%.1f %@", value, suffix)
    }
}
