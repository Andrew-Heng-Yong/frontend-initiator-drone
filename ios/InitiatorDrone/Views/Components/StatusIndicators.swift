import SwiftUI

/// A small coloured pill used across the overlays for connection, calibration
/// and tracking state.
struct StatusPill: View {
    enum Level {
        case good
        case warning
        case bad
        case neutral

        var color: Color {
            switch self {
            case .good: return .green
            case .warning: return .orange
            case .bad: return .red
            case .neutral: return .gray
            }
        }
    }

    var title: String
    var value: String
    var level: Level
    var systemImage: String?

    var body: some View {
        HStack(spacing: 6) {
            if let systemImage {
                Image(systemName: systemImage)
                    .font(.caption2)
            } else {
                Circle()
                    .fill(level.color)
                    .frame(width: 8, height: 8)
            }

            VStack(alignment: .leading, spacing: 0) {
                Text(title)
                    .font(.system(size: 9, weight: .semibold))
                    .foregroundStyle(.secondary)
                    .textCase(.uppercase)
                Text(value)
                    .font(.system(size: 12, weight: .medium, design: .rounded))
            }
        }
        .padding(.horizontal, 9)
        .padding(.vertical, 6)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 9, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: 9, style: .continuous)
                .strokeBorder(level.color.opacity(0.55), lineWidth: 1)
        )
        .foregroundStyle(level == .neutral ? Color.primary : level.color)
        .accessibilityElement(children: .combine)
        .accessibilityLabel("\(title): \(value)")
    }
}

/// A labelled number, used in the metrics strip and on the diagnostics screen.
struct MetricTile: View {
    var label: String
    var value: String
    var caption: String?
    var isDimmed: Bool = false

    var body: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(label)
                .font(.system(size: 9, weight: .semibold))
                .foregroundStyle(.secondary)
                .textCase(.uppercase)
            Text(value)
                .font(.system(size: 15, weight: .semibold, design: .monospaced))
                .lineLimit(1)
                .minimumScaleFactor(0.6)
            if let caption {
                Text(caption)
                    .font(.system(size: 9))
                    .foregroundStyle(.secondary)
                    .lineLimit(1)
            }
        }
        .opacity(isDimmed ? 0.45 : 1)
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

extension RosbridgeConnectionState {
    var pillLevel: StatusPill.Level {
        switch self {
        case .connected: return .good
        case .connecting, .reconnecting: return .warning
        case .failed: return .bad
        case .idle: return .neutral
        }
    }
}

extension RobotTrackingStatus {
    var pillLevel: StatusPill.Level {
        switch self {
        case .tracking: return .good
        case .visualTrackingLost: return .warning
        case .stale, .notCalibrated: return .bad
        case .unknown: return .neutral
        }
    }
}

extension VIONodeStatus {
    var pillLevel: StatusPill.Level {
        switch self {
        case .running: return .good
        // Calibrating is the expected state for the first few seconds after a
        // launch, so it reads as "wait", not as a fault.
        case .calibrating: return .warning
        case .notRunning, .silent: return .bad
        case .graphStopped, .unknown: return .neutral
        }
    }
}
