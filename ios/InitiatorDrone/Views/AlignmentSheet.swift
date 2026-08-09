#if canImport(ARKit)
import SwiftUI

/// The "Align robot" workflow.
///
/// The whole point of this screen is to answer one question the app cannot work
/// out on its own: where, in the room the phone can see, is the origin of the
/// robot's odometry, and which way was it facing? Everything drawn in AR hangs
/// off that answer.
struct AlignmentSheet: View {
    @EnvironmentObject private var model: AppModel
    @EnvironmentObject private var alignment: AlignmentController
    @EnvironmentObject private var arSession: ARSessionController
    @EnvironmentObject private var connection: RobotConnection
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationStack {
            Form {
                if !arSession.isTrackingUsable {
                    Section {
                        Label(arSession.trackingStateDetail.isEmpty
                                ? "Move the phone slowly until tracking settles."
                                : arSession.trackingStateDetail,
                              systemImage: "exclamationmark.triangle.fill")
                            .foregroundStyle(.orange)
                    } header: {
                        Text("Phone tracking: \(arSession.trackingStateLabel)")
                    } footer: {
                        Text("Aligning while the phone pose is unreliable puts the robot in the wrong place.")
                    }
                }

                switch alignment.phase {
                case .idle:
                    idleSections
                case .pickingPosition:
                    pickingSection
                case .adjustingHeading:
                    headingSection
                }

                if let message = alignment.statusMessage {
                    Section {
                        Text(message).font(.footnote)
                    }
                }
            }
            .navigationTitle("Align robot")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Done") {
                        if alignment.isPlacing { alignment.cancel() }
                        dismiss()
                    }
                }
            }
        }
    }

    // MARK: - Idle

    @ViewBuilder
    private var idleSections: some View {
        Section {
            Button {
                alignment.alignAtPhone(arSession.currentPose)
                dismiss()
            } label: {
                Label("The robot is here", systemImage: "location.fill")
            }
            .disabled(!arSession.isTrackingUsable)
        } header: {
            Text("Stand at the robot")
        } footer: {
            Text("""
            Hold the phone where the robot started, pointing the way the robot was facing, then tap. \
            This is the quickest flow: open the app at the robot, align, then walk around. \
            Only the heading is taken from the phone, so it does not matter how the phone is tilted.
            """)
        }

        Section {
            Button {
                alignment.beginSurfacePlacement()
            } label: {
                Label("Place on a surface", systemImage: "hand.tap.fill")
            }
            .disabled(!arSession.isTrackingUsable)
        } footer: {
            Text("Aim the crosshair at the floor where the robot started and tap to drop the origin.")
        }

        Section("Origin height") {
            HStack {
                Text("Offset")
                Spacer()
                Text(String(format: "%.2f m", alignment.originHeightOffset))
                    .monospacedDigit()
                    .foregroundStyle(.secondary)
            }
            Slider(value: $alignment.originHeightOffset, in: -2.0...1.0, step: 0.05)
            Text("Raises or lowers the robot's odometry origin relative to the point you pick.")
                .font(.caption)
                .foregroundStyle(.secondary)
        }

        if alignment.isAligned {
            Section("Current alignment") {
                Text(alignment.summary)
                    .font(.system(.footnote, design: .monospaced))

                nudgeControls

                Button(role: .destructive) {
                    alignment.clearAlignment()
                } label: {
                    Label("Clear alignment", systemImage: "trash")
                }
            }
        } else if let saved = model.savedAlignmentForCurrentRobot {
            Section {
                Button {
                    alignment.restore(saved)
                } label: {
                    Label("Restore saved alignment", systemImage: "clock.arrow.circlepath")
                }
            } footer: {
                Text("""
                Saved alignments are relative to the AR world origin, which moves every time the AR \
                session restarts. Restore it only if this session has been running since you set it.
                """)
            }
        }

        Section {
            Button(role: .destructive) {
                model.resetARTracking()
                dismiss()
            } label: {
                Label("Reset AR tracking", systemImage: "arrow.counterclockwise")
            }
        } footer: {
            Text("Starts a new AR world origin and clears the alignment.")
        }
    }

    private var nudgeControls: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Nudge")
                .font(.caption)
                .foregroundStyle(.secondary)

            HStack(spacing: 12) {
                nudgeButton("Left", "arrow.left") { alignment.adjust(dx: -0.05) }
                nudgeButton("Right", "arrow.right") { alignment.adjust(dx: 0.05) }
                nudgeButton("Fwd", "arrow.up") { alignment.adjust(dz: -0.05) }
                nudgeButton("Back", "arrow.down") { alignment.adjust(dz: 0.05) }
            }

            HStack(spacing: 12) {
                nudgeButton("Up", "arrow.up.to.line") { alignment.adjust(dy: 0.05) }
                nudgeButton("Down", "arrow.down.to.line") { alignment.adjust(dy: -0.05) }
                nudgeButton("Yaw −", "rotate.left") { alignment.adjust(dyaw: -.pi / 36) }
                nudgeButton("Yaw +", "rotate.right") { alignment.adjust(dyaw: .pi / 36) }
            }
        }
    }

    private func nudgeButton(_ title: String, _ image: String, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            VStack(spacing: 2) {
                Image(systemName: image)
                Text(title).font(.system(size: 9))
            }
            .frame(maxWidth: .infinity)
        }
        .buttonStyle(.bordered)
    }

    // MARK: - Picking

    private var pickingSection: some View {
        Section {
            if let preview = alignment.previewPosition {
                Label(
                    String(format: "Surface at (%.2f, %.2f, %.2f)", preview.x, preview.y, preview.z),
                    systemImage: "checkmark.circle.fill"
                )
                .foregroundStyle(.green)
                .font(.system(.footnote, design: .monospaced))
            } else {
                Label("Looking for a horizontal surface…", systemImage: "magnifyingglass")
                    .foregroundStyle(.secondary)
            }

            Button("Cancel", role: .cancel) { alignment.cancel() }
        } header: {
            Text("Aim and tap")
        } footer: {
            Text("Close this sheet to see the camera, aim the crosshair at the floor, and tap the screen.")
        }
    }

    // MARK: - Heading

    private var headingSection: some View {
        Section {
            HStack {
                Text("Heading")
                Spacer()
                Text(String(format: "%.0f°", alignment.pendingYaw * 180 / .pi))
                    .monospacedDigit()
                    .foregroundStyle(.secondary)
            }
            Slider(value: $alignment.pendingYaw, in: -Double.pi...Double.pi)

            Button {
                alignment.usePhoneHeading(arSession.currentPose)
            } label: {
                Label("Use the phone's heading", systemImage: "location.north.line.fill")
            }

            Button {
                alignment.confirmPlacement()
                dismiss()
            } label: {
                Label("Confirm alignment", systemImage: "checkmark.circle.fill")
            }
            .buttonStyle(.borderedProminent)

            Button("Cancel", role: .cancel) { alignment.cancel() }
        } header: {
            Text("Which way was the robot facing?")
        } footer: {
            Text("The blue arrow in the camera view shows the robot's forward direction.")
        }
    }
}
#endif
