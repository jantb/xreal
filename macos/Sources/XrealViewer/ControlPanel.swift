import AppKit
import SwiftUI
import XrealCore

private let refreshInterval = 0.25
private let screenRecordingSettings = URL(
    string: "x-apple.systempreferences:com.apple.settings.PrivacySecurity.extension?Privacy_ScreenCapture")!

/// How the viewer is doing, in a line or two for people rather than the
/// status lines' diagnostics.
struct Headline {
    enum Level { case ok, busy, idle, problem }

    var level: Level
    var title: String
    var detail: String

    init(source: SourceStatus, tracking: TrackingSnapshot) {
        switch source {
        case .starting, .creatingCanvas:
            (level, title, detail) = (.busy, "Starting", "Creating the canvas…")
        case .noGlasses:
            (level, title, detail) = (.idle, "Glasses not connected", "Connect the glasses to show the canvas.")
        case .refused:
            (level, title, detail) = (.problem, "macOS refused the canvas", "Pick another canvas size.")
        case .failed(let problem):
            (level, title, detail) = (.problem, "No picture", problem)
        case .live(let width, let height) where tracking.status == .searching:
            (level, title, detail) = (.busy, "Head tracking not found", "Canvas \(width) × \(height). Reconnecting…")
        case .live(let width, let height):
            (level, title, detail) = (
                .ok, "Showing the canvas",
                String(format: "Canvas %d × %d, head tracking at %.0f Hz", width, height, tracking.sampleRateHz)
            )
        }
    }

    var symbol: String {
        switch level {
        case .ok: "checkmark.circle.fill"
        case .busy: "clock.fill"
        case .idle: "eyeglasses"
        case .problem: "exclamationmark.triangle.fill"
        }
    }

    var color: Color {
        switch level {
        case .ok: .green
        case .busy: .orange
        case .idle: .secondary
        case .problem: .red
        }
    }
}

/// What the controls window shows, read from the viewer while it is open.
@MainActor @Observable final class ControlModel {
    private let viewer: Viewer
    private(set) var settings: XrealCore.Settings
    private(set) var info: HudInfo
    private(set) var source: SourceStatus
    private(set) var screenRecordingAllowed = true
    private(set) var windowControlAllowed = true
    /// Windows that can be pinned above the canvas, as last looked up.
    private(set) var pinnableWindows: [PinnableWindow] = []
    private(set) var windowsLoading = false
    private(set) var windowsError: String?
    private(set) var pinnedWindowStatus: String?
    private(set) var pinnedWindowID: CGWindowID?
    private var windowsTask: Task<Void, Never>?

    init(viewer: Viewer) {
        self.viewer = viewer
        settings = viewer.settings
        (info, source) = viewer.statusInfo()
        refresh()
    }

    var headline: Headline { Headline(source: source, tracking: info.tracking) }
    var statusLines: [String] { hudLines(info) }

    func refresh() {
        // Set only when changed: each set redraws what reads it.
        func update<T: Equatable>(_ value: ReferenceWritableKeyPath<ControlModel, T>, _ new: T) {
            if self[keyPath: value] != new { self[keyPath: value] = new }
        }
        update(\.settings, viewer.settings)
        let status = viewer.statusInfo()
        info = status.info
        update(\.source, status.source)
        update(\.screenRecordingAllowed, CGPreflightScreenCaptureAccess())
        update(\.windowControlAllowed, WindowControl.allowed(prompt: false))
        update(\.pinnedWindowStatus, viewer.pinnedWindowStatus)
        update(\.pinnedWindowID, viewer.preferredPinnedWindowID)
    }

    func perform(_ command: ViewerCommand) {
        viewer.perform(command)
        refresh()
    }

    /// A binding for a setting that `command` flips.
    func toggle(_ value: KeyPath<ControlModel, Bool>, _ command: ViewerCommand) -> Binding<Bool> {
        Binding(
            get: { self[keyPath: value] },
            set: { if $0 != self[keyPath: value] { self.perform(command) } })
    }

    func refreshPinnableWindows() {
        windowsTask?.cancel()
        windowsLoading = true
        windowsError = nil
        windowsTask = Task {
            do {
                let windows = try await ScreenCapture.pinnableWindows()
                guard !Task.isCancelled else { return }
                pinnableWindows = windows
            } catch {
                guard !Task.isCancelled else { return }
                windowsError = error.localizedDescription
            }
            windowsLoading = false
        }
    }

    func askForWindowControl() {
        _ = WindowControl.allowed(prompt: true)
        refresh()
    }
}

/// The window with every setting and action, opened from the menu bar.
@MainActor final class ControlPanel: NSObject, NSWindowDelegate {
    private let model: ControlModel
    private var window: NSWindow?
    private var timer: Timer?

    init(viewer: Viewer) {
        model = ControlModel(viewer: viewer)
    }

    func show() {
        let window = window ?? makeWindow()
        self.window = window
        model.refresh()
        model.refreshPinnableWindows()
        if timer == nil {
            let timer = Timer(timeInterval: refreshInterval, repeats: true) { [weak self] _ in
                MainActor.assumeIsolated { self?.model.refresh() }
            }
            RunLoop.main.add(timer, forMode: .common)
            self.timer = timer
        }
        NSApp.activate()
        window.makeKeyAndOrderFront(nil)
    }

    func windowWillClose(_ notification: Notification) {
        timer?.invalidate()
        timer = nil
    }

    private func makeWindow() -> NSWindow {
        let hosting = NSHostingController(rootView: ControlView(model: model))
        hosting.sizingOptions = .minSize
        let window = NSWindow(contentViewController: hosting)
        window.title = "XREAL Viewer"
        window.styleMask = [.titled, .closable, .miniaturizable, .resizable]
        window.isReleasedWhenClosed = false
        window.delegate = self
        window.setContentSize(NSSize(width: 480, height: 760))
        window.center()
        window.setFrameAutosaveName("Controls")
        return window
    }
}

private struct ControlView: View {
    let model: ControlModel

    var body: some View {
        Form {
            StatusSection(model: model)
            CanvasSection(model: model)
            AboveCanvasSection(model: model)
            ViewSection(model: model)
            TrackingSection(model: model)
            WindowsSection(model: model)
            ShortcutsSection()
            DiagnosticsSection(model: model)
        }
        .formStyle(.grouped)
        .frame(minWidth: 420, minHeight: 420)
    }
}

private struct StatusSection: View {
    let model: ControlModel

    var body: some View {
        Section {
            let headline = model.headline
            HStack(spacing: 12) {
                Image(systemName: headline.symbol)
                    .font(.title)
                    .foregroundStyle(headline.color)
                VStack(alignment: .leading, spacing: 2) {
                    Text(headline.title).font(.headline)
                    Text(headline.detail).font(.callout).foregroundStyle(.secondary)
                }
            }
            .padding(.vertical, 4)
            if !model.screenRecordingAllowed {
                Problem(
                    "Screen Recording is off",
                    detail: "The viewer needs it to show the canvas. Allow XREAL Viewer, then quit and reopen it."
                ) {
                    Button("Open Settings") { NSWorkspace.shared.open(screenRecordingSettings) }
                    Button("Quit") { NSApp.terminate(nil) }
                }
            }
        }
    }
}

private struct CanvasSection: View {
    let model: ControlModel

    private struct Size: Hashable {
        var width: Int
        var height: Int
        var scale: Int
    }

    var body: some View {
        let settings = model.settings
        let current = Size(width: settings.canvas.width, height: settings.canvas.height, scale: settings.canvas.scale)
        let sizes = canvasSizes.map { Size(width: $0.width, height: $0.height, scale: $0.scale) }
        Section("Canvas") {
            Picker(
                "Size",
                selection: Binding(
                    get: { current },
                    set: { model.perform(.setCanvasSize(width: $0.width, height: $0.height, scale: $0.scale)) })
            ) {
                ForEach(sizes.contains(current) ? sizes : sizes + [current], id: \.self) { size in
                    Text(verbatim: "\(size.width) × \(size.height)" + (size.scale == 2 ? " HiDPI" : "")).tag(size)
                }
            }
            if settings.canvas.scale == 2 {
                Text("HiDPI draws text at twice the detail and filters it down: sharper, but less fits on the canvas.")
                    .font(.caption).foregroundStyle(.secondary)
            }
            Picker(
                "Refresh Rate",
                selection: Binding(
                    get: { settings.canvasRefreshRate }, set: { model.perform(.setCanvasRefreshRate($0)) })
            ) {
                ForEach(canvasRefreshRates, id: \.self) { rate in Text(verbatim: "\(rate) Hz").tag(rate) }
            }
            Text(
                "How often what is on the canvas can change. Head movement is drawn at the glasses' own rate either way; 60 Hz leaves the GPU more room."
            )
            .font(.caption).foregroundStyle(.secondary)
            VStack(alignment: .leading) {
                Toggle("Ambient Light", isOn: model.toggle(\.settings.ambientLight, .toggleAmbientLight))
                Text("Lights the room round the canvas in the colours of its edges, like a TV's backlight.")
                    .font(.caption).foregroundStyle(.secondary)
            }
            VStack(alignment: .leading) {
                Picker(
                    "Shape",
                    selection: Binding(get: { settings.canvas.shape }, set: { model.perform(.setShape($0)) })
                ) {
                    ForEach(CanvasShape.allCases, id: \.self) { shape in Text(shape.rawValue).tag(shape) }
                }
                Text(
                    "Wrap Around You curves the canvas up and down as well, like the inside of a ball centred on you, so every pixel faces you."
                )
                .font(.caption).foregroundStyle(.secondary)
            }
            if settings.canvas.spherical {
                VStack(alignment: .leading) {
                    LabeledContent("Wrap Up and Down") {
                        Text(String(format: "%.0f%%", settings.canvas.verticalWrap * 100)).monospacedDigit()
                    }
                    Slider(
                        value: Binding(
                            get: { settings.canvas.verticalWrap }, set: { model.perform(.setVerticalWrap($0)) }),
                        in: 0...1)
                    Text(
                        "At 100% the canvas is part of a ball round you: every pixel faces you and keeps its shape, a little smaller towards the top and bottom. Less bends it less up and down."
                    )
                    .font(.caption).foregroundStyle(.secondary)
                    Toggle(
                        "Even Out Text Size", isOn: model.toggle(\.settings.canvas.evenSize, .toggleEvenTextSize))
                    Text(
                        "Text is then a few percent larger in the middle and smaller at the top and bottom, rather than full size in the middle and smallest at the edges."
                    )
                    .font(.caption).foregroundStyle(.secondary)
                }
            } else {
                VStack(alignment: .leading) {
                    LabeledContent("Curve") {
                        Text(String(format: "Radius %.2f × distance", settings.curveRadius)).monospacedDigit()
                    }
                    // Stronger to the right, which is a smaller radius.
                    Slider(
                        value: Binding(
                            get: { -log(settings.curveRadius) }, set: { model.perform(.setCurveRadius(exp(-$0))) }),
                        in: -log(maxCurveRadius)...(-log(minCurveRadius)))
                    Text("At 1 the canvas surrounds you evenly; further right bends it more.")
                        .font(.caption).foregroundStyle(.secondary)
                }
                .disabled(!settings.canvas.curved)
            }
            VStack(alignment: .leading) {
                LabeledContent("Viewing Distance") {
                    Text(String(format: "%.2f m", settings.metresPerRoomUnit)).monospacedDigit()
                }
                Slider(
                    value: Binding(
                        get: { log(settings.metresPerRoomUnit) },
                        set: { model.perform(.setDepthScale(snappedViewingDistance(exp($0)))) }),
                    in: log(minViewingDistance)...log(maxViewingDistance))
                if settings.metresPerRoomUnit == glassesFocusDistance {
                    Label("Eyes aim and focus at the same distance", systemImage: "checkmark.circle.fill")
                        .font(.caption).foregroundStyle(.green)
                }
                Text(
                    "Nearer shows more depth between the eyes' views; the canvas keeps its size. The glasses' optics focus at about 4 m, where the slider snaps: there the eyes aim and focus at the same distance, easiest on them over hours."
                )
                .font(.caption).foregroundStyle(.secondary)
            }
            Button("Put Canvas Back Straight Ahead") { model.perform(.resetView) }
        }
    }
}

private struct ViewSection: View {
    let model: ControlModel

    var body: some View {
        Section("View") {
            Toggle("Follow Head Tilt", isOn: model.toggle(\.settings.followRoll, .toggleRoll))
            Toggle("Sharpen Text", isOn: model.toggle(\.settings.sharpFiltering, .toggleSharpFiltering))
            Toggle("Soft Edges", isOn: model.toggle(\.settings.softEdges, .toggleSoftEdges))
            VStack(alignment: .leading) {
                Toggle(
                    "Turn Off the Laptop Screen While Wearing the Glasses",
                    isOn: model.toggle(\.settings.laptopScreenOff, .toggleLaptopScreenOff))
                Text(
                    "With the laptop's screen on alongside the canvas, the glasses drop frames every few seconds; switched off, not at all. It comes back on when the glasses are unplugged, this is turned off or the viewer quits, and if the viewer dies, at once."
                )
                .font(.caption).foregroundStyle(.secondary)
            }
        }
    }
}

private struct AboveCanvasSection: View {
    let model: ControlModel
    @State private var choosingWindow = false

    var body: some View {
        let pinned = model.settings.pinnedWindow
        Section("Above the Canvas") {
            Toggle("Dashboard", isOn: model.toggle(\.settings.statusStrip, .toggleStatusStrip))
            LabeledContent("Pinned Window") {
                HStack {
                    Text(pinned?.title ?? "None").lineLimit(1).truncationMode(.middle)
                    Button("Choose…") {
                        model.refreshPinnableWindows()
                        choosingWindow = true
                    }
                    if pinned != nil {
                        Button("Remove") { model.perform(.setPinnedWindow(nil)) }
                    }
                }
            }
            .sheet(isPresented: $choosingWindow) { PinnedWindowChooser(model: model) }
            if let status = model.pinnedWindowStatus {
                Text(status).font(.caption).foregroundStyle(.secondary)
            }
            Text(
                "Look up to see them: the Mac's CPU, memory, GPU, network, battery and disk, its busiest apps, the glasses and latency, and the pinned window beside them. The pinned window can stay anywhere, even on the glasses' own display behind the view."
            )
            .font(.caption).foregroundStyle(.secondary)
        }
    }
}

private struct PinnedWindowChooser: View {
    let model: ControlModel
    @Environment(\.dismiss) private var dismiss
    @State private var search = ""

    private var choices: [PinnableWindow] { model.pinnableWindows.filter { $0.matches(search) } }

    private func detail(_ choice: PinnableWindow) -> String {
        let duplicates = model.pinnableWindows.filter { $0.window == choice.window }
        let number = duplicates.count > 1
            ? " · \((duplicates.firstIndex { $0.id == choice.id } ?? 0) + 1) of \(duplicates.count)"
            : ""
        return "\(choice.appName) · \(choice.width) × \(choice.height)\(number)"
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Choose a Pinned Window").font(.title2)
            TextField("Search apps and window titles", text: $search)
                .textFieldStyle(.roundedBorder)
            if let error = model.windowsError {
                Text("Could not list windows: \(error)").foregroundStyle(.red)
            }
            if model.windowsLoading { ProgressView("Looking for windows…") }
            List(choices) { choice in
                Button {
                    model.perform(.pinWindow(choice))
                    dismiss()
                } label: {
                    HStack {
                        VStack(alignment: .leading, spacing: 3) {
                            Text(choice.window.title).lineLimit(2)
                            Text(detail(choice))
                                .font(.caption).foregroundStyle(.secondary)
                        }
                        Spacer()
                        if choice.id == model.pinnedWindowID { Image(systemName: "checkmark") }
                    }
                    .contentShape(Rectangle())
                }
                .buttonStyle(.plain)
                .padding(.vertical, 4)
            }
            .overlay {
                if choices.isEmpty && !model.windowsLoading && model.windowsError == nil {
                    Text(search.isEmpty ? "No windows available. Open a window and refresh." : "No matching windows.")
                        .foregroundStyle(.secondary).padding()
                }
            }
            HStack {
                Button("Refresh") { model.refreshPinnableWindows() }.disabled(model.windowsLoading)
                Spacer()
                Button("Cancel") { dismiss() }.keyboardShortcut(.cancelAction)
            }
        }
        .padding(20)
        .frame(width: 560, height: 460)
    }
}

private struct TrackingSection: View {
    let model: ControlModel

    var body: some View {
        let tracking = model.info.tracking
        let connected = tracking.status == .connected
        Section("Head Tracking") {
            LabeledContent("Glasses") {
                Text(connected ? String(format: "%.0f Hz", tracking.sampleRateHz) : "Searching…").monospacedDigit()
            }
            if let temperature = tracking.temperature {
                LabeledContent("Temperature") { Text(String(format: "%.1f °C", temperature)).monospacedDigit() }
            }
            VStack(alignment: .leading, spacing: 6) {
                HStack {
                    Button("Calibrate Gyro") { model.perform(.calibrate) }
                        .disabled(!connected || isCalibrating(tracking.calibration))
                    calibrationState(tracking.calibration)
                }
                Text("Put the glasses down somewhere steady and leave them still until it finishes.")
                    .font(.caption).foregroundStyle(.secondary)
            }
            VStack(alignment: .leading, spacing: 6) {
                HStack {
                    Button("Recenter") { model.perform(.recenter) }
                    Spacer()
                    Text("⌃⌥⌘C").font(.callout.monospaced()).foregroundStyle(.secondary)
                }
                Text(driftText(model.info.lastDrift)).font(.caption).foregroundStyle(.secondary)
            }
        }
    }

    private func isCalibrating(_ state: CalibrationState) -> Bool {
        if case .running = state { true } else { false }
    }

    @ViewBuilder private func calibrationState(_ state: CalibrationState) -> some View {
        switch state {
        case .idle:
            EmptyView()
        case .running(let progress):
            ProgressView(value: progress).frame(maxWidth: 160)
            Text("Keep still").foregroundStyle(.secondary)
        case .succeeded:
            Label("Calibrated", systemImage: "checkmark.circle.fill").foregroundStyle(.green)
        case .failed:
            Label("The glasses moved; try again", systemImage: "xmark.circle.fill").foregroundStyle(.red)
        }
    }

    private func driftText(_ drift: DriftObservation?) -> String {
        let degreesPerMinute = { (rate: Float) in rate * 180 / .pi * 60 }
        switch drift {
        case nil:
            return "Recentering again at least 20 s later teaches the viewer how far the view drifts."
        case .anchored:
            return "Drift reference set. Recenter again later to correct the drift."
        case .tooSoon:
            return "Too soon after the last recenter to measure drift; wait at least 20 s."
        case .learned(let measured, _):
            return String(format: "Corrected a drift of %+.2f° per minute.", degreesPerMinute(measured))
        case .rejected(let measured):
            return String(
                format: "%+.1f° per minute is too fast for drift, so it was taken as a turn.",
                degreesPerMinute(measured))
        }
    }
}

private struct WindowsSection: View {
    let model: ControlModel

    var body: some View {
        Section("Windows") {
            if !model.windowControlAllowed {
                Problem(
                    "Accessibility access is off",
                    detail: "Moving and fitting windows to where you look needs it."
                ) {
                    Button("Allow…") { model.askForWindowControl() }
                }
            }
            Button("Bring Back Windows Hidden Behind the Glasses") { model.perform(.window(.gather)) }
                .disabled(!model.windowControlAllowed)
        }
    }
}

private struct ShortcutsSection: View {
    private let shortcuts: [(keys: String, action: String)] = [
        ("⌃⌥⌘C", "Recenter"),
        ("⌃⌥⌘G", "Hold while looking at the canvas to carry it"),
        ("⌃⌥⌘=  ⌃⌥⌘-", "Bring the canvas closer, push it away"),
        ("⌃⌥⌘]  ⌃⌥⌘[", "Next canvas size up, down"),
        ("⌃⌥⌘W", "Move the focused window to where you look"),
        ("⌃⌥⌘F", "Fit the focused window to the zone you look at"),
        ("⌃⌥⌘M", "Move the pointer to where you look"),
    ]

    var body: some View {
        Section("Shortcuts") {
            ForEach(shortcuts, id: \.keys) { shortcut in
                LabeledContent(shortcut.action) {
                    Text(shortcut.keys).font(.callout.monospaced())
                }
            }
        }
    }
}

private struct DiagnosticsSection: View {
    let model: ControlModel
    @AppStorage("diagnosticsVisible") private var visible = true

    var body: some View {
        Section {
            DisclosureGroup("Diagnostics", isExpanded: $visible) {
                Text(model.statusLines.joined(separator: "\n"))
                    .font(.caption.monospaced())
                    .foregroundStyle(.secondary)
                    .textSelection(.enabled)
                    .frame(maxWidth: .infinity, alignment: .leading)
                if let p95 = model.info.timing.workP95 {
                    Text(String(format: "Pose-to-GPU workload: median %.1f ms, p95 %.1f ms", (model.info.timing.workMedian ?? 0) * 1000, p95 * 1000))
                        .font(.caption.monospaced())
                }
            }
        }
    }
}

/// A problem with what to do about it.
private struct Problem<Actions: View>: View {
    let title: String
    let detail: String
    @ViewBuilder let actions: Actions

    init(_ title: String, detail: String, @ViewBuilder actions: () -> Actions) {
        self.title = title
        self.detail = detail
        self.actions = actions()
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            Label(title, systemImage: "exclamationmark.triangle.fill")
                .font(.headline)
                .foregroundStyle(.orange)
            Text(detail).font(.callout).foregroundStyle(.secondary)
            HStack { actions }
        }
        .padding(.vertical, 4)
    }
}
