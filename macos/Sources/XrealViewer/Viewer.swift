import AppKit
import Carbon.HIToolbox
import Synchronization
import XrealCore

private let captureFps = 120
// The glasses show a frame about 7 ms after it arrives over DisplayPort.
private let glassesDisplayDelay = 0.007
private let sensitivityStep: Float = 1.05
private let virtualScreenTimeout = 10.0

/// Something the glasses can show.
struct SourceChoice: Sendable {
    var title: String
    var source: CaptureSource
    var size: (width: Int, height: Int)?

    // Sizes macOS 27 creates as asked. Nearby standard sizes such as
    // 3840x2160, 5120x2880 or 5760x2160 are refused or come up smaller.
    static let all = [
        SourceChoice(title: "Mirror Main Display", source: .mirror),
        SourceChoice(title: "Virtual Screen 2880 × 1620", source: .virtual, size: (2880, 1620)),
        SourceChoice(title: "Virtual Screen 3832 × 2160", source: .virtual, size: (3832, 2160)),
        SourceChoice(title: "Virtual Screen 5120 × 1440 (Wide)", source: .virtual, size: (5120, 1440)),
        SourceChoice(title: "Virtual Screen 5752 × 2160", source: .virtual, size: (5752, 2160)),
    ]

    func isSelected(in settings: Settings) -> Bool {
        guard source == settings.source else { return false }
        guard let size else { return true }
        return size.width == settings.virtualWidth && size.height == settings.virtualHeight
    }
}

enum ViewerCommand {
    case recenter
    case resetView
    case toggleFreeze
    case zoomIn
    case zoomOut
    case setZoom(Int)
    case increaseSensitivity
    case decreaseSensitivity
    case resetSensitivity
    case cycleDeadzone
    case setDeadzone(Int)
    case togglePrediction
    case setProjection(Projection)
    case toggleRoll
    case setEdge(EdgeMode)
    case toggleFollowCursor
    case toggleStatus
    case calibrate
    case pan(dx: Float, dy: Float)
    case setSource(SourceChoice)
    case toggleSource
}

/// Everything both the render thread and the main thread (menu, keys) use.
struct ViewerState: Sendable {
    var settings: Settings
    var viewport: ViewportController
    var cursorFollow = CursorFollow()
    var drift = DriftLearner()
    var lastDrift: DriftObservation?
    var trackingSession: UInt64 = 0
    var biasRevisionSaved: UInt32 = 0
    var lastPose = HeadPose()
    var sourceDescription = "SOURCE STARTING"
    /// The captured display in global coordinates (points, top-left
    /// origin), to find the cursor on it.
    var sourceBounds: CGRect?

    // The latest frame, for the status lines.
    var stats = RenderStats(now: monotonicNow())
    var snapshot: TrackingSnapshot
    var geometry: ViewGeometry?
    var sourceSize: (width: Int, height: Int)?
    var output = (width: 0, height: 0)
    var newFrame = false

    init(settings: Settings) {
        self.settings = settings
        viewport = ViewportController(settings: settings)
        snapshot = TrackingSnapshot(gyroBias: settings.gyroBias)
    }

    mutating func recenter(tracking: Tracking) {
        let observation = drift.observeRecenter(now: monotonicNow(), yaw: lastPose.yaw)
        if case .learned(_, let correction) = observation {
            tracking.correctYawDrift(correction)
        }
        lastDrift = observation
        viewport.recenter(lastPose)
    }

    /// Moves on to the frame that reaches the display at `presentingAt`, in
    /// `monotonicNow` time. `cursor` is the mouse in global coordinates.
    /// Returns what to draw, and whether the gyro bias changed and should
    /// be saved.
    mutating func advance(
        now: Double, dt: Float, presentingAt: Double, snapshot: TrackingSnapshot, captureGeneration: UInt64,
        newFrame: Bool, sourceSize: (width: Int, height: Int)?, output: (width: Int, height: Int),
        cursor: CGPoint?
    ) -> (geometry: ViewGeometry?, biasChanged: Bool) {
        stats.tick(now: now, captureGeneration: captureGeneration)
        self.snapshot = snapshot
        self.newFrame = newFrame
        self.sourceSize = sourceSize
        self.output = output

        let biasChanged = snapshot.biasRevision != biasRevisionSaved
        biasRevisionSaved = snapshot.biasRevision
        let pose =
            settings.prediction
            ? snapshot.predict(now: now, lead: max(presentingAt - now, 0) + glassesDisplayDelay)
            : snapshot.pose
        lastPose = pose
        if snapshot.session != trackingSession {
            trackingSession = snapshot.session
            drift.reset()
            viewport.recenter(pose)
        }
        viewport.track(pose: pose, dt: dt)

        guard let sourceSize else {
            geometry = nil
            return (nil, biasChanged)
        }
        let viewport = viewport
        func view(scale: Float) -> ViewGeometry {
            viewport.geometry(
                outputWidth: output.width, outputHeight: output.height, sourceWidth: sourceSize.width,
                sourceHeight: sourceSize.height, scale: scale)
        }
        let cursorPixel = cursor.flatMap { sourcePixel(of: $0, sourceSize: sourceSize) }
        let scale = cursorFollow.update(cursor: cursorPixel, enabled: settings.followCursor, now: now, dt: dt) {
            scale, margin in
            cursorPixel.map { view(scale: scale).shows($0, margin: margin) } ?? false
        }
        var visible = view(scale: scale)
        if case .crop(var rect) = visible, viewport.zoom == 1, scale == 1 {
            // Whole source pixels keep text sharp at 1:1.
            rect.x.round()
            rect.y.round()
            visible = .crop(rect)
        }
        self.geometry = visible
        return (visible, biasChanged)
    }

    private func sourcePixel(of point: CGPoint, sourceSize: (width: Int, height: Int)) -> SIMD2<Float>? {
        guard let bounds = sourceBounds, bounds.contains(point), bounds.width > 0, bounds.height > 0 else {
            return nil
        }
        return SIMD2(
            Float((point.x - bounds.minX) / bounds.width) * Float(sourceSize.width),
            Float((point.y - bounds.minY) / bounds.height) * Float(sourceSize.height))
    }

    func hudInfo(now: Double) -> HudInfo {
        HudInfo(
            viewport: viewport, geometry: geometry, followScale: cursorFollow.scale, source: sourceSize,
            sourceDescription: sourceDescription, output: output, newFrame: newFrame, stats: stats,
            tracking: snapshot, pose: lastPose, prediction: settings.prediction, lastDrift: lastDrift, now: now)
    }
}

final class SharedState: Sendable {
    let mutex: Mutex<ViewerState>

    init(_ state: ViewerState) {
        mutex = Mutex(state)
    }
}

/// Builds one frame per refresh of the glasses, on the display link thread.
final class FrameLoop: @unchecked Sendable {
    private let shared: SharedState
    private let tracking: Tracking
    private let latest: LatestFrame
    private let renderer: Renderer
    /// Set before the first frame; called on the display link thread.
    var onBiasChanged: @Sendable () -> Void = {}
    // Touched only on the display link thread.
    private var frame: CapturedFrame?
    private var frameGeneration: UInt64 = 0
    private var lastRenderAt = monotonicNow()

    init(shared: SharedState, tracking: Tracking, latest: LatestFrame, renderer: Renderer) {
        self.shared = shared
        self.tracking = tracking
        self.latest = latest
        self.renderer = renderer
    }

    func render(to drawable: CAMetalDrawable, presentingAt: Double) {
        let now = monotonicNow()
        let dt = Float(min(max(now - lastRenderAt, 0), 0.1))
        lastRenderAt = now

        let current = latest.current()
        let newFrame = current.generation != frameGeneration
        if newFrame {
            frame = current.frame
            frameGeneration = current.generation
        }
        let cursor = CGEvent(source: nil)?.location
        let output = (width: drawable.texture.width, height: drawable.texture.height)
        let sourceSize = frame.map { (width: $0.width, height: $0.height) }

        // Sample the pose as late as possible, just before building the frame.
        let snapshot = tracking.snapshot()
        let result = shared.mutex.withLock { state in
            state.advance(
                now: now, dt: dt, presentingAt: presentingAt, snapshot: snapshot,
                captureGeneration: current.generation, newFrame: newFrame, sourceSize: sourceSize, output: output,
                cursor: cursor)
        }
        renderer.draw(to: drawable, frame: frame, geometry: result.geometry)
        if result.biasChanged {
            onBiasChanged()
        }
    }
}

/// Ties head tracking, capture and rendering together, and applies the
/// commands from the menu and keyboard.
@MainActor final class Viewer {
    private let shared: SharedState
    private let tracking: Tracking
    private let capture: ScreenCapture
    private let window: GlassesWindow
    private let displayLink: DisplayLinkThread
    private var virtualScreen: VirtualScreen?
    private var sourceDisplayID: CGDirectDisplayID?
    private var sourceTask: Task<Void, Never>?
    private var recenterHotKey: GlobalHotKey?

    init(settings: Settings) throws {
        let shared = SharedState(ViewerState(settings: settings))
        let tracking = Tracking(initialBias: settings.gyroBias)
        let renderer = try Renderer()
        let capture = try ScreenCapture(device: renderer.device)
        self.shared = shared
        self.tracking = tracking
        self.capture = capture

        let frameLoop = FrameLoop(shared: shared, tracking: tracking, latest: capture.latest, renderer: renderer)
        displayLink = DisplayLinkThread { drawable, presentingAt in
            frameLoop.render(to: drawable, presentingAt: presentingAt)
        }
        window = GlassesWindow(device: renderer.device, displayLink: displayLink)
        frameLoop.onBiasChanged = { [weak self] in Task { @MainActor in self?.saveSettings() } }

        window.view.onKey = { [unowned self] event in handleKey(event) }
        recenterHotKey = GlobalHotKey(
            keyCode: kVK_ANSI_C, modifiers: controlKey | optionKey | cmdKey
        ) { [unowned self] in perform(.recenter) }
        NotificationCenter.default.addObserver(
            forName: NSApplication.didChangeScreenParametersNotification, object: nil, queue: .main
        ) { [weak self] _ in
            MainActor.assumeIsolated { self?.screensChanged() }
        }

        window.show()
        startSource()
    }

    /// The settings and view as they are now, for the menu.
    var current: (settings: Settings, viewport: ViewportController) {
        shared.mutex.withLock { ($0.settings, $0.viewport) }
    }

    func statusLines() -> [String] {
        let info = shared.mutex.withLock { $0.hudInfo(now: monotonicNow()) }
        return hudLines(info)
    }

    func saveSettings() {
        let bias = tracking.snapshot().gyroBias
        let settings = shared.mutex.withLock { state in
            state.viewport.store(into: &state.settings)
            state.settings.gyroBias = bias
            return state.settings
        }
        do {
            try settings.save()
        } catch {
            eprint("Failed to save settings: \(error)")
        }
    }

    func perform(_ command: ViewerCommand) {
        var persist = true
        var restartSource = false
        shared.mutex.withLock { state in
            switch command {
            case .recenter:
                state.recenter(tracking: tracking)
                persist = false
            case .resetView:
                state.drift.reset()
                state.viewport.reset(state.lastPose)
            case .toggleFreeze:
                state.viewport.toggleFreeze()
                persist = false
            case .zoomIn: state.viewport.zoomIn()
            case .zoomOut: state.viewport.zoomOut()
            case .setZoom(let index): state.viewport.zoomIndex = min(max(index, 0), zoomLevels.count - 1)
            case .increaseSensitivity: state.viewport.adjustSensitivity(sensitivityStep)
            case .decreaseSensitivity: state.viewport.adjustSensitivity(1 / sensitivityStep)
            case .resetSensitivity: state.viewport.resetSensitivity()
            case .cycleDeadzone: state.viewport.cycleDeadzone()
            case .setDeadzone(let index): state.viewport.setDeadzone(index)
            case .togglePrediction: state.settings.prediction.toggle()
            case .setProjection(let projection): state.viewport.projection = projection
            case .toggleRoll: state.viewport.followsRoll.toggle()
            case .setEdge(let edge): state.viewport.edge = edge
            case .toggleFollowCursor: state.settings.followCursor.toggle()
            case .toggleStatus: state.settings.overlayVisible.toggle()
            case .calibrate:
                tracking.calibrate()
                persist = false
            case .pan(let dx, let dy):
                state.viewport.pan(dx: dx, dy: dy)
                persist = false
            case .setSource(let choice):
                state.settings.source = choice.source
                if let size = choice.size {
                    state.settings.virtualWidth = size.width
                    state.settings.virtualHeight = size.height
                }
                restartSource = true
            case .toggleSource:
                state.settings.source = state.settings.source == .mirror ? .virtual : .mirror
                restartSource = true
            }
        }
        if persist {
            saveSettings()
        }
        if restartSource {
            startSource()
        }
    }

    private func screensChanged() {
        window.place()
        let bounds = sourceDisplayID.map(CGDisplayBounds)
        shared.mutex.withLock { $0.sourceBounds = bounds }
    }

    // MARK: Source

    private func startSource() {
        sourceTask?.cancel()
        sourceTask = Task { await switchSource() }
    }

    private func setSource(description: String, displayID: CGDirectDisplayID?) {
        sourceDisplayID = displayID
        let bounds = displayID.map(CGDisplayBounds)
        shared.mutex.withLock { state in
            state.sourceDescription = description
            state.sourceBounds = bounds
        }
    }

    private func switchSource() async {
        await capture.stop()
        setSource(description: "SOURCE STARTING", displayID: nil)
        let settings = shared.mutex.withLock { $0.settings }
        switch settings.source {
        case .mirror:
            virtualScreen = nil
            await startCapture(displayID: Displays.mirrorSource(), description: "MIRROR MAIN DISPLAY")
        case .virtual:
            let requested = (width: settings.virtualWidth, height: settings.virtualHeight)
            if let existing = virtualScreen, existing.width != requested.width || existing.height != requested.height {
                virtualScreen = nil
            }
            if virtualScreen == nil {
                setSource(description: "CREATING VIRTUAL SCREEN \(requested.width)X\(requested.height)", displayID: nil)
                virtualScreen = VirtualScreen(width: requested.width, height: requested.height)
            }
            guard let screen = virtualScreen,
                let size = await ScreenCapture.shareableSize(of: screen.displayID, timeout: virtualScreenTimeout)
            else {
                virtualScreen = nil
                await startCapture(
                    displayID: Displays.mirrorSource(),
                    description: "VIRTUAL SCREEN \(requested.width)X\(requested.height) DID NOT COME ONLINE - MIRRORING")
                return
            }
            // macOS sometimes brings a virtual screen up smaller than asked.
            let shrunk = size != requested ? " (ASKED FOR \(requested.width)X\(requested.height))" : ""
            await startCapture(
                displayID: screen.displayID, pixelSize: size,
                description: "VIRTUAL SCREEN \(size.width)X\(size.height)\(shrunk)")
        }
    }

    private func startCapture(
        displayID: CGDirectDisplayID, pixelSize: (width: Int, height: Int)? = nil, description: String
    ) async {
        do {
            try await capture.start(
                displayID: displayID, pixelSize: pixelSize ?? Displays.pixelSize(of: displayID), fps: captureFps,
                excludedWindowID: window.windowID)
            setSource(description: description, displayID: displayID)
        } catch {
            let permission = CGPreflightScreenCaptureAccess() ? "" : " - ALLOW SCREEN RECORDING AND RELAUNCH"
            setSource(
                description: "CAPTURE FAILED: \(error.localizedDescription.uppercased())\(permission)", displayID: nil)
            eprint("Screen capture failed: \(error)")
        }
    }

    // MARK: Keys

    private func handleKey(_ event: NSEvent) {
        switch Int(event.keyCode) {
        case kVK_RightArrow: perform(.pan(dx: 96, dy: 0))
        case kVK_LeftArrow: perform(.pan(dx: -96, dy: 0))
        case kVK_UpArrow: perform(.pan(dx: 0, dy: -72))
        case kVK_DownArrow: perform(.pan(dx: 0, dy: 72))
        case kVK_Space: perform(.calibrate)
        case kVK_Escape: NSApp.terminate(nil)
        default: handleCharacter(event.charactersIgnoringModifiers?.lowercased() ?? "")
        }
    }

    private func handleCharacter(_ character: String) {
        switch character {
        case "c": perform(.recenter)
        case "r": perform(.resetView)
        case "f": perform(.toggleFreeze)
        case "d": perform(.cycleDeadzone)
        case "p": perform(.togglePrediction)
        case "=", "+": perform(.zoomIn)
        case "-", "_": perform(.zoomOut)
        case ".": perform(.increaseSensitivity)
        case ",": perform(.decreaseSensitivity)
        case "v": perform(.toggleSource)
        default: break
        }
    }
}
