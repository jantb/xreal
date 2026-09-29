import AppKit
import Carbon.HIToolbox
import Synchronization
import simd
import XrealCore

// As often as the glasses show a new frame; more is capture work for
// nothing.
private let captureFps = 90
// Parts of the canvas out of view still update this often, so they are
// fresh enough when the head turns to them.
private let outOfViewFps = 10
// Captures this far outside the view, as a fraction of it, count as in view,
// so a quick turn finds them already at the full rate.
private let inViewMargin: Float = 1
// A capture stays at the full rate this long after it was last in view.
// Changing a capture's rate makes the capture hitch for a moment, so a fast
// pan back and forth should not change it at every tile edge.
private let inViewHold = 1.5  // seconds
private let captureRateInterval = 0.25
// The display arrangement churns for a moment when the lid opens or closes.
private let displaysSettleTime = 1.5
// Windows brought back from behind the glasses view are staggered this much.
private let gatherStep: CGFloat = 40
// The glasses show a frame about 7 ms after it arrives over DisplayPort.
private let glassesDisplayDelay = 0.007
// The glasses light their rows top to bottom over about this long after the
// top one (Breezy Desktop uses 8 ms for the Air series).
private let scanoutTime = 0.008
private let virtualScreenTimeout = 10.0
// The glasses' refresh rate side by side, which the canvas follows so
// macOS draws it in step with them.
let glassesRefreshRate = 90.0
// The canvas let go this close to level is set level, so it is easy to
// straighten; tilted further it keeps its tilt.
private let levelSnapTilt: Float = 2 * .pi / 180  // rad
// Each step of bringing the canvas closer or pushing it away.
private let distanceStep: Float = 1.1
/// The range of the viewing distance slider, in metres: how far away the
/// canvas at distance 1 is.
let minViewingDistance: Float = 0.5
let maxViewingDistance: Float = 20

/// The range of the curve slider, as multiples of the canvas's distance: the
/// smaller the radius, the stronger the curve.
let minCurveRadius: Float = 0.5
let maxCurveRadius: Float = 5
// How long the canvas stays outlined after it was moved closer or away.
private let outlineTime = 1.0
// macOS nudges displays apart after an arrangement is applied; they are
// where they stay this long after.
private let arrangementSettleTime = 2.0

/// Virtual screen sizes that macOS 27 creates as asked, smallest first: the
/// sizes the canvas steps through when resized. Nearby standard sizes such
/// as 3840 × 2160, 5120 × 2880 or 5760 × 2160 are refused or come up
/// smaller, and so are 5752 × 3240 and 5752 × 4320. 7672 × 2160 wraps about
/// 170° at the glasses' pixel density; 5752 × 2880 about 125°, and a third
/// taller; 7672 × 4320 is as wide and twice as tall.
let canvasSizes: [(width: Int, height: Int)] = [
    (1920, 1080), (2880, 1620), (5120, 1440), (3832, 2160), (5752, 2160), (5752, 2880), (7672, 2160), (7672, 4320),
]

/// What the glasses show, or why they show nothing.
enum SourceStatus: Equatable, Sendable {
    case starting
    case noGlasses
    case creatingCanvas
    /// macOS would not create the canvas at its size.
    case refused
    case live(width: Int, height: Int)
    /// The canvas exists but did not come online or could not be captured.
    case failed(String)

    /// The status as the diagnostics show it.
    var diagnosticsDescription: String {
        switch self {
        case .starting: "Source starting"
        case .noGlasses: "Glasses not connected"
        case .creatingCanvas: "Creating the canvas"
        case .refused: "macOS refused the canvas, try another size"
        case .live(let width, let height): "Canvas \(width)×\(height)"
        case .failed(let problem): problem
        }
    }
}

enum ViewerCommand {
    case recenter
    /// Puts the canvas back straight ahead, level and at the glasses' own
    /// pixel density, and recenters.
    case resetView
    case togglePrediction
    case toggleRoll
    case toggleFollowCursor
    case toggleDiagnostics
    case setLatencyTrim(Float)
    case setCurveRadius(Float)
    case toggleLensCorrection
    case setDepthScale(Float)
    case calibrate
    case toggleCurved
    /// Picks up (true) or lets go of (false) the canvas, if looked at.
    case grab(Bool)
    /// Brings the canvas closer (true) or pushes it away (false).
    case moveCanvas(closer: Bool)
    /// Makes the canvas larger (true) or smaller.
    case resizeCanvas(bigger: Bool)
    case setCanvasSize(width: Int, height: Int)
    case window(WindowCommand)
}

/// What can be done to windows, for `ViewerCommand.window`.
enum WindowCommand {
    /// Moves the focused window to where the viewer is looking.
    case moveToGaze
    /// Fits the focused window into the zone the viewer is looking at.
    case fitToZone
    case movePointerToGaze
    /// Brings back windows hidden behind the glasses view.
    case gather
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
    /// Where the glasses' own display is in the arrangement, kept free of
    /// the mouse.
    var cursorFence: CGRect?
    var source = SourceStatus.starting
    /// The canvas's display in global coordinates, to find the cursor on it.
    var canvasBounds: CGRect?
    /// One per capture tile of the canvas, left to right.
    var captures: [LatestFrame] = []
    /// The canvas while carried by the head, and how far away it is.
    var grab: ScreenGrab?
    var grabDistance: Float = 1
    /// The pixel of the canvas looked at in the latest frame, if any.
    var gaze: SIMD2<Float>?
    /// For each capture, whether it was in view or nearly in the latest frame.
    var capturesInView: [Bool] = []
    var outlineUntil = 0.0

    // The latest frame, for the status lines.
    var stats = RenderStats(now: monotonicNow())
    var snapshot: TrackingSnapshot
    var sourceSize: (width: Int, height: Int)?
    var output = (width: 0, height: 0)
    var newFrame = false

    init(settings: Settings) {
        self.settings = settings
        viewport = ViewportController(settings: settings)
        snapshot = TrackingSnapshot(gyroBias: settings.gyroBias, biasSlope: settings.gyroBiasSlope)
    }

    mutating func recenter(tracking: Tracking) {
        let observation = drift.observeRecenter(now: monotonicNow(), yaw: lastPose.yaw)
        if case .learned(_, let correction) = observation {
            tracking.correctYawDrift(correction)
        }
        lastDrift = observation
        viewport.recenter(lastPose)
    }

    /// Picks up the canvas if it is being looked at. Returns false if not.
    mutating func startGrab() -> Bool {
        guard grab == nil, gaze != nil else { return false }
        grab = ScreenGrab(placement: settings.canvas.placement, headRotation: viewport.headRotation)
        grabDistance = settings.canvas.placement.distance
        return true
    }

    /// Lets go of the carried canvas where it is now. Returns false if it
    /// was not carried.
    mutating func endGrab() -> Bool {
        guard let grab else { return false }
        var placement = grab.placement(headRotation: viewport.headRotation, distance: grabDistance)
        if abs(placement.tilt) < levelSnapTilt {
            placement.tilt = 0
        }
        settings.canvas.placement = placement
        self.grab = nil
        return true
    }

    mutating func moveCanvas(closer: Bool, now: Double) {
        let factor = closer ? 1 / distanceStep : distanceStep
        if grab != nil {
            grabDistance = ScreenPlacement(direction: SIMD3(0, 0, -1), distance: grabDistance * factor).distance
        } else {
            settings.canvas.placement = settings.canvas.placement.movedAway(by: factor)
        }
        outlineUntil = now + outlineTime
    }

    /// Gives the canvas the next larger or smaller size in `canvasSizes`.
    /// Returns whether it changed, so the canvas is created again at its
    /// new size.
    mutating func resizeCanvas(bigger: Bool, now: Double) -> Bool {
        let area = settings.canvas.width * settings.canvas.height
        let size =
            bigger
            ? canvasSizes.first { $0.width * $0.height > area } : canvasSizes.last { $0.width * $0.height < area }
        guard let size else { return false }
        return setCanvasSize(width: size.width, height: size.height, now: now)
    }

    /// Returns whether the size changed, so the canvas is created again.
    mutating func setCanvasSize(width: Int, height: Int, now: Double) -> Bool {
        guard width != settings.canvas.width || height != settings.canvas.height else { return false }
        settings.canvas.width = width
        settings.canvas.height = height
        grab = nil
        gaze = nil
        outlineUntil = now + outlineTime
        return true
    }

    /// Moves on to the frame that reaches the display at `presentingAt`, in
    /// `monotonicNow` time. `frameSizes` holds the size of the latest frame
    /// of each capture, and `cursor` is the mouse in global coordinates.
    /// Returns what to draw, nil until the glasses show side by side, and
    /// whether the gyro bias changed and should be saved.
    mutating func advance(
        now: Double, dt: Float, presentingAt: Double, snapshot: TrackingSnapshot, captureGeneration: UInt64,
        newFrame: Bool, frameSizes: [(width: Int, height: Int)?], output: (width: Int, height: Int),
        cursor: CGPoint?
    ) -> (room: RoomView?, biasChanged: Bool) {
        stats.tick(now: now, captureGeneration: captureGeneration)
        self.snapshot = snapshot
        self.newFrame = newFrame
        self.sourceSize = frameSizes.first ?? nil
        self.output = output

        let biasChanged = snapshot.biasRevision != biasRevisionSaved
        biasRevisionSaved = snapshot.biasRevision
        let lead = max(presentingAt - now, 0) + glassesDisplayDelay + Double(settings.latencyTrimMs) / 1000
        let pose = settings.prediction ? snapshot.predict(now: now, lead: lead) : snapshot.pose
        lastPose = pose
        if snapshot.session != trackingSession {
            trackingSession = snapshot.session
            drift.reset()
            viewport.recenter(pose)
        }
        viewport.track(pose: pose)
        var room = roomView(now: now, dt: dt, output: output, frameSizes: frameSizes, cursor: cursor)
        if settings.prediction {
            let end = snapshot.predict(now: now, lead: lead + scanoutTime)
            let turn = SIMD3(wrapAngle(end.yaw - pose.yaw), end.pitch - pose.pitch, wrapAngle(end.roll - pose.roll))
            room.scanEndRotation = viewport.headRotation(advancedBy: turn)
        }
        // Until the glasses switch to side by side, the output is still one
        // view wide.
        return (output.width >= 3 * output.height ? room : nil, biasChanged)
    }

    private mutating func roomView(
        now: Double, dt: Float, output: (width: Int, height: Int), frameSizes: [(width: Int, height: Int)?],
        cursor: CGPoint?
    ) -> RoomView {
        let rotation = viewport.headRotation
        if let grab {
            settings.canvas.placement = grab.placement(headRotation: rotation, distance: grabDistance)
        }
        let (canvas, curveRadius) = (settings.canvas, settings.curveRadius)
        gaze = gazeTarget(rotation * SIMD3(0, 0, -1), on: canvas, curveRadius: curveRadius)
        var room = viewport.roomView(
            canvas: canvas, outputWidth: output.width / 2, outputHeight: output.height,
            highlighted: grab != nil || now < outlineUntil, curveRadius: curveRadius)
        // Zoom out while the mouse moves where it cannot be seen; behind the
        // viewer no zoom would show it.
        let cursorPoint = cursor.flatMap(roomPoint(ofCursor:)).flatMap { room.isAhead($0) ? $0 : nil }
        let scale = cursorFollow.update(
            cursor: cursor.map { SIMD2(Float($0.x), Float($0.y)) }.flatMap { cursorPoint == nil ? nil : $0 },
            enabled: settings.followCursor, now: now, dt: dt
        ) { scale, margin in
            cursorPoint.map { room.shows($0, scale: scale, margin: margin) } ?? false
        }
        room.tanHalfFov /= scale
        // Each eye as the glasses' calibration describes it, zoomed out with
        // the rest of the view.
        let optics = snapshot.display ?? .nominal
        room.eyes = [optics.left, optics.right].map { eye in
            var eye = eye
            eye.position /= settings.metresPerRoomUnit
            eye.focal *= scale
            if !settings.lensCorrection {
                eye.distortion = nil
            }
            return eye
        }
        capturesInView = Array(repeating: false, count: frameSizes.count)
        for panel in room.panels where panel.source < frameSizes.count && room.shows(panel, margin: inViewMargin) {
            capturesInView[panel.source] = true
        }
        // A canvas still starting up has nothing to show yet.
        room.panels.removeAll { $0.source >= frameSizes.count || frameSizes[$0.source] == nil }
        return room
    }

    /// Where the mouse at `point`, in global coordinates, is in the room, if
    /// it is on the canvas.
    private func roomPoint(ofCursor point: CGPoint) -> SIMD3<Float>? {
        guard let bounds = canvasBounds, bounds.contains(point) else { return nil }
        let canvas = settings.canvas
        let pixel = SIMD2(
            Float((point.x - bounds.minX) / bounds.width) * Float(canvas.width),
            Float((point.y - bounds.minY) / bounds.height) * Float(canvas.height))
        return canvas.roomPoint(ofPixel: pixel, curveRadius: settings.curveRadius)
    }

    func hudInfo(now: Double) -> HudInfo {
        HudInfo(
            gaze: gaze, source: sourceSize, sourceDescription: source.diagnosticsDescription, output: output,
            newFrame: newFrame, stats: stats, tracking: snapshot, pose: lastPose, prediction: settings.prediction,
            lastDrift: lastDrift, now: now)
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
    private let renderer: Renderer
    /// Set before the first frame; called on the display link thread.
    var onBiasChanged: @Sendable () -> Void = {}
    // Touched only on the display link thread.
    private var sources: [ObjectIdentifier] = []
    private var frames: [CapturedFrame?] = []
    private var generations: [UInt64] = []
    private var lastRenderAt = monotonicNow()
    private var lastFreeCursor: CGPoint?

    init(shared: SharedState, tracking: Tracking, renderer: Renderer) {
        self.shared = shared
        self.tracking = tracking
        self.renderer = renderer
    }

    func render(to drawable: CAMetalDrawable, presentingAt: Double) {
        let now = monotonicNow()
        let dt = Float(min(max(now - lastRenderAt, 0), 0.1))
        lastRenderAt = now

        let (captures, fence) = shared.mutex.withLock { ($0.captures, $0.cursorFence) }
        let ids = captures.map(ObjectIdentifier.init)
        if ids != sources {
            sources = ids
            frames = Array(repeating: nil, count: captures.count)
            generations = Array(repeating: 0, count: captures.count)
        }
        var newFrame = false
        var captureGeneration: UInt64 = 0
        for (index, capture) in captures.enumerated() {
            let current = capture.current()
            captureGeneration &+= current.generation
            if current.generation != generations[index] {
                frames[index] = current.frame
                generations[index] = current.generation
                newFrame = true
            }
        }
        let cursor = CGEvent(source: nil)?.location
        if let fence, let cursor {
            // The glasses' own display sits behind this view; keep the mouse
            // off it.
            if !fence.contains(cursor) {
                lastFreeCursor = cursor
            } else if let free = lastFreeCursor {
                CGWarpMouseCursorPosition(free)
            }
        }
        let output = (width: drawable.texture.width, height: drawable.texture.height)
        let frameSizes = frames.map { frame in frame.map { (width: $0.width, height: $0.height) } }

        // Sample the pose as late as possible, just before building the frame.
        let snapshot = tracking.snapshot()
        let result = shared.mutex.withLock { state in
            state.advance(
                now: now, dt: dt, presentingAt: presentingAt, snapshot: snapshot,
                captureGeneration: captureGeneration, newFrame: newFrame, frameSizes: frameSizes, output: output,
                cursor: cursor)
        }
        renderer.draw(to: drawable, frames: frames, room: result.room)
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
    private let device: MTLDevice
    private let window: GlassesWindow
    private let displayLink: DisplayLinkThread
    /// One per capture tile of the canvas, left to right.
    private var captures: [ScreenCapture] = [] {
        didSet { lastInView = Array(repeating: monotonicNow(), count: captures.count) }
    }
    /// When each capture was last in view, for `inViewHold`.
    private var lastInView: [Double] = []
    /// Whether the glasses were connected at the last display change.
    private var glassesConnected = Displays.glassesDisplay() != nil
    private var canvasScreen: VirtualScreen? {
        didSet { updateCanvasBounds() }
    }
    /// The real displays when the canvas was last arranged, to arrange it
    /// again when one comes or goes, as when the lid opens or closes.
    private var arrangedDisplays: Set<CGDirectDisplayID> = []
    private var sourceTask: Task<Void, Never>?
    private var hotKeys: [GlobalHotKey] = []
    private var settleTask: Task<Void, Never>?
    private var displaysTask: Task<Void, Never>?
    private var rateTimer: Timer?

    init(settings: Settings) throws {
        // The viewer needs the glasses as a display of their own.
        Displays.unmirrorGlasses()
        let shared = SharedState(ViewerState(settings: settings))
        let tracking = Tracking(
            initialBias: settings.gyroBias, biasSlope: settings.gyroBiasSlope, displayMode: .highRefreshRateSBS)
        let renderer = try Renderer()
        self.shared = shared
        self.tracking = tracking
        device = renderer.device

        let frameLoop = FrameLoop(shared: shared, tracking: tracking, renderer: renderer)
        displayLink = DisplayLinkThread { drawable, presentingAt in
            frameLoop.render(to: drawable, presentingAt: presentingAt)
        }
        window = GlassesWindow(device: renderer.device, displayLink: displayLink)
        frameLoop.onBiasChanged = { [weak self] in Task { @MainActor in self?.saveSettings() } }

        window.view.onKey = { [unowned self] event in handleKey(event) }
        let modifiers = controlKey | optionKey | cmdKey
        func hotKey(_ keyCode: Int, _ command: ViewerCommand) -> GlobalHotKey {
            GlobalHotKey(keyCode: keyCode, modifiers: modifiers) { [unowned self] in perform(command) }
        }
        hotKeys = [
            hotKey(kVK_ANSI_C, .recenter),
            GlobalHotKey(
                keyCode: kVK_ANSI_G, modifiers: modifiers, onPress: { [unowned self] in perform(.grab(true)) },
                onRelease: { [unowned self] in perform(.grab(false)) }),
            hotKey(kVK_ANSI_Equal, .moveCanvas(closer: true)),
            hotKey(kVK_ANSI_Minus, .moveCanvas(closer: false)),
            hotKey(kVK_ANSI_RightBracket, .resizeCanvas(bigger: true)),
            hotKey(kVK_ANSI_LeftBracket, .resizeCanvas(bigger: false)),
            hotKey(kVK_ANSI_W, .window(.moveToGaze)),
            hotKey(kVK_ANSI_F, .window(.fitToZone)),
            hotKey(kVK_ANSI_M, .window(.movePointerToGaze)),
        ]
        NotificationCenter.default.addObserver(
            forName: NSApplication.didChangeScreenParametersNotification, object: nil, queue: .main
        ) { [weak self] _ in
            MainActor.assumeIsolated { self?.screensChanged() }
        }
        let timer = Timer(timeInterval: captureRateInterval, repeats: true) { [weak self] _ in
            MainActor.assumeIsolated { self?.updateCaptureRates() }
        }
        RunLoop.main.add(timer, forMode: .common)
        rateTimer = timer

        // The window shows once the source starts, if the glasses are there.
        updateCursorFence()
        startSource()
    }

    /// The settings and view as they are now, for the menu.
    var current: (settings: Settings, viewport: ViewportController) {
        shared.mutex.withLock { ($0.settings, $0.viewport) }
    }

    /// What the status lines are made from.
    func statusInfo() -> (info: HudInfo, source: SourceStatus) {
        shared.mutex.withLock { ($0.hudInfo(now: monotonicNow()), $0.source) }
    }

    /// Leaves the glasses showing their own picture, as before the viewer ran.
    func restoreGlasses() {
        tracking.restoreDisplayMode()
    }

    func saveSettings() {
        let bias = tracking.snapshot().thermalBias
        let settings = shared.mutex.withLock { state in
            state.viewport.store(into: &state.settings)
            state.settings.gyroBias = bias.reference
            state.settings.gyroBiasSlope = bias.slope
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
        if case .window(let windowCommand) = command {
            controlWindows(windowCommand)
            return
        }
        shared.mutex.withLock { state in
            switch command {
            case .recenter:
                state.recenter(tracking: tracking)
                persist = false
            case .resetView:
                state.grab = nil
                state.settings.canvas.placement = .straightAhead
                state.drift.reset()
                state.viewport.recenter(state.lastPose)
            case .togglePrediction: state.settings.prediction.toggle()
            case .toggleRoll: state.viewport.followsRoll.toggle()
            case .toggleFollowCursor: state.settings.followCursor.toggle()
            case .toggleDiagnostics: state.settings.diagnosticsVisible.toggle()
            case .setLatencyTrim(let ms): state.settings.latencyTrimMs = min(max(ms, 0), maxLatencyTrimMs)
            case .toggleLensCorrection: state.settings.lensCorrection.toggle()
            case .setDepthScale(let metres): state.settings.metresPerRoomUnit = metres
            case .setCurveRadius(let radius): state.settings.curveRadius = radius
            case .calibrate:
                tracking.calibrate()
                persist = false
            case .toggleCurved: state.settings.canvas.curved.toggle()
            case .grab(let pickUp):
                // Saved once it is let go.
                persist = pickUp ? false : state.endGrab()
                if pickUp {
                    _ = state.startGrab()
                }
            case .moveCanvas(let closer):
                state.moveCanvas(closer: closer, now: monotonicNow())
            case .resizeCanvas(let bigger):
                restartSource = state.resizeCanvas(bigger: bigger, now: monotonicNow())
                persist = restartSource
            case .setCanvasSize(let width, let height):
                restartSource = state.setCanvasSize(width: width, height: height, now: monotonicNow())
                persist = restartSource
            case .window:
                break
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
        // The next change notification carries on from here.
        if Displays.unmirrorGlasses() {
            eprint("Took the glasses out of mirroring")
            return
        }
        let connected = Displays.glassesDisplay() != nil
        if connected != glassesConnected {
            glassesConnected = connected
            eprint(connected ? "The glasses are connected" : "The glasses were disconnected")
            startSource()
        }
        window.place()
        updateCanvasBounds()
        updateCursorFence()
        displaysTask?.cancel()
        displaysTask = Task {
            try? await Task.sleep(for: .seconds(displaysSettleTime))
            guard !Task.isCancelled else { return }
            // The glasses drop off and come back while switching to side by
            // side; if they were gone just as the canvas was made, make it
            // now.
            if Displays.glassesDisplay() != nil, canvasScreen == nil {
                eprint("The glasses are back without a canvas")
                startSource()
            } else if Self.realDisplays() != arrangedDisplays {
                eprint("A real display came or went")
                arrangeCanvas()
            }
        }
    }

    /// The real displays in use, such as the laptop's own screen while its
    /// lid is open.
    private static func realDisplays() -> Set<CGDirectDisplayID> {
        Set(Displays.active().filter(Displays.isReal))
    }

    private func updateCursorFence() {
        let fence = Displays.glassesDisplay().map(CGDisplayBounds)
        shared.mutex.withLock { $0.cursorFence = fence }
    }

    /// Arranges the displays around the canvas. The canvas becomes the main
    /// display, for the menu bar and Dock; real displays go in a row below
    /// it, where a laptop sits below the eyes; and the glasses' own display
    /// is tucked into a corner, out of the way of the mouse. Once macOS has
    /// settled the arrangement, windows left on the glasses' own display,
    /// as from the laptop's screen when the lid closes, are brought back.
    private func arrangeCanvas() {
        guard let canvasScreen else { return }
        arrangedDisplays = Self.realDisplays()
        var config: CGDisplayConfigRef?
        guard CGBeginDisplayConfiguration(&config) == .success, let config else { return }
        // macOS remembers mirroring per set of displays and may clone a real
        // display onto the canvas as it appears; each stays its own.
        let ours = Set([canvasScreen.displayID] + [Displays.glassesDisplay()].compactMap { $0 })
        var unmirrored: Set<CGDirectDisplayID> = []
        for display in Displays.online() {
            let original = CGDisplayMirrorsDisplay(display)
            if original != kCGNullDirectDisplay, ours.contains(original) || ours.contains(display) {
                CGConfigureDisplayMirrorOfDisplay(config, display, kCGNullDirectDisplay)
                unmirrored.insert(display)
            }
        }
        CGConfigureDisplayOrigin(config, canvasScreen.displayID, 0, 0)
        var arranged = CGRect(x: 0, y: 0, width: canvasScreen.width, height: canvasScreen.height)
        // A display mirroring another follows it, unless it was just taken
        // out of mirroring one of ours. Asleep and switched-off displays are
        // left where they are.
        let reals = Displays.online().filter { display in
            Displays.isReal(display) && CGDisplayIsAsleep(display) == 0
                && (CGDisplayIsActive(display) != 0 || unmirrored.contains(display))
                && (CGDisplayMirrorsDisplay(display) == kCGNullDirectDisplay || unmirrored.contains(display))
        }
        let sizes = reals.map { CGDisplayBounds($0).size }
        var x = (arranged.midX - sizes.reduce(0) { $0 + $1.width } / 2).rounded()
        let y = arranged.maxY
        for (real, size) in zip(reals, sizes) {
            CGConfigureDisplayOrigin(config, real, Int32(x), Int32(y))
            arranged = arranged.union(CGRect(origin: CGPoint(x: x, y: y), size: size))
            x += size.width
        }
        if let glasses = Displays.glassesDisplay() {
            CGConfigureDisplayOrigin(config, glasses, Int32(arranged.maxX), Int32(arranged.maxY))
        }
        let result = CGCompleteDisplayConfiguration(config, .forSession)
        if result != .success {
            eprint("Could not arrange the canvas: \(result.rawValue)")
        }
        settleTask?.cancel()
        settleTask = Task {
            try? await Task.sleep(for: .seconds(arrangementSettleTime))
            guard !Task.isCancelled else { return }
            updateCanvasBounds()
            updateCursorFence()
            if WindowControl.allowed(prompt: false) {
                gatherWindows()
            }
        }
    }

    /// Tells the frame loop where the canvas is in macOS, to find the mouse
    /// on it.
    private func updateCanvasBounds() {
        let bounds = canvasScreen.map { CGDisplayBounds($0.displayID) }
        shared.mutex.withLock { $0.canvasBounds = bounds }
    }

    /// Captures of parts of the canvas out of view update less often.
    private func updateCaptureRates() {
        let inView = shared.mutex.withLock { $0.capturesInView }
        let now = monotonicNow()
        for (index, capture) in captures.enumerated() where capture.fps != 0 {
            if index >= inView.count || inView[index] {
                lastInView[index] = now
            }
            let fps = now - lastInView[index] < inViewHold ? captureFps : outOfViewFps
            if capture.fps != fps {
                Task { await capture.setFps(fps) }
            }
        }
    }

    // MARK: Windows

    /// Where the viewer is looking, in global points, with the bounds of the
    /// canvas and the zone of it looked at.
    private func gazeInArrangement() -> (point: CGPoint, display: CGRect, zone: CGRect)? {
        let (gaze, canvas) = shared.mutex.withLock { ($0.gaze, $0.settings.canvas) }
        guard let gaze, let canvasScreen else { return nil }
        let bounds = CGDisplayBounds(canvasScreen.displayID)
        let scale = CGSize(width: bounds.width / CGFloat(canvas.width), height: bounds.height / CGFloat(canvas.height))
        let zone = zone(around: gaze, width: canvas.width, height: canvas.height)
        return (
            CGPoint(x: bounds.minX + CGFloat(gaze.x) * scale.width, y: bounds.minY + CGFloat(gaze.y) * scale.height),
            bounds,
            CGRect(
                x: bounds.minX + zone.minX * scale.width, y: bounds.minY + zone.minY * scale.height,
                width: zone.width * scale.width, height: zone.height * scale.height)
        )
    }

    private func controlWindows(_ command: WindowCommand) {
        let gaze = gazeInArrangement()
        if case .movePointerToGaze = command {
            guard let gaze else { return }
            CGWarpMouseCursorPosition(gaze.point)
            CGAssociateMouseAndMouseCursorPosition(1)
            return
        }
        guard WindowControl.allowed(prompt: true) else { return }
        switch command {
        case .moveToGaze:
            guard let gaze, let window = WindowControl.focusedWindow(), let frame = WindowControl.frame(of: window)
            else { return }
            WindowControl.move(window, to: windowOrigin(size: frame.size, centeredOn: gaze.point, within: gaze.display))
        case .fitToZone:
            guard let gaze, let window = WindowControl.focusedWindow() else { return }
            WindowControl.setFrame(window, to: gaze.zone)
        case .gather:
            gatherWindows()
        case .movePointerToGaze:
            break
        }
    }

    /// Moves every window sitting on the glasses' own display, where it is
    /// hidden behind the glasses view, onto the middle of the canvas.
    private func gatherWindows() {
        guard let glasses = Displays.glassesDisplay(), let canvasScreen else { return }
        let target = CGDisplayBounds(canvasScreen.displayID)
        let hidden = CGDisplayBounds(glasses)
        var step: CGFloat = 0
        for window in WindowControl.allWindows() {
            guard let frame = WindowControl.frame(of: window), hidden.contains(CGPoint(x: frame.midX, y: frame.midY))
            else { continue }
            let middle = CGPoint(x: target.midX + step, y: target.midY + step)
            WindowControl.move(window, to: windowOrigin(size: frame.size, centeredOn: middle, within: target))
            step += gatherStep
        }
    }

    // MARK: Source

    private func startSource() {
        sourceTask?.cancel()
        sourceTask = Task { await switchSource() }
    }

    private func setSource(_ status: SourceStatus, captures: [ScreenCapture]) {
        self.captures = captures
        let latest = captures.map(\.latest)
        shared.mutex.withLock { state in
            state.source = status
            state.captures = latest
            state.capturesInView = []
        }
    }

    private func switchSource() async {
        for capture in captures {
            await capture.stop()
        }
        // Without the glasses there is nothing to show: no canvas, no
        // capture, no window.
        guard Displays.glassesDisplay() != nil else {
            canvasScreen = nil
            setSource(.noGlasses, captures: [])
            window.hide()
            return
        }
        window.show()
        let canvas = shared.mutex.withLock { $0.settings.canvas }
        setSource(.creatingCanvas, captures: [])
        // A canvas of the same size is kept, so it stays put.
        if canvasScreen.map({ $0.width != canvas.width || $0.height != canvas.height }) ?? true {
            canvasScreen = VirtualScreen(index: 0, width: canvas.width, height: canvas.height, refreshRate: glassesRefreshRate)
        }
        guard let screen = canvasScreen else {
            setSource(.refused, captures: [])
            return
        }
        let frame = await ScreenCapture.shareableFrame(of: screen.displayID, timeout: virtualScreenTimeout)
        guard !Task.isCancelled else { return }
        arrangeCanvas()

        var started: [ScreenCapture] = []
        var problems: [String] = []
        // A wide canvas is captured in tiles, so the parts out of view can
        // update less often.
        for columns in captureTiles(width: screen.width) {
            let tile = CGRect(x: columns.lowerBound, y: 0, width: columns.count, height: screen.height)
            let (capture, problem) = await startCapture(
                displayID: screen.displayID, pixelSize: (columns.count, screen.height),
                sourceRect: screen.width == columns.count ? nil : tile)
            started.append(capture)
            guard !Task.isCancelled else {
                // A newer source took over; these never reached it.
                for capture in started {
                    await capture.stop()
                }
                return
            }
            if let problem {
                problems.append(problem)
            }
        }
        if frame == nil {
            problems.append("The canvas did not come online")
        }
        setSource(
            problems.first.map(SourceStatus.failed) ?? .live(width: screen.width, height: screen.height),
            captures: started)
    }

    /// Starts capturing `displayID`, or the part `sourceRect` of it. The
    /// capture is returned even when it fails, so captures stay in step with
    /// the tiles; the second value then says what went wrong.
    private func startCapture(
        displayID: CGDirectDisplayID, pixelSize: (width: Int, height: Int), sourceRect: CGRect? = nil
    ) async -> (ScreenCapture, String?) {
        do {
            let capture = try ScreenCapture(device: device)
            do {
                try await capture.start(
                    displayID: displayID, pixelSize: pixelSize, sourceRect: sourceRect, fps: captureFps,
                    excludedWindowID: window.windowID)
                return (capture, nil)
            } catch {
                let permission = CGPreflightScreenCaptureAccess() ? "" : " - allow Screen Recording and relaunch"
                eprint("Screen capture failed: \(error)")
                return (capture, "Capture failed: \(error.localizedDescription)\(permission)")
            }
        } catch {
            fatalError("Could not create a Metal texture cache: \(error)")
        }
    }

    // MARK: Keys

    private func handleKey(_ event: NSEvent) {
        switch Int(event.keyCode) {
        case kVK_Space: perform(.calibrate)
        case kVK_Escape: NSApp.terminate(nil)
        default: handleCharacter(event.charactersIgnoringModifiers?.lowercased() ?? "")
        }
    }

    private func handleCharacter(_ character: String) {
        switch character {
        case "c": perform(.recenter)
        case "r": perform(.resetView)
        case "p": perform(.togglePrediction)
        default: break
        }
    }
}
