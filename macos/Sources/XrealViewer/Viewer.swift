import AppKit
import Carbon.HIToolbox
import Synchronization
import simd
import XrealCore

private let captureFps = 120
// Parts of the room out of view still update this often, so they are fresh
// enough when the head turns to them.
private let outOfViewFps = 10
// Captures this far outside the view, as a fraction of it, count as in view.
private let inViewMargin: Float = 0.5
private let captureRateInterval = 0.25
// The display arrangement churns for a moment when the lid opens or closes.
private let modeSettleTime = 1.5
// Windows brought back from behind the glasses view are staggered this much.
private let gatherStep: CGFloat = 40
// The glasses show a frame about 7 ms after it arrives over DisplayPort.
private let glassesDisplayDelay = 0.007
private let sensitivityStep: Float = 1.05
private let virtualScreenTimeout = 10.0
// Each step of bringing a screen closer or pushing it away.
private let distanceStep: Float = 1.1
// How long a screen stays outlined after it was moved closer or away.
private let outlineTime = 1.0
// macOS nudges displays apart after an arrangement is applied. Where they
// sit this long after is the baseline; later moves are the user's.
private let arrangementSettleTime = 2.0

/// Virtual screen sizes that macOS 27 creates as asked. Nearby standard
/// sizes such as 3840 × 2160, 5120 × 2880 or 5760 × 2160 are refused or come
/// up smaller.
let virtualScreenSizes: [(width: Int, height: Int)] = [(1920, 1080), (2880, 1620), (3832, 2160), (5120, 1440), (5752, 2160)]
/// Wide canvases to curve around the viewer. 7672 × 2160 wraps about 170° at
/// the glasses' pixel density; 7672 × 4320 is as wide and twice as tall.
let canvasSizes: [(width: Int, height: Int)] = [(7672, 2160), (7672, 4320)]

/// Something the glasses can show.
struct SourceChoice: Sendable {
    var title: String
    var source: CaptureSource

    static let all = [
        SourceChoice(title: "Mirror Main Display", source: .mirror),
        SourceChoice(title: "Virtual Screens", source: .virtual),
    ]

    func isSelected(in settings: Settings) -> Bool {
        source == settings.source
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
    case addScreen(width: Int, height: Int, curved: Bool)
    case removeScreen(Int)
    case toggleCurved(Int)
    case standardLayout
    /// Picks up (true) or lets go of (false) the screen being looked at.
    case grab(Bool)
    /// Brings the screen being looked at or carried closer (true) or pushes
    /// it away (false).
    case moveScreen(closer: Bool)
    /// Moves the focused window to where the viewer is looking.
    case moveWindowToGaze
    /// Fits the focused window into the zone the viewer is looking at.
    case fitWindowToZone
    case movePointerToGaze
    /// Brings back windows hidden behind the glasses view.
    case gatherWindows
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
    /// The glasses are the only display, as with the laptop lid closed:
    /// the room is all virtual screens, from `settings.glassesOnlyScreens`.
    var glassesOnly = false
    /// Where the glasses' own display is in the arrangement, kept free of
    /// the mouse while the glasses are the only display.
    var cursorFence: CGRect?
    var sourceDescription = "SOURCE STARTING"
    /// The captured display in global coordinates (points, top-left
    /// origin), to find the cursor on it. Mirror mode only.
    var sourceBounds: CGRect?
    /// One per captured display: the mirrored one, or each virtual screen in
    /// the order of `settings.screens`.
    var captures: [LatestFrame] = []
    /// The virtual screen being carried by the head, and how far away it is.
    var grab: ScreenGrab?
    var grabDistance: Float = 1
    /// The virtual screen looked at in the latest frame, and the pixel on it.
    var lookedAt: Int?
    var gaze: (index: Int, pixel: SIMD2<Float>)?
    /// For each capture, whether it was in view or nearly in the latest frame.
    var capturesInView: [Bool] = []
    var outlineUntil = 0.0

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

    /// The virtual screens of the current mode.
    var screens: [RoomScreen] {
        get { glassesOnly ? settings.glassesOnlyScreens : settings.screens }
        set {
            if glassesOnly {
                settings.glassesOnlyScreens = newValue
            } else {
                settings.screens = newValue
            }
        }
    }

    /// Whether the glasses show the virtual screens rather than a mirror.
    var showsRoom: Bool { glassesOnly || settings.source == .virtual }

    mutating func recenter(tracking: Tracking) {
        let observation = drift.observeRecenter(now: monotonicNow(), yaw: lastPose.yaw)
        if case .learned(_, let correction) = observation {
            tracking.correctYawDrift(correction)
        }
        lastDrift = observation
        viewport.recenter(lastPose)
    }

    /// Picks up the screen being looked at. Returns false if there is none.
    mutating func startGrab() -> Bool {
        guard showsRoom, grab == nil, let index = lookedAt,
            screens.indices.contains(index), let placement = screens[index].placement
        else { return false }
        grab = ScreenGrab(index: index, placement: placement, headRotation: viewport.headRotation)
        grabDistance = placement.distance
        return true
    }

    /// Lets go of the carried screen where it is now. Returns false if none
    /// was carried.
    mutating func endGrab() -> Bool {
        guard let grab else { return false }
        screens[grab.index].placement = grab.placement(headRotation: viewport.headRotation, distance: grabDistance)
        self.grab = nil
        return true
    }

    mutating func moveScreen(closer: Bool, now: Double) {
        let factor = closer ? 1 / distanceStep : distanceStep
        if grab != nil {
            grabDistance = ScreenPlacement(direction: SIMD3(0, 0, -1), distance: grabDistance * factor).distance
        } else if let index = lookedAt, screens.indices.contains(index), let placement = screens[index].placement {
            screens[index].placement = placement.movedAway(by: factor)
        } else {
            return
        }
        outlineUntil = now + outlineTime
    }

    /// Moves on to the frame that reaches the display at `presentingAt`, in
    /// `monotonicNow` time. `frameSizes` holds the size of the latest frame
    /// of each capture, and `cursor` is the mouse in global coordinates.
    /// Returns what to draw, and whether the gyro bias changed and should
    /// be saved.
    mutating func advance(
        now: Double, dt: Float, presentingAt: Double, snapshot: TrackingSnapshot, captureGeneration: UInt64,
        newFrame: Bool, frameSizes: [(width: Int, height: Int)?], output: (width: Int, height: Int),
        cursor: CGPoint?
    ) -> (geometry: ViewGeometry?, biasChanged: Bool) {
        stats.tick(now: now, captureGeneration: captureGeneration)
        self.snapshot = snapshot
        self.newFrame = newFrame
        self.sourceSize = frameSizes.first ?? nil
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

        if showsRoom {
            geometry = .room(roomView(now: now, output: output, frameSizes: frameSizes))
            return (geometry, biasChanged)
        }

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

    private mutating func roomView(
        now: Double, output: (width: Int, height: Int), frameSizes: [(width: Int, height: Int)?]
    ) -> RoomView {
        let rotation = viewport.headRotation
        var screens = screens
        if let grab {
            screens[grab.index].placement = grab.placement(headRotation: rotation, distance: grabDistance)
            self.screens = screens
        }
        gaze = gazeTarget(rotation * SIMD3(0, 0, -1), among: screens)
        lookedAt = grab?.index ?? gaze?.index
        let outlined = grab != nil || now < outlineUntil ? lookedAt : nil
        var room = viewport.roomView(
            screens: screens, outputWidth: output.width, outputHeight: output.height, highlighted: outlined)
        capturesInView = Array(repeating: false, count: frameSizes.count)
        for panel in room.panels where panel.source < frameSizes.count && room.shows(panel, margin: inViewMargin) {
            capturesInView[panel.source] = true
        }
        // Screens still starting up have nothing to show yet.
        room.panels.removeAll { $0.source >= frameSizes.count || frameSizes[$0.source] == nil }
        return room
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
            // on the virtual screens.
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
        renderer.draw(to: drawable, frames: frames, geometry: result.geometry)
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
    /// One per captured display, or per tile of each virtual screen in the
    /// order of the screens.
    private var captures: [ScreenCapture] = []
    private var virtualScreens: [VirtualScreen] = []
    private var sourceDisplayID: CGDirectDisplayID?
    private var sourceTask: Task<Void, Never>?
    private var hotKeys: [GlobalHotKey] = []
    /// Where the virtual screens sat once macOS settled the last
    /// arrangement; nil while it is settling.
    private var settledFrames: [CGRect]?
    private var settleTask: Task<Void, Never>?
    private var modeTask: Task<Void, Never>?
    private var rateTimer: Timer?

    init(settings: Settings) throws {
        var state = ViewerState(settings: settings)
        state.glassesOnly = Displays.glassesOnly(besides: [])
        let shared = SharedState(state)
        let tracking = Tracking(initialBias: settings.gyroBias)
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
            hotKey(kVK_ANSI_Equal, .moveScreen(closer: true)),
            hotKey(kVK_ANSI_Minus, .moveScreen(closer: false)),
            hotKey(kVK_ANSI_W, .moveWindowToGaze),
            hotKey(kVK_ANSI_F, .fitWindowToZone),
            hotKey(kVK_ANSI_M, .movePointerToGaze),
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

        window.show()
        updateCursorFence()
        startSource()
    }

    /// The settings and view as they are now, for the menu.
    var current: (settings: Settings, viewport: ViewportController, screens: [RoomScreen], glassesOnly: Bool) {
        shared.mutex.withLock { ($0.settings, $0.viewport, $0.screens, $0.glassesOnly) }
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
        var arrange = false
        var removedScreen: Int?
        switch command {
        case .moveWindowToGaze, .fitWindowToZone, .movePointerToGaze, .gatherWindows:
            controlWindows(command)
            return
        default:
            break
        }
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
                restartSource = true
            case .toggleSource:
                state.settings.source = state.settings.source == .mirror ? .virtual : .mirror
                restartSource = true
            case .addScreen(let width, let height, let curved):
                state.screens.append(RoomScreen(width: width, height: height, curved: curved))
                state.settings.source = .virtual
                restartSource = true
            case .removeScreen(let index):
                guard state.screens.count > 1, state.screens.indices.contains(index) else {
                    persist = false
                    return
                }
                state.grab = nil
                state.lookedAt = nil
                state.screens.remove(at: index)
                removedScreen = index
                restartSource = true
            case .toggleCurved(let index):
                guard state.screens.indices.contains(index) else { return }
                state.screens[index].curved.toggle()
            case .standardLayout:
                let ahead = state.glassesOnly ? CGRect.zero : CGDisplayBounds(Displays.mirrorSource())
                let frames =
                    state.glassesOnly
                    ? glassesOnlyArrangement(for: state.screens)
                    : standardArrangement(for: state.screens, around: ahead)
                for (index, frame) in frames.enumerated() {
                    state.screens[index].placement = ScreenPlacement(arrangedAt: frame, around: ahead)
                }
                arrange = true
            case .grab(let pickUp):
                // Letting go ends the move; the arrangement follows the room.
                arrange = pickUp ? false : state.endGrab()
                persist = arrange
                if pickUp {
                    _ = state.startGrab()
                }
            case .moveScreen(let closer):
                state.moveScreen(closer: closer, now: monotonicNow())
            case .moveWindowToGaze, .fitWindowToZone, .movePointerToGaze, .gatherWindows:
                break
            }
        }
        // The screens after a removed one keep their displays, and so their
        // place in the room and in macOS.
        if let removedScreen, virtualScreens.indices.contains(removedScreen) {
            virtualScreens.remove(at: removedScreen)
        }
        if arrange {
            arrangeVirtualScreens()
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
        updateCursorFence()
        followArrangement()
        modeTask?.cancel()
        modeTask = Task {
            try? await Task.sleep(for: .seconds(modeSettleTime))
            guard !Task.isCancelled else { return }
            updateMode()
        }
    }

    /// Switches between the screens around the real display and the
    /// glasses-only screens when the lid closes or opens.
    private func updateMode() {
        let glassesOnly = Displays.glassesOnly(besides: Set(virtualScreens.map(\.displayID)))
        let changed = shared.mutex.withLock { state -> Bool in
            guard state.glassesOnly != glassesOnly else { return false }
            state.glassesOnly = glassesOnly
            state.grab = nil
            state.lookedAt = nil
            state.gaze = nil
            return true
        }
        guard changed else { return }
        eprint(glassesOnly ? "The glasses are the only display" : "A real display is back")
        updateCursorFence()
        startSource()
    }

    private func updateCursorFence() {
        let glassesOnly = shared.mutex.withLock { $0.glassesOnly }
        let fence = glassesOnly ? Displays.glassesDisplay().map(CGDisplayBounds) : nil
        shared.mutex.withLock { $0.cursorFence = fence }
    }

    /// Where straight ahead is in the arrangement as it is now: the middle of
    /// the real display, or with the glasses alone, measured from where
    /// macOS has the first virtual screen.
    private func roomReference(screens: [RoomScreen], glassesOnly: Bool) -> CGRect {
        guard glassesOnly else { return CGDisplayBounds(Displays.mirrorSource()) }
        guard let display = virtualScreens.first, let screen = screens.first else { return .zero }
        return aheadReference(for: screen, arrangedAt: CGDisplayBounds(display.displayID).origin)
    }

    /// Moves virtual screens in the room to where they were dragged in
    /// System Settings > Displays > Arrange.
    private func followArrangement() {
        guard let settled = settledFrames else { return }
        let frames = virtualScreens.map { CGDisplayBounds($0.displayID) }
        guard frames.count == settled.count else { return }
        settledFrames = frames
        let (screens, glassesOnly) = shared.mutex.withLock { ($0.screens, $0.glassesOnly) }
        let ahead = roomReference(screens: screens, glassesOnly: glassesOnly)
        let moved = shared.mutex.withLock { state -> Bool in
            guard state.showsRoom, state.grab == nil, state.glassesOnly == glassesOnly else { return false }
            var moved = false
            for (index, frame) in frames.enumerated() where index < state.screens.count {
                guard frame != settled[index], !frame.isEmpty, let placement = state.screens[index].placement
                else { continue }
                state.screens[index].placement = ScreenPlacement(
                    direction: ScreenPlacement(arrangedAt: frame, around: ahead).direction,
                    distance: placement.distance)
                moved = true
            }
            return moved
        }
        if moved {
            saveSettings()
        }
    }

    /// Arranges the virtual screens in macOS the way they hang in the room,
    /// so the mouse crosses between displays where the eye expects. Around
    /// the real display, which becomes the main one again if it was not.
    /// With the glasses alone the first virtual screen becomes the main
    /// display, for the menu bar and Dock, and the glasses' own display is
    /// tucked into a corner, out of the way of the mouse.
    private func arrangeVirtualScreens() {
        let (screens, glassesOnly) = shared.mutex.withLock { ($0.screens, $0.glassesOnly) }
        var config: CGDisplayConfigRef?
        guard CGBeginDisplayConfiguration(&config) == .success, let config else { return }
        let ahead: CGRect
        if glassesOnly, let first = screens.first {
            ahead = aheadReference(for: first, arrangedAt: .zero)
        } else {
            let real = Displays.mirrorSource()
            var bounds = CGDisplayBounds(real)
            // Left over from the glasses alone, a virtual screen may still be
            // the main display.
            if !Displays.isReal(CGMainDisplayID()), !Displays.isGlasses(CGMainDisplayID()), Displays.isReal(real) {
                CGConfigureDisplayOrigin(config, real, 0, 0)
                bounds.origin = .zero
            }
            ahead = bounds
        }
        var arranged = CGRect.null
        for (screen, display) in zip(screens, virtualScreens) {
            guard let placement = screen.placement else { continue }
            let origin = placement.arrangedOrigin(width: screen.width, height: screen.height, around: ahead)
            CGConfigureDisplayOrigin(config, display.displayID, Int32(origin.x), Int32(origin.y))
            arranged = arranged.union(CGRect(origin: origin, size: CGSize(width: screen.width, height: screen.height)))
        }
        if glassesOnly, let glasses = Displays.glassesDisplay(), !arranged.isNull {
            CGConfigureDisplayOrigin(config, glasses, Int32(arranged.maxX), Int32(arranged.maxY))
        }
        let result = CGCompleteDisplayConfiguration(config, .forSession)
        if result != .success {
            eprint("Could not arrange the virtual screens: \(result.rawValue)")
        }
        settledFrames = nil
        settleTask?.cancel()
        settleTask = Task {
            try? await Task.sleep(for: .seconds(arrangementSettleTime))
            guard !Task.isCancelled else { return }
            settledFrames = virtualScreens.map { CGDisplayBounds($0.displayID) }
            updateCursorFence()
        }
    }

    /// Captures of screens out of view update less often; in mirror mode
    /// everything updates at the full rate.
    private func updateCaptureRates() {
        let (inView, showsRoom) = shared.mutex.withLock { ($0.capturesInView, $0.showsRoom) }
        for (index, capture) in captures.enumerated() where capture.fps != 0 {
            let seen = !showsRoom || index >= inView.count || inView[index]
            let fps = seen ? captureFps : outOfViewFps
            if capture.fps != fps {
                Task { await capture.setFps(fps) }
            }
        }
    }

    // MARK: Windows

    /// Where the viewer is looking, in global points, with the bounds of the
    /// virtual screen looked at and the zone of it looked at.
    private func gazeInArrangement() -> (point: CGPoint, display: CGRect, zone: CGRect)? {
        let (gaze, screens) = shared.mutex.withLock { ($0.gaze, $0.screens) }
        guard let gaze, gaze.index < screens.count, virtualScreens.indices.contains(gaze.index) else { return nil }
        let screen = screens[gaze.index]
        let bounds = CGDisplayBounds(virtualScreens[gaze.index].displayID)
        let scale = CGSize(width: bounds.width / CGFloat(screen.width), height: bounds.height / CGFloat(screen.height))
        let zone = zone(around: gaze.pixel, width: screen.width, height: screen.height)
        return (
            CGPoint(
                x: bounds.minX + CGFloat(gaze.pixel.x) * scale.width,
                y: bounds.minY + CGFloat(gaze.pixel.y) * scale.height),
            bounds,
            CGRect(
                x: bounds.minX + zone.minX * scale.width, y: bounds.minY + zone.minY * scale.height,
                width: zone.width * scale.width, height: zone.height * scale.height)
        )
    }

    private func controlWindows(_ command: ViewerCommand) {
        let gaze = gazeInArrangement()
        if case .movePointerToGaze = command {
            guard let gaze else { return }
            CGWarpMouseCursorPosition(gaze.point)
            CGAssociateMouseAndMouseCursorPosition(1)
            return
        }
        guard WindowControl.allowed(prompt: true) else { return }
        switch command {
        case .moveWindowToGaze:
            guard let gaze, let window = WindowControl.focusedWindow(), let frame = WindowControl.frame(of: window)
            else { return }
            WindowControl.move(window, to: windowOrigin(size: frame.size, centeredOn: gaze.point, within: gaze.display))
        case .fitWindowToZone:
            guard let gaze, let window = WindowControl.focusedWindow() else { return }
            WindowControl.setFrame(window, to: gaze.zone)
        case .gatherWindows:
            gatherWindows(to: gaze?.display)
        default:
            break
        }
    }

    /// Moves every window sitting on the glasses' own display, where it is
    /// hidden behind the glasses view, onto the virtual screen `target`, or
    /// the first one.
    private func gatherWindows(to target: CGRect?) {
        guard let glasses = Displays.glassesDisplay(),
            let target = target ?? virtualScreens.first.map({ CGDisplayBounds($0.displayID) })
        else { return }
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

    private func setSource(description: String, displayID: CGDirectDisplayID?, captures: [ScreenCapture]) {
        sourceDisplayID = displayID
        self.captures = captures
        let bounds = displayID.map(CGDisplayBounds)
        let latest = captures.map(\.latest)
        shared.mutex.withLock { state in
            state.sourceDescription = description
            state.sourceBounds = bounds
            state.captures = latest
            state.capturesInView = []
        }
    }

    private func switchSource() async {
        for capture in captures {
            await capture.stop()
        }
        setSource(description: "SOURCE STARTING", displayID: nil, captures: [])
        let (showsRoom, screens, glassesOnly) = shared.mutex.withLock { ($0.showsRoom, $0.screens, $0.glassesOnly) }
        guard showsRoom else {
            virtualScreens = []
            let displayID = Displays.mirrorSource()
            let capture = await startCapture(displayID: displayID, pixelSize: Displays.pixelSize(of: displayID))
            guard !Task.isCancelled else { return }
            setSource(description: capture.1 ?? "MIRROR MAIN DISPLAY", displayID: displayID, captures: [capture.0])
            return
        }
        await startVirtualScreens(screens, glassesOnly: glassesOnly)
        guard glassesOnly, !Task.isCancelled else { return }
        // Windows from the laptop's screen land on the glasses' own display,
        // behind this view, when the lid closes.
        try? await Task.sleep(for: .seconds(arrangementSettleTime))
        if !Task.isCancelled, WindowControl.allowed(prompt: false) {
            gatherWindows(to: nil)
        }
    }

    private func startVirtualScreens(_ screens: [RoomScreen], glassesOnly: Bool) async {
        setSource(description: "CREATING \(screens.count) VIRTUAL SCREENS", displayID: nil, captures: [])
        // Screens whose size is unchanged are kept, so they stay put.
        var kept: [VirtualScreen?] = []
        for (index, screen) in screens.enumerated() {
            if index < virtualScreens.count, virtualScreens[index].width == screen.width,
                virtualScreens[index].height == screen.height
            {
                kept.append(virtualScreens[index])
            } else {
                kept.append(VirtualScreen(index: index, width: screen.width, height: screen.height))
            }
        }
        guard kept.allSatisfy({ $0 != nil }) else {
            // Without every display the rest would pair with the wrong screens.
            virtualScreens = []
            setSource(description: "MACOS REFUSED A VIRTUAL SCREEN - TRY ANOTHER SIZE", displayID: nil, captures: [])
            return
        }
        virtualScreens = kept.compactMap { $0 }

        var frames: [CGRect?] = []
        for screen in virtualScreens {
            frames.append(await ScreenCapture.shareableFrame(of: screen.displayID, timeout: virtualScreenTimeout))
            guard !Task.isCancelled else { return }
        }
        // New screens take their place in the standard layout; macOS's
        // arrangement then follows the room.
        let main = CGDisplayBounds(Displays.mirrorSource())
        shared.mutex.withLock { state in
            guard state.glassesOnly == glassesOnly else { return }
            let (standard, ahead) =
                glassesOnly
                ? (glassesOnlyArrangement(for: state.screens), CGRect.zero)
                : (standardArrangement(for: state.screens, around: main), main)
            for (index, frame) in standard.enumerated() where state.screens[index].placement == nil {
                state.screens[index].placement = ScreenPlacement(arrangedAt: frame, around: ahead)
            }
        }
        arrangeVirtualScreens()
        saveSettings()

        var started: [ScreenCapture] = []
        var problems: [String] = []
        for (index, screen) in virtualScreens.enumerated() {
            // Wide screens are captured in tiles, so the parts out of view
            // can update less often.
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
            if frames[index] == nil {
                problems.append("SCREEN \(index + 1) DID NOT COME ONLINE")
            }
        }
        let sizes = screens.map { "\($0.width)X\($0.height)" }.joined(separator: ", ")
        let mode = glassesOnly ? "GLASSES ONLY " : ""
        setSource(
            description: problems.first ?? "\(mode)VIRTUAL SCREENS \(sizes)", displayID: nil, captures: started)
    }

    /// Starts capturing `displayID`, or the part `sourceRect` of it. The
    /// capture is returned even when it fails, so captures stay in step with
    /// the screens; the second value then says what went wrong.
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
                let permission = CGPreflightScreenCaptureAccess() ? "" : " - ALLOW SCREEN RECORDING AND RELAUNCH"
                eprint("Screen capture failed: \(error)")
                return (capture, "CAPTURE FAILED: \(error.localizedDescription.uppercased())\(permission)")
            }
        } catch {
            fatalError("Could not create a Metal texture cache: \(error)")
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
