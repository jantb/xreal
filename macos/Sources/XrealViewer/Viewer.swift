import AppKit
import Carbon.HIToolbox
import ScreenCaptureKit
import Synchronization
import simd
import XrealCore

// Parts of the canvas out of view still update this often, so they are
// fresh enough when the head turns to them.
private let outOfViewFps = 10
// Captures this far outside the view, as a fraction of it, count as in view,
// so a quick turn finds them already at the full rate.
private let inViewMargin: Float = 1
// How far past the view's edges the canvas, the dashboard or the pinned
// window can go, as a share of its size, before an arrow points back.
private let pointBackMargin: Float = 0.05
// A capture stays at the full rate this long after it was last in view.
// Changing a capture's rate makes the capture hitch for a moment, so a fast
// pan back and forth should not change it at every tile edge.
private let inViewHold = 1.5  // seconds
private let captureRateInterval = 0.25
// The display arrangement churns for a moment when the lid opens or closes.
private let displaysSettleTime = 1.5
// A capture macOS stopped starts again after this, so tiles that stop
// together start again once.
private let captureRestartDelay = 1.0  // seconds
// After the Mac wakes its captures start afresh this much later, once the
// displays are back.
private let wakeSettleTime = 3.0  // seconds
// Windows brought back from behind the glasses view are staggered this much.
private let gatherStep: CGFloat = 40
// The glasses show a frame about 7 ms after it arrives over DisplayPort.
private let glassesDisplayDelay = 0.007
// The glasses light their rows top to bottom over about this long after the
// top one (Breezy Desktop uses 8 ms for the Air series).
private let scanoutTime = 0.008
private let virtualScreenTimeout = 10.0
// The pinned window updates this often; it is looked at now and then.
private let pinnedFps = 30
// Its capture's longest side, in pixels.
private let maxPinnedPixels: CGFloat = 4096
// How often the pinned window is checked for moving, resizing or closing,
// and the dashboard and the pointer's shape brought up to date.
private let pinnedCheckInterval = 2.0
// Ten times a second: numbers and graphs move smoothly, and a look at the
// Mac and a redraw take a few milliseconds on a queue of their own.
private let statusInterval = 0.1
private let pointerInterval = 1.0 / 15
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
// How far the glow round the canvas reaches past its edges, in points, and
// how bright it is at the edge.
private let ambientReach: Float = 480
private let ambientBrightness: Float = 0.75
// The ring round the mouse while zoomed out to show it: its radius in
// glasses pixels, and how far zoomed out it takes to show it fully.
private let locatorRadius: Float = 40
private let locatorFadeIn: Float = 0.15
// How far the dashboard and the pinned window hang, as a share of how far
// they would hang on the canvas's own surface.
private let overheadNearness: Float = 0.8
// How long the canvas stays outlined after it was moved closer or away.
private let outlineTime = 1.0
// macOS nudges displays apart after an arrangement is applied; they are
// where they stay this long after.
private let arrangementSettleTime = 2.0

/// Virtual screen sizes that macOS 27 creates as asked, in points, smallest
/// first, each HiDPI one (scale 2) after its plain twin where there is one: the sizes the
/// canvas steps through when resized. Nearby standard sizes such as 3840 ×
/// 2160, 5120 × 2880 or 5760 × 2160 are refused or come up smaller, and so
/// are 5752 × 3240 and 5752 × 4320. 7672 × 2160 wraps about 170° at the
/// glasses' pixel density; 5752 × 2880 about 125°, and a third taller;
/// 7672 × 4320 is as wide and twice as tall. The HiDPI ones draw text at
/// twice the detail, 3840 × 2160, 5760 × 3240 and 7664 × 4320 pixels,
/// filtered down; 5120 × 1440 and 5120 × 2160 at 2x are the two allowed past
/// `maxVirtualScreenSide` (see `sizesProbedBeyondLimit`), and by far the
/// heaviest to draw and capture.
let canvasSizes: [(width: Int, height: Int, scale: Int)] = [
    (1920, 1080, 1), (1920, 1080, 2), (2880, 1620, 1), (2880, 1620, 2), (5120, 1440, 1), (5120, 1440, 2),
    (3832, 2160, 1), (3832, 2160, 2), (5120, 2160, 2), (5752, 2160, 1), (5752, 2880, 1), (7672, 2160, 1), (7672, 4320, 1),
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
    /// How many times further the view turns than the head, 1 or more.
    case setHeadGain(Float)
    case toggleFollowCursor
    case toggleTurnZoom
    case toggleDiagnostics
    case setLatencyTrim(Float)
    case setCurveRadius(Float)
    case toggleLensCorrection
    case setDepthScale(Float)
    case calibrate
    case toggleCurved
    case toggleSpherical
    case toggleEvenTextSize
    case toggleSoftEdges
    case toggleSteadyLaptopScreen
    case toggleLaptopScreenOff
    case toggleAmbientLight
    /// How far a wrapped canvas bends up and down, 0 to 1.
    case setVerticalWrap(Float)
    /// Picks up (true) or lets go of (false) the canvas, if looked at.
    case grab(Bool)
    /// Brings the canvas closer (true) or pushes it away (false).
    case moveCanvas(closer: Bool)
    /// Back to the canvas's own distance, one of its pixels to each of the
    /// glasses' pixels.
    case resetZoom
    /// Makes the canvas larger (true) or smaller.
    case resizeCanvas(bigger: Bool)
    case setCanvasSize(width: Int, height: Int, scale: Int)
    case setCanvasRefreshRate(Int)
    case toggleLatePoseSampling
    case toggleSharpFiltering
    case toggleLivePointer
    case toggleStatusStrip
    case setPinnedWindow(PinnedWindow?)
    /// Pins one window of those listed, even among several with its title.
    case pinWindow(PinnableWindow)
    case window(WindowCommand)
}

/// The sizes, in points, of what is shown with the canvas this frame; nil
/// for what is not there.
struct ExtraSizes: Sendable {
    var status: SIMD2<Float>?
    var pinned: SIMD2<Float>?
    /// The pointer and its hot spot from its top-left corner.
    var pointer: (size: SIMD2<Float>, hotSpot: SIMD2<Float>)?
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
    var turnZoom = TurnZoom()
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
    /// The pinned window's capture, while there is one.
    var pinned: LatestFrame?
    /// When frames reached the display, as of the latest frame.
    var timing = FrameTiming()
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
    /// The overhead panels as last laid out, facing straight ahead: they
    /// only move with the canvas's size and distance, not every frame.
    private var overheadCache: (
        canvas: RoomScreen, curveRadius: Float, layout: OverheadLayout,
        dashboard: (surface: ScreenSurface, rect: SurfaceRect)?, pinned: (surface: ScreenSurface, rect: SurfaceRect)?
    )?

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

    /// Picks up the canvas, wherever it is, to carry it with the head: it
    /// glides to straight ahead and stays there, tilting as the head does.
    /// While it is carried, straight ahead follows the head, as putting the
    /// canvas back straight ahead makes it, so everything round it keeps
    /// its shape. Returns false if it was already carried.
    mutating func startGrab(now: Double = monotonicNow()) -> Bool {
        guard grab == nil else { return false }
        grab = ScreenGrab(placement: settings.canvas.placement, headRotation: viewport.headRotation, at: now)
        grabDistance = settings.canvas.placement.distance
        return true
    }

    /// Lets go of the carried canvas where it is now, set level if nearly
    /// so. Returns false if it was not carried.
    mutating func endGrab(now: Double = monotonicNow()) -> Bool {
        guard let grab else { return false }
        var placement = grab.placement(headRotation: viewport.headRotation, distance: grabDistance, at: now)
        if abs(placement.tilt) < levelSnapTilt {
            placement.tilt = 0
        }
        settings.canvas.placement = placement
        self.grab = nil
        // Straight ahead moved with the head, so drift measured against
        // the old one no longer holds.
        drift.reset()
        return true
    }

    mutating func resetZoom(now: Double) {
        if grab != nil {
            grabDistance = 1
        } else {
            settings.canvas.placement.distance = 1
        }
        outlineUntil = now + outlineTime
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
        let canvas = settings.canvas
        let size: (width: Int, height: Int, scale: Int)?
        if let index = canvasSizes.firstIndex(where: {
            $0.width == canvas.width && $0.height == canvas.height && $0.scale == canvas.scale
        }) {
            let next = bigger ? index + 1 : index - 1
            size = canvasSizes.indices.contains(next) ? canvasSizes[next] : nil
        } else {
            // A size of its own, from an older run: the nearest listed one.
            let area = canvas.width * canvas.height
            size =
                bigger
                ? canvasSizes.first { $0.width * $0.height > area } : canvasSizes.last { $0.width * $0.height < area }
        }
        guard let size else { return false }
        return setCanvasSize(width: size.width, height: size.height, scale: size.scale, now: now)
    }

    /// Returns whether the size changed, so the canvas is created again.
    mutating func setCanvasSize(width: Int, height: Int, scale: Int = 1, now: Double) -> Bool {
        let canvas = settings.canvas
        guard width != canvas.width || height != canvas.height || scale != canvas.scale else { return false }
        settings.canvas.width = width
        settings.canvas.height = height
        settings.canvas.scale = scale
        if grab != nil {
            // Straight ahead followed the head while it was carried.
            grab = nil
            drift.reset()
        }
        gaze = nil
        outlineUntil = now + outlineTime
        return true
    }

    /// Moves on to the frame that reaches the display at `presentingAt`, in
    /// `monotonicNow` time, by the display link's promise; `timing` says how
    /// much later frames really reach it. `frameSizes` holds the size of the
    /// latest frame of each capture, `cursor` is the mouse in global
    /// coordinates and `extras` what else there is to show. Returns what to
    /// draw, nil until the glasses show side by side, and whether the gyro
    /// bias changed and should be saved.
    mutating func advance(
        now: Double, dt: Float, presentingAt: Double, snapshot: TrackingSnapshot, captureGeneration: UInt64,
        newFrame: Bool, frameSizes: [(width: Int, height: Int)?], output: (width: Int, height: Int),
        cursor: CGPoint?, timing: FrameTiming = FrameTiming(), extras: ExtraSizes = ExtraSizes()
    ) -> (room: RoomView?, biasChanged: Bool) {
        stats.tick(now: now, captureGeneration: captureGeneration)
        self.timing = timing
        self.snapshot = snapshot
        self.newFrame = newFrame
        self.sourceSize = frameSizes.first ?? nil
        self.output = output

        let biasChanged = snapshot.biasRevision != biasRevisionSaved
        biasRevisionSaved = snapshot.biasRevision
        // Predicted for when the frame is really seen: when the display link
        // promised it, as much later as frames have lately been, and the
        // glasses' own delay.
        let lead =
            max(presentingAt - now, 0) + (timing.extraDelay ?? 0) + glassesDisplayDelay
            + Double(settings.latencyTrimMs) / 1000
        let pose = settings.prediction ? snapshot.predict(now: now, lead: lead) : snapshot.pose
        lastPose = pose
        if snapshot.session != trackingSession {
            trackingSession = snapshot.session
            drift.reset()
            viewport.recenter(pose)
        }
        if grab != nil {
            viewport.recenter(pose)
        }
        viewport.track(pose: pose, dt: dt)
        var room = roomView(now: now, dt: dt, output: output, frameSizes: frameSizes, cursor: cursor, extras: extras)
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
        cursor: CGPoint?, extras: ExtraSizes
    ) -> RoomView {
        let rotation = viewport.headRotation
        if let grab {
            settings.canvas.placement = grab.placement(headRotation: rotation, distance: grabDistance, at: now)
        }
        let (canvas, curveRadius) = (settings.canvas, settings.curveRadius)
        gaze = gazeTarget(rotation * SIMD3(0, 0, -1), on: canvas, curveRadius: curveRadius)
        var room = viewport.roomView(
            canvas: canvas, outputWidth: output.width / 2, outputHeight: output.height,
            highlighted: grab != nil || now < outlineUntil, curveRadius: curveRadius)
        // Zoom out while the mouse moves where it cannot be seen; behind the
        // viewer no zoom would show it.
        let cursorPoint = cursor.flatMap(roomPoint(ofCursor:)).flatMap { room.isAhead($0) ? $0 : nil }
        let cursorScale = cursorFollow.update(
            cursor: cursor.map { SIMD2(Float($0.x), Float($0.y)) }.flatMap { cursorPoint == nil ? nil : $0 },
            enabled: settings.followCursor, now: now, dt: dt
        ) { scale, margin in
            cursorPoint.map { room.shows($0, scale: scale, margin: margin) } ?? false
        }
        // And while the head turns quickly, as far as showing the whole
        // canvas; whichever wants more wins.
        var whole: Float = 1
        if settings.zoomOutWhenTurning {
            let edges: [SIMD2<Float>] = [[0, 0], [0.5, 0], [1, 0], [0, 0.5], [1, 0.5], [0, 1], [0.5, 1], [1, 1]]
            let size = SIMD2(Float(canvas.width), Float(canvas.height))
            // What is behind no zoom can show; the rest of it, then.
            let points = edges.map { canvas.roomPoint(ofPixel: $0 * size, curveRadius: curveRadius) }
                .filter(room.isAhead)
            whole = TurnZoom.wholeCanvasScale(points) { room.shows($0, scale: $1, margin: $2) }
        }
        let turnScale = turnZoom.update(
            speed: viewport.headSpeed, enabled: settings.zoomOutWhenTurning, whole: whole, dt: dt)
        let scale = min(cursorScale, turnScale)
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
        for panel in room.panels {
            if let tile = panel.tile, tile < frameSizes.count, room.shows(panel, margin: inViewMargin) {
                capturesInView[tile] = true
            }
        }
        // A canvas still starting up has nothing to show yet.
        room.panels.removeAll { panel in panel.tile.map { $0 >= frameSizes.count || frameSizes[$0] == nil } ?? false }

        // The glow round the canvas, drawn first so everything else lies on
        // it; the dashboard's cards and the pinned window hide it behind
        // them.
        let surface = canvas.surface(curveRadius: curveRadius)
        if settings.ambientLight, room.panels.contains(where: { $0.tile != nil }) {
            let size = SIMD2(Float(canvas.width), Float(canvas.height))
            let reach = ambientReach * 2 / size
            let halo = SIMD4(size.x, size.y, ambientReach, ambientBrightness)
            let sides = [
                SurfaceRect(left: -1 - reach.x, right: -1, top: 1 + reach.y, bottom: -1 - reach.y),
                SurfaceRect(left: 1, right: 1 + reach.x, top: 1 + reach.y, bottom: -1 - reach.y),
                SurfaceRect(left: -1, right: 1, top: 1 + reach.y, bottom: 1),
                SurfaceRect(left: -1, right: 1, top: -1, bottom: -1 - reach.y),
            ]
            room.panels.insert(
                contentsOf: sides.map { RoomView.Panel(source: .ambient, surface: surface, rect: $0, halo: halo) }, at: 0)
        }

        // Above the canvas, in a row tilted to face the viewer.
        let overhead = overheadLayout(
            canvas: canvas, dashboard: settings.statusStrip ? extras.status : nil, pinned: extras.pinned)
        if overhead.height > 0 {
            let row = overheadViews(canvas: canvas, curveRadius: curveRadius, layout: overhead)
            if let dashboard = row.dashboard {
                room.panels.append(RoomView.Panel(source: .status, surface: dashboard.surface, rect: dashboard.rect))
            }
            if let pinned = row.pinned {
                room.panels.append(RoomView.Panel(source: .pinned, surface: pinned.surface, rect: pinned.rect))
            }
        }
        // While zoomed out to show the mouse, a ring round it, the same
        // size to the eye however far out, to find it by.
        let finding = min(max((1 - cursorScale) / locatorFadeIn, 0), 1)
        if finding > 0, let point = cursor.flatMap(canvasPoint(ofCursor:)) {
            let radius = locatorRadius * canvas.placement.distance / scale
            let rect = pointerRect(
                canvas: canvas, at: point, size: SIMD2(repeating: 2 * radius), hotSpot: SIMD2(repeating: radius))
            room.panels.append(
                RoomView.Panel(source: .locator, surface: surface, rect: rect, halo: SIMD4(0, 0, 0, finding)))
        }
        // The pointer last, over everything it lies on, keeping its size to
        // the eye while the view zooms out.
        if settings.livePointer, let pointer = extras.pointer, let point = cursor.flatMap(canvasPoint(ofCursor:)) {
            let rect = pointerRect(
                canvas: canvas, at: point, size: pointer.size / scale, hotSpot: pointer.hotSpot / scale)
            room.panels.append(RoomView.Panel(source: .pointer, surface: surface, rect: rect))
        }
        // Looking away from all of it, an arrow shows the way back. Looking
        // at the canvas counts too: close up, it can fill the view with
        // every point `shows` tries outside it.
        let shown = room.panels.filter { $0.tile != nil || $0.source == .status || $0.source == .pinned }
        if gaze == nil, shown.contains(where: { $0.tile != nil }),
            !shown.contains(where: { room.shows($0, margin: pointBackMargin) })
        {
            let way = room.headRotation.transpose * canvas.placement.direction
            let across = SIMD2(way.x, way.y)
            // Straight behind, either way round is as short.
            room.pointBack = simd_length(across) > 1e-3 ? simd_normalize(across) : SIMD2(1, 0)
        }
        return room
    }

    /// Layout depends on size and distance, not on the head pose or the
    /// canvas's orientation. Keep the clearance solver off the frame path.
    private mutating func overheadViews(canvas: RoomScreen, curveRadius: Float, layout: OverheadLayout)
        -> (dashboard: (surface: ScreenSurface, rect: SurfaceRect)?, pinned: (surface: ScreenSurface, rect: SurfaceRect)?)
    {
        var local = canvas
        local.placement = ScreenPlacement(direction: SIMD3(0, 0, -1), distance: canvas.placement.distance)
        if overheadCache.map({ $0.canvas != local || $0.curveRadius != curveRadius || $0.layout != layout }) ?? true {
            let panels = overheadPanels(canvas: local, curveRadius: curveRadius, layout: layout)
            overheadCache = (local, curveRadius, layout, panels.dashboard, panels.pinned)
        }
        func oriented(_ panel: (surface: ScreenSurface, rect: SurfaceRect)?) -> (surface: ScreenSurface, rect: SurfaceRect)? {
            guard var panel else { return nil }
            panel.surface.spin = canvas.placement.orientation * panel.surface.spin
            // Nearer than the canvas, looking the same size: the eyes see
            // it in front of the canvas and its glow.
            panel.surface = panel.surface.scaled(by: overheadNearness)
            return panel
        }
        return (oriented(overheadCache?.dashboard), oriented(overheadCache?.pinned))
    }

    /// Where the mouse at `point`, in global coordinates, is on the canvas,
    /// in its points from the top-left corner; nil when it is elsewhere.
    private func canvasPoint(ofCursor point: CGPoint) -> SIMD2<Float>? {
        guard let bounds = canvasBounds, bounds.contains(point) else { return nil }
        let canvas = settings.canvas
        return SIMD2(
            Float((point.x - bounds.minX) / bounds.width) * Float(canvas.width),
            Float((point.y - bounds.minY) / bounds.height) * Float(canvas.height))
    }

    /// Where the mouse at `point`, in global coordinates, is in the room, if
    /// it is on the canvas.
    private func roomPoint(ofCursor point: CGPoint) -> SIMD3<Float>? {
        canvasPoint(ofCursor: point).map { settings.canvas.roomPoint(ofPixel: $0, curveRadius: settings.curveRadius) }
    }

    func hudInfo(now: Double) -> HudInfo {
        HudInfo(
            gaze: gaze, source: sourceSize, sourceDescription: source.diagnosticsDescription, output: output,
            newFrame: newFrame, stats: stats, tracking: snapshot, pose: lastPose, prediction: settings.prediction,
            lastDrift: lastDrift, now: now, timing: timing, latePoseSampling: settings.latePoseSampling)
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
    private let status: LatestFrame
    private let pointer: Guarded<(frame: CapturedFrame?, hotSpot: SIMD2<Float>)>
    private let timing = Guarded(FrameTiming())
    /// Set before the first frame; called on the display link thread.
    var onBiasChanged: @Sendable () -> Void = {}
    // Touched only on the display link thread.
    private var sources: [ObjectIdentifier] = []
    private var frames: [CapturedFrame?] = []
    private var generations: [UInt64] = []
    private var lastRenderAt = monotonicNow()
    private var lastFreeCursor: CGPoint?

    init(
        shared: SharedState, tracking: Tracking, renderer: Renderer, status: LatestFrame,
        pointer: Guarded<(frame: CapturedFrame?, hotSpot: SIMD2<Float>)>
    ) {
        self.shared = shared
        self.tracking = tracking
        self.renderer = renderer
        self.status = status
        self.pointer = pointer
    }

    /// Fills `drawable`, which the display link promises for `presentingAt`
    /// if it is committed by `deadline`.
    func render(to drawable: CAMetalDrawable, presentingAt: Double, deadline: Double) {
        // The later the pose is taken, the less there is to predict: wait
        // until just enough time is left for the frame's work.
        if shared.mutex.withLock({ $0.settings.latePoseSampling }) {
            let wake = deadline - timing.mutex.withLock { $0.workBudget }
            sleep(until: wake)
            let overslept = monotonicNow() - wake
            if overslept > 0.002 {
                timingLog.notice("Render thread woke \(String(format: "%.1f", overslept * 1000), privacy: .public) ms late")
            }
        }
        let now = monotonicNow()
        let dt = Float(min(max(now - lastRenderAt, 0), 0.1))
        lastRenderAt = now

        let (captures, pinned, fence, sharpen, softEdges) = shared.mutex.withLock {
            ($0.captures, $0.pinned, $0.cursorFence, $0.settings.sharpFiltering, $0.settings.softEdges)
        }
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
            } else if let free = lastFreeCursor ?? escape(from: fence) {
                lastFreeCursor = nil
                CGWarpMouseCursorPosition(free)
                // Without this the mouse stays frozen for a quarter of a
                // second after every warp.
                CGAssociateMouseAndMouseCursorPosition(1)
            }
        }
        let output = (width: drawable.texture.width, height: drawable.texture.height)
        let frameSizes = frames.map { frame in frame.map { (width: $0.width, height: $0.height) } }
        let images = PanelImages(
            canvas: frames, status: status.current().frame, pinned: pinned?.current().frame,
            pointer: pointer.mutex.withLock { $0.frame })
        let hotSpot = pointer.mutex.withLock { $0.hotSpot }
        func points(_ frame: CapturedFrame) -> SIMD2<Float> {
            SIMD2(Float(frame.width), Float(frame.height)) / frame.pixelsPerPoint
        }
        let extras = ExtraSizes(
            status: images.status.map(points), pinned: images.pinned.map(points),
            pointer: images.pointer.map { (points($0), hotSpot) })
        let timing = self.timing
        let timingNow = timing.mutex.withLock { $0 }

        // Sample the pose as late as possible, just before building the frame.
        let snapshot = tracking.snapshot()
        let result = shared.mutex.withLock { state in
            state.advance(
                now: now, dt: dt, presentingAt: presentingAt, snapshot: snapshot,
                captureGeneration: captureGeneration, newFrame: newFrame, frameSizes: frameSizes, output: output,
                cursor: cursor, timing: timingNow, extras: extras)
        }
        let period = 1 / glassesRefreshRate
        drawable.addPresentedHandler { shown in
            // Zero when the frame was never shown.
            let at = shown.presentedTime
            if at == 0 {
                timingLog.notice("Frame dropped")
            } else if timing.mutex.withLock({ $0.isLate(promised: presentingAt, presented: at, period: period) }) {
                timingLog.notice(
                    "Frame late by \(String(format: "%.1f", (at - presentingAt) * 1000), privacy: .public) ms")
            }
            timing.mutex.withLock {
                $0.presented(promised: presentingAt, at: at > 0 ? at : nil, sampledAt: now, period: period)
            }
        }
        renderer.draw(to: drawable, images: images, room: result.room, sharpen: sharpen, softEdges: softEdges) {
            gpuEnd in
            if gpuEnd - now > period {
                timingLog.notice(
                    "Frame took \(String(format: "%.1f", (gpuEnd - now) * 1000), privacy: .public) ms from pose to GPU done")
            }
            timing.mutex.withLock { $0.worked(gpuEnd - now, period: period) }
        }
        if result.biasChanged {
            onBiasChanged()
        }
    }

    /// Somewhere off `fence` for the mouse when it has nowhere it was
    /// before: the middle of a display other than the glasses'.
    private func escape(from fence: CGRect) -> CGPoint? {
        Displays.active().lazy.map(CGDisplayBounds).first { !$0.intersects(fence) }
            .map { CGPoint(x: $0.midX, y: $0.midY) }
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
    private var captureRestartTask: Task<Void, Never>?
    private var timers: [Timer] = []
    private let dashboard: Dashboard
    private let pointerImage: PointerImage
    private var pinnedCapture: ScreenCapture?
    /// The window being pinned and its size in points.
    private var pinnedTarget: (id: CGWindowID, size: CGSize)?
    private var pinnedTask: Task<Void, Never>?
    /// The window chosen in the controls, which the pinned window follows
    /// while its title changes; nil to go by the saved app and title.
    private(set) var preferredPinnedWindowID: CGWindowID?
    /// Why the pinned window is not shown, for the controls.
    private(set) var pinnedWindowStatus: String?
    private var settingsSaveTask: Task<Void, Never>?
    private let laptopScreen = LaptopScreen()

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

        dashboard = Dashboard(device: renderer.device)
        pointerImage = PointerImage(device: renderer.device)
        let frameLoop = FrameLoop(
            shared: shared, tracking: tracking, renderer: renderer, status: dashboard.latest,
            pointer: pointerImage.latest)
        displayLink = DisplayLinkThread { drawable, presentingAt, deadline in
            frameLoop.render(to: drawable, presentingAt: presentingAt, deadline: deadline)
        }
        window = GlassesWindow(device: renderer.device, displayLink: displayLink)
        frameLoop.onBiasChanged = { [weak self] in Task { @MainActor in self?.saveSettings() } }

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
            hotKey(kVK_ANSI_0, .resetZoom),
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
        // Captures can stop, or stop delivering, across sleep without the
        // glasses ever going away; start them afresh once the Mac is awake.
        for name in [NSWorkspace.didWakeNotification, NSWorkspace.screensDidWakeNotification] {
            NSWorkspace.shared.notificationCenter.addObserver(forName: name, object: nil, queue: .main) {
                [weak self] _ in
                MainActor.assumeIsolated { self?.restartCaptures(after: wakeSettleTime) }
            }
        }
        func every(_ interval: Double, _ action: @escaping @MainActor (Viewer) -> Void) -> Timer {
            let timer = Timer(timeInterval: interval, repeats: true) { [weak self] _ in
                MainActor.assumeIsolated { if let self { action(self) } }
            }
            RunLoop.main.add(timer, forMode: .common)
            return timer
        }
        timers = [
            every(captureRateInterval) { $0.updateCaptureRates() },
            every(statusInterval) { $0.updateDashboard() },
            every(pointerInterval) { $0.updatePointer() },
            every(pinnedCheckInterval) { $0.checkPinnedWindow() },
        ]

        // The window shows once the source starts, if the glasses are there.
        updateCursorFence()
        startSource()
        restartPinned()
    }

    /// The settings and view as they are now, for the menu.
    var current: (settings: Settings, viewport: ViewportController) {
        shared.mutex.withLock { ($0.settings, $0.viewport) }
    }

    /// What the status lines are made from.
    func statusInfo() -> (info: HudInfo, source: SourceStatus) {
        shared.mutex.withLock { ($0.hudInfo(now: monotonicNow()), $0.source) }
    }

    /// Leaves the glasses showing their own picture, and the laptop's
    /// screen its own refresh rate, as before the viewer ran.
    func restoreGlasses() {
        tracking.restoreDisplayMode()
        laptopScreen.release()
    }

    /// Holds the laptop's screen at 60 Hz while the glasses are in use, if
    /// asked to.
    private func holdLaptopScreen() {
        let (off, steady) = shared.mutex.withLock { ($0.settings.laptopScreenOff, $0.settings.steadyLaptopScreen) }
        let wearing = Displays.glassesDisplay() != nil
        laptopScreen.update(off: off && wearing, steady: steady && wearing)
    }

    func saveSettings() {
        settingsSaveTask?.cancel()
        timingLog.notice("Saving settings")
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
        var repin = false
        if case .window(let windowCommand) = command {
            controlWindows(windowCommand)
            return
        }
        let preferredID = preferredPinnedWindowID
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
            case .setHeadGain(let gain):
                state.viewport.setGain(gain)
                state.settings.headGain = state.viewport.gain
            case .toggleFollowCursor: state.settings.followCursor.toggle()
            case .toggleTurnZoom: state.settings.zoomOutWhenTurning.toggle()
            case .toggleDiagnostics: state.settings.diagnosticsVisible.toggle()
            case .setLatencyTrim(let ms): state.settings.latencyTrimMs = min(max(ms, 0), maxLatencyTrimMs)
            case .toggleLensCorrection: state.settings.lensCorrection.toggle()
            case .setDepthScale(let metres): state.settings.metresPerRoomUnit = metres
            case .setCurveRadius(let radius): state.settings.curveRadius = radius
            case .calibrate:
                tracking.calibrate()
                persist = false
            case .toggleCurved: state.settings.canvas.curved.toggle()
            case .toggleSpherical: state.settings.canvas.spherical.toggle()
            case .toggleEvenTextSize: state.settings.canvas.evenSize.toggle()
            case .toggleSoftEdges: state.settings.softEdges.toggle()
            case .toggleSteadyLaptopScreen: state.settings.steadyLaptopScreen.toggle()
            case .toggleLaptopScreenOff: state.settings.laptopScreenOff.toggle()
            case .toggleAmbientLight: state.settings.ambientLight.toggle()
            case .setVerticalWrap(let wrap): state.settings.canvas.verticalWrap = min(max(wrap, 0), 1)
            case .grab(let pickUp):
                // Saved once it is let go.
                persist = pickUp ? false : state.endGrab()
                if pickUp {
                    _ = state.startGrab()
                }
            case .moveCanvas(let closer):
                state.moveCanvas(closer: closer, now: monotonicNow())
            case .resetZoom:
                state.resetZoom(now: monotonicNow())
            case .resizeCanvas(let bigger):
                restartSource = state.resizeCanvas(bigger: bigger, now: monotonicNow())
                persist = restartSource
            case .setCanvasSize(let width, let height, let scale):
                restartSource = state.setCanvasSize(width: width, height: height, scale: scale, now: monotonicNow())
                persist = restartSource
            case .setCanvasRefreshRate(let rate):
                restartSource = canvasRefreshRates.contains(rate) && rate != state.settings.canvasRefreshRate
                if restartSource {
                    state.settings.canvasRefreshRate = rate
                }
                persist = restartSource
            case .toggleLatePoseSampling: state.settings.latePoseSampling.toggle()
            case .toggleSharpFiltering: state.settings.sharpFiltering.toggle()
            case .toggleLivePointer:
                // The captures leave the pointer out while it is drawn live.
                state.settings.livePointer.toggle()
                restartSource = true
            case .toggleStatusStrip: state.settings.statusStrip.toggle()
            case .setPinnedWindow(let pinned):
                repin = pinned != state.settings.pinnedWindow
                state.settings.pinnedWindow = pinned
                persist = repin
            case .pinWindow(let choice):
                repin = choice.id != preferredID || choice.window != state.settings.pinnedWindow
                state.settings.pinnedWindow = choice.window
                persist = repin
            case .window:
                break
            }
        }
        if persist {
            scheduleSettingsSave()
        }
        if restartSource {
            startSource()
        }
        if repin {
            if case .pinWindow(let choice) = command {
                preferredPinnedWindowID = choice.id
            } else {
                preferredPinnedWindowID = nil
            }
            restartPinned()
        }
        switch command {
        case .toggleStatusStrip: updateDashboard()
        case .toggleSteadyLaptopScreen, .toggleLaptopScreenOff: holdLaptopScreen()
        case .toggleLivePointer: updatePointer()
        default: break
        }
    }

    private func scheduleSettingsSave() {
        settingsSaveTask?.cancel()
        settingsSaveTask = Task {
            do { try await Task.sleep(for: .milliseconds(300)) } catch { return }
            saveSettings()
        }
    }

    /// Windows that can be pinned above the canvas, for the controls.
    func pinnableWindows() async throws -> [PinnableWindow] {
        try await ScreenCapture.pinnableWindows()
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
            restartPinned()
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
            // The laptop's screen comes back in its own mode when the lid
            // opens.
            holdLaptopScreen()
        }
    }

    /// Starts the canvas's and the pinned window's captures again after
    /// `delay`, keeping the canvas. Every request within the delay is one
    /// restart, so tiles stopping together, or a wake right after, start
    /// everything once.
    private func restartCaptures(after delay: Double) {
        captureRestartTask?.cancel()
        captureRestartTask = Task {
            do { try await Task.sleep(for: .seconds(delay)) } catch { return }
            eprint("Starting the captures again")
            startSource()
            restartPinned()
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
        let (inView, fullFps) = shared.mutex.withLock { ($0.capturesInView, $0.settings.canvasRefreshRate) }
        let now = monotonicNow()
        for (index, capture) in captures.enumerated() where capture.fps != 0 {
            if index >= inView.count || inView[index] {
                lastInView[index] = now
            }
            let fps = now - lastInView[index] < inViewHold ? fullFps : outOfViewFps
            if capture.fps != fps {
                timingLog.notice("Capture tile \(index, privacy: .public) to \(fps, privacy: .public) fps")
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
        // The pinned window may sit there on purpose, out of the way: it
        // shows above the canvas all the same.
        let pinned = shared.mutex.withLock { $0.settings.pinnedWindow }
        var step: CGFloat = 0
        for (window, bundleID) in WindowControl.allWindows() {
            guard let frame = WindowControl.frame(of: window), hidden.contains(CGPoint(x: frame.midX, y: frame.midY))
            else { continue }
            if let pinned, bundleID == pinned.bundleID, WindowControl.title(of: window) == pinned.title {
                continue
            }
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
            holdLaptopScreen()
            return
        }
        window.place()
        holdLaptopScreen()
        // For measuring the glasses' output on its own: no canvas, no
        // captures, only black frames, still timed.
        if ProcessInfo.processInfo.environment["XREAL_NO_CANVAS"] != nil {
            canvasScreen = nil
            setSource(.failed("No canvas, for testing"), captures: [])
            return
        }
        let (canvas, rate, livePointer) = shared.mutex.withLock {
            ($0.settings.canvas, $0.settings.canvasRefreshRate, $0.settings.livePointer)
        }
        setSource(.creatingCanvas, captures: [])
        // A canvas of the same size and rate is kept, so it stays put.
        if canvasScreen.map({
            $0.width != canvas.width || $0.height != canvas.height || $0.scale != canvas.scale
                || $0.refreshRate != Double(rate)
        }) ?? true {
            // The old one goes first: the new one may have the same identity.
            canvasScreen = nil
            canvasScreen = VirtualScreen(
                index: 0, width: canvas.width, height: canvas.height, scale: canvas.scale, refreshRate: Double(rate))
        }
        guard let screen = canvasScreen else {
            setSource(.refused, captures: [])
            return
        }
        let frame = await ScreenCapture.shareableFrame(of: screen.displayID, timeout: virtualScreenTimeout)
        guard !Task.isCancelled else { return }
        arrangeCanvas()
        // For measuring the canvas's display on its own, never captured.
        if ProcessInfo.processInfo.environment["XREAL_NO_CAPTURE"] != nil {
            setSource(.failed("Canvas not captured, for testing"), captures: [])
            return
        }
        // Looked up once for every tile.
        let content = try? await ScreenCapture.content()
        guard !Task.isCancelled else { return }

        var started: [ScreenCapture] = []
        var problems: [String] = []
        // A wide canvas is captured in tiles, so the parts out of view can
        // update less often. Tiles are laid out in points and captured at
        // the screen's own pixels.
        for columns in captureTiles(width: screen.width) {
            let tile = CGRect(x: columns.lowerBound, y: 0, width: columns.count, height: screen.height)
            let (capture, problem) = await startCapture(
                displayID: screen.displayID, content: content,
                pixelSize: (columns.count * screen.scale, screen.height * screen.scale),
                sourceRect: screen.width == columns.count ? nil : tile, fps: rate, showsCursor: !livePointer)
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
        displayID: CGDirectDisplayID, content: SCShareableContent?, pixelSize: (width: Int, height: Int),
        sourceRect: CGRect?, fps: Int, showsCursor: Bool
    ) async -> (ScreenCapture, String?) {
        do {
            let capture = try ScreenCapture(device: device)
            capture.onStop = { [weak self] in self?.restartCaptures(after: captureRestartDelay) }
            do {
                guard let content else { throw CaptureError.displayNotShareable }
                try await capture.start(
                    displayID: displayID, in: content, pixelSize: pixelSize, sourceRect: sourceRect, fps: fps,
                    showsCursor: showsCursor, excludedWindowID: window.windowID)
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

    // MARK: Above the canvas

    private func updateDashboard() {
        let (enabled, info) = shared.mutex.withLock { ($0.settings.statusStrip, $0.hudInfo(now: monotonicNow())) }
        guard enabled, glassesConnected else {
            dashboard.update(glasses: nil)
            return
        }
        let tracking = info.tracking
        dashboard.update(
            glasses: GlassesReadings(
                temperature: tracking.temperature,
                trackingHz: tracking.status == .connected ? tracking.sampleRateHz : nil, fps: info.stats.fps,
                latency: info.timing.lastLead.map { $0 + glassesDisplayDelay },
                lateFramesPerSecond: info.timing.lateFramesPerSecond(now: info.now)))
    }

    private func updatePointer() {
        guard glassesConnected, shared.mutex.withLock({ $0.settings.livePointer }) else {
            pointerImage.hide()
            return
        }
        pointerImage.update()
    }

    private func restartPinned() {
        timingLog.notice("Looking for the pinned window")
        pinnedTask?.cancel()
        pinnedTask = Task { await switchPinned() }
    }

    /// Captures the pinned window, if there is one to show.
    private func switchPinned() async {
        pinnedWindowStatus = nil
        if let old = pinnedCapture {
            pinnedCapture = nil
            await old.stop()
        }
        guard !Task.isCancelled else { return }
        pinnedTarget = nil
        shared.mutex.withLock { $0.pinned = nil }
        guard let wanted = shared.mutex.withLock({ $0.settings.pinnedWindow }) else { return }
        guard Displays.glassesDisplay() != nil else {
            pinnedWindowStatus = "Connect the glasses to show this window."
            return
        }
        do {
            let content = try await ScreenCapture.content()
            guard !Task.isCancelled else { return }
            guard let found = ScreenCapture.find(wanted, in: content, preferredID: preferredPinnedWindowID) else {
                pinnedWindowStatus = "Window unavailable or ambiguous. Open it or choose a window again."
                return
            }
            let capture = try ScreenCapture(device: device)
            // As when the window closes: found again once it is back.
            capture.onStop = { [weak self] in
                Task {
                    try? await Task.sleep(for: .seconds(captureRestartDelay))
                    self?.restartPinned()
                }
            }
            try await capture.start(window: found, fps: pinnedFps, maxPixels: maxPinnedPixels)
            guard !Task.isCancelled else {
                await capture.stop()
                return
            }
            pinnedCapture = capture
            preferredPinnedWindowID = found.windowID
            pinnedTarget = (found.windowID, found.frame.size)
            shared.mutex.withLock { $0.pinned = capture.latest }
        } catch {
            guard !Task.isCancelled else { return }
            pinnedWindowStatus = "Could not capture the window: \(error.localizedDescription)"
            eprint("Could not capture the pinned window: \(error.localizedDescription)")
        }
    }

    /// Follows the pinned window as it is resized, and finds it again once
    /// it is closed and opened again.
    private func checkPinnedWindow() {
        guard shared.mutex.withLock({ $0.settings.pinnedWindow }) != nil, glassesConnected else { return }
        guard let target = pinnedTarget, let capture = pinnedCapture else {
            restartPinned()
            return
        }
        let info = CGWindowListCopyWindowInfo([.optionIncludingWindow], target.id) as? [[String: Any]]
        guard let bounds = info?.first?[kCGWindowBounds as String] as? NSDictionary,
            let frame = CGRect(dictionaryRepresentation: bounds)
        else {
            restartPinned()
            return
        }
        if frame.size != target.size {
            Task {
                if await capture.resizeWindow(to: frame.size), capture === pinnedCapture {
                    pinnedTarget = (target.id, frame.size)
                }
            }
        }
    }

}
