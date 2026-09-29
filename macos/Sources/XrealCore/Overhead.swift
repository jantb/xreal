import Foundation
import simd

// Points between the canvas's top edge and what hangs above it.
private let overheadGap: Float = 72
// Points between the dashboard and the pinned window.
private let overheadSpacing: Float = 32

/// A part of a screen's surface, in its positions: from -1 to 1 across, and
/// from -1 at the bottom to 1 at the top, beyond 1 above it. The surface
/// carries on past its edges the way it runs: a curved screen's rows keep
/// their arc and its columns stay straight.
public struct SurfaceRect: Equatable, Sendable {
    public var left: Float
    public var right: Float
    public var top: Float
    public var bottom: Float

    public init(left: Float, right: Float, top: Float, bottom: Float) {
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom
    }

    /// The whole screen.
    public static let whole = SurfaceRect(left: -1, right: 1, top: 1, bottom: -1)
}

/// Where the dashboard and the pinned window hang above the canvas: how
/// tall the row they hang in is, in points, and where each is in it, on
/// `overheadSurface`.
public struct OverheadLayout: Equatable, Sendable {
    public var height: Float
    public var dashboard: SurfaceRect?
    public var pinned: SurfaceRect?
}

/// Lays out the dashboard and the pinned window, `dashboard` and `pinned`
/// points in size: the dashboard centred and the window beside it on the
/// right, both resting on the row's bottom edge. Where the canvas is too
/// narrow for both, the window goes above the dashboard instead; either is
/// shrunk to the canvas's width if it is wider.
public func overheadLayout(canvas: RoomScreen, dashboard: SIMD2<Float>?, pinned: SIMD2<Float>?) -> OverheadLayout {
    let width = Float(max(canvas.width, 1))
    func fitted(_ extent: SIMD2<Float>?) -> SIMD2<Float>? {
        guard let extent, extent.x > 0, extent.y > 0 else { return nil }
        return extent * min(1, width / extent.x)
    }
    let board = fitted(dashboard)
    let window = fitted(pinned)
    let beside = board.map { board in window.map { board.x / 2 + overheadSpacing + $0.x <= width / 2 } ?? true } ?? true
    let height: Float
    if let board, let window {
        height = beside ? max(board.y, window.y) : board.y + overheadSpacing + window.y
    } else {
        height = board?.y ?? window?.y ?? 0
    }
    guard height > 0 else { return OverheadLayout(height: 0) }
    // Points across from the middle and up from the row's bottom edge, to
    // surface positions.
    func rect(left: Float, bottom: Float, _ extent: SIMD2<Float>) -> SurfaceRect {
        SurfaceRect(
            left: left * 2 / width, right: (left + extent.x) * 2 / width, top: -1 + (bottom + extent.y) * 2 / height,
            bottom: -1 + bottom * 2 / height)
    }
    let boardRect = board.map { rect(left: -$0.x / 2, bottom: 0, $0) }
    let windowRect = window.map { window -> SurfaceRect in
        guard let board else { return rect(left: -window.x / 2, bottom: 0, window) }
        return beside
            ? rect(left: board.x / 2 + overheadSpacing, bottom: 0, window)
            : rect(left: -window.x / 2, bottom: board.y + overheadSpacing, window)
    }
    return OverheadLayout(height: height, dashboard: boardRect, pinned: windowRect)
}

/// The surface of the row `height` points tall above the canvas: as wide as
/// the canvas and curved like it, starting just above the highest its top
/// edge reaches and leaning towards the viewer as it rises, so it faces the
/// eyes like a monitor hung overhead and angled down. Curved, it leans the
/// same way all the way round, so its bottom edge follows the canvas's top
/// edge.
///
/// `raised` points lifts its bottom edge further, for a panel resting on
/// another.
public func overheadSurface(canvas: RoomScreen, curveRadius: Float = 1, height: Float, raised: Float = 0)
    -> ScreenSurface
{
    let canvasSurface = canvas.surface(curveRadius: curveRadius)
    let distance = canvas.placement.distance
    let turn = canvas.placement.orientation
    // How high the canvas's top edge reaches, seen from the eyes as the
    // canvas's own up goes: a wrapped canvas's reaches higher than a flat
    // or curved one's of the same size.
    var edge = -Float.pi / 2
    for step in 0...32 {
        let point = turn.inverse.act(canvasSurface.point(at: SIMD2(-1 + Float(step) / 16, 1)))
        edge = max(edge, atan2(point.y, simd_length(SIMD2(point.x, point.z))))
    }
    let bottom = distance * tan(min(edge, 1.3)) + (overheadGap + raised) * roomUnitsPerPixel
    let halfRow = max(height, 1) * roomUnitsPerPixel / 2
    // Square to the line of sight at the row's middle, which itself moves
    // with the lean: a few rounds settle it. Upright it is shortened, so
    // the leaning row keeps its height along its slope.
    var lean = (bottom + halfRow) / distance
    var halfUp = halfRow / (1 + lean * lean).squareRoot()
    for _ in 0..<8 {
        lean = (bottom + halfUp) / max(distance - lean * halfUp, 1e-3)
        halfUp = halfRow / (1 + lean * lean).squareRoot()
    }
    // Curved like the canvas; over a wrapped one, round the eyes as its rows
    // are, so its bottom edge runs level.
    let halfWidth = Float(canvas.width) * roomUnitsPerPixel / 2
    return ScreenSurface(
        center: SIMD3(0, bottom + halfUp, -distance), right: SIMD3(halfWidth, 0, 0), up: SIMD3(0, halfUp, 0),
        halfArc: canvas.spherical ? min(halfWidth / distance, 2.9) : canvasSurface.halfArc, spin: turn, lean: lean)
}

/// Where the dashboard and the pinned window hang, each as a surface and
/// the part of it they cover: each a strip of `overheadSurface` from its own
/// bottom, tilted to face the viewer at its own middle. Over a wrapped
/// canvas each is instead part of the same sphere round the eyes, laid out
/// as the canvas's rows are, so it curves both ways like the canvas below
/// it and faces the eyes from every pixel.
public func overheadPanels(canvas: RoomScreen, curveRadius: Float = 1, layout: OverheadLayout)
    -> (dashboard: (surface: ScreenSurface, rect: SurfaceRect)?, pinned: (surface: ScreenSurface, rect: SurfaceRect)?)
{
    let turn = canvas.placement.orientation
    let distance = canvas.placement.distance
    func panel(_ rect: SurfaceRect) -> (surface: ScreenSurface, rect: SurfaceRect) {
        let points = { (y: Float) in (y + 1) / 2 * layout.height }
        let height = points(rect.top) - points(rect.bottom)
        let strip = overheadSurface(
            canvas: canvas, curveRadius: curveRadius, height: height, raised: points(rect.bottom))
        guard canvas.spherical else { return (strip, SurfaceRect(left: rect.left, right: rect.right, top: 1, bottom: -1)) }
        // On the sphere round the eyes, laid out as the canvas's rows are:
        // its bottom edge runs level along the canvas's top edge, and every
        // pixel faces the eyes, square. Mercator rows shrink by cos(rise), so
        // the panel is drawn larger by as much at its middle.
        let seen = { (point: SIMD3<Float>) -> (across: Float, rise: Float) in
            let local = turn.inverse.act(point)
            return (atan2(local.x, -local.z), atan2(local.y, simd_length(SIMD2(local.x, local.z))))
        }
        let across = seen(strip.point(at: SIMD2((rect.left + rect.right) / 2, 0))).across
        let bottom = mercatorHeight(seen(strip.point(at: SIMD2((rect.left + rect.right) / 2, -1))).rise)
        let perPoint = roomUnitsPerPixel / distance
        let width = (rect.right - rect.left) / 2 * Float(canvas.width) * perPoint
        var top = bottom + height * perPoint
        for _ in 0..<4 {
            top = bottom + height * perPoint / cos(mercatorRise((bottom + top) / 2))
        }
        let halfWidth = width / 2 / cos(mercatorRise((bottom + top) / 2))
        // Across as an angle over π, up as the Mercator height itself.
        let sphere = ScreenSurface(
            center: SIMD3(0, 0, -distance), right: SIMD3(distance * .pi, 0, 0), up: SIMD3(0, distance, 0),
            halfArc: .pi, spin: turn, wrap: 1)
        return (
            sphere,
            SurfaceRect(left: (across - halfWidth) / .pi, right: (across + halfWidth) / .pi, top: top, bottom: bottom)
        )
    }
    let dashboard = layout.dashboard.map(panel)
    guard canvas.spherical, let window = layout.pinned else { return (dashboard, layout.pinned.map(panel)) }
    // The pinned window is a monitor of its own: a small wrapped screen
    // centred on its own middle and turned to face the eyes, curving round
    // its middle as the canvas does round its own. Placed by angle, clear of
    // the canvas below and of the dashboard beside it.
    let perPoint = roomUnitsPerPixel / distance
    let halfWidth = (window.right - window.left) / 2 * Float(canvas.width) * roomUnitsPerPixel / 2
    let halfHeight = (window.top - window.bottom) / 2 * layout.height * roomUnitsPerPixel / 2
    let (acrossHalf, riseHalf) = (halfWidth / distance, halfHeight / distance)
    let gap = overheadSpacing * perPoint
    let base = panel(SurfaceRect(left: window.left, right: window.right, top: window.bottom + 0.01, bottom: window.bottom))
    var across = base.rect.left * .pi + acrossHalf
    var bottom = mercatorRise(base.rect.bottom)
    if let board = dashboard?.rect, let under = layout.dashboard {
        if window.left > under.right {
            across = board.right * .pi + gap + acrossHalf
        } else {
            across = (board.left + board.right) / 2 * .pi
            bottom = mercatorRise(board.top) + gap
        }
    }
    // Turned up to face the eyes, its bottom corners fall a little below its
    // bottom middle; lifted by as much.
    var rise = bottom + riseHalf
    for _ in 0..<4 {
        rise = bottom + riseHalf + acrossHalf * acrossHalf / 2 * tan(rise)
    }
    func monitor(across: Float) -> ScreenSurface {
        let forward = SIMD3(sin(across) * cos(rise), sin(rise), -cos(across) * cos(rise))
        let right = normalize(cross(forward, SIMD3(0, 1, 0)))
        let facing = simd_quatf(simd_float3x3(right, cross(right, forward), -forward))
        return ScreenSurface(
            center: SIMD3(0, 0, -distance), right: SIMD3(halfWidth, 0, 0), up: SIMD3(0, halfHeight, 0),
            halfArc: acrossHalf, spin: turn * facing, wrap: 1)
    }
    var placed = monitor(across: across)
    // Its sides lean once it is turned up, so its corners come closer to the
    // dashboard than its middle: moved over until all of its near side
    // keeps the gap.
    if let board = dashboard?.rect, let under = layout.dashboard, window.left > under.right {
        let azimuth = { (point: SIMD3<Float>) -> Float in
            let local = turn.inverse.act(point)
            return atan2(local.x, -local.z)
        }
        for _ in 0..<4 {
            let nearest = [Float(-1), -0.5, 0, 0.5, 1].map { azimuth(placed.point(at: SIMD2(-1, $0))) }.min() ?? across
            let short = board.right * .pi + gap - nearest
            guard short > 1e-5 else { break }
            across += short
            placed = monitor(across: across)
        }
    }
    return (dashboard, (placed, .whole))
}

/// Where a pointer image `size` points large, with its hot spot `hotSpot`
/// points from its top-left corner, covers the canvas when the mouse is at
/// `point`, in the canvas's points from its top-left corner.
public func pointerRect(canvas: RoomScreen, at point: SIMD2<Float>, size: SIMD2<Float>, hotSpot: SIMD2<Float>)
    -> SurfaceRect
{
    let extent = SIMD2(Float(max(canvas.width, 1)), Float(max(canvas.height, 1)))
    let topLeft = (point - hotSpot) / extent
    let bottomRight = (point - hotSpot + size) / extent
    return SurfaceRect(
        left: topLeft.x * 2 - 1, right: bottomRight.x * 2 - 1, top: 1 - topLeft.y * 2, bottom: 1 - bottomRight.y * 2)
}

