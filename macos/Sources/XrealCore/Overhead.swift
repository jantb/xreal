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
/// the part of it they cover. Over a flat or curved canvas each is a strip
/// of `overheadSurface` from its own bottom, flat and tilted to face the
/// viewer at its own middle. Over a wrapped canvas each is instead a patch
/// of a sphere round the eyes turned towards its own middle, placed by
/// angle clear of the canvas and of the other.
public func overheadPanels(canvas: RoomScreen, curveRadius: Float = 1, layout: OverheadLayout)
    -> (dashboard: (surface: ScreenSurface, rect: SurfaceRect)?, pinned: (surface: ScreenSurface, rect: SurfaceRect)?)
{
    guard layout.height > 0 else { return (nil, nil) }
    guard canvas.spherical else {
        func strip(_ rect: SurfaceRect) -> (surface: ScreenSurface, rect: SurfaceRect) {
            let points = { (y: Float) in (y + 1) / 2 * layout.height }
            let surface = overheadSurface(
                canvas: canvas, curveRadius: curveRadius, height: points(rect.top) - points(rect.bottom),
                raised: points(rect.bottom))
            return (surface, SurfaceRect(left: rect.left, right: rect.right, top: 1, bottom: -1))
        }
        return (layout.dashboard.map(strip), layout.pinned.map(strip))
    }
    let turn = canvas.placement.orientation
    let distance = canvas.placement.distance
    let perPoint = roomUnitsPerPixel / distance
    let canvasSurface = canvas.surface(curveRadius: curveRadius)
    func elevation(_ p: SIMD3<Float>) -> Float { atan2(p.y, length(SIMD2(p.x, p.z))) }
    let edge = (0...128).map { step in
        elevation(turn.inverse.act(canvasSurface.point(at: SIMD2(-1 + Float(step) / 64, 1))))
    }.max() ?? 0
    // The highest a panel may reach, short of straight overhead.
    let ceiling: Float = 1.48
    let gap = min(overheadGap * perPoint, max(ceiling - edge, 0.001) / 6)
    let spacing = min(overheadSpacing * perPoint, gap)

    func extent(_ rect: SurfaceRect) -> SIMD2<Float> {
        SIMD2((rect.right - rect.left) * Float(canvas.width), (rect.top - rect.bottom) * layout.height) * perPoint / 4
    }
    let beside = layout.dashboard.map { board in layout.pinned.map { $0.left > board.right } ?? true } ?? true
    let boardExtent = layout.dashboard.map(extent)
    let windowExtent = layout.pinned.map(extent)
    // Fit both panels uniformly, preserving their aspect ratios. A stacked
    // pair shares the available height; a side-by-side pair shares its row.
    let needed = beside
        ? 2 * max(boardExtent?.y ?? 0, windowExtent?.y ?? 0)
        : 2 * ((boardExtent?.y ?? 0) + (windowExtent?.y ?? 0)) + spacing
    let widest = max(boardExtent?.x ?? 0, windowExtent?.x ?? 0)
    let fit = min(1, max(ceiling - edge - 2 * gap, 0.001) / max(needed, 0.001), 0.9 / max(widest, 0.001))

    func surface(half: SIMD2<Float>, across: Float, rise: Float) -> ScreenSurface {
        let forward = SIMD3(sin(across) * cos(rise), sin(rise), -cos(across) * cos(rise))
        let right = normalize(cross(forward, SIMD3(0, 1, 0)))
        let facing = simd_quatf(simd_float3x3(right, cross(right, forward), -forward))
        return ScreenSurface(
            center: SIMD3(0, 0, -distance), right: SIMD3(half.x * distance, 0, 0), up: SIMD3(0, half.y * distance, 0),
            halfArc: half.x, spin: turn * facing, wrap: 1)
    }
    func bounds(_ panel: ScreenSurface) -> (left: Float, right: Float, bottom: Float, top: Float) {
        var result = (left: Float.infinity, right: -Float.infinity, bottom: Float.infinity, top: -Float.infinity)
        for x in 0...16 {
            for y in 0...16 {
                let p = turn.inverse.act(panel.point(at: SIMD2(-1 + Float(x) / 8, -1 + Float(y) / 8)))
                let a = atan2(p.x, -p.z)
                let e = elevation(p)
                result.left = min(result.left, a)
                result.right = max(result.right, a)
                result.bottom = min(result.bottom, e)
                result.top = max(result.top, e)
            }
        }
        return result
    }
    func placed(_ extent: SIMD2<Float>, above bottom: Float, below top: Float) -> ScreenSurface {
        var half = extent * fit
        for _ in 0..<80 {
            let latitude = mercatorRise(half.y)
            // The bottom corners, not the centre of the bottom edge,
            // determine clearance after the patch is tilted upwards.
            let a = cos(latitude) * cos(half.x)
            let b = sin(latitude)
            let radius = sqrt(a * a + b * b)
            if sin(bottom) < radius {
                let rise = atan2(b, a) + asin(sin(bottom) / radius) + 1e-5
                let azimuth = atan2(sin(half.x), cos(rise) * cos(half.x) - sin(rise) * tan(latitude))
                if rise + latitude <= top && azimuth < 0.65 {
                    return surface(half: half, across: 0, rise: rise)
                }
            }
            half *= 0.9
        }
        return surface(half: half, across: 0, rise: bottom + mercatorRise(half.y))
    }
    let stackHeight = max(ceiling - edge - 2 * gap - spacing, 0.001)
    let boardCeiling = !beside && windowExtent != nil
        ? edge + gap + stackHeight * (boardExtent?.y ?? 0) / max((boardExtent?.y ?? 0) + (windowExtent?.y ?? 0), 0.001)
        : ceiling
    let board = boardExtent.map { placed($0, above: edge + gap, below: boardCeiling) }
    let windowBottom = beside ? edge + gap : board.map { bounds($0).top + spacing } ?? edge + gap
    var window = windowExtent.map { placed($0, above: windowBottom, below: ceiling) }
    if beside, let board, let original = window {
        // A yaw rotation preserves elevation, so this separation cannot
        // undo clearance from the canvas or the ceiling.
        let b = bounds(board)
        let w = bounds(original)
        let shift = b.right + spacing - w.left
        var moved = original
        moved.spin = turn * simd_quatf(angle: -shift, axis: SIMD3(0, 1, 0)) * turn.inverse * original.spin
        window = moved
    }
    return (board.map { ($0, .whole) }, window.map { ($0, .whole) })
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
