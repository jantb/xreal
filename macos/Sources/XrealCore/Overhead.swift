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
/// the canvas and curved like it, starting just above its top edge and
/// leaning towards the viewer as it rises, so it faces the eyes like a
/// monitor hung overhead and angled down. Curved, it leans the same way all
/// the way round, so its bottom edge follows the canvas's top edge.
public func overheadSurface(canvas: RoomScreen, curveRadius: Float = 1, height: Float) -> ScreenSurface {
    let canvasSurface = canvas.surface(curveRadius: curveRadius)
    let distance = canvas.placement.distance
    let bottom = Float(canvas.height) * roomUnitsPerPixel / 2 + overheadGap * roomUnitsPerPixel
    let halfRow = max(height, 1) * roomUnitsPerPixel / 2
    // Square to the line of sight at the row's middle.
    let lean = (bottom + halfRow) / distance
    // Shortened upright so the leaning row keeps its height along its slope.
    let halfUp = halfRow / (1 + lean * lean).squareRoot()
    return ScreenSurface(
        center: SIMD3(0, bottom + halfUp, -distance), right: SIMD3(Float(canvas.width) * roomUnitsPerPixel / 2, 0, 0),
        up: SIMD3(0, halfUp, 0), halfArc: canvasSurface.halfArc, spin: canvas.placement.orientation, lean: lean)
}

/// The surface the row above the canvas hangs on and where the dashboard
/// and the pinned window are on it: part of a sphere round the eyes, so
/// every pixel of them faces the viewer from the same distance and keeps
/// its size, whatever the canvas's shape. Its bottom edge runs level, a
/// little above the highest the canvas's top edge reaches. A row so tall it
/// would near the point straight overhead leans on `overheadSurface`
/// instead.
public func overheadPanels(canvas: RoomScreen, curveRadius: Float = 1, layout: OverheadLayout)
    -> (surface: ScreenSurface, dashboard: SurfaceRect?, pinned: SurfaceRect?)
{
    let canvasSurface = canvas.surface(curveRadius: curveRadius)
    let distance = canvas.placement.distance
    let turn = canvas.placement.orientation
    // How high the canvas's top edge reaches, seen from the eyes, as the
    // canvas's own up goes.
    var edge = -Float.pi / 2
    for step in 0...32 {
        let point = turn.inverse.act(canvasSurface.point(at: SIMD2(-1 + Float(step) / 16, 1)))
        edge = max(edge, atan2(point.y, simd_length(SIMD2(point.x, point.z))))
    }
    // Round the sphere, a point of the canvas is this many radians.
    let perPoint = roomUnitsPerPixel / distance
    let bottom = edge + overheadGap * perPoint
    let top = bottom + layout.height * perPoint
    let halfWidth = Float(canvas.width) * roomUnitsPerPixel / 2
    let halfArc = halfWidth / distance
    // Rows are widened by 1 / cos(rise) to keep their pixels' size.
    let widest = [layout.dashboard, layout.pinned].compactMap { $0 }.map { max(abs($0.left), abs($0.right)) }.max() ?? 0
    guard top < 1.4, widest * halfArc / cos(top) < 0.95 * .pi else {
        return (
            overheadSurface(canvas: canvas, curveRadius: curveRadius, height: layout.height), layout.dashboard,
            layout.pinned
        )
    }
    // `up` as long as the radius, so positions up it are angles.
    let sphere = ScreenSurface(
        center: SIMD3(0, 0, -distance), right: SIMD3(halfWidth, 0, 0), up: SIMD3(0, distance, 0), halfArc: halfArc,
        spin: turn, wrap: 1)
    func lifted(_ rect: SurfaceRect?) -> SurfaceRect? {
        rect.map {
            SurfaceRect(
                left: $0.left, right: $0.right, top: bottom + ($0.top + 1) / 2 * layout.height * perPoint,
                bottom: bottom + ($0.bottom + 1) / 2 * layout.height * perPoint)
        }
    }
    return (sphere, lifted(layout.dashboard), lifted(layout.pinned))
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

