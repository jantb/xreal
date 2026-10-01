import Foundation

// Zooms out from this turning speed to fully at the next, faster than
// panning along the canvas: only a quick look elsewhere zooms.
private let zoomFrom: Float = 0.15  // rad/s, about 9°/s
private let zoomFully: Float = 0.8  // rad/s, about 46°/s
// How quickly it glides, as a spring that settles without overshooting:
// out gently, most of the way in about 0.8 s, and back in within 0.4 s,
// to land sharp.
private let zoomOutRate: Float = 6  // 1/s
private let zoomInRate: Float = 12  // 1/s
// The furthest out it zooms, however large the canvas or however far to
// its side the head points: past this the view's edges stretch too far.
let widestTurnZoom: Float = 0.1
// The whole canvas fits inside this part of the view.
private let wholeCanvasMargin: Float = 0.95

/// Zooms out while the head turns quickly, far enough to show the whole
/// canvas on the way to where it is going, and back in as it slows, to
/// land sharp. Slower turning, from `zoomFrom` down, never zooms.
public struct TurnZoom: Sendable {
    /// How far the view is zoomed out, 1 meaning not, as `CursorFollow`'s.
    public private(set) var scale: Float = 1
    /// How fast the scale is changing, per second.
    private var rate: Float = 0

    public init() {}

    /// `speed` is how fast the head turns, in rad/s; `whole` the scale that
    /// shows the whole canvas, from `wholeCanvasScale`. Returns the new
    /// scale.
    public mutating func update(speed: Float, enabled: Bool, whole: Float, dt: Float) -> Float {
        let quick = enabled ? smoothstep(zoomFrom, zoomFully, speed) : 0
        let wider = 1 / min(max(whole, widestTurnZoom), 1)
        let target = 1 / (1 + (wider - 1) * quick)
        // Starts gently rather than at its fastest, so it glides out
        // instead of lurching.
        // Stepped exactly, so a late frame cannot throw it off.
        let spring = target < scale ? zoomOutRate : zoomInRate
        let t = max(dt, 0)
        let away = scale - target
        let carry = rate + spring * away
        let decay = exp(-spring * t)
        scale = target + (away + carry * t) * decay
        rate = (rate - spring * carry * t) * decay
        // A target easing back as the scale races towards it can carry it a
        // little past; never past no zoom, nor further out than allowed.
        if scale > 1 || scale < widestTurnZoom {
            scale = min(max(scale, widestTurnZoom), 1)
            rate = 0
        }
        // Land exactly on no zoom, where the canvas has its own pixel density.
        if abs(target - scale) < 1e-3, abs(rate) < 1e-2 {
            scale = target
            rate = 0
        }
        return scale
    }

    /// The least zoomed out scale at which the view shows every one of
    /// `points`, the canvas's corners and edges, as `shows(point, scale,
    /// margin)` says; `widestTurnZoom` if even that does not, as with part
    /// of the canvas behind the viewer.
    public static func wholeCanvasScale(
        _ points: [SIMD3<Float>], shows: (SIMD3<Float>, Float, Float) -> Bool
    ) -> Float {
        func showsAll(_ scale: Float) -> Bool { points.allSatisfy { shows($0, scale, wholeCanvasMargin) } }
        if showsAll(1) { return 1 }
        guard showsAll(widestTurnZoom) else { return widestTurnZoom }
        var shown = widestTurnZoom
        var hidden: Float = 1
        for _ in 0..<12 {
            let middle = (shown + hidden) * 0.5
            if showsAll(middle) {
                shown = middle
            } else {
                hidden = middle
            }
        }
        return shown
    }
}
