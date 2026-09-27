import Foundation
import simd

// While following, the cursor is kept inside this part of the view...
private let followMargin: Float = 0.9
// ...and following ends once it is back well inside the view at the chosen
// zoom, so the zoom does not flicker at the edge.
private let releaseMargin: Float = 0.8
// A cursor left alone this long stops holding the view zoomed out.
private let idleRelease = 3.0  // seconds
private let minScale: Float = 0.05
private let zoomOutTime: Float = 0.08  // seconds
private let zoomInTime: Float = 0.25  // seconds
// Smaller moves, in source pixels, are not treated as the mouse moving.
private let moveThreshold: Float = 0.5

/// Zooms out while the mouse moves outside the view, so the cursor stays in
/// sight, and back in once the cursor is inside the view again.
public struct CursorFollow: Sendable {
    /// How far the view is zoomed out beyond the chosen zoom, 1 meaning not.
    public private(set) var scale: Float = 1
    private var following = false
    private var lastCursor: SIMD2<Float>?
    private var movedAt = -Double.infinity

    public init() {}

    /// `cursor` is in source pixels, or nil when it is on another display.
    /// `shows(scale, margin)` says whether the view zoomed out by `scale`
    /// shows the cursor inside `margin` of its size. Returns the new scale.
    public mutating func update(
        cursor: SIMD2<Float>?, enabled: Bool, now: Double, dt: Float, shows: (Float, Float) -> Bool
    ) -> Float {
        let moved = cursor.flatMap { cursor in lastCursor.map { simd_distance($0, cursor) > moveThreshold } } ?? false
        lastCursor = cursor
        if moved {
            movedAt = now
        }

        if !enabled || cursor == nil || shows(1, releaseMargin) || now - movedAt > idleRelease {
            following = false
        } else if moved {
            following = true
        }

        let target = following ? largestScale(shows) : 1
        let time = target < scale ? zoomOutTime : zoomInTime
        scale += (target - scale) * (1 - exp(-max(dt, 0) / time))
        // Land exactly on the chosen zoom, where the crop is pixel-exact.
        if abs(target - scale) < 1e-3 {
            scale = target
        }
        return scale
    }

    private func largestScale(_ shows: (Float, Float) -> Bool) -> Float {
        if shows(1, followMargin) { return 1 }
        var visible = minScale
        guard shows(visible, followMargin) else { return minScale }
        var hidden: Float = 1
        for _ in 0..<12 {
            let middle = (visible + hidden) * 0.5
            if shows(middle, followMargin) {
                visible = middle
            } else {
                hidden = middle
            }
        }
        return visible
    }
}
