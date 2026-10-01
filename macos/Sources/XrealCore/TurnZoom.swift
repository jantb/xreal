import Foundation

// At the fastest turns the view takes in this many times as much canvas
// each way.
let turnZoomOut: Float = 1.5
// Zooms out from this turning speed to fully at the next, faster than
// panning along the canvas: only a quick look elsewhere zooms.
private let zoomFrom: Float = 0.15  // rad/s, about 9°/s
private let zoomFully: Float = 0.8  // rad/s, about 46°/s
private let zoomOutTime: Float = 0.1  // seconds
private let zoomInTime: Float = 0.15  // seconds: nearly back in under half a second

/// Zooms out while the head turns quickly, to show more of the canvas on
/// the way to where it is going, and back in as it slows, to land sharp.
/// Slower turning, from `zoomFrom` down, never zooms.
public struct TurnZoom: Sendable {
    /// How far the view is zoomed out, 1 meaning not, as `CursorFollow`'s.
    public private(set) var scale: Float = 1

    public init() {}

    /// `speed` is how fast the head turns, in rad/s. Returns the new scale.
    public mutating func update(speed: Float, enabled: Bool, dt: Float) -> Float {
        let quick = enabled ? smoothstep(zoomFrom, zoomFully, speed) : 0
        let target = 1 / (1 + (turnZoomOut - 1) * quick)
        let time = target < scale ? zoomOutTime : zoomInTime
        scale += (target - scale) * (1 - exp(-max(dt, 0) / time))
        // Land exactly on no zoom, where the canvas has its own pixel density.
        if abs(target - scale) < 1e-3 {
            scale = target
        }
        return scale
    }
}
