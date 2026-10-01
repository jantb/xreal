import Foundation
import simd

// The mouse left alone this long counts as not in use.
private let idleBeforeGlide = 0.5  // seconds
// How quickly the pointer glides to where the viewer looks: most of the way
// in about a third of a second.
private let glideTime: Float = 0.12  // seconds
// Closer than this, in points, the pointer has arrived.
private let arrivedWithin: Float = 2
// Where the viewer looks moving less than this a frame, in points, has
// settled, and the glide ends once the pointer is there.
private let settledWithin: Float = 1
// Moves smaller than this, in points, are not the mouse being used: macOS
// rounds where the pointer is put by up to about 0.7 of a point.
private let moveThreshold: Float = 1.5
// The most the glide aims ahead of the gaze, in points, so a sudden jump in
// where the viewer looks cannot fling the pointer past it.
private let maxLead: Float = 800

/// Brings the mouse pointer to where the viewer looks once it is left out
/// of view, gliding rather than jumping, and keeps it with the gaze until
/// the head settles. While it is in view, or while the mouse is in use, it
/// stays where it is.
public struct PointerGlide: Sendable {
    public private(set) var gliding = false
    /// Where the pointer should be if only this moved it, and where it
    /// should have been a frame earlier, as a move can show a frame late.
    private var expected: SIMD2<Float>?
    private var previous: SIMD2<Float>?
    private var lastTarget: SIMD2<Float>?
    private var usedAt = -Double.infinity

    public init() {}

    /// `cursor` is the pointer in global points while it is on the canvas,
    /// `inView` whether the view shows it, and `target` where the viewer
    /// looks, nil when not at the canvas; `bounds` where the canvas is, in
    /// global points, which the pointer is never moved out of. Returns where
    /// to move the pointer to, or nil to leave it.
    public mutating func update(
        cursor: SIMD2<Float>?, inView: Bool, target: SIMD2<Float>?, enabled: Bool, now: Double, dt: Float,
        bounds: (min: SIMD2<Float>, max: SIMD2<Float>)? = nil
    ) -> SIMD2<Float>? {
        defer { lastTarget = target }
        guard var cursor else {
            gliding = false
            expected = nil
            previous = nil
            return nil
        }
        func near(_ point: SIMD2<Float>?) -> Bool { point.map { simd_distance(cursor, $0) <= moveThreshold } ?? false }
        if let expected, !near(expected) {
            if near(previous) {
                // The last move has not shown yet: carry on from it.
                cursor = expected
            } else {
                usedAt = now
                gliding = false
            }
        } else if let expected {
            // Where it was put, not where macOS rounded it to.
            cursor = expected
        }
        previous = expected ?? cursor
        expected = cursor
        guard enabled, let target else {
            gliding = false
            return nil
        }
        if !gliding, !inView, now - usedAt > idleBeforeGlide {
            gliding = true
        }
        guard gliding else { return nil }
        // Aimed ahead by how far the gaze moves in the glide's time, so it
        // keeps up with a moving gaze instead of trailing behind it.
        var lead = dt > 0 ? (lastTarget.map { (target - $0) / dt } ?? .zero) * glideTime : .zero
        if simd_length(lead) > maxLead {
            lead *= maxLead / simd_length(lead)
        }
        var next = cursor + (target + lead - cursor) * (1 - exp(-max(dt, 0) / glideTime))
        let settled = lastTarget.map { simd_distance($0, target) < settledWithin } ?? false
        if simd_distance(next, target) < arrivedWithin {
            next = target
            // Still with the gaze while the head moves on.
            if settled {
                gliding = false
            }
        }
        if let bounds {
            next = simd_clamp(next, bounds.min, bounds.max)
        }
        expected = next
        return next
    }
}
