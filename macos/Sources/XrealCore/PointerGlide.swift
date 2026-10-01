import Foundation
import simd

// The mouse left alone this long counts as not in use.
private let idleBeforeGlide = 0.5  // seconds
// How quickly the pointer glides to where the viewer looks: most of the way
// in about a third of a second.
private let glideTime: Float = 0.12  // seconds
// Closer than this, in points, the pointer has arrived.
private let arrivedWithin: Float = 2
// Moves smaller than this, in points, are not the mouse being used.
private let moveThreshold: Float = 0.5

/// Brings the mouse pointer to where the viewer looks once it is left out
/// of view, gliding rather than jumping. While it is in view, or while the
/// mouse is in use, it stays where it is.
public struct PointerGlide: Sendable {
    public private(set) var gliding = false
    /// Where the pointer should be if only this moved it, and where it
    /// should have been a frame earlier, as a move can show a frame late.
    private var expected: SIMD2<Float>?
    private var previous: SIMD2<Float>?
    private var usedAt = -Double.infinity

    public init() {}

    /// `cursor` is the pointer in global points while it is on the canvas,
    /// `inView` whether the view shows it, and `target` where the viewer
    /// looks, nil when not at the canvas. Returns where to move the pointer
    /// to, or nil to leave it.
    public mutating func update(
        cursor: SIMD2<Float>?, inView: Bool, target: SIMD2<Float>?, enabled: Bool, now: Double, dt: Float
    ) -> SIMD2<Float>? {
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
        var next = cursor + (target - cursor) * (1 - exp(-max(dt, 0) / glideTime))
        if simd_distance(next, target) < arrivedWithin {
            next = target
            gliding = false
        }
        expected = next
        return next
    }
}
