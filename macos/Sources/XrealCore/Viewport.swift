import Foundation
import simd

// Horizontal field of view of each of the Air 2's displays, from the focal
// length in its factory calibration, about 2707 pixels across 1920 (the
// 46° diagonal it is sold with is slightly wider). The canvas at distance 1
// then has one pixel per glasses pixel. Each eye is drawn from its own
// calibrated optics; this sets the canvas's pixel size and what counts as
// in view.
let horizontalFov: Float = 2 * atan(960 / 2707)  // rad, about 39°

// The canvas ignores head wobble within this angle, so text holds on the
// same glasses pixels instead of shimmering as every tremor moves it a
// fraction of a pixel. About three glasses pixels.
let steadyRadius: Float = 3 * horizontalFov / 1920  // rad

/// Holds a value still until it moves more than `radius` away, then drags
/// along behind it at that distance: small wobble is ignored, and a real
/// turn is followed at once, never more than `radius` behind.
struct SoftLeash: Sendable {
    private var held: SIMD3<Float>?

    mutating func reset(_ value: SIMD3<Float>) {
        held = value
    }

    /// `value` holds angles; x is yaw, which wraps.
    mutating func follow(_ value: SIMD3<Float>, radius: Float) -> SIMD3<Float> {
        guard let held else {
            self.held = value
            return value
        }
        var offset = value - held
        offset.x = wrapAngle(offset.x)
        let distance = magnitude(offset)
        guard distance > radius else { return held }
        let dragged = value - offset * (radius / distance)
        self.held = dragged
        return dragged
    }
}

/// The most head movement is multiplied by: turning the head by a third
/// of the way turns the view the whole way.
public let maxHeadGain: Float = 3

/// The view never nods past straight up or down, however far the head
/// movement is multiplied.
private let maxViewPitch: Float = .pi / 2 - 0.01

// Head movement is multiplied only while the head keeps turning one way,
// however slowly. Holding still, the head's tremor and the pulse's jolts
// of a few pixels go back and forth and average out to under `slowTurn`,
// so the canvas stays still in the room; from `fastTurn`, a slow pan of
// under a degree and a half a second, the head is multiplied fully.
let slowTurn: Float = 0.01  // rad/s, about 0.6°/s
let fastTurn: Float = 0.025  // rad/s, about 1.4°/s
// How long the head's turning is averaged over, which back-and-forth
// wobble cancels out of.
private let turnSpeedSmoothing: Float = 0.3  // seconds
// How long the head's speed, either way, is smoothed over: briefly, so it
// follows a quick turn starting and stopping at once.
private let headSpeedSmoothing: Float = 0.05  // seconds

/// Turns head pose into how the head is turned in the room.
public struct ViewportController: Sendable {
    private var center = HeadPose()
    private var leash = SoftLeash()
    /// The turn and nod from straight ahead, as the head made them.
    private var headOffset = SIMD2<Float>.zero
    /// How much further than the head the view is turned and nodded:
    /// `gain - 1` times the head's offset once anchored, less while slow
    /// movement has left it behind.
    private var extraTurn = SIMD2<Float>.zero
    /// How fast and which way the head keeps turning and nodding, in rad/s.
    private var turning = SIMD2<Float>.zero
    /// How fast the head keeps turning and nodding one way, in rad/s.
    private var turnSpeed: Float { simd_length(turning) }
    /// How fast the head is turning and nodding just now, whichever way,
    /// in rad/s.
    public private(set) var headSpeed: Float = 0
    private var offsetYaw: Float = 0
    private var offsetPitch: Float = 0
    private var offsetRoll: Float = 0
    private var initialized = false
    public var followsRoll: Bool
    /// How many times further the view turns and nods than the head, so a
    /// wide canvas can be looked round with less head movement. Tilt is
    /// never multiplied. At 1 the canvas stays put in the room; above it,
    /// it slides the other way as the head turns. Each head direction
    /// keeps one place on the canvas, however slowly or quickly the head
    /// got there; only the wobble of holding still is not multiplied, so
    /// the canvas keeps still then, and what that leaves out of place is
    /// taken back by the next turn.
    public private(set) var gain: Float

    public init(settings: Settings) {
        followsRoll = settings.followRoll
        gain = clampedHeadGain(settings.headGain)
    }

    public func store(into settings: inout Settings) {
        settings.followRoll = followsRoll
        settings.headGain = gain
    }

    /// Changes how much head movement is multiplied. The view stays where
    /// it is and moves to its new place with the next quick turns.
    public mutating func setGain(_ gain: Float) {
        self.gain = clampedHeadGain(gain)
    }

    /// How many times further than the head the view turns at the head's
    /// present speed.
    private var currentGain: Float {
        1 + (gain - 1) * smoothstep(slowTurn, fastTurn, turnSpeed)
    }

    /// Makes `pose` the new straight ahead. Only the turn and nod are
    /// taken: tilt is measured against gravity, so level stays level
    /// however the head was tilted when recentering.
    public mutating func recenter(_ pose: HeadPose) {
        center = HeadPose(yaw: pose.yaw, pitch: pose.pitch)
        offsetYaw = 0
        offsetPitch = 0
        offsetRoll = 0
        headOffset = .zero
        extraTurn = .zero
        turning = .zero
        headSpeed = 0
        leash.reset(.zero)
        initialized = true
    }

    /// Follows the head to `pose`, `dt` seconds after the last one. The
    /// canvas turns with the head exactly, and further while it turns
    /// quickly, as any delay makes it lag behind, apart from wobble within
    /// `steadyRadius` of the view.
    public mutating func track(pose: HeadPose, dt: Float = 1 / 90) {
        if !initialized {
            recenter(pose)
        }
        let offset = SIMD2(wrapAngle(pose.yaw - center.yaw), pose.pitch - center.pitch)
        var moved = offset - headOffset
        moved.x = wrapAngle(moved.x)
        headOffset = offset
        if dt > 0 {
            let follow = 1 - exp(-dt / turnSpeedSmoothing)
            turning += (moved / dt - turning) * follow
            headSpeed += (simd_length(moved) / dt - headSpeed) * (1 - exp(-dt / headSpeedSmoothing))
        }
        let quick = smoothstep(slowTurn, fastTurn, turnSpeed)
        extraTurn += moved * (quick * (gain - 1))
        // Back towards its anchored place, only as far as turning hides it:
        // never while the head holds still. At most as fast as the
        // multiplying itself, so the view never turns against the head: at
        // worst it goes one to one for a moment, or twice as far ahead
        // again. Not with the head turned round behind, where the anchored
        // place, multiplied from a turn that wraps, jumps.
        var outOfPlace = offset * (gain - 1) - extraTurn
        outOfPlace.x = wrapAngle(outOfPlace.x)
        let facing = 1 - smoothstep(0.6 * .pi, 0.75 * .pi, abs(offset.x))
        let most = simd_length(moved) * quick * (gain - 1) * facing
        let distance = simd_length(outOfPlace)
        if distance > 0 {
            extraTurn += outOfPlace * min(1, most / distance)
        }
        extraTurn.x = wrapAngle(extraTurn.x)
        // What would nod the view past straight up or down is dropped, not
        // kept to pull it back later; the head alone may still go past.
        extraTurn.y = min(max(extraTurn.y, min(0, -maxViewPitch - offset.y)), max(0, maxViewPitch - offset.y))
        let turned = offset + extraTurn
        let raw = SIMD3(wrapAngle(turned.x), min(max(turned.y, -maxViewPitch), maxViewPitch), wrapAngle(pose.roll))
        let steady = leash.follow(raw, radius: steadyRadius)
        (offsetYaw, offsetPitch, offsetRoll) = (steady.x, steady.y, steady.z)
    }

    /// Turns head directions into room directions.
    public var headRotation: simd_float3x3 {
        headRotation(advancedBy: .zero)
    }

    /// `headRotation` with the head turned on by `turn` (yaw, pitch, roll),
    /// as it will be a moment later.
    public func headRotation(advancedBy turn: SIMD3<Float>) -> simd_float3x3 {
        let roll = followsRoll ? offsetRoll + turn.z : 0
        let gain = currentGain
        let pitch = min(max(offsetPitch + turn.y * gain, -maxViewPitch), maxViewPitch)
        return rotationY(offsetYaw + turn.x * gain) * rotationX(-pitch) * rotationZ(-roll)
    }

    /// The canvas as seen from the head, one panel per capture tile,
    /// outlined when `highlighted`.
    public func roomView(
        canvas: RoomScreen, outputWidth: Int, outputHeight: Int, highlighted: Bool, curveRadius: Float = 1
    ) -> RoomView {
        let tanX = tan(horizontalFov * 0.5)
        let aspect = Float(max(outputHeight, 1)) / Float(max(outputWidth, 1))
        let surface = canvas.surface(curveRadius: curveRadius)
        let width = Float(canvas.width)
        let panels = captureTiles(width: canvas.width).enumerated().map { tile, columns in
            RoomView.Panel(
                source: .canvas(tile), surface: surface,
                rect: SurfaceRect(
                    left: Float(columns.lowerBound) / width * 2 - 1, right: Float(columns.upperBound) / width * 2 - 1,
                    top: 1, bottom: -1),
                highlighted: highlighted)
        }
        return RoomView(headRotation: headRotation, tanHalfFov: SIMD2(tanX, tanX * aspect), panels: panels)
    }
}

func smoothstep(_ edge0: Float, _ edge1: Float, _ x: Float) -> Float {
    let t = min(max((x - edge0) / (edge1 - edge0), 0), 1)
    return t * t * (3 - 2 * t)
}

private func clampedHeadGain(_ gain: Float) -> Float {
    gain.isFinite ? min(max(gain, 1), maxHeadGain) : 1
}

private func rotationX(_ angle: Float) -> simd_float3x3 {
    let c = cos(angle)
    let s = sin(angle)
    return simd_float3x3(SIMD3(1, 0, 0), SIMD3(0, c, s), SIMD3(0, -s, c))
}

private func rotationY(_ angle: Float) -> simd_float3x3 {
    let c = cos(angle)
    let s = sin(angle)
    return simd_float3x3(SIMD3(c, 0, -s), SIMD3(0, 1, 0), SIMD3(s, 0, c))
}

private func rotationZ(_ angle: Float) -> simd_float3x3 {
    let c = cos(angle)
    let s = sin(angle)
    return simd_float3x3(SIMD3(c, s, 0), SIMD3(-s, c, 0), SIMD3(0, 0, 1))
}
