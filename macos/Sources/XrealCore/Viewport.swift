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

/// Turns head pose into how the head is turned in the room.
public struct ViewportController: Sendable {
    private var center = HeadPose()
    private var leash = SoftLeash()
    private var offsetYaw: Float = 0
    private var offsetPitch: Float = 0
    private var offsetRoll: Float = 0
    private var initialized = false
    /// Whether the view tilts with the head, as `Settings.followRoll` says.
    public var followsRoll: Bool

    public init(settings: Settings) {
        followsRoll = settings.followRoll
    }

    /// Makes `pose` the new straight ahead. Only the turn and nod are
    /// taken: tilt is measured against gravity, so level stays level
    /// however the head was tilted when recentering.
    public mutating func recenter(_ pose: HeadPose) {
        center = HeadPose(yaw: pose.yaw, pitch: pose.pitch)
        offsetYaw = 0
        offsetPitch = 0
        offsetRoll = 0
        leash.reset(.zero)
        initialized = true
    }

    /// Follows the head to `pose`. The canvas turns with the head exactly,
    /// as any delay makes it lag behind, apart from wobble within
    /// `steadyRadius`.
    public mutating func track(pose: HeadPose) {
        if !initialized {
            recenter(pose)
        }
        let raw = SIMD3(wrapAngle(pose.yaw - center.yaw), pose.pitch - center.pitch, wrapAngle(pose.roll))
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
        return rotationY(offsetYaw + turn.x) * rotationX(-(offsetPitch + turn.y)) * rotationZ(-roll)
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
