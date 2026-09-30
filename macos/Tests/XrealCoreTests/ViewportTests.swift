import Testing
import simd

@testable import XrealCore

@Test func theCanvasTurnsWithTheHeadAtOnce() {
    var viewport = ViewportController(settings: Settings())
    viewport.recenter(HeadPose())
    // A small turn and a larger one, each in one frame.
    for yaw in [Float(0.01), 0.4] {
        viewport.track(pose: HeadPose(yaw: yaw))
        let ahead = viewport.headRotation * SIMD3(0, 0, -1)
        // At most the few pixels of wobble the view ignores behind.
        #expect(abs(atan2(-ahead.x, -ahead.z) - yaw) <= steadyRadius + 1e-5, "yaw \(yaw)")
    }
}

@Test func headTremorDoesNotMoveTheCanvas() {
    var viewport = ViewportController(settings: Settings())
    viewport.recenter(HeadPose(yaw: 0.3, pitch: 0.1))
    viewport.track(pose: HeadPose(yaw: 0.3, pitch: 0.1))
    let settled = viewport.headRotation
    // Holding the head still, as well as a head can: wobble of about a
    // glasses pixel either way.
    let pixel = horizontalFov / 1920
    for frame in 0..<180 {
        let wobble = pixel * sin(Float(frame) * 0.7)
        viewport.track(pose: HeadPose(yaw: 0.3 + wobble, pitch: 0.1 - wobble * 0.5, roll: wobble * 0.3))
        #expect(viewport.headRotation == settled, "frame \(frame)")
    }
}

@Test func recenteringWithTheHeadTiltedKeepsTheCanvasLevel() {
    var viewport = ViewportController(settings: Settings())
    // Recentered while leaning the head to one side...
    viewport.recenter(HeadPose(yaw: 0.2, pitch: 0.1, roll: 0.25))
    // ...then sitting up straight.
    viewport.track(pose: HeadPose(yaw: 0.2, pitch: 0.1, roll: 0))
    let up = viewport.headRotation * SIMD3(0, 1, 0)
    // The head's up is the room's up again: the canvas looks level.
    #expect(simd_distance(up, SIMD3(0, 1, 0)) <= steadyRadius + 1e-4, "up \(up)")
}

@Test func tiltingTheHeadKeepsTheCanvasLevelUnlessTurnedOff() {
    func rightInView(followRoll: Bool) -> SIMD3<Float> {
        var settings = Settings()
        settings.followRoll = followRoll
        var viewport = ViewportController(settings: settings)
        viewport.recenter(HeadPose())
        viewport.track(pose: HeadPose(roll: 0.2))
        return viewport.headRotation.transpose * SIMD3(1, 0, 0)
    }
    // Right ear down: the room's horizontal line rises on the right.
    #expect(rightInView(followRoll: true).y > 0.1)
    #expect(abs(rightInView(followRoll: false).y) < 1e-6)
}
