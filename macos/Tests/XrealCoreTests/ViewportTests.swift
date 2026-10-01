import Testing
import simd

@testable import XrealCore

private let dt: Float = 1.0 / 90.0

private struct CursorScene {
    let canvas = Settings().canvas
    let room: RoomView
    var follow = CursorFollow()
    var now = 0.0

    init() {
        var viewport = ViewportController(settings: Settings())
        viewport.recenter(HeadPose())
        room = viewport.roomView(canvas: canvas, outputWidth: 1920, outputHeight: 1080, highlighted: false)
    }

    /// Whether the view, zoomed out as far as the cursor has it now, shows
    /// `pixel` of the canvas.
    func shows(_ pixel: SIMD2<Float>) -> Bool {
        room.shows(canvas.roomPoint(ofPixel: pixel), scale: follow.scale, margin: 1)
    }

    /// Runs `seconds` of frames with the cursor at `cursor`, a pixel of the
    /// canvas.
    mutating func run(cursor: SIMD2<Float>, seconds: Double) {
        let point = canvas.roomPoint(ofPixel: cursor)
        for _ in 0..<Int(seconds * 90) {
            now += Double(dt)
            _ = follow.update(cursor: cursor, enabled: true, now: now, dt: dt) { [room] scale, margin in
                room.shows(point, scale: scale, margin: margin)
            }
        }
    }
}

private let middle = SIMD2<Float>(2876, 1080)
private let farCorner = SIMD2<Float>(5500, 2000)

@Test func movingTheMouseOutOfViewZoomsOutUntilTheCursorShows() {
    var scene = CursorScene()
    scene.run(cursor: middle, seconds: 0.1)
    #expect(!scene.shows(farCorner))

    scene.run(cursor: farCorner, seconds: 1)
    #expect(scene.shows(farCorner))
}

@Test func cursorBackInViewZoomsBackIn() {
    var scene = CursorScene()
    scene.run(cursor: middle, seconds: 0.1)
    scene.run(cursor: farCorner, seconds: 1)

    scene.run(cursor: middle, seconds: 2)
    #expect(scene.follow.scale == 1)
}

@Test func cursorLeftOutOfViewWithoutMovingDoesNotZoomOut() {
    var scene = CursorScene()
    scene.run(cursor: farCorner, seconds: 1)
    #expect(scene.follow.scale == 1)
}

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

private func viewYaw(_ viewport: ViewportController) -> Float {
    let ahead = viewport.headRotation * SIMD3(0, 0, -1)
    return atan2(-ahead.x, -ahead.z)
}

private func viewPitch(_ viewport: ViewportController) -> Float {
    let ahead = viewport.headRotation * SIMD3(0, 0, -1)
    return -asin(ahead.y)
}

@Test func headMovementCanTurnTheViewFurther() {
    var settings = Settings()
    settings.headGain = 2
    var viewport = ViewportController(settings: settings)
    viewport.recenter(HeadPose())
    viewport.track(pose: HeadPose(yaw: 0.2, pitch: 0.1))
    #expect(abs(viewYaw(viewport) - 0.4) <= steadyRadius + 1e-4)
    #expect(abs(viewPitch(viewport) - 0.2) <= steadyRadius + 1e-4)
}

@Test func changingHowFarTheViewTurnsLeavesItWhereItIs() {
    var viewport = ViewportController(settings: Settings())
    viewport.recenter(HeadPose())
    viewport.track(pose: HeadPose(yaw: 0.3))
    let before = viewYaw(viewport)

    viewport.setGain(3)
    viewport.track(pose: HeadPose(yaw: 0.3))
    #expect(abs(viewYaw(viewport) - before) < 1e-4)

    // From there on, the head turns it three times as far.
    viewport.track(pose: HeadPose(yaw: 0.4))
    #expect(abs(viewYaw(viewport) - (before + 0.3)) <= steadyRadius + 1e-4)
}

@Test func recenteringPutsTheViewStraightAheadWhateverTheGain() {
    var settings = Settings()
    settings.headGain = 2
    var viewport = ViewportController(settings: settings)
    viewport.recenter(HeadPose())
    viewport.track(pose: HeadPose(yaw: 0.5, pitch: -0.2))
    viewport.recenter(HeadPose(yaw: 0.5, pitch: -0.2))
    viewport.track(pose: HeadPose(yaw: 0.5, pitch: -0.2))
    #expect(abs(viewYaw(viewport)) < 1e-5 && abs(viewPitch(viewport)) < 1e-5)
}
