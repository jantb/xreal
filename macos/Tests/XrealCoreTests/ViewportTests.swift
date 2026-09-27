import Testing
import simd

@testable import XrealCore

private let dt: Float = 1.0 / 120.0

private func noDeadzone() -> ViewportController {
    var settings = Settings()
    settings.deadzoneIndex = 0
    return ViewportController(settings: settings)
}

private func pose(_ yaw: Float, _ pitch: Float) -> HeadPose {
    HeadPose(yaw: yaw, pitch: pitch)
}

private func settle(_ viewport: inout ViewportController, _ pose: HeadPose) -> ViewportRect {
    var rect = ViewportRect()
    for _ in 0..<240 {
        rect = viewport.update(
            pose: pose, dt: dt, outputWidth: 1920, outputHeight: 1080, sourceWidth: 7680,
            sourceHeight: 4320)
    }
    return rect
}

@Test func viewportRecentersAndClampsToSource() {
    var viewport = ViewportController(settings: Settings())
    viewport.recenter(pose(1.0, 0.5))
    viewport.pan(dx: 10_000, dy: -10_000)

    let rect = viewport.update(
        pose: pose(1.0, 0.5), dt: dt, outputWidth: 1920, outputHeight: 1080, sourceWidth: 3840,
        sourceHeight: 1080)
    #expect(rect == ViewportRect(x: 1920, y: 0, width: 1920, height: 1080))
}

@Test func freezeHoldsPosition() {
    var viewport = ViewportController(settings: Settings())
    let before = viewport.update(
        pose: pose(0, 0), dt: dt, outputWidth: 1920, outputHeight: 1080, sourceWidth: 3840,
        sourceHeight: 1080)

    viewport.toggleFreeze()
    let after = viewport.update(
        pose: pose(1, 1), dt: dt, outputWidth: 1920, outputHeight: 1080, sourceWidth: 3840,
        sourceHeight: 1080)

    #expect(after.x == before.x)
    #expect(after.y == before.y)
}

@Test(arguments: [2, 5])
func headTurnMovesContentByTheSameAngleAtAnyZoom(zoomIndex: Int) {
    let turn: Float = 0.1
    let expectedOutputPx = turn * 1920 / horizontalFov

    var viewport = noDeadzone()
    viewport.zoomIndex = zoomIndex
    let start = settle(&viewport, pose(0, 0))
    let turned = settle(&viewport, pose(turn, 0))

    let shiftOutputPx = (start.x - turned.x) * viewport.zoom
    #expect(
        abs(shiftOutputPx - expectedOutputPx) < 1,
        "zoom \(viewport.zoom): shifted \(shiftOutputPx) px, expected \(expectedOutputPx)")
}

@Test func turningAcrossTheYawSeamIsASmallMove() {
    var viewport = noDeadzone()
    viewport.recenter(pose(3.1, 0))
    let start = settle(&viewport, pose(3.1, 0))
    let crossed = settle(&viewport, pose(-3.1, 0))

    let expected = (2 * Float.pi - 6.2) * 1920 / horizontalFov
    #expect(abs((start.x - crossed.x) - expected) < 1)
}

@Test func fastHeadTurnIsFollowedWithinAFewFrames() {
    var viewport = noDeadzone()
    let start = settle(&viewport, pose(0, 0))
    var reference = noDeadzone()
    _ = settle(&reference, pose(0, 0))
    let target = settle(&reference, pose(0.2, 0))

    var rect = start
    for _ in 0..<3 {
        rect = viewport.update(
            pose: pose(0.2, 0), dt: dt, outputWidth: 1920, outputHeight: 1080, sourceWidth: 7680,
            sourceHeight: 4320)
    }
    let progress = (start.x - rect.x) / (start.x - target.x)
    #expect(progress > 0.9, "only \(progress) of the way after 25 ms")
}

@Test func smallJitterWhileStillIsDamped() {
    var viewport = noDeadzone()
    let jitter: Float = 0.002
    let center = settle(&viewport, pose(0, 0))

    var maxOffset: Float = 0
    for i in 0..<240 {
        let yaw = i % 2 == 0 ? jitter : -jitter
        let rect = viewport.update(
            pose: pose(yaw, 0), dt: dt, outputWidth: 1920, outputHeight: 1080, sourceWidth: 7680,
            sourceHeight: 4320)
        maxOffset = max(maxOffset, abs(rect.x - center.x))
    }
    let unfiltered = jitter * 1920 / horizontalFov
    #expect(maxOffset < unfiltered * 0.3, "jitter \(maxOffset) px vs raw \(unfiltered) px")
}

private func roomView(
    _ projection: Projection, pose: HeadPose, followRoll: Bool = true, zoomIndex: Int = 2
) -> SpatialView {
    var settings = Settings()
    settings.deadzoneIndex = 0
    settings.projection = projection
    settings.followRoll = followRoll
    settings.zoomIndex = zoomIndex
    var viewport = ViewportController(settings: settings)
    viewport.recenter(HeadPose())
    for _ in 0..<240 {
        viewport.track(pose: pose, dt: dt)
    }
    return viewport.spatialView(outputWidth: 1920, outputHeight: 1080, sourceWidth: 3832, sourceHeight: 2160)
}

@Test(arguments: [Projection.flat, .curved])
func lookingAheadShowsTheMiddleOfTheRoomScreen(projection: Projection) throws {
    let view = roomView(projection, pose: HeadPose())
    let middle = try #require(view.sourcePoint(atOutput: .zero))
    #expect(simd_distance(middle, SIMD2(1916, 1080)) < 0.5)
}

@Test(arguments: [Projection.flat, .curved])
func roomScreenStaysPutWhenTheHeadTurns(projection: Projection) throws {
    let turn: Float = 0.15
    let ahead = roomView(projection, pose: HeadPose())
    let turnedLeft = roomView(projection, pose: HeadPose(yaw: turn))
    let middle = try #require(ahead.sourcePoint(atOutput: .zero))

    // Turning left moves what was straight ahead to the right of the view,
    // by the angle turned.
    let seen = try #require(turnedLeft.outputPoint(ofSource: middle))
    #expect(abs(seen.x - tan(turn) / turnedLeft.tanHalfFov.x) < 1e-3)
    #expect(abs(seen.y) < 1e-3)
}

@Test func curvedScreenKeepsItsPixelDensityAllTheWayAround() throws {
    let view = roomView(.curved, pose: HeadPose())
    let right = roomView(.curved, pose: HeadPose(yaw: -0.4))
    let pixelsPerRadian = 1920 * 0.5 / view.tanHalfFov.x

    let middle = try #require(view.sourcePoint(atOutput: .zero))
    let looked = try #require(right.sourcePoint(atOutput: .zero))
    #expect(abs((looked.x - middle.x) - 0.4 * pixelsPerRadian) < 1)
    #expect(abs(looked.y - middle.y) < 0.5)
}

@Test func tiltingTheHeadKeepsTheRoomScreenLevel() throws {
    let ahead = roomView(.flat, pose: HeadPose())
    let rightOfMiddle = try #require(ahead.sourcePoint(atOutput: SIMD2(0.5, 0)))

    // Right ear down: the screen's horizontal line rises on the right.
    let tilted = roomView(.flat, pose: HeadPose(roll: 0.2))
    let seen = try #require(tilted.outputPoint(ofSource: rightOfMiddle))
    #expect(seen.y > 0.05)

    let ignoringTilt = roomView(.flat, pose: HeadPose(roll: 0.2), followRoll: false)
    let unmoved = try #require(ignoringTilt.outputPoint(ofSource: rightOfMiddle))
    #expect(abs(unmoved.y) < 1e-3)
}

@Test func blackEdgeLetsTheViewMovePastTheSource() {
    var settings = Settings()
    settings.edge = .black
    var viewport = ViewportController(settings: settings)
    viewport.recenter(HeadPose())
    viewport.pan(dx: 10_000, dy: 0)
    let rect = viewport.update(
        pose: HeadPose(), dt: dt, outputWidth: 1920, outputHeight: 1080, sourceWidth: 3840, sourceHeight: 1080)
    #expect(rect.x > 3840)
    #expect(rect.width == 1920)
}

private struct CursorScene {
    var viewport = noDeadzone()
    var follow = CursorFollow()
    var now = 0.0

    init() {
        viewport.recenter(HeadPose())
    }

    func geometry(scale: Float) -> ViewGeometry {
        viewport.geometry(outputWidth: 1920, outputHeight: 1080, sourceWidth: 7680, sourceHeight: 4320, scale: scale)
    }

    /// Runs `seconds` of frames with the cursor at `cursor` and returns the
    /// view at the end.
    mutating func run(cursor: SIMD2<Float>, seconds: Double) -> ViewGeometry {
        var scale: Float = 1
        for _ in 0..<Int(seconds * 120) {
            now += Double(dt)
            let scene = self
            scale = follow.update(cursor: cursor, enabled: true, now: now, dt: dt) { scale, margin in
                scene.geometry(scale: scale).shows(cursor, margin: margin)
            }
        }
        return geometry(scale: scale)
    }
}

@Test func movingTheMouseOutOfViewZoomsOutUntilTheCursorShows() {
    var scene = CursorScene()
    let middle = SIMD2<Float>(3840, 2160)
    let farCorner = SIMD2<Float>(7000, 4000)
    _ = scene.run(cursor: middle, seconds: 0.1)
    #expect(!scene.run(cursor: middle, seconds: 0.1).shows(farCorner, margin: 1))

    #expect(scene.run(cursor: farCorner, seconds: 1).shows(farCorner, margin: 1))
}

@Test func cursorBackInViewZoomsBackIn() {
    var scene = CursorScene()
    let middle = SIMD2<Float>(3840, 2160)
    let chosen = scene.geometry(scale: 1)
    _ = scene.run(cursor: middle, seconds: 0.1)
    _ = scene.run(cursor: SIMD2(7000, 4000), seconds: 1)

    guard case .crop(let back) = scene.run(cursor: middle, seconds: 2), case .crop(let normal) = chosen else {
        Issue.record("expected a crop view")
        return
    }
    #expect(back == normal)
}

@Test func cursorLeftOutOfViewWithoutMovingDoesNotZoomOut() {
    var scene = CursorScene()
    let chosen = scene.geometry(scale: 1)
    guard case .crop(let still) = scene.run(cursor: SIMD2(7000, 4000), seconds: 1), case .crop(let normal) = chosen
    else {
        Issue.record("expected a crop view")
        return
    }
    #expect(still == normal)
}
