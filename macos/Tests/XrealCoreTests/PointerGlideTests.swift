import Testing
import simd

@testable import XrealCore

private let dt: Float = 1.0 / 90.0

/// Runs `seconds` of frames, moving the pointer wherever the glide says,
/// and returns where it ends up.
private func run(
    _ glide: inout PointerGlide, from start: SIMD2<Float>, to target: SIMD2<Float>, inView: Bool, seconds: Double,
    enabled: Bool = true, now: inout Double
) -> SIMD2<Float> {
    var cursor = start
    for _ in 0..<Int(seconds * 90) {
        now += Double(dt)
        if let move = glide.update(cursor: cursor, inView: inView, target: target, enabled: enabled, now: now, dt: dt) {
            cursor = move
        }
    }
    return cursor
}

private let leftBehind = SIMD2<Float>(100, 500)
private let lookedAt = SIMD2<Float>(4000, 900)

@Test func aPointerLeftOutOfViewGlidesToWhereTheViewerLooks() {
    var glide = PointerGlide()
    var now = 0.0
    let end = run(&glide, from: leftBehind, to: lookedAt, inView: false, seconds: 1.5, now: &now)
    #expect(simd_distance(end, lookedAt) < 1)
}

@Test func aPointerInViewStaysPut() {
    var glide = PointerGlide()
    var now = 0.0
    let end = run(&glide, from: leftBehind, to: lookedAt, inView: true, seconds: 2, now: &now)
    #expect(end == leftBehind)
}

@Test func movingTheMouseStopsTheGlide() {
    var glide = PointerGlide()
    var now = 0.0
    let partway = run(&glide, from: leftBehind, to: lookedAt, inView: false, seconds: 0.6, now: &now)
    #expect(simd_distance(partway, leftBehind) > 10)
    // The viewer takes the mouse.
    let taken = partway + SIMD2(-40, 0)
    now += Double(dt)
    #expect(glide.update(cursor: taken, inView: false, target: lookedAt, enabled: true, now: now, dt: dt) == nil)
    // And while they keep using it, it is theirs.
    var cursor = taken
    for _ in 0..<20 {
        now += Double(dt)
        cursor.x -= 3
        #expect(glide.update(cursor: cursor, inView: false, target: lookedAt, enabled: true, now: now, dt: dt) == nil)
    }
}

@Test func turnedOffThePointerStaysPut() {
    var glide = PointerGlide()
    var now = 0.0
    let end = run(&glide, from: leftBehind, to: lookedAt, inView: false, seconds: 2, enabled: false, now: &now)
    #expect(end == leftBehind)
}

@Test func aMoveShowingAFrameLateDoesNotStopTheGlide() {
    var glide = PointerGlide()
    var now = 0.0
    var cursor = leftBehind
    var shown = leftBehind
    for _ in 0..<Int(1.5 * 90) {
        now += Double(dt)
        // The pointer reads where the previous move left it.
        let move = glide.update(cursor: shown, inView: false, target: lookedAt, enabled: true, now: now, dt: dt)
        shown = cursor
        if let move {
            cursor = move
        }
    }
    #expect(simd_distance(cursor, lookedAt) < 1)
}

@Test func macOSRoundingWherePointerIsPutDoesNotStopTheGlide() {
    var glide = PointerGlide()
    var now = 0.0
    var cursor = leftBehind
    for _ in 0..<Int(1.5 * 90) {
        now += Double(dt)
        if let move = glide.update(cursor: cursor, inView: false, target: lookedAt, enabled: true, now: now, dt: dt) {
            cursor = move.rounded(.toNearestOrAwayFromZero)
        }
    }
    #expect(simd_distance(cursor, lookedAt) < 1)
}

@Test func thePointerKeepsUpWithTheGazeWhileTheHeadTurnsQuickly() {
    var glide = PointerGlide()
    var now = 0.0
    var cursor = leftBehind
    var target = lookedAt
    var furthest: Float = 0
    for frame in 0..<Int(2 * 90) {
        now += Double(dt)
        // The gaze sweeping along the canvas at 6000 points a second, as
        // with head movement multiplied.
        target.x += 6000 * dt
        let inView = simd_distance(cursor, target) < 800
        if let move = glide.update(cursor: cursor, inView: inView, target: target, enabled: true, now: now, dt: dt) {
            cursor = move
        }
        if frame > 45 {
            furthest = max(furthest, simd_distance(cursor, target))
        }
    }
    // With the gaze, well inside the view, all the way.
    #expect(furthest < 100)
}

@Test func aSuddenJumpInGazeNeverFlingsThePointerOffTheCanvas() {
    var glide = PointerGlide()
    var now = 0.0
    var cursor = leftBehind
    let bounds = (min: SIMD2<Float>(0, 0), max: SIMD2<Float>(5751, 2159))
    var target = SIMD2<Float>(1000, 900)
    for frame in 0..<Int(2.5 * 90) {
        now += Double(dt)
        // Where the viewer looks jumps across the canvas at once.
        if frame == 60 {
            target = SIMD2(5600, 900)
        }
        if let move = glide.update(
            cursor: cursor, inView: false, target: target, enabled: true, now: now, dt: dt, bounds: bounds)
        {
            cursor = move
            #expect(cursor.x >= bounds.min.x && cursor.x <= bounds.max.x)
            #expect(simd_distance(cursor, target) < 4700)
        }
    }
    #expect(simd_distance(cursor, target) < 1)
}
