import Testing

@testable import XrealCore

private let dt: Float = 1.0 / 90.0

// A canvas three times as wide as the view.
private let wholeCanvas: Float = 1.0 / 3

private func run(_ zoom: inout TurnZoom, speed: Float, seconds: Float, enabled: Bool = true) {
    for _ in 0..<Int(seconds * 90) {
        _ = zoom.update(speed: speed, enabled: enabled, whole: wholeCanvas, dt: dt)
    }
}

@Test func aQuickTurnZoomsOutToShowTheWholeCanvas() {
    var zoom = TurnZoom()
    // Gliding out: gently at first...
    run(&zoom, speed: 3, seconds: 0.1)
    #expect(zoom.scale > 0.85 && zoom.scale < 1)
    // ...and the whole canvas in view after about two seconds.
    run(&zoom, speed: 3, seconds: 2.9)
    #expect(abs(zoom.scale - wholeCanvas) < 0.01)
}

@Test func theScaleThatShowsTheWholeCanvasShowsItAndNoMore() {
    // Points spread out to three times what the view shows unzoomed.
    let points: [SIMD3<Float>] = [[-3, 0, 0], [3, 0, 0], [0, 1, 0]]
    let shows = { (point: SIMD3<Float>, scale: Float, margin: Float) in abs(point.x) * scale <= margin }
    let scale = TurnZoom.wholeCanvasScale(points, shows: shows)
    #expect(points.allSatisfy { shows($0, scale, 0.95) })
    #expect(!points.allSatisfy { shows($0, scale * 1.05, 0.95) })
    // Nothing beyond the view: no zoom.
    #expect(TurnZoom.wholeCanvasScale([[0.5, 0, 0]], shows: shows) == 1)
}

@Test func slowMovementNeverZooms() {
    var zoom = TurnZoom()
    run(&zoom, speed: 0.1, seconds: 2)
    #expect(zoom.scale == 1)
}

@Test func slowingDownZoomsBackInToTheCanvasOwnPixels() {
    var zoom = TurnZoom()
    run(&zoom, speed: 3, seconds: 0.5)
    run(&zoom, speed: 0, seconds: 1)
    #expect(zoom.scale == 1)
}

@Test func turnedOffItNeverZooms() {
    var zoom = TurnZoom()
    run(&zoom, speed: 3, seconds: 1, enabled: false)
    #expect(zoom.scale == 1)
}

@Test func aLateFrameDoesNotThrowTheZoomOff() {
    var zoom = TurnZoom()
    for _ in 0..<20 {
        _ = zoom.update(speed: 3, enabled: true, whole: wholeCanvas, dt: 0.1)
    }
    #expect(zoom.scale >= wholeCanvas - 1e-3 && zoom.scale <= 1)
}
