import Testing

@testable import XrealCore

private let dt: Float = 1.0 / 90.0

private func run(_ zoom: inout TurnZoom, speed: Float, seconds: Float, enabled: Bool = true) {
    for _ in 0..<Int(seconds * 90) {
        _ = zoom.update(speed: speed, enabled: enabled, dt: dt)
    }
}

@Test func aQuickTurnZoomsOutAtOnce() {
    var zoom = TurnZoom()
    run(&zoom, speed: 3, seconds: 0.15)
    // Most of the way to showing half as much again each way.
    #expect(zoom.scale < 0.75)
}

@Test func slowMovementNeverZooms() {
    var zoom = TurnZoom()
    run(&zoom, speed: slowTurn, seconds: 2)
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
