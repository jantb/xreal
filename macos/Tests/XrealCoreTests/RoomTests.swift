import CoreGraphics
import Testing
import simd

@testable import XrealCore

private let laptop = CGRect(x: 0, y: 0, width: 2056, height: 1329)
private let ahead = matrix_identity_float3x3

private func turned(yaw: Float) -> simd_float3x3 {
    var viewport = ViewportController(settings: Settings())
    viewport.setDeadzone(0)
    viewport.recenter(HeadPose())
    for _ in 0..<480 {
        viewport.track(pose: HeadPose(yaw: yaw), dt: 1.0 / 120)
    }
    return viewport.headRotation
}

@Test func screenArrangedRightOfTheLaptopStartsToTheRight() {
    let right = ScreenPlacement(arrangedAt: CGRect(x: 2056, y: 0, width: 1920, height: 1080), around: laptop)
    let above = ScreenPlacement(arrangedAt: CGRect(x: 68, y: -1080, width: 1920, height: 1080), around: laptop)

    #expect(right.direction.x > 0.3)
    #expect(abs(right.direction.y) < 0.2)
    #expect(above.direction.y > 0.3)
    #expect(abs(above.direction.x) < 0.05)
}

@Test func placementFromTheArrangementArrangesBackTheSame() {
    for frame in [
        CGRect(x: 2056, y: 0, width: 1920, height: 1080), CGRect(x: 68, y: -1080, width: 1920, height: 1080),
        CGRect(x: -2880, y: 200, width: 2880, height: 1620),
    ] {
        let placement = ScreenPlacement(arrangedAt: frame, around: laptop)
        let origin = placement.arrangedOrigin(width: Int(frame.width), height: Int(frame.height), around: laptop)
        #expect(abs(origin.x - frame.minX) <= 1 && abs(origin.y - frame.minY) <= 1, "\(frame) came back at \(origin)")
    }
}

@Test func lookingAtAScreenPicksIt() {
    let screens = [
        RoomScreen(width: 1920, height: 1080, placement: ScreenPlacement(direction: SIMD3(-1, 0, -1))),
        RoomScreen(width: 1920, height: 1080, placement: ScreenPlacement(direction: SIMD3(1, 0, -1))),
    ]
    #expect(screenLooked(at: normalize(SIMD3(-1, 0, -1)), among: screens) == 0)
    #expect(screenLooked(at: normalize(SIMD3(1, 0.1, -1)), among: screens) == 1)
    #expect(screenLooked(at: SIMD3(0, 0, 1), among: screens) == nil)
}

@Test func nearerScreenWinsWhereScreensOverlap() {
    let screens = [
        RoomScreen(width: 1920, height: 1080, placement: ScreenPlacement(direction: SIMD3(0, 0, -1), distance: 2)),
        RoomScreen(width: 1920, height: 1080, placement: ScreenPlacement(direction: SIMD3(0, 0, -1), distance: 1)),
    ]
    #expect(screenLooked(at: SIMD3(0, 0, -1), among: screens) == 1)
}

@Test func grabbedScreenFollowsTheHeadAndStaysWhereItIsLetGo() throws {
    let start = ScreenPlacement(direction: SIMD3(0, 0, -1))
    let grab = ScreenGrab(index: 0, placement: start, headRotation: ahead)

    func middleOfScreen(_ placement: ScreenPlacement, seenWith headRotation: simd_float3x3) -> SIMD2<Float>? {
        let view = RoomView(headRotation: headRotation, tanHalfFov: SIMD2(1, 0.5625), panels: [])
        return view.outputPoint(ofRoom: placement.frame(width: 1920, height: 1080).center)
    }

    // Carried along a head turn to the left, it stays in the middle of the view...
    let turnedLeft = turned(yaw: 0.5)
    let dropped = grab.placement(headRotation: turnedLeft, distance: start.distance)
    let carried = try #require(middleOfScreen(dropped, seenWith: turnedLeft))
    #expect(simd_length(carried) < 1e-3)

    // ...and once let go, looking ahead again leaves it off to the left.
    let afterwards = try #require(middleOfScreen(dropped, seenWith: ahead))
    #expect(afterwards.x < -0.4)
}

@Test func movingAScreenAwayMakesItLookSmaller() throws {
    let near = ScreenPlacement(direction: SIMD3(0, 0, -1))
    let far = near.movedAway(by: 2)
    let view = { (placement: ScreenPlacement) -> Float in
        var viewport = ViewportController(settings: Settings())
        viewport.recenter(HeadPose())
        let room = viewport.roomView(
            screens: [RoomScreen(width: 1920, height: 1080, placement: placement)], outputWidth: 1920,
            outputHeight: 1080, highlighted: nil)
        let panel = room.panels[0]
        let edge = room.outputPoint(ofRoom: panel.center + panel.right)!
        return edge.x
    }
    // At distance 1 the screen's 1920 pixels fill the view's 1920 pixels.
    #expect(abs(view(near) - 1) < 1e-3)
    #expect(abs(view(far) - 0.5) < 1e-3)
}

@Test func screensStayUprightAboveAndBelow() {
    let overhead = ScreenPlacement(direction: SIMD3(0, 1, 0))
    #expect(overhead.direction.y < 0.99)
    let (_, right, up) = overhead.frame(width: 1920, height: 1080)
    #expect(abs(right.y) < 1e-4)
    #expect(up.y > 0)
}

@Test func standardLayoutPutsAWideScreenAboveAndOneOnEachSide() {
    let screens = [
        RoomScreen(width: 5120, height: 1440), RoomScreen(width: 2880, height: 1620),
        RoomScreen(width: 2880, height: 1620), RoomScreen(width: 1920, height: 1080),
    ]
    let frames = standardArrangement(for: screens, around: laptop)
    #expect(frames[0].maxY <= laptop.minY && abs(frames[0].midX - laptop.midX) <= 1)
    #expect(frames[1].minX >= laptop.maxX)
    #expect(frames[2].maxX <= laptop.minX)
    #expect(frames[3].minX >= frames[1].maxX)
    for (index, frame) in frames.enumerated() {
        #expect(!frame.intersects(laptop), "screen \(index) overlaps the laptop")
        for other in frames[(index + 1)...] {
            #expect(!frame.intersects(other))
        }
    }

    let placements = frames.map { ScreenPlacement(arrangedAt: $0, around: laptop) }
    #expect(placements[0].direction.y > 0.3)
    #expect(placements[1].direction.x > 0.3 && placements[2].direction.x < -0.3)
}
