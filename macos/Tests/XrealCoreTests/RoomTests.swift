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
        let edge = room.outputPoint(ofRoom: panel.surface.center + panel.surface.right)!
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

private let canvas = RoomScreen(
    width: 7672, height: 2160, placement: ScreenPlacement(direction: SIMD3(0, 0, -1)), curved: true)

private func azimuth(_ point: SIMD3<Float>) -> Float { atan2(point.x, -point.z) }

@Test func curvedCanvasWrapsAroundTheViewerAtAnEvenDistance() throws {
    let surface = try #require(canvas.surface)
    let left = surface.point(at: SIMD2(-1, 0))
    let right = surface.point(at: SIMD2(1, 0))
    // An 8K-wide canvas at the glasses' pixel density wraps most of the way round.
    #expect(azimuth(right) > 1.3 && azimuth(left) < -1.3)
    for x in stride(from: Float(-1), through: 1, by: 0.25) {
        let point = surface.point(at: SIMD2(x, 0))
        #expect(abs(simd_length(point) - 1) < 1e-4)
    }
}

@Test func curvedScreenBroughtVeryCloseStopsWrappingFurther() throws {
    var close = canvas
    close.placement = ScreenPlacement(direction: SIMD3(0, 0, -1), distance: 0.3)
    let surface = try #require(close.surface)
    let edge = surface.point(at: SIMD2(1, 0))
    // Never wraps so far that the edges meet behind the viewer.
    #expect(azimuth(edge) > 0 && azimuth(edge) < .pi)
}

@Test func lookingFarToTheSideStillFindsTheCurvedCanvas() throws {
    let screens = [canvas]
    let sideways = SIMD3<Float>(sin(1.2), 0, -cos(1.2))
    let target = try #require(gazeTarget(sideways, among: screens))
    #expect(target.index == 0)
    #expect(target.pixel.x > 7672 * 0.8)
    #expect(abs(target.pixel.y - 1080) < 20)
    #expect(gazeTarget(SIMD3(0, 0, 1), among: screens) == nil)
}

@Test func lookingStraightAtAScreenTargetsItsMiddle() throws {
    let screen = RoomScreen(width: 1920, height: 1080, placement: ScreenPlacement(direction: SIMD3(0, 0, -1)))
    let target = try #require(gazeTarget(SIMD3(0, 0, -1), among: [screen]))
    #expect(simd_distance(target.pixel, SIMD2(960, 540)) < 1)
}

@Test func screensSideBySideInTheArrangementSitSideBySideInTheRoom() throws {
    let ahead = CGRect.zero
    let frames = glassesOnlyArrangement(for: [canvas, RoomScreen(width: 2880, height: 1620)], ahead: ahead.origin)
    var side = RoomScreen(width: 2880, height: 1620)
    side.placement = ScreenPlacement(arrangedAt: frames[1], around: ahead)
    let canvasEdge = azimuth(try #require(canvas.surface).point(at: SIMD2(1, 0)))
    let sideEdge = azimuth(try #require(side.surface).point(at: SIMD2(-1, 0)))
    #expect(abs(sideEdge - canvasEdge) < 0.1, "gap or overlap of \(sideEdge - canvasEdge) rad")
}

@Test func glassesOnlyLayoutPutsTheFirstScreenAheadAndTheRestBeside() {
    let screens = [canvas, RoomScreen(width: 2880, height: 1620), RoomScreen(width: 1920, height: 1080)]
    let frames = glassesOnlyArrangement(for: screens)
    #expect(abs(frames[0].midX) <= 1 && abs(frames[0].midY) <= 1)
    #expect(frames[1].minX >= frames[0].maxX)
    #expect(frames[2].maxX <= frames[0].minX)
    for (index, frame) in frames.enumerated() {
        for other in frames[(index + 1)...] {
            #expect(!frame.intersects(other))
        }
    }
}

@Test func referenceFromAScreenArrangesItWhereAsked() {
    for screen in [
        canvas,
        RoomScreen(width: 2880, height: 1620, placement: ScreenPlacement(direction: SIMD3(0.5, 0.2, -1))),
    ] {
        let reference = aheadReference(for: screen, arrangedAt: .zero)
        let origin = screen.placement!.arrangedOrigin(width: screen.width, height: screen.height, around: reference)
        #expect(abs(origin.x) <= 1 && abs(origin.y) <= 1)
    }
}

@Test func wideScreensAreCapturedInTilesCoveringEveryColumn() {
    for width in [1920, 3832, 5752, 7672, 8192] {
        let tiles = captureTiles(width: width)
        #expect(tiles.first?.lowerBound == 0 && tiles.last?.upperBound == width)
        for (tile, next) in zip(tiles, tiles.dropFirst()) {
            #expect(tile.upperBound == next.lowerBound)
        }
    }
    #expect(captureTiles(width: 7672).count > 1)
}

@Test func onlyPanelsNearTheViewCountAsShown() {
    var viewport = ViewportController(settings: Settings())
    viewport.recenter(HeadPose())
    let screens = [
        RoomScreen(width: 1920, height: 1080, placement: ScreenPlacement(direction: SIMD3(0, 0, -1))),
        RoomScreen(width: 1920, height: 1080, placement: ScreenPlacement(direction: SIMD3(0, 0, 1))),
    ]
    let room = viewport.roomView(screens: screens, outputWidth: 1920, outputHeight: 1080, highlighted: nil)
    let ahead = room.panels.first { $0.screen == 0 }!
    let behind = room.panels.first { $0.screen == 1 }!
    #expect(room.shows(ahead, margin: 0.5))
    #expect(!room.shows(behind, margin: 0.5))
}

@Test func zoneIsAboutOneViewOfACanvasAroundThePoint() {
    let point = SIMD2<Float>(5000, 900)
    let found = zone(around: point, width: 7672, height: 2160)
    #expect(found.contains(CGPoint(x: 5000, y: 900)))
    #expect(found.width > 1500 && found.width < 2600)
    #expect(found.minX >= 0 && found.maxX <= 7672 && found.minY >= 0 && found.maxY <= 2160)

    let small = zone(around: SIMD2(100, 100), width: 1920, height: 1080)
    #expect(small == CGRect(x: 0, y: 0, width: 1920, height: 1080))
}

@Test func windowMovedToAPointStaysOnTheScreen() {
    let screen = CGRect(x: 1000, y: -500, width: 2880, height: 1620)
    let size = CGSize(width: 800, height: 600)
    let middle = windowOrigin(size: size, centeredOn: CGPoint(x: 2000, y: 200), within: screen)
    #expect(middle == CGPoint(x: 1600, y: -100))

    let corner = windowOrigin(size: size, centeredOn: CGPoint(x: 3870, y: 1110), within: screen)
    #expect(CGRect(origin: corner, size: size).maxX <= screen.maxX)
    #expect(CGRect(origin: corner, size: size).maxY <= screen.maxY)
}
