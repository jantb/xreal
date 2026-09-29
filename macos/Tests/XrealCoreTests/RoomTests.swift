import CoreGraphics
import Testing
import simd

@testable import XrealCore

private let ahead = matrix_identity_float3x3

@Test func lookingAtTheCanvasFindsItAndLookingAwayDoesNot() throws {
    let screen = RoomScreen(width: 1920, height: 1080, placement: ScreenPlacement(direction: SIMD3(-1, 0, -1)))
    let pixel = try #require(gazeTarget(normalize(SIMD3(-1, 0, -1)), on: screen))
    #expect(simd_distance(pixel, SIMD2(960, 540)) < 1)
    #expect(gazeTarget(normalize(SIMD3(1, 0, -1)), on: screen) == nil)
    #expect(gazeTarget(SIMD3(0, 0, 1), on: screen) == nil)
}

@Test func grabbedScreenFollowsTheHeadAndStaysWhereItIsLetGo() throws {
    let start = ScreenPlacement(direction: SIMD3(0, 0, -1))
    let grab = ScreenGrab(placement: start, headRotation: ahead)

    func middleOfScreen(_ placement: ScreenPlacement, seenWith headRotation: simd_float3x3) -> SIMD2<Float>? {
        let view = RoomView(headRotation: headRotation, tanHalfFov: SIMD2(1, 0.5625), panels: [])
        return view.outputPoint(ofRoom: placement.frame(width: 1920, height: 1080).center)
    }

    // Carried along a head turn to the left, it stays in the middle of the view...
    let turnedLeft = headRotation(yaw: 0.5, roll: 0)
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
            canvas: RoomScreen(width: 1920, height: 1080, placement: placement), outputWidth: 1920,
            outputHeight: 1080, highlighted: false)
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

private let canvas = RoomScreen(
    width: 7672, height: 2160, placement: ScreenPlacement(direction: SIMD3(0, 0, -1)), curved: true)

private func azimuth(_ point: SIMD3<Float>) -> Float { atan2(point.x, -point.z) }

@Test func curvedScreenBroughtVeryCloseStopsWrappingFurther() throws {
    var close = canvas
    close.placement = ScreenPlacement(direction: SIMD3(0, 0, -1), distance: 0.3)
    let edge = close.surface().point(at: SIMD2(1, 0))
    // Never wraps so far that the edges meet behind the viewer.
    #expect(azimuth(edge) > 0 && azimuth(edge) < .pi)
}

private let raisedCurved = RoomScreen(
    width: 5120, height: 1440, placement: ScreenPlacement(direction: SIMD3(0, sin(0.53), -cos(0.53))),
    curved: true)

private func elevation(_ point: SIMD3<Float>) -> Float {
    atan2(point.y, simd_length(SIMD2(point.x, point.z)))
}

@Test func curvedScreenColumnsAreStraightLines() throws {
    for screen in [canvas, raisedCurved] {
        let surface = screen.surface()
        for x in [Float(-1), -0.3, 0.6, 1] {
            let top = surface.point(at: SIMD2(x, 1))
            let bottom = surface.point(at: SIMD2(x, -1))
            // Every point down the column lies on the line from top to bottom,
            // so the side edges look straight.
            for y in [Float(0.5), 0, -0.5] {
                let point = surface.point(at: SIMD2(x, y))
                let expected = bottom + (top - bottom) * (y + 1) / 2
                #expect(simd_distance(point, expected) < 1e-4, "x \(x) y \(y)")
            }
        }
    }
}

@Test func lookingFarToTheSideStillFindsTheCurvedCanvas() throws {
    let sideways = SIMD3<Float>(sin(1.2), 0, -cos(1.2))
    let pixel = try #require(gazeTarget(sideways, on: canvas))
    #expect(pixel.x > 7672 * 0.8)
    #expect(abs(pixel.y - 1080) < 20)
    #expect(gazeTarget(SIMD3(0, 0, 1), on: canvas) == nil)
}

@Test func lookingStraightAtAScreenTargetsItsMiddle() throws {
    let screen = RoomScreen(width: 1920, height: 1080, placement: ScreenPlacement(direction: SIMD3(0, 0, -1)))
    let pixel = try #require(gazeTarget(SIMD3(0, 0, -1), on: screen))
    #expect(simd_distance(pixel, SIMD2(960, 540)) < 1)
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

@Test func onlyTilesNearTheViewCountAsShown() {
    var viewport = ViewportController(settings: Settings())
    viewport.recenter(HeadPose())
    let room = viewport.roomView(canvas: canvas, outputWidth: 1920, outputHeight: 1080, highlighted: false)
    let middle = room.panels.filter { $0.span.x <= 0 && $0.span.y >= 0 }
    let outer = room.panels.filter { $0.span.x == -1 || $0.span.y == 1 }
    #expect(!middle.isEmpty && !outer.isEmpty)
    for panel in middle {
        #expect(room.shows(panel, margin: 0.5), "\(panel.span)")
    }
    for panel in outer {
        #expect(!room.shows(panel, margin: 0.5), "\(panel.span)")
    }
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

private func arcLength(_ surface: ScreenSurface, from start: SIMD2<Float>, to end: SIMD2<Float>) -> Float {
    var length: Float = 0
    var previous = surface.point(at: start)
    for step in 1...200 {
        let point = surface.point(at: start + (end - start) * Float(step) / 200)
        length += simd_distance(point, previous)
        previous = point
    }
    return length
}

@Test(arguments: [Float(1), 0.5, 2.5])
func aScreenCurvedLikeAMonitorIsAsWideAtTheTopAsInTheMiddle(radius: Float) throws {
    for screen in [canvas, raisedCurved] {
        let surface = screen.surface(curveRadius: radius)
        let middleRow = arcLength(surface, from: SIMD2(-1, 0), to: SIMD2(1, 0))
        let middleColumn = arcLength(surface, from: SIMD2(0, -1), to: SIMD2(0, 1))
        #expect(abs(middleRow - Float(screen.width) * roomUnitsPerPixel) < 0.01)
        #expect(abs(middleColumn - Float(screen.height) * roomUnitsPerPixel) < 0.01)
        for y in [Float(1), -1] {
            #expect(abs(arcLength(surface, from: SIMD2(-1, y), to: SIMD2(1, y)) - middleRow) < 1e-3, "row \(y)")
        }
        for x in [Float(1), -1] {
            #expect(abs(arcLength(surface, from: SIMD2(x, -1), to: SIMD2(x, 1)) - middleColumn) < 1e-3, "column \(x)")
        }
    }
}

@Test func aScreenCurvedLikeAMonitorWrapsAroundTheViewerWithStraightSides() throws {
    let surface = canvas.surface(curveRadius: 1)
    let middle = simd_length(surface.point(at: .zero))
    // Across it wraps around the viewer, as far away at its ends.
    #expect(abs(simd_length(surface.point(at: SIMD2(1, 0))) - middle) < 1e-3)
    // Its sides run straight from top to bottom.
    for x in [Float(-1), 1] {
        let (top, bottom) = (surface.point(at: SIMD2(x, 1)), surface.point(at: SIMD2(x, -1)))
        #expect(simd_distance(surface.point(at: SIMD2(x, 0)), (top + bottom) / 2) < 1e-4)
    }
}

@Test(arguments: [Float(0.5), 1, 5])
func lookingAtAPointOnAScreenCurvedLikeAMonitorFindsThatPixel(radius: Float) throws {
    let surface = raisedCurved.surface(curveRadius: radius)
    for position in [SIMD2<Float>(0, 0), SIMD2(0.8, 0.9), SIMD2(-0.6, -0.7), SIMD2(0.99, -0.99)] {
        let gaze = simd_normalize(surface.point(at: position))
        let pixel = try #require(gazeTarget(gaze, on: raisedCurved, curveRadius: radius))
        let expected = SIMD2((position.x + 1) * 0.5 * 5120, (1 - position.y) * 0.5 * 1440)
        #expect(simd_distance(pixel, expected) < 2, "\(position): \(pixel)")
    }
}

@Test func aScreenPixelIsFoundWhereItHangsInTheRoom() {
    let placement = ScreenPlacement(direction: SIMD3(sin(0.6), 0.1, -cos(0.6)))
    let screen = RoomScreen(width: 1920, height: 1080, placement: placement, curved: true)
    let surface = screen.surface()
    let topLeft = screen.roomPoint(ofPixel: SIMD2(0, 0))
    let middle = screen.roomPoint(ofPixel: SIMD2(960, 540))
    #expect(simd_distance(topLeft, surface.point(at: SIMD2(-1, 1))) < 1e-4)
    #expect(simd_distance(simd_normalize(middle), placement.direction) < 1e-4)
}

@Test func zoomingOutBringsAPointBesideTheViewIntoSight() {
    let tanHalf = tan(horizontalFov * 0.5)
    let room = RoomView(headRotation: matrix_identity_float3x3, tanHalfFov: SIMD2(tanHalf, tanHalf * 9 / 16), panels: [])
    let beside = SIMD3<Float>(sin(0.7), 0, -cos(0.7))
    #expect(!room.shows(beside, scale: 1, margin: 0.9))
    #expect(room.shows(beside, scale: 0.3, margin: 0.9))
    // Behind the viewer no zoom helps.
    #expect(room.isAhead(beside))
    #expect(!room.isAhead(SIMD3(0, 0, 1)))
}

@Test func aGentlerCurveBendsTheEdgesLess() throws {
    let even = canvas.surface(curveRadius: 1)
    let gentle = canvas.surface(curveRadius: 2.5)
    // The middle stays where the screen hangs.
    #expect(simd_distance(gentle.point(at: .zero), even.point(at: .zero)) < 1e-4)
    for edge in [SIMD2<Float>(1, 0), SIMD2(-1, 0), SIMD2(0.8, -0.9)] {
        #expect(simd_length(gentle.point(at: edge)) > simd_length(even.point(at: edge)) + 0.01, "\(edge)")
    }
}

@Test func lookingAtAPointOnAGentlyCurvedScreenFindsThatPixel() throws {
    let surface = raisedCurved.surface(curveRadius: 2.5)
    for position in [SIMD2<Float>(0, 0), SIMD2(0.8, 0.9), SIMD2(-0.6, -0.7)] {
        let gaze = simd_normalize(surface.point(at: position))
        let pixel = try #require(gazeTarget(gaze, on: raisedCurved, curveRadius: 2.5))
        let expected = SIMD2((position.x + 1) * 0.5 * 5120, (1 - position.y) * 0.5 * 1440)
        #expect(simd_distance(pixel, expected) < 2, "\(position): \(pixel)")
    }
}

/// The head turned by `yaw` and tilted by `roll`, as the viewport builds it.
private func headRotation(yaw: Float = 0, roll: Float) -> simd_float3x3 {
    var viewport = ViewportController(settings: Settings())
    viewport.recenter(HeadPose())
    viewport.track(pose: HeadPose(yaw: yaw, roll: roll))
    return viewport.headRotation
}

@Test(arguments: [false, true])
func aScreenCarriedWithTheHeadTiltedKeepsThatTilt(curved: Bool) throws {
    let start = ScreenPlacement(direction: SIMD3(0, 0, -1))
    let grab = ScreenGrab(placement: start, headRotation: headRotation(roll: 0))

    // Carried a little to the side with the head tilted, then let go.
    let tiltedHead = headRotation(yaw: 0.3, roll: 0.25)
    let dropped = grab.placement(headRotation: tiltedHead, distance: start.distance)
    let screen = RoomScreen(width: 1920, height: 1080, placement: dropped, curved: curved)
    let surface = screen.surface()

    // Seen with the head tilted like that, it looks upright...
    let top = tiltedHead.transpose * surface.point(at: SIMD2(0, 1))
    let bottom = tiltedHead.transpose * surface.point(at: SIMD2(0, -1))
    let upInView = normalize(top - bottom)
    #expect(abs(upInView.x) < 0.01, "up in view \(upInView)")

    // ...so with the head straight again it leans by the tilt.
    let straight = headRotation(yaw: 0.3, roll: 0)
    let upStraight = normalize(straight.transpose * surface.point(at: SIMD2(0, 1)) - straight.transpose * surface.point(at: SIMD2(0, -1)))
    #expect(abs(abs(asin(upStraight.x)) - 0.25) < 0.02, "leans \(asin(upStraight.x)) rad")
}

@Test(arguments: [false, true])
func lookingAtATiltedScreenFindsThePixelThere(curved: Bool) throws {
    let placement = ScreenPlacement(direction: normalize(SIMD3(0.3, 0.1, -1)), distance: 1.2, tilt: 0.4)
    let screen = RoomScreen(width: 2880, height: 1620, placement: placement, curved: curved)
    let surface = screen.surface()
    for position in [SIMD2<Float>(0.6, 0.4), SIMD2(-0.7, -0.5), SIMD2(0, 0.9)] {
        let gaze = normalize(surface.point(at: position))
        let hit = try #require(surface.hit(gaze: gaze))
        #expect(simd_distance(hit.at, SIMD2(position.x, -position.y)) < 1e-3, "at \(position): \(hit.at)")
    }
}

@Test func theCanvasTiltIsKeptAfterARestart() {
    var settings = Settings()
    settings.canvas.placement = ScreenPlacement(direction: SIMD3(0, 0, -1), tilt: -0.3)
    #expect(abs(Settings.parse(settings.serialize()).canvas.placement.tilt - -0.3) < 1e-6)
    settings.canvas.placement.tilt = 0
    #expect(Settings.parse(settings.serialize()).canvas.placement.tilt == 0)
}

@Test func aCurvedScreenPutAnywhereLooksAsItDoesStraightAheadWhenFaced() {
    let placements = [
        ScreenPlacement(direction: SIMD3(0, sin(0.5), -cos(0.5))),
        ScreenPlacement(direction: SIMD3(0.6, -0.4, -0.7), tilt: 0.3),
        ScreenPlacement(direction: SIMD3(-1, 0.2, 0), distance: 1.4, tilt: -0.2),
    ]
    for placement in placements {
        var screen = RoomScreen(width: 5752, height: 2160, curved: true)
        screen.placement = placement
        let moved = screen.surface()
        screen.placement = ScreenPlacement(direction: SIMD3(0, 0, -1), distance: placement.distance)
        let ahead = screen.surface()
        let middle = (ahead.point(at: .zero), moved.point(at: .zero))
        // The same distance to every point, and between each point and the
        // middle, as straight ahead: the same shape, only turned.
        for x in [Float(-1), -0.4, 0.5, 1] {
            for y in [Float(-1), 0, 1] {
                let (a, b) = (ahead.point(at: SIMD2(x, y)), moved.point(at: SIMD2(x, y)))
                #expect(abs(simd_length(a) - simd_length(b)) < 1e-4, "x \(x) y \(y)")
                #expect(abs(simd_distance(a, middle.0) - simd_distance(b, middle.1)) < 1e-4, "x \(x) y \(y)")
            }
        }
    }
}
