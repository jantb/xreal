import Testing
import simd

@testable import XrealCore

private let canvas = RoomScreen(width: 5752, height: 2160, curved: true)

private func elevation(_ point: SIMD3<Float>) -> Float {
    atan2(point.y, simd_length(SIMD2(point.x, point.z)))
}

@Test func theDashboardIsCentredAndThePinnedWindowRestsBesideIt() throws {
    let layout = overheadLayout(canvas: canvas, dashboard: SIMD2(2000, 172), pinned: SIMD2(800, 600))
    let board = try #require(layout.dashboard)
    let window = try #require(layout.pinned)
    #expect(abs(board.left + board.right) < 1e-5)
    #expect(window.left > board.right)
    // Both rest on the row's bottom edge, which is as tall as the taller.
    #expect(board.bottom == -1 && window.bottom == -1)
    #expect(layout.height == 600)
    // Sizes in points are kept: the dashboard is 2000 of 5752 points wide.
    #expect(abs((board.right - board.left) / 2 * 5752 - 2000) < 0.5)
}

@Test func onANarrowCanvasThePinnedWindowGoesAboveTheDashboard() throws {
    let small = RoomScreen(width: 1920, height: 1080)
    let layout = overheadLayout(canvas: small, dashboard: SIMD2(1900, 172), pinned: SIMD2(800, 600))
    let board = try #require(layout.dashboard)
    let window = try #require(layout.pinned)
    #expect(window.bottom > board.top)
    #expect(window.top <= 1 + 1e-5)
}

@Test func aWindowWiderThanTheCanvasIsShrunkToItsWidth() throws {
    let small = RoomScreen(width: 1920, height: 1080)
    let layout = overheadLayout(canvas: small, dashboard: nil, pinned: SIMD2(3840, 1000))
    let window = try #require(layout.pinned)
    #expect(window.left >= -1 - 1e-5 && window.right <= 1 + 1e-5)
    // Kept in proportion.
    #expect(abs(layout.height - 500) < 0.5)
}

@Test func theRowHangsJustAboveTheCanvasTurnedToFaceTheViewer() {
    let placements = [
        ScreenPlacement.straightAhead, ScreenPlacement(direction: SIMD3(0.5, -0.3, -0.8), tilt: 0.2),
    ]
    for placement in placements {
        for curved in [true, false] {
            var screen = canvas
            screen.curved = curved
            screen.placement = placement
            let row = overheadSurface(canvas: screen, height: 400)
            let rowMiddle = row.point(at: .zero)
            let upward = row.point(at: SIMD2(0, 0.01)) - row.point(at: SIMD2(0, -0.01))
            // Seen just above the canvas's top edge, as the canvas's up goes,
            // right out to the sides.
            let turn = placement.orientation.inverse
            let seen = { (point: SIMD3<Float>) in elevation(turn.act(point)) }
            for x in [Float(-0.9), -0.5, 0, 0.5, 0.9] {
                let (edge, bottom) = (screen.surface().point(at: SIMD2(x, 1)), row.point(at: SIMD2(x, -1)))
                #expect(seen(bottom) > seen(edge), "curved \(curved) x \(x)")
                #expect(seen(bottom) - seen(edge) < 0.05, "curved \(curved) x \(x)")
            }
            // Its middle faces the eyes: straight up it runs across the line
            // of sight, not along the canvas.
            #expect(abs(dot(normalize(upward), normalize(rowMiddle))) < 1e-2, "curved \(curved)")
        }
    }
}

@Test func thePointersHotSpotLandsWhereTheMouseIs() {
    let point = SIMD2<Float>(1000, 400)
    let hotSpot = SIMD2<Float>(4, 4)
    let rect = pointerRect(canvas: canvas, at: point, size: SIMD2(32, 32), hotSpot: hotSpot)
    // Back from surface positions to canvas points.
    let topLeft = SIMD2((rect.left + 1) / 2 * 5752, (1 - rect.top) / 2 * 2160)
    #expect(simd_distance(topLeft + hotSpot, point) < 0.01)
    #expect(abs((rect.right - rect.left) / 2 * 5752 - 32) < 0.01)
}

@Test func cpuLoadIsTheShareOfTimeSpentBusy() {
    let before = CpuTicks(busy: 1000, idle: 5000)
    #expect(CpuTicks(busy: 1030, idle: 5070).load(since: before) == 0.3)
    #expect(before.load(since: before) == nil)
    #expect(CpuTicks.now() != nil)
    #expect(CpuTicks.perCore()?.isEmpty == false)
}

@Test func aHistoryKeepsOnlyTheLatestValuesOldestFirst() {
    var history = History(capacity: 3)
    for value in [1, 2, 3, 4, 5] as [Float] {
        history.append(value)
    }
    #expect(history.values == [3, 4, 5])
}

@Test func trafficIsBytesPerSecondAndACounterGoingBackIsNoReading() throws {
    let before = NetworkCounters(received: 1000, sent: 500)
    let rate = try #require(NetworkCounters(received: 3000, sent: 1500).rate(since: before, seconds: 2))
    #expect(rate.down == 1000 && rate.up == 500)
    #expect(NetworkCounters(received: 10, sent: 10).rate(since: before, seconds: 1) == nil)
    let disk = try #require(DiskCounters(read: 400, written: 200).rate(since: DiskCounters(read: 0, written: 0), seconds: 4))
    #expect(disk.read == 100 && disk.write == 50)
}

@Test func theBusiestProcessesComeFirstWithHelpersCountedTogether() {
    let before = ProcessTimes(
        times: [1: 0, 2: 0, 3: 0, 4: 0], names: [1: "Xcode", 2: "Helper", 3: "Helper", 4: "Idle"])
    let after = ProcessTimes(
        times: [1: 1_500_000_000, 2: 600_000_000, 3: 600_000_000, 4: 0], names: before.names)
    let busiest = after.busiest(since: before, seconds: 1, count: 2)
    #expect(busiest.map(\.name) == ["Xcode", "Helper"])
    #expect(abs(busiest[0].share - 1.5) < 1e-5)
    #expect(abs(busiest[1].share - 1.2) < 1e-5)
}

@Test func lookingAtTheMacTwiceGivesLoadsForEveryCore() {
    var monitor = SystemMonitor()
    _ = monitor.sample(now: 0)
    let sample = monitor.sample(now: 0.1)
    #expect(sample.cpu != nil)
    #expect(sample.cores.count == CpuTicks.perCore()?.count)
    #expect(sample.memory != nil)
}

/// How high the canvas's top edge is, seen from the eyes as the canvas's
/// own up goes, in the direction `azimuth` across: nil beyond its sides.
private func topEdge(of screen: RoomScreen, atAzimuth azimuth: Float) -> Float? {
    let turn = screen.placement.orientation.inverse
    let edge = (0...256).map { step -> (azimuth: Float, elevation: Float) in
        let point = turn.act(screen.surface().point(at: SIMD2(-1 + Float(step) / 128, 1)))
        return (atan2(point.x, -point.z), elevation(point))
    }
    guard let index = edge.indices.dropLast().first(where: { edge[$0].azimuth <= azimuth && azimuth <= edge[$0 + 1].azimuth })
    else { return nil }
    let (a, b) = (edge[index], edge[index + 1])
    let t = b.azimuth > a.azimuth ? (azimuth - a.azimuth) / (b.azimuth - a.azimuth) : 0
    return a.elevation + (b.elevation - a.elevation) * t
}

@Test func whatHangsAboveStaysClearOfEveryCanvasShapeAndFacesTheEyes() throws {
    let shapes = [
        RoomScreen(width: 5752, height: 2160), RoomScreen(width: 5752, height: 2160, curved: true),
        RoomScreen(width: 5752, height: 2160, spherical: true),
        RoomScreen(width: 3832, height: 4320, spherical: true, verticalWrap: 0.5),
    ]
    for (index, shape) in shapes.enumerated() {
        var screen = shape
        screen.placement = ScreenPlacement(direction: SIMD3(0.2, 0.1, -1), distance: 1.3, tilt: 0.1)
        let layout = overheadLayout(canvas: screen, dashboard: SIMD2(2400, 172), pinned: SIMD2(900, 700))
        let panels = overheadPanels(canvas: screen, layout: layout)
        let turn = screen.placement.orientation.inverse
        for (surface, rect) in [try #require(panels.dashboard), try #require(panels.pinned)] {
            for x in [rect.left, (rect.left + rect.right) / 2, rect.right] {
                // Above the canvas's top edge in the same direction.
                let bottom = turn.act(surface.point(at: SIMD2(x, rect.bottom)))
                let edge = try #require(topEdge(of: screen, atAzimuth: atan2(bottom.x, -bottom.z)))
                #expect(elevation(bottom) > edge, "shape \(index) x \(x)")
            }
            // Its middle faces the eyes.
            let (midX, midY) = ((rect.left + rect.right) / 2, (rect.top + rect.bottom) / 2)
            let middle = surface.point(at: SIMD2(midX, midY))
            let upward = surface.point(at: SIMD2(midX, midY + 0.01)) - surface.point(at: SIMD2(midX, midY - 0.01))
            #expect(abs(dot(normalize(upward), normalize(middle))) < 1e-2, "shape \(index)")
        }
    }
}

@Test func overAWrappedCanvasThePinnedWindowCurvesLikeItFacingTheEyesEverywhere() throws {
    var screen = RoomScreen(width: 5752, height: 2160, spherical: true)
    screen.placement = ScreenPlacement(direction: SIMD3(0, 0, -1), distance: 1.3)
    let layout = overheadLayout(canvas: screen, dashboard: SIMD2(2400, 172), pinned: SIMD2(900, 700))
    let panels = overheadPanels(canvas: screen, layout: layout)
    // The dashboard too faces the eyes from every pixel.
    let (board, boardRect) = try #require(panels.dashboard)
    for x in [boardRect.left, 0, boardRect.right] {
        let point = board.point(at: SIMD2(x, 0))
        let across = board.point(at: SIMD2(x + 0.001, 0)) - board.point(at: SIMD2(x - 0.001, 0))
        #expect(abs(simd_length(point) - 1.3) < 1e-3 && abs(dot(normalize(across), normalize(point))) < 1e-2, "x \(x)")
    }
    let (window, rect) = try #require(panels.pinned)
    // Beside the dashboard, not over it: its left edge is further round
    // than the dashboard's right one, all the way up.
    let azimuth = { (point: SIMD3<Float>) in atan2(point.x, -point.z) }
    let boardRight = azimuth(board.point(at: SIMD2(boardRect.right, boardRect.bottom)))
    for y in [Float(-1), 0, 1] {
        #expect(azimuth(window.point(at: SIMD2(rect.left, y))) > boardRight, "y \(y)")
    }
    let (midX, midY) = ((rect.left + rect.right) / 2, (rect.top + rect.bottom) / 2)
    // A monitor of its own: curving round its own middle, which faces the
    // eyes straight on.
    let middle = window.point(at: SIMD2(midX, midY))
    let across = window.point(at: SIMD2(midX + 0.001, midY)) - window.point(at: SIMD2(midX - 0.001, midY))
    let upward = window.point(at: SIMD2(midX, midY + 0.001)) - window.point(at: SIMD2(midX, midY - 0.001))
    #expect(abs(simd_length(window.point(at: SIMD2(-1, 0))) - simd_length(window.point(at: SIMD2(1, 0)))) < 1e-4)
    #expect(abs(dot(normalize(across), normalize(middle))) < 1e-3 && abs(dot(normalize(upward), normalize(middle))) < 1e-3)
    #expect(middle.x > 0)
    for x in [rect.left, midX, rect.right] {
        for y in [rect.bottom, midY, rect.top] {
            let point = window.point(at: SIMD2(x, y))
            let across = window.point(at: SIMD2(x + 0.001, y)) - window.point(at: SIMD2(x - 0.001, y))
            let upward = window.point(at: SIMD2(x, y + 0.001)) - window.point(at: SIMD2(x, y - 0.001))
            // Every pixel as far away as the canvas and square to the eyes.
            #expect(abs(simd_length(point) - 1.3) < 1e-3, "x \(x) y \(y)")
            #expect(abs(dot(normalize(across), normalize(point))) < 1e-2, "x \(x) y \(y)")
            #expect(abs(dot(normalize(upward), normalize(point))) < 1e-2, "x \(x) y \(y)")
        }
    }
    // As many points across and up as the window has, at its middle.
    let perPoint = 2 * tan(horizontalFov / 2) / 1920
    let (dx, dy) = ((rect.right - rect.left) * 0.01, (rect.top - rect.bottom) * 0.01)
    let width = simd_distance(window.point(at: SIMD2(midX - dx, midY)), window.point(at: SIMD2(midX + dx, midY))) / 0.02
    let height = simd_distance(window.point(at: SIMD2(midX, midY - dy)), window.point(at: SIMD2(midX, midY + dy))) / 0.02
    #expect(abs(width / perPoint - 900) < 9)
    #expect(abs(height / perPoint - 700) < 7)
}
