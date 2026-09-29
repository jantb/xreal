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

@Test func onAWrappedCanvasTheRowCarriesOnUpTheSameSphereAboveItsTopEdge() throws {
    let wrapped = RoomScreen(width: 5752, height: 2160, spherical: true)
    let layout = overheadLayout(canvas: wrapped, dashboard: SIMD2(2400, 172), pinned: SIMD2(800, 600))
    let row = overheadPanels(canvas: wrapped, layout: layout)
    let board = try #require(row.dashboard)
    for x in [board.left, 0, board.right] {
        let bottom = row.surface.point(at: SIMD2(x, board.bottom))
        let edge = wrapped.surface().point(at: SIMD2(x, 1))
        #expect(elevation(bottom) > elevation(edge), "x \(x)")
        // Still on the sphere, facing the viewer.
        #expect(abs(simd_length(bottom) - 1) < 1e-4, "x \(x)")
    }
}
