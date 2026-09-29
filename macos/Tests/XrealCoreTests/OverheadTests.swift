import Testing
import simd

@testable import XrealCore

private let canvas = RoomScreen(width: 5752, height: 2160, curved: true)

@Test func theStatusStripHangsCentredAboveTheCanvasAndThePinnedWindowAboveIt() throws {
    let layout = overheadLayout(canvas: canvas, strip: SIMD2(1200, 36), pinned: SIMD2(800, 600))
    let strip = try #require(layout.strip)
    let pinned = try #require(layout.pinned)
    #expect(strip.bottom > 1)
    #expect(pinned.bottom > strip.top)
    for rect in [strip, pinned] {
        #expect(abs(rect.left + rect.right) < 1e-5)
    }
    // Their sizes in points are kept: the strip is 1200 of 5752 points wide.
    #expect(abs((strip.right - strip.left) / 2 * 5752 - 1200) < 0.5)
    #expect(abs((pinned.top - pinned.bottom) / 2 * 2160 - 600) < 0.5)
}

@Test func thePinnedWindowTakesTheStripsPlaceWithoutOne() throws {
    let layout = overheadLayout(canvas: canvas, strip: nil, pinned: SIMD2(800, 600))
    #expect(layout.strip == nil)
    let pinned = try #require(layout.pinned)
    #expect(pinned.bottom > 1)
    #expect(pinned.bottom < 1.1)
}

@Test func aWindowWiderThanTheCanvasIsShrunkToItsWidth() throws {
    let small = RoomScreen(width: 1920, height: 1080)
    let pinned = try #require(overheadLayout(canvas: small, strip: nil, pinned: SIMD2(3840, 1000)).pinned)
    #expect(pinned.left >= -1 - 1e-5 && pinned.right <= 1 + 1e-5)
    // Kept in proportion.
    #expect(abs((pinned.top - pinned.bottom) / 2 * 1080 - 500) < 0.5)
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

@Test func theStatusShowsWhatIsKnownAndLeavesOutWhatIsNot() {
    let everything = statusItems(
        StatusReadings(
            clock: "14:05", battery: Battery(percent: 82, charging: true), cpuLoad: 0.12,
            memory: MemoryUse(used: 18 << 30, total: 32 << 30), glassesTemperature: 31.2, trackingHz: 1000,
            fps: 90, latency: 0.024, lateFramesPerSecond: 1.5))
    let line = everything.joined(separator: " ")
    for part in ["14:05", "82%", "12%", "18.0", "32", "31.2", "1000", "90", "24", "1.5"] {
        #expect(line.contains(part), "\(part) in \(line)")
    }

    let desktop = statusItems(StatusReadings(clock: "14:05"))
    #expect(!desktop.joined().contains("Battery"))
    #expect(!desktop.joined().contains("late"))
}

@Test func cpuLoadIsTheShareOfTimeSpentBusy() {
    let before = CpuTicks(busy: 1000, idle: 5000)
    #expect(CpuTicks(busy: 1030, idle: 5070).load(since: before) == 0.3)
    #expect(before.load(since: before) == nil)
    #expect(CpuTicks.now() != nil)
}
