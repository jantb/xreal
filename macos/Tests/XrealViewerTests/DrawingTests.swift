import Metal
import Testing
import XrealCore

@testable import XrealViewer

@Test func theRendererAndItsShadersStartOnThisMac() throws {
    _ = try Renderer()
}

@Test func theDashboardShowsTilesAndNothingOnceHidden() throws {
    let device = try #require(MTLCreateSystemDefaultDevice())
    let dashboard = Dashboard(device: device)
    let glasses = GlassesReadings(temperature: 31, trackingHz: 1000, fps: 90, latency: 0.02, lateFramesPerSecond: 0)
    dashboard.update(glasses: glasses)
    dashboard.settle()
    let image = try #require(dashboard.latest.current().frame)
    // A row of tiles: far wider than tall, drawn at its points' density.
    #expect(image.width > 4 * image.height)
    #expect(image.pixelsPerPoint == overlayPixelsPerPoint)
    dashboard.update(glasses: nil)
    #expect(dashboard.latest.current().frame == nil)
}

@MainActor @Test func thePointerIsPickedUpAsAnImageWithItsHotSpotInside() throws {
    let device = try #require(MTLCreateSystemDefaultDevice())
    let pointer = PointerImage(device: device)
    pointer.update()
    let (frame, hotSpot) = pointer.latest.mutex.withLock { $0 }
    let image = try #require(frame)
    let size = SIMD2(Float(image.width), Float(image.height)) / overlayPixelsPerPoint
    #expect(hotSpot.x >= 0 && hotSpot.y >= 0 && hotSpot.x <= size.x && hotSpot.y <= size.y)
    pointer.hide()
    #expect(pointer.latest.mutex.withLock { $0.frame } == nil)
}
