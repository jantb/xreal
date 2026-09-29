import Metal
import Testing
import XrealCore

@testable import XrealViewer

@Test func theRendererAndItsShadersStartOnThisMac() throws {
    _ = try Renderer()
}

@MainActor @Test func theStatusStripShowsItsItemsAndNothingOnceHidden() throws {
    let device = try #require(MTLCreateSystemDefaultDevice())
    let strip = StatusStrip(device: device)
    strip.show(["14:05", "CPU 12%"])
    let short = try #require(strip.latest.current().frame)
    strip.show(["14:05", "CPU 12%", "RAM 18.0 of 32 GB", "Tracking 1000 Hz"])
    let long = try #require(strip.latest.current().frame)
    #expect(long.width > short.width)
    #expect(long.height == short.height)
    strip.show(nil)
    #expect(strip.latest.current().frame == nil)
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
