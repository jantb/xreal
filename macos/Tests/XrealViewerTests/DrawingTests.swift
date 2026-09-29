import Metal
import QuartzCore
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

/// A white image of `width` × `height` pixels.
private func whiteImage(_ device: MTLDevice, width: Int, height: Int) throws -> CapturedFrame {
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(
        pixelFormat: .bgra8Unorm_srgb, width: width, height: height, mipmapped: false)
    let texture = try #require(device.makeTexture(descriptor: descriptor))
    let bytes = [UInt8](repeating: 255, count: width * height * 4)
    texture.replace(region: MTLRegionMake2D(0, 0, width, height), mipmapLevel: 0, withBytes: bytes, bytesPerRow: width * 4)
    return CapturedFrame(texture: texture)
}

/// A frame of a small white flat canvas straight ahead, with the view
/// reaching past its edges, drawn as the glasses would get it.
private func drawWhiteCanvas(softEdges: Bool, ambientLight: Bool = false) async throws -> MTLTexture {
    let renderer = try Renderer()
    let layer = CAMetalLayer()
    layer.device = renderer.device
    layer.pixelFormat = .bgra8Unorm_srgb
    layer.framebufferOnly = false
    layer.drawableSize = CGSize(width: 3840, height: 1080)
    let drawable = try #require(layer.nextDrawable())
    var settings = XrealCore.Settings()
    settings.canvas = RoomScreen(width: 1920, height: 1080)
    settings.canvas.placement = ScreenPlacement(direction: SIMD3(0, 0, -1), distance: 2)
    settings.ambientLight = ambientLight
    var state = ViewerState(settings: settings)
    let now = monotonicNow()
    let room = try #require(
        state.advance(
            now: now, dt: 1 / 90, presentingAt: now + 1 / 90, snapshot: TrackingSnapshot(gyroBias: .zero),
            captureGeneration: 0, newFrame: true, frameSizes: [(1920, 1080)], output: (3840, 1080), cursor: nil
        ).room)
    let images = PanelImages(canvas: [try whiteImage(renderer.device, width: 1920, height: 1080)])
    await withCheckedContinuation { (done: CheckedContinuation<Void, Never>) in
        renderer.draw(to: drawable, images: images, room: room, sharpen: true, softEdges: softEdges) { _ in
            done.resume()
        }
    }
    return drawable.texture
}

/// The red, green and blue of every pixel of row `y`.
private func row(_ texture: MTLTexture, _ y: Int) -> [UInt8] {
    var bytes = [UInt8](repeating: 0, count: texture.width * 4)
    texture.getBytes(&bytes, bytesPerRow: texture.width * 4, from: MTLRegionMake2D(0, y, texture.width, 1), mipmapLevel: 0)
    return bytes
}

@Test func aFrameOfAWhiteCanvasComesOutWhiteInTheMiddleOfEachEyeAndDarkBeyondIt() async throws {
    let frame = try await drawWhiteCanvas(softEdges: true)
    let (middle, top) = (row(frame, 540), row(frame, 5))
    for eye in 0..<2 {
        let x = (eye * 1920 + 960) * 4
        #expect(middle[x..<x + 3].allSatisfy { $0 > 240 }, "eye \(eye) middle")
        let corner = (eye * 1920 + 5) * 4
        #expect(top[corner..<corner + 3].allSatisfy { $0 < 10 }, "eye \(eye) corner")
    }
}

@Test func softEdgesDimOnlyTheCanvasBorder() async throws {
    let soft = row(try await drawWhiteCanvas(softEdges: true), 540)
    let hard = row(try await drawWhiteCanvas(softEdges: false), 540)
    let dimmed = zip(soft, hard).filter { $0 < $1 }.count
    // Some pixels at the left and right borders of each eye, a few tens
    // of values, and nothing else.
    #expect(dimmed > 0)
    #expect(dimmed < 200)
    #expect(zip(soft, hard).allSatisfy { $0 <= $1 })
}

@Test func ambientLightGlowsInTheRoomAroundTheCanvasOnlyWhenOn() async throws {
    let lit = row(try await drawWhiteCanvas(softEdges: true, ambientLight: true), 540)
    let plain = row(try await drawWhiteCanvas(softEdges: true), 540)
    // Pixels dark beside the canvas without it, and lit with it.
    let glowing = stride(from: 0, to: plain.count, by: 4).filter { plain[$0] < 10 && lit[$0] > 40 }.count
    #expect(glowing > 40)
    // The canvas itself is left as it is.
    #expect(lit[960 * 4] == plain[960 * 4])
}
