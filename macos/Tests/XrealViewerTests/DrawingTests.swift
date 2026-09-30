import Metal
import QuartzCore
import Testing
import XrealCore
import simd

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

/// A white image of `width` × `height` pixels, with a black border
/// `border` pixels wide.
private func whiteImage(_ device: MTLDevice, width: Int, height: Int, border: Int = 0) throws -> CapturedFrame {
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(
        pixelFormat: .bgra8Unorm_srgb, width: width, height: height, mipmapped: false)
    let texture = try #require(device.makeTexture(descriptor: descriptor))
    var bytes = [UInt8](repeating: 255, count: width * height * 4)
    for y in 0..<height {
        for x in 0..<width where min(x, y, width - 1 - x, height - 1 - y) < border {
            bytes.replaceSubrange((y * width + x) * 4..<(y * width + x) * 4 + 3, with: [0, 0, 0])
        }
    }
    texture.replace(region: MTLRegionMake2D(0, 0, width, height), mipmapLevel: 0, withBytes: bytes, bytesPerRow: width * 4)
    return CapturedFrame(texture: texture)
}

/// A frame of a small white flat canvas straight ahead, with the view
/// reaching past its edges, drawn as the glasses would get it.
private func drawWhiteCanvas(
    softEdges: Bool, ambientLight: Bool = false, border: Int = 0, dashboard: DashboardCards? = nil
) async throws -> MTLTexture {
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
    let dashboard = try dashboard.map { try cardsImage(renderer.device, width: 1600, height: 340, $0) }
    let room = try #require(
        state.advance(
            now: now, dt: 1 / 90, presentingAt: now + 1 / 90, snapshot: TrackingSnapshot(gyroBias: .zero),
            captureGeneration: 0, newFrame: true, frameSizes: [(1920, 1080)], output: (3840, 1080), cursor: nil,
            extras: ExtraSizes(status: dashboard.map { _ in SIMD2(800, 170) })
        ).room)
    let images = PanelImages(
        canvas: [try whiteImage(renderer.device, width: 1920, height: 1080, border: border)], status: dashboard)
    await withCheckedContinuation { (done: CheckedContinuation<Void, Never>) in
        renderer.draw(to: drawable, images: images, room: room, sharpen: true, softEdges: softEdges) { _ in
            done.resume()
        }
    }
    return drawable.texture
}

/// A dashboard's picture: one faint card over all of it, or two with a
/// see-through gap between them.
private enum DashboardCards {
    case one, twoWithGap
}

/// A dashboard picture of `width` × `height` pixels, at two pixels a point,
/// its cards a faint white wash.
private func cardsImage(_ device: MTLDevice, width: Int, height: Int, _ cards: DashboardCards) throws -> CapturedFrame {
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(
        pixelFormat: .bgra8Unorm_srgb, width: width, height: height, mipmapped: false)
    let texture = try #require(device.makeTexture(descriptor: descriptor))
    var bytes = [UInt8](repeating: 18, count: width * height * 4)
    if cards == .twoWithGap {
        for y in 0..<height {
            bytes.replaceSubrange((y * width + width / 3) * 4..<(y * width + 2 * width / 3) * 4, with: repeatElement(0, count: width / 3 * 4))
        }
    }
    texture.replace(region: MTLRegionMake2D(0, 0, width, height), mipmapLevel: 0, withBytes: bytes, bytesPerRow: width * 4)
    return CapturedFrame(texture: texture, pixelsPerPoint: 2)
}

/// A frame of nothing but the arrow back to the canvas, pointing `way`.
private func drawArrowAlone(pointing way: SIMD2<Float>) async throws -> MTLTexture {
    let renderer = try Renderer()
    let layer = CAMetalLayer()
    layer.device = renderer.device
    layer.pixelFormat = .bgra8Unorm_srgb
    layer.framebufferOnly = false
    layer.drawableSize = CGSize(width: 3840, height: 1080)
    let drawable = try #require(layer.nextDrawable())
    var state = ViewerState(settings: XrealCore.Settings())
    let now = monotonicNow()
    var room = try #require(
        state.advance(
            now: now, dt: 1 / 90, presentingAt: now + 1 / 90, snapshot: TrackingSnapshot(gyroBias: .zero),
            captureGeneration: 0, newFrame: true, frameSizes: [], output: (3840, 1080), cursor: nil
        ).room)
    room.panels = []
    room.pointBack = way
    await withCheckedContinuation { (done: CheckedContinuation<Void, Never>) in
        renderer.draw(to: drawable, images: PanelImages(), room: room, sharpen: true) { _ in done.resume() }
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

@Test func ambientLightShowsWhatIsOnTheCanvasBeyondItsDarkEdge() async throws {
    // White inside a black border, and all black.
    let framed = row(try await drawWhiteCanvas(softEdges: true, ambientLight: true, border: 200), 540)
    let black = row(try await drawWhiteCanvas(softEdges: true, ambientLight: true, border: 1080), 540)
    // Beside the canvas, the white inside the border lights the room.
    let lit = stride(from: 0, to: framed.count, by: 4).filter { framed[$0] > black[$0] + 30 }.count
    #expect(lit > 40)
}

@Test func theDashboardsCardsHideTheGlowAndTheGapsBetweenThemShowIt() async throws {
    let open = try await drawWhiteCanvas(softEdges: true, ambientLight: true)
    let covered = try await drawWhiteCanvas(softEdges: true, ambientLight: true, dashboard: .one)
    let gapped = try await drawWhiteCanvas(softEdges: true, ambientLight: true, dashboard: .twoWithGap)
    // Above the canvas, where the dashboard hangs: pixels of glow a faint
    // card hides, and pixels the gap between two cards shows again.
    var hidden = 0
    var shown = 0
    for y in stride(from: 0, to: 300, by: 10) {
        let (lit, behind, between) = (row(open, y), row(covered, y), row(gapped, y))
        for x in stride(from: 0, to: lit.count, by: 4) where lit[x + 2] > 40 && behind[x + 2] < 30 {
            hidden += 1
            if between[x + 2] > 40 {
                shown += 1
            }
        }
    }
    #expect(hidden > 100)
    #expect(shown > 30)
    #expect(shown < hidden)
}

@Test(arguments: [-0.8, 0, 0.8] as [Float])
func identicalEyesSeeIdenticalPixelsWhenPanningAcrossAWideCanvas(yaw: Float) async throws {
    let renderer = try Renderer()
    let layer = CAMetalLayer()
    layer.device = renderer.device
    layer.pixelFormat = .bgra8Unorm_srgb
    layer.framebufferOnly = false
    layer.drawableSize = CGSize(width: 3840, height: 1080)
    let drawable = try #require(layer.nextDrawable())
    var settings = Settings()
    settings.canvas = RoomScreen(width: 5752, height: 2160)
    settings.prediction = false
    var state = ViewerState(settings: settings)
    let now = monotonicNow()
    var room = try #require(state.advance(
        now: now, dt: 1 / 90, presentingAt: now + 1 / 90, snapshot: TrackingSnapshot(gyroBias: .zero),
        captureGeneration: 0, newFrame: true, frameSizes: [(5752, 2160)], output: (3840, 1080), cursor: nil
    ).room)
    // Remove intentional stereo disparity. The two views must now agree,
    // even when the canvas extends beyond each eye's field of view.
    var eye = DisplayCalibration.nominal.left
    eye.position = .zero
    room.eyes = [eye, eye]
    room.headRotation = simd_float3x3(simd_quatf(angle: yaw, axis: SIMD3(0, 1, 0)))
    room.scanEndRotation = simd_float3x3(simd_quatf(angle: yaw + 0.012, axis: SIMD3(0, 1, 0)))
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(
        pixelFormat: .bgra8Unorm_srgb, width: 5752, height: 1, mipmapped: false)
    let texture = try #require(renderer.device.makeTexture(descriptor: descriptor))
    let bytes = (0..<5752).flatMap { x -> [UInt8] in
        let value: UInt8 = (x / 32) % 2 == 0 ? 255 : 0
        return [value, value, value, 255]
    }
    texture.replace(region: MTLRegionMake2D(0, 0, 5752, 1), mipmapLevel: 0, withBytes: bytes, bytesPerRow: 5752 * 4)
    // One wide panel deliberately reaches outside both eye viewports.
    room.panels = [RoomView.Panel(source: .canvas(0), surface: settings.canvas.surface(curveRadius: 1))]
    let images = PanelImages(canvas: [CapturedFrame(texture: texture)])
    await withCheckedContinuation { (done: CheckedContinuation<Void, Never>) in
        renderer.draw(to: drawable, images: images, room: room, sharpen: false, softEdges: false) { _ in
            done.resume()
        }
    }
    let pixels = row(drawable.texture, 540)
    let mismatches = (0..<1920).filter { x in
        abs(Int(pixels[x * 4]) - Int(pixels[(x + 1920) * 4])) > 2
    }.count
    #expect(mismatches == 0, "Identical eye views differ at \(mismatches) pixels")
}

@Test func resizingThroughOneToOneFilteringDoesNotSuddenlyChangeAThinStroke() async throws {
    let renderer = try Renderer()
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(
        pixelFormat: .bgra8Unorm_srgb, width: 64, height: 1, mipmapped: false)
    let source = try #require(renderer.device.makeTexture(descriptor: descriptor))
    let bytes = (0..<64).flatMap { x -> [UInt8] in
        let value: UInt8 = x == 32 ? 255 : 0
        return [value, value, value, 255]
    }
    source.replace(region: MTLRegionMake2D(0, 0, 64, 1), mipmapLevel: 0, withBytes: bytes, bytesPerRow: 64 * 4)
    var values: [UInt8] = []
    for footprint: Float in [1.0199, 1.0201] {
        let layer = CAMetalLayer()
        layer.device = renderer.device
        layer.pixelFormat = .bgra8Unorm_srgb
        layer.framebufferOnly = false
        layer.drawableSize = CGSize(width: 3840, height: 1080)
        let drawable = try #require(layer.nextDrawable())
        var state = ViewerState(settings: Settings())
        var room = try #require(state.advance(now: 100, dt: 1 / 90, presentingAt: 100.01,
            snapshot: TrackingSnapshot(gyroBias: .zero), captureGeneration: 0, newFrame: true,
            frameSizes: [(64, 1)], output: (3840, 1080), cursor: nil).room)
        let eye = EyeOptics(focal: SIMD2(repeating: 960), center: SIMD2(960.5, 540.5), size: SIMD2(1920, 1080),
                           rotation: matrix_identity_float3x3, position: .zero)
        room.eyes = [eye, eye]
        let halfWidth = 64 / (2 * 960 * footprint)
        let surface = ScreenSurface(center: SIMD3(-halfWidth / 64, 0, -1),
                                    right: SIMD3(halfWidth, 0, 0), up: SIMD3(0, 0.5, 0))
        room.panels = [RoomView.Panel(source: .canvas(0), surface: surface)]
        await withCheckedContinuation { (done: CheckedContinuation<Void, Never>) in
            renderer.draw(to: drawable, images: PanelImages(canvas: [CapturedFrame(texture: source)]),
                          room: room, sharpen: true, softEdges: false) { _ in done.resume() }
        }
        values.append(row(drawable.texture, 540)[960 * 4])
    }
    #expect(abs(Int(values[0]) - Int(values[1])) <= 2,
            "A tiny resize changed the stroke from \(values[0]) to \(values[1])")
}

@Test func theArrowBackToTheCanvasShowsInEachEyeOnTheSideItPoints() async throws {
    let middle = row(try await drawArrowAlone(pointing: SIMD2(1, 0)), 540)
    // Blue, the way the canvas's outline is.
    func lit(_ x: Int) -> Bool { middle[x * 4] > 100 && middle[x * 4] > middle[x * 4 + 2] }
    for eye in 0..<2 {
        let left = (eye * 1920)..<(eye * 1920 + 960)
        let right = (eye * 1920 + 960)..<(eye * 1920 + 1920)
        #expect(right.contains(where: lit), "eye \(eye)")
        #expect(!left.contains(where: lit), "eye \(eye)")
    }
}
