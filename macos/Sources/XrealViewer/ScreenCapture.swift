import CoreMedia
import CoreVideo
import Metal
import ScreenCaptureKit
import Synchronization
import XrealCore

/// An image to draw: a captured screen image as a Metal texture over
/// ScreenCaptureKit's IOSurface, so nothing is copied, or one the app made.
final class CapturedFrame: @unchecked Sendable {
    let texture: MTLTexture
    let width: Int
    let height: Int
    /// How many of its pixels make a point, for images shown at their size
    /// in points.
    let pixelsPerPoint: Float
    // Keeps the IOSurface out of ScreenCaptureKit's reuse pool while in use.
    private let backing: CVMetalTexture?

    init(texture: MTLTexture, backing: CVMetalTexture? = nil, pixelsPerPoint: Float = 1) {
        self.texture = texture
        self.backing = backing
        self.pixelsPerPoint = pixelsPerPoint
        width = texture.width
        height = texture.height
    }
}

/// Holds only the newest captured frame. The render loop never waits on
/// capture: it reuses the last frame until a newer one arrives.
final class LatestFrame: Sendable {
    private let state = Mutex<(frame: CapturedFrame?, generation: UInt64)>((nil, 0))

    /// The newest frame and a counter that increments with every new frame.
    func current() -> (frame: CapturedFrame?, generation: UInt64) {
        state.withLock { $0 }
    }

    func publish(_ frame: CapturedFrame?) {
        state.withLock { state in
            state.frame = frame
            state.generation &+= 1
        }
    }
}

enum CaptureError: LocalizedError {
    case displayNotShareable
    case noTextureCache

    var errorDescription: String? {
        switch self {
        case .displayNotShareable: "the display is not available to screen capture"
        case .noTextureCache: "could not create a Metal texture cache"
        }
    }
}

/// Streams one display, or one window, through ScreenCaptureKit into
/// `latest`.
final class ScreenCapture: NSObject, SCStreamOutput, SCStreamDelegate, @unchecked Sendable {
    let latest = LatestFrame()
    /// Pixels per point of what is captured, given to every frame.
    private let pixelsPerPoint = Mutex<Float>(1)
    private let queue = DispatchQueue(label: "xreal.capture", qos: .userInteractive)
    private let textureCache: CVMetalTextureCache
    @MainActor private var stream: SCStream?
    @MainActor private var configuration: SCStreamConfiguration?
    @MainActor private var updatingConfiguration = false
    @MainActor private(set) var fps = 0
    /// Called when the capture stops by itself, as when macOS ends it, so
    /// it can be started again.
    @MainActor var onStop: (() -> Void)?
    /// The captured window's size in points and density, and the longest
    /// side allowed, while capturing a window.
    @MainActor private var windowCapture: (size: CGSize, scale: CGFloat, maxPixels: CGFloat)?

    init(device: MTLDevice) throws {
        var cache: CVMetalTextureCache?
        CVMetalTextureCacheCreate(nil, nil, device, nil, &cache)
        guard let cache else { throw CaptureError.noTextureCache }
        textureCache = cache
    }

    /// What there is to capture now.
    static func content() async throws -> SCShareableContent {
        try await SCShareableContent.excludingDesktopWindows(false, onScreenWindowsOnly: false)
    }

    /// Captures `displayID` from `content` at `pixelSize`, leaving out the
    /// window with `excludedWindowID` so the viewer never captures itself.
    /// The app's other windows, such as its menu bar menu, stay in the
    /// picture. `sourceRect`, in the display's points, captures only that
    /// part of it.
    @MainActor func start(
        displayID: CGDirectDisplayID, in content: SCShareableContent, pixelSize: (width: Int, height: Int),
        sourceRect: CGRect? = nil, fps: Int, excludedWindowID: CGWindowID
    ) async throws {
        guard let display = content.displays.first(where: { $0.displayID == displayID }) else {
            throw CaptureError.displayNotShareable
        }
        let viewerWindow = content.windows.filter { $0.windowID == excludedWindowID }
        let configuration = Self.configuration(pixelSize: pixelSize, fps: fps)
        if let sourceRect {
            configuration.sourceRect = sourceRect
        }
        try await start(
            filter: SCContentFilter(display: display, excludingWindows: viewerWindow), configuration: configuration)
    }

    /// Captures just `window`, wherever it is and whatever covers it, at
    /// its own pixel density but at most `maxPixels` on its longer side.
    @MainActor func start(window: SCWindow, fps: Int, maxPixels: CGFloat) async throws {
        let filter = SCContentFilter(desktopIndependentWindow: window)
        let scale = Self.pixelScale(of: filter)
        let pixels = Self.pixelSize(of: window.frame.size, scale: scale, maxPixels: maxPixels)
        let configuration = Self.configuration(pixelSize: pixels, fps: fps)
        // Shown at the window's size in points, however many pixels it has.
        pixelsPerPoint.withLock { $0 = Float(pixels.width) / Float(max(window.frame.width, 1)) }
        configuration.ignoreShadowsSingleWindow = true
        // The window fills the whole image, whatever density it is drawn at.
        configuration.scalesToFit = true
        try await start(filter: filter, configuration: configuration)
        windowCapture = (window.frame.size, scale, maxPixels)
    }

    /// Follows the captured window to `size`, in points.
    @MainActor func resizeWindow(to size: CGSize) async -> Bool {
        guard let stream, let configuration, let window = windowCapture else { return false }
        guard window.size != size else { return true }
        guard !updatingConfiguration else { return false }
        updatingConfiguration = true
        defer { updatingConfiguration = false }
        let pixels = Self.pixelSize(of: size, scale: window.scale, maxPixels: window.maxPixels)
        let previous = (configuration.width, configuration.height)
        configuration.width = pixels.width
        configuration.height = pixels.height
        do {
            try await stream.updateConfiguration(configuration)
            guard self.stream === stream else { return false }
            windowCapture?.size = size
            pixelsPerPoint.withLock { $0 = Float(pixels.width) / Float(max(size.width, 1)) }
            return true
        } catch {
            configuration.width = previous.0
            configuration.height = previous.1
            eprint("Could not change the capture size: \(error.localizedDescription)")
            return false
        }
    }

    /// Pixels per point of the display `filter`'s content is on.
    private static func pixelScale(of filter: SCContentFilter) -> CGFloat {
        max(CGFloat(SCShareableContent.info(for: filter).pointPixelScale), 1)
    }

    private static func pixelSize(of size: CGSize, scale: CGFloat, maxPixels: CGFloat) -> (width: Int, height: Int) {
        let fit = min(1, maxPixels / max(size.width * scale, size.height * scale, 1))
        return (max(Int(size.width * scale * fit), 1), max(Int(size.height * scale * fit), 1))
    }

    private static func configuration(pixelSize: (width: Int, height: Int), fps: Int)
        -> SCStreamConfiguration
    {
        let configuration = SCStreamConfiguration()
        configuration.width = pixelSize.width
        configuration.height = pixelSize.height
        configuration.minimumFrameInterval = CMTime(value: 1, timescale: CMTimeScale(fps))
        configuration.pixelFormat = kCVPixelFormatType_32BGRA
        configuration.colorSpaceName = CGColorSpace.sRGB
        configuration.queueDepth = 4
        // The pointer is drawn live over the captures instead.
        configuration.showsCursor = false
        return configuration
    }

    @MainActor private func start(filter: SCContentFilter, configuration: SCStreamConfiguration) async throws {
        await stop()
        let newStream = SCStream(filter: filter, configuration: configuration, delegate: self)
        try newStream.addStreamOutput(self, type: .screen, sampleHandlerQueue: queue)
        try await newStream.startCapture()
        stream = newStream
        self.configuration = configuration
        fps = Int(configuration.minimumFrameInterval.timescale)
    }

    /// Changes how often frames are captured, while capturing.
    @MainActor func setFps(_ fps: Int) async {
        guard let stream, let configuration, fps != self.fps, !updatingConfiguration else { return }
        updatingConfiguration = true
        defer { updatingConfiguration = false }
        let previous = configuration.minimumFrameInterval
        configuration.minimumFrameInterval = CMTime(value: 1, timescale: CMTimeScale(fps))
        do {
            try await stream.updateConfiguration(configuration)
            guard self.stream === stream else { return }
            self.fps = fps
        } catch {
            configuration.minimumFrameInterval = previous
            eprint("Could not change the capture rate: \(error.localizedDescription)")
        }
    }

    @MainActor func stop() async {
        guard let current = stream else { return }
        stream = nil
        configuration = nil
        fps = 0
        windowCapture = nil
        try? await current.stopCapture()
        latest.publish(nil)
    }

    /// Waits until ScreenCaptureKit lists `displayID`, which takes a moment
    /// for a display that was just created, and returns where it is in the
    /// arrangement, in global points with a top-left origin.
    static func shareableFrame(of displayID: CGDirectDisplayID, timeout: Double) async -> CGRect? {
        let deadline = monotonicNow() + timeout
        while monotonicNow() < deadline, !Task.isCancelled {
            if let content = try? await SCShareableContent.excludingDesktopWindows(false, onScreenWindowsOnly: false),
                let display = content.displays.first(where: { $0.displayID == displayID })
            {
                return display.frame
            }
            // A cancelled sleep throws at once; stop instead of polling flat out.
            do {
                try await Task.sleep(for: .milliseconds(500))
            } catch {
                return nil
            }
        }
        return nil
    }

    func stream(_ stream: SCStream, didOutputSampleBuffer sampleBuffer: CMSampleBuffer, of type: SCStreamOutputType) {
        // Idle frames (unchanged screen) carry no image; keep the last one.
        guard type == .screen,
            let attachments = CMSampleBufferGetSampleAttachmentsArray(sampleBuffer, createIfNecessary: false)
                as? [[SCStreamFrameInfo: Any]],
            let rawStatus = attachments.first?[.status] as? Int,
            SCFrameStatus(rawValue: rawStatus) == .complete,
            let pixelBuffer = sampleBuffer.imageBuffer
        else { return }

        var backing: CVMetalTexture?
        CVMetalTextureCacheCreateTextureFromImage(
            nil, textureCache, pixelBuffer, nil, .bgra8Unorm_srgb, CVPixelBufferGetWidth(pixelBuffer),
            CVPixelBufferGetHeight(pixelBuffer), 0, &backing)
        guard let backing, let texture = CVMetalTextureGetTexture(backing) else { return }
        latest.publish(
            CapturedFrame(texture: texture, backing: backing, pixelsPerPoint: pixelsPerPoint.withLock { $0 }))
    }

    func stream(_ stream: SCStream, didStopWithError error: Error) {
        eprint("Screen capture stopped: \(error.localizedDescription)")
        let stopped = ObjectIdentifier(stream)
        Task { @MainActor in
            // Not one stopped on purpose, or replaced since.
            guard self.stream.map(ObjectIdentifier.init) == stopped else { return }
            await stop()
            onStop?()
        }
    }
}
