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
    // Keeps the IOSurface out of ScreenCaptureKit's reuse pool while in use.
    private let backing: CVMetalTexture?

    init(texture: MTLTexture, backing: CVMetalTexture? = nil) {
        self.texture = texture
        self.backing = backing
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
    private let queue = DispatchQueue(label: "xreal.capture", qos: .userInteractive)
    private let textureCache: CVMetalTextureCache
    @MainActor private var stream: SCStream?
    @MainActor private var configuration: SCStreamConfiguration?
    @MainActor private(set) var fps = 0

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
        sourceRect: CGRect? = nil, fps: Int, showsCursor: Bool, excludedWindowID: CGWindowID
    ) async throws {
        guard let display = content.displays.first(where: { $0.displayID == displayID }) else {
            throw CaptureError.displayNotShareable
        }
        let viewerWindow = content.windows.filter { $0.windowID == excludedWindowID }
        let configuration = Self.configuration(pixelSize: pixelSize, fps: fps, showsCursor: showsCursor)
        if let sourceRect {
            configuration.sourceRect = sourceRect
        }
        try await start(
            filter: SCContentFilter(display: display, excludingWindows: viewerWindow), configuration: configuration)
    }

    /// Captures just `window`, wherever it is and whatever covers it, at
    /// `pixelSize`.
    @MainActor func start(window: SCWindow, pixelSize: (width: Int, height: Int), fps: Int) async throws {
        let configuration = Self.configuration(pixelSize: pixelSize, fps: fps, showsCursor: false)
        configuration.ignoreShadowsSingleWindow = true
        try await start(filter: SCContentFilter(desktopIndependentWindow: window), configuration: configuration)
    }

    /// Captures at `pixelSize` from now on.
    @MainActor func setPixelSize(_ pixelSize: (width: Int, height: Int)) async {
        guard let stream, let configuration else { return }
        configuration.width = pixelSize.width
        configuration.height = pixelSize.height
        do {
            try await stream.updateConfiguration(configuration)
        } catch {
            eprint("Could not change the capture size: \(error.localizedDescription)")
        }
    }

    private static func configuration(pixelSize: (width: Int, height: Int), fps: Int, showsCursor: Bool)
        -> SCStreamConfiguration
    {
        let configuration = SCStreamConfiguration()
        configuration.width = pixelSize.width
        configuration.height = pixelSize.height
        configuration.minimumFrameInterval = CMTime(value: 1, timescale: CMTimeScale(fps))
        configuration.pixelFormat = kCVPixelFormatType_32BGRA
        configuration.colorSpaceName = CGColorSpace.sRGB
        configuration.queueDepth = 4
        configuration.showsCursor = showsCursor
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
        guard let stream, let configuration, fps != self.fps else { return }
        self.fps = fps
        configuration.minimumFrameInterval = CMTime(value: 1, timescale: CMTimeScale(fps))
        do {
            try await stream.updateConfiguration(configuration)
        } catch {
            eprint("Could not change the capture rate: \(error.localizedDescription)")
        }
    }

    @MainActor func stop() async {
        guard let current = stream else { return }
        stream = nil
        configuration = nil
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
        latest.publish(CapturedFrame(texture: texture, backing: backing))
    }

    func stream(_ stream: SCStream, didStopWithError error: Error) {
        eprint("Screen capture stopped: \(error.localizedDescription)")
    }
}
