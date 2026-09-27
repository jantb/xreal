import CoreMedia
import CoreVideo
import Metal
import ScreenCaptureKit
import Synchronization
import XrealCore

/// A captured screen image as a Metal texture over ScreenCaptureKit's
/// IOSurface, so nothing is copied.
final class CapturedFrame: @unchecked Sendable {
    let texture: MTLTexture
    let width: Int
    let height: Int
    // Keeps the IOSurface out of ScreenCaptureKit's reuse pool while in use.
    private let backing: CVMetalTexture

    init(texture: MTLTexture, backing: CVMetalTexture) {
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

/// Streams one display through ScreenCaptureKit into `latest`.
final class ScreenCapture: NSObject, SCStreamOutput, SCStreamDelegate, @unchecked Sendable {
    let latest = LatestFrame()
    private let queue = DispatchQueue(label: "xreal.capture", qos: .userInteractive)
    private let textureCache: CVMetalTextureCache
    @MainActor private var stream: SCStream?

    init(device: MTLDevice) throws {
        var cache: CVMetalTextureCache?
        CVMetalTextureCacheCreate(nil, nil, device, nil, &cache)
        guard let cache else { throw CaptureError.noTextureCache }
        textureCache = cache
    }

    /// Captures `displayID` at `pixelSize`, leaving out the window with
    /// `excludedWindowID` so the viewer never captures itself. The app's
    /// other windows, such as its menu bar menu, stay in the picture.
    @MainActor func start(
        displayID: CGDirectDisplayID, pixelSize: (width: Int, height: Int), fps: Int, excludedWindowID: CGWindowID
    ) async throws {
        await stop()
        let content = try await SCShareableContent.excludingDesktopWindows(false, onScreenWindowsOnly: false)
        guard let display = content.displays.first(where: { $0.displayID == displayID }) else {
            throw CaptureError.displayNotShareable
        }
        let viewerWindow = content.windows.filter { $0.windowID == excludedWindowID }
        let filter = SCContentFilter(display: display, excludingWindows: viewerWindow)

        let configuration = SCStreamConfiguration()
        configuration.width = pixelSize.width
        configuration.height = pixelSize.height
        configuration.minimumFrameInterval = CMTime(value: 1, timescale: CMTimeScale(fps))
        configuration.pixelFormat = kCVPixelFormatType_32BGRA
        configuration.colorSpaceName = CGColorSpace.sRGB
        configuration.queueDepth = 4
        configuration.showsCursor = true

        let newStream = SCStream(filter: filter, configuration: configuration, delegate: self)
        try newStream.addStreamOutput(self, type: .screen, sampleHandlerQueue: queue)
        try await newStream.startCapture()
        stream = newStream
    }

    @MainActor func stop() async {
        guard let current = stream else { return }
        stream = nil
        try? await current.stopCapture()
        latest.publish(nil)
    }

    /// Waits until ScreenCaptureKit lists `displayID`, which takes a moment
    /// for a display that was just created, and returns its size in points.
    static func shareableSize(of displayID: CGDirectDisplayID, timeout: Double) async -> (width: Int, height: Int)? {
        let deadline = monotonicNow() + timeout
        while monotonicNow() < deadline {
            if let content = try? await SCShareableContent.excludingDesktopWindows(false, onScreenWindowsOnly: false),
                let display = content.displays.first(where: { $0.displayID == displayID })
            {
                return (display.width, display.height)
            }
            try? await Task.sleep(for: .milliseconds(500))
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
            nil, textureCache, pixelBuffer, nil, .bgra8Unorm, CVPixelBufferGetWidth(pixelBuffer),
            CVPixelBufferGetHeight(pixelBuffer), 0, &backing)
        guard let backing, let texture = CVMetalTextureGetTexture(backing) else { return }
        latest.publish(CapturedFrame(texture: texture, backing: backing))
    }

    func stream(_ stream: SCStream, didStopWithError error: Error) {
        eprint("Screen capture stopped: \(error.localizedDescription)")
    }
}
