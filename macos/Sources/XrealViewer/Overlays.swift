import AppKit
import Metal
import ScreenCaptureKit
import Synchronization
import XrealCore

/// Everything drawn with the canvas but not captured from it is made at
/// this many pixels per point, so it stays crisp on a HiDPI canvas and is
/// filtered down smoothly on a plain one.
let overlayPixelsPerPoint: Float = 2

/// A value shared between threads behind a lock.
final class Guarded<Value: Sendable>: Sendable {
    let mutex: Mutex<Value>

    init(_ value: Value) {
        mutex = Mutex(value)
    }
}

/// A premultiplied BGRA bitmap of `width` × `height` pixels drawn by `draw`
/// into a context with the origin at its bottom left.
func drawBitmap(width: Int, height: Int, draw: (CGContext) -> Void) -> Data? {
    guard width > 0, height > 0, let space = CGColorSpace(name: CGColorSpace.sRGB),
        let context = CGContext(
            data: nil, width: width, height: height, bitsPerComponent: 8, bytesPerRow: width * 4, space: space,
            bitmapInfo: CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue)
    else { return nil }
    NSGraphicsContext.saveGraphicsState()
    NSGraphicsContext.current = NSGraphicsContext(cgContext: context, flipped: false)
    draw(context)
    NSGraphicsContext.restoreGraphicsState()
    guard let data = context.data else { return nil }
    return Data(bytes: data, count: width * height * 4)
}

/// `bitmap`, from `drawBitmap`, as an image to draw.
func image(of bitmap: Data, width: Int, height: Int, device: MTLDevice) -> CapturedFrame? {
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(
        pixelFormat: .bgra8Unorm_srgb, width: width, height: height, mipmapped: false)
    descriptor.usage = .shaderRead
    guard let texture = device.makeTexture(descriptor: descriptor) else { return nil }
    bitmap.withUnsafeBytes { bytes in
        texture.replace(
            region: MTLRegionMake2D(0, 0, width, height), mipmapLevel: 0, withBytes: bytes.baseAddress!,
            bytesPerRow: width * 4)
    }
    return CapturedFrame(texture: texture, pixelsPerPoint: overlayPixelsPerPoint)
}

/// The system's current mouse pointer as an image, for drawing it where the
/// mouse is at each frame rather than where the capture last saw it.
@MainActor final class PointerImage {
    /// The pointer and its hot spot, in points from its top-left corner.
    let latest = Guarded<(frame: CapturedFrame?, hotSpot: SIMD2<Float>)>((nil, .zero))
    private let device: MTLDevice
    private var shownKey: Int?

    init(device: MTLDevice) {
        self.device = device
    }

    /// Picks up a change of pointer, such as to a text beam or a resize arrow.
    func update() {
        // The system's pointer, whichever app set it.
        let cursor = NSCursor.currentSystem ?? .arrow
        let size = cursor.image.size
        let scale = CGFloat(overlayPixelsPerPoint)
        let (width, height) = (Int((size.width * scale).rounded(.up)), Int((size.height * scale).rounded(.up)))
        let drawn = CGRect(origin: .zero, size: CGSize(width: size.width * scale, height: size.height * scale))
        guard let bitmap = drawBitmap(width: width, height: height, draw: { _ in cursor.image.draw(in: drawn) })
        else { return }
        // Only a changed pointer is made into a new image.
        var hasher = Hasher()
        hasher.combine(bitmap)
        hasher.combine(cursor.hotSpot.x)
        hasher.combine(cursor.hotSpot.y)
        let key = hasher.finalize()
        guard key != shownKey, let frame = image(of: bitmap, width: width, height: height, device: device) else {
            return
        }
        shownKey = key
        let hotSpot = SIMD2(Float(cursor.hotSpot.x), Float(cursor.hotSpot.y))
        latest.mutex.withLock { $0 = (frame, hotSpot) }
    }

    func hide() {
        shownKey = nil
        latest.mutex.withLock { $0 = (nil, .zero) }
    }
}

/// A window that can be pinned above the canvas, as the controls list it.
struct PinnableWindow: Hashable, Identifiable, Sendable {
    var id: CGWindowID
    var window: PinnedWindow
    var appName: String
    var width: Int
    var height: Int

    var label: String { "\(appName) – \(window.title)" }

    func matches(_ query: String) -> Bool {
        query.isEmpty || label.localizedStandardContains(query)
    }
}

extension ScreenCapture {
    /// Windows worth pinning: ordinary, titled, on screen, not this app's.
    static func pinnableWindows() async throws -> [PinnableWindow] {
        let content = try await content()
        let own = Bundle.main.bundleIdentifier
        return content.windows.compactMap { window -> PinnableWindow? in
            guard window.windowLayer == 0, window.isOnScreen, let title = window.title, !title.isEmpty,
                window.frame.width >= 50, window.frame.height >= 50,
                let app = window.owningApplication, app.bundleIdentifier != own, !app.bundleIdentifier.isEmpty
            else { return nil }
            return PinnableWindow(
                id: window.windowID,
                window: PinnedWindow(bundleID: app.bundleIdentifier, title: title),
                appName: app.applicationName, width: Int(window.frame.width), height: Int(window.frame.height))
        }
        .sorted { $0.label.localizedStandardCompare($1.label) == .orderedAscending }
    }

    /// Preserve a live selection across title changes. After reopening,
    /// only an unambiguous title match may replace it.
    static func find(_ pinned: PinnedWindow, in content: SCShareableContent, preferredID: CGWindowID? = nil) -> SCWindow? {
        let windows = content.windows.filter {
            $0.owningApplication?.bundleIdentifier == pinned.bundleID && $0.windowLayer == 0
                && $0.frame.width >= 50 && $0.frame.height >= 50
        }
        let choices = windows.map {
            PinnableWindow(id: $0.windowID, window: PinnedWindow(bundleID: pinned.bundleID, title: $0.title ?? ""),
                           appName: "", width: Int($0.frame.width), height: Int($0.frame.height))
        }
        guard let id = pinnedWindowID(for: pinned, preferredID: preferredID, choices: choices) else { return nil }
        return windows.first { $0.windowID == id }
    }
}

func pinnedWindowID(for wanted: PinnedWindow, preferredID: CGWindowID?, choices: [PinnableWindow]) -> CGWindowID? {
    let sameApp = choices.filter { $0.window.bundleID == wanted.bundleID }
    if let preferredID, sameApp.contains(where: { $0.id == preferredID }) { return preferredID }
    let exact = sameApp.filter { $0.window.title == wanted.title }
    return exact.count == 1 ? exact[0].id : nil
}
