import AppKit
import Metal
import ScreenCaptureKit
import Synchronization
import XrealCore

/// Everything drawn with the canvas but not captured from it is made at
/// this many pixels per point, so it stays crisp on a HiDPI canvas and is
/// filtered down smoothly on a plain one.
let overlayPixelsPerPoint: Float = 2

// The status strip's text size and height, in points.
private let statusFontSize: CGFloat = 20
private let statusHeight: CGFloat = 36
private let statusPadding: CGFloat = 14
private let statusSeparator = "      "

/// A value shared between threads behind a lock.
final class Guarded<Value: Sendable>: Sendable {
    let mutex: Mutex<Value>

    init(_ value: Value) {
        mutex = Mutex(value)
    }
}

/// A premultiplied BGRA bitmap of `width` × `height` pixels drawn by `draw`
/// into a context with the origin at its bottom left.
@MainActor private func drawBitmap(width: Int, height: Int, draw: (CGContext) -> Void) -> Data? {
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
private func image(of bitmap: Data, width: Int, height: Int, device: MTLDevice) -> CapturedFrame? {
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(
        pixelFormat: .bgra8Unorm_srgb, width: width, height: height, mipmapped: false)
    descriptor.usage = .shaderRead
    guard let texture = device.makeTexture(descriptor: descriptor) else { return nil }
    bitmap.withUnsafeBytes { bytes in
        texture.replace(
            region: MTLRegionMake2D(0, 0, width, height), mipmapLevel: 0, withBytes: bytes.baseAddress!,
            bytesPerRow: width * 4)
    }
    return CapturedFrame(texture: texture)
}

/// The line of status above the canvas, redrawn only when what it says
/// changes.
@MainActor final class StatusStrip {
    let latest = LatestFrame()
    private let device: MTLDevice
    private var shown: [String]?
    private var ticks: CpuTicks?

    init(device: MTLDevice) {
        self.device = device
    }

    /// The share of the CPU used since the last call.
    func cpuLoad() -> Float? {
        let now = CpuTicks.now()
        defer { ticks = now }
        guard let now, let ticks else { return nil }
        return now.load(since: ticks)
    }

    /// Shows `items`, or nothing when nil.
    func show(_ items: [String]?) {
        guard items != shown else { return }
        shown = items
        guard let items else {
            latest.publish(nil)
            return
        }
        let text = NSAttributedString(
            string: items.joined(separator: statusSeparator),
            attributes: [
                .font: NSFont.monospacedDigitSystemFont(ofSize: statusFontSize, weight: .medium),
                .foregroundColor: NSColor(white: 0.85, alpha: 1),
            ])
        let size = CGSize(width: (text.size().width + 2 * statusPadding).rounded(.up), height: statusHeight)
        let scale = CGFloat(overlayPixelsPerPoint)
        let (width, height) = (Int(size.width * scale), Int(size.height * scale))
        let bitmap = drawBitmap(width: width, height: height) { context in
            context.scaleBy(x: scale, y: scale)
            // A faint frame, so the strip reads as one thing. Black shows
            // nothing in the glasses.
            let frame = CGRect(origin: .zero, size: size).insetBy(dx: 1, dy: 1)
            context.setStrokeColor(CGColor(gray: 0.3, alpha: 1))
            context.setLineWidth(1.5)
            context.addPath(CGPath(roundedRect: frame, cornerWidth: 8, cornerHeight: 8, transform: nil))
            context.strokePath()
            text.draw(at: CGPoint(x: statusPadding, y: (size.height - text.size().height) / 2))
        }
        latest.publish(bitmap.flatMap { image(of: $0, width: width, height: height, device: device) })
    }
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
struct PinnableWindow: Hashable, Sendable {
    var window: PinnedWindow
    var label: String
}

extension ScreenCapture {
    /// Windows worth pinning: ordinary, titled, on screen, not this app's.
    static func pinnableWindows() async -> [PinnableWindow] {
        guard let content = try? await content() else { return [] }
        let own = Bundle.main.bundleIdentifier
        return content.windows.compactMap { window -> PinnableWindow? in
            guard window.windowLayer == 0, window.isOnScreen, let title = window.title, !title.isEmpty,
                window.frame.width >= 50, window.frame.height >= 50,
                let app = window.owningApplication, app.bundleIdentifier != own, !app.bundleIdentifier.isEmpty
            else { return nil }
            return PinnableWindow(
                window: PinnedWindow(bundleID: app.bundleIdentifier, title: title),
                label: "\(app.applicationName) – \(title)")
        }
        .sorted { $0.label.localizedStandardCompare($1.label) == .orderedAscending }
    }

    /// The window `pinned` names in `content`: the app's window with that
    /// title, or else its first ordinary one, as titles change.
    static func find(_ pinned: PinnedWindow, in content: SCShareableContent) -> SCWindow? {
        let windows = content.windows.filter {
            $0.owningApplication?.bundleIdentifier == pinned.bundleID && $0.windowLayer == 0
                && $0.frame.width >= 50 && $0.frame.height >= 50
        }
        return windows.first { $0.title == pinned.title } ?? windows.first { $0.isOnScreen } ?? windows.first
    }
}
