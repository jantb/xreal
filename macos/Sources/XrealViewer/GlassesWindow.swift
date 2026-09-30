import AppKit
import QuartzCore

/// Identifies the glasses and the other displays.
enum Displays {
    /// EDID manufacturer "MRG", which all XREAL glasses report.
    private static let glassesVendor: UInt32 = 0x3647

    static func isGlasses(_ id: CGDirectDisplayID) -> Bool {
        CGDisplayVendorNumber(id) == glassesVendor
    }

    /// Vendor number of the virtual screen this app creates.
    static let virtualScreenVendor: UInt32 = 0x5852  // "XR"

    static func glassesScreen() -> NSScreen? {
        NSScreen.screens.first { $0.displayID.map(isGlasses) ?? false }
    }

    static func glassesDisplay() -> CGDirectDisplayID? {
        active().first(where: isGlasses)
    }

    /// A display outside the glasses, such as the laptop's own screen:
    /// neither the glasses nor this app's canvas.
    static func isReal(_ id: CGDirectDisplayID) -> Bool {
        !isGlasses(id) && CGDisplayVendorNumber(id) != virtualScreenVendor
    }

    /// Takes the glasses out of any mirroring, of another display or by
    /// another display. macOS sometimes mirrors them when displays come and
    /// go, which hides the viewer's window and starves it of frames. Returns
    /// whether anything changed.
    @discardableResult
    static func unmirrorGlasses() -> Bool {
        guard let glasses = online().first(where: isGlasses), CGDisplayIsInMirrorSet(glasses) != 0 else { return false }
        var config: CGDisplayConfigRef?
        guard CGBeginDisplayConfiguration(&config) == .success, let config else { return false }
        CGConfigureDisplayMirrorOfDisplay(config, glasses, kCGNullDirectDisplay)
        for other in online() where CGDisplayMirrorsDisplay(other) == glasses {
            CGConfigureDisplayMirrorOfDisplay(config, other, kCGNullDirectDisplay)
        }
        return CGCompleteDisplayConfiguration(config, .forSession) == .success
    }

    /// Every connected display, including ones mirroring another.
    static func online() -> [CGDirectDisplayID] {
        var count: UInt32 = 0
        var ids = [CGDirectDisplayID](repeating: 0, count: 32)
        CGGetOnlineDisplayList(UInt32(ids.count), &ids, &count)
        return Array(ids.prefix(Int(count)))
    }

    static func active() -> [CGDirectDisplayID] {
        var count: UInt32 = 0
        var ids = [CGDirectDisplayID](repeating: 0, count: 32)
        CGGetActiveDisplayList(UInt32(ids.count), &ids, &count)
        return Array(ids.prefix(Int(count)))
    }
}

extension NSScreen {
    var displayID: CGDirectDisplayID? {
        (deviceDescription[NSDeviceDescriptionKey("NSScreenNumber")] as? NSNumber)?.uint32Value
    }
}

/// A view backed by an opaque CAMetalLayer. Nothing is drawn over it, so the
/// window server can show the layer without compositing it.
final class MetalView: NSView {
    let metalLayer = CAMetalLayer()

    init(device: MTLDevice) {
        super.init(frame: .zero)
        metalLayer.device = device
        metalLayer.pixelFormat = .bgra8Unorm_srgb
        metalLayer.colorspace = CGColorSpace(name: CGColorSpace.sRGB)
        metalLayer.framebufferOnly = true
        metalLayer.displaySyncEnabled = true
        metalLayer.isOpaque = true
        wantsLayer = true
    }

    @available(*, unavailable)
    required init?(coder: NSCoder) { fatalError("not used") }

    override func makeBackingLayer() -> CALayer { metalLayer }
    override var isOpaque: Bool { true }

    override func viewDidMoveToWindow() {
        super.viewDidMoveToWindow()
        updateDrawableSize()
    }

    override func setFrameSize(_ newSize: NSSize) {
        super.setFrameSize(newSize)
        updateDrawableSize()
    }

    override func viewDidChangeBackingProperties() {
        super.viewDidChangeBackingProperties()
        updateDrawableSize()
    }

    private func updateDrawableSize() {
        let scale = window?.backingScaleFactor ?? 1
        metalLayer.contentsScale = scale
        metalLayer.drawableSize = CGSize(width: bounds.width * scale, height: bounds.height * scale)
    }
}

/// Full screen on the glasses, hidden while they are not connected.
@MainActor final class GlassesWindow {
    let view: MetalView
    private let window: NSWindow
    private let displayLink: DisplayLinkThread
    /// The screen and rate the display link runs for.
    private var linked: (display: CGDirectDisplayID, fps: Int)?

    init(device: MTLDevice, displayLink: DisplayLinkThread) {
        self.displayLink = displayLink
        view = MetalView(device: device)
        window = NSWindow(
            contentRect: NSRect(x: 0, y: 0, width: 960, height: 540),
            styleMask: [.titled, .closable, .resizable], backing: .buffered, defer: false)
        window.title = "XREAL Viewer"
        window.isReleasedWhenClosed = false
        window.backgroundColor = .black
        window.isOpaque = true
        window.collectionBehavior = [.canJoinAllSpaces, .fullScreenAuxiliary]
        window.contentView = view
    }

    var windowID: CGWindowID {
        CGWindowID(window.windowNumber)
    }

    func hide() {
        window.orderOut(nil)
        // The glasses may come back as a new screen.
        linked = nil
    }

    /// Moves the window full screen onto the glasses, or out of sight when
    /// they are not connected. It never takes the keyboard, which stays with
    /// the app being worked in.
    func place() {
        guard let screen = Displays.glassesScreen() else {
            hide()
            return
        }
        window.styleMask = [.borderless]
        // Above the menu bar and Dock of that screen.
        window.level = .statusBar
        window.setFrame(screen.frame, display: true)
        window.orderFrontRegardless()
        // Recreated so the link runs at the refresh rate of this screen,
        // only when that changes: recreating it drops frames.
        let wanted = (display: screen.displayID ?? 0, fps: screen.maximumFramesPerSecond)
        if linked.map({ $0 != wanted }) ?? true {
            linked = wanted
            displayLink.attach(to: view.metalLayer, fps: wanted.fps)
        }
    }
}
