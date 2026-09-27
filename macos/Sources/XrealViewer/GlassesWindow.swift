import AppKit
import QuartzCore

/// Identifies the glasses and picks what to mirror.
enum Displays {
    /// EDID manufacturer "MRG", which all XREAL glasses report.
    private static let glassesVendor: UInt32 = 0x3647

    static func isGlasses(_ id: CGDirectDisplayID) -> Bool {
        CGDisplayVendorNumber(id) == glassesVendor
    }

    static func glassesScreen() -> NSScreen? {
        NSScreen.screens.first { $0.displayID.map(isGlasses) ?? false }
    }

    /// The main display, unless that is the glasses.
    static func mirrorSource() -> CGDirectDisplayID {
        let main = CGMainDisplayID()
        if !isGlasses(main) { return main }
        var count: UInt32 = 0
        var ids = [CGDirectDisplayID](repeating: 0, count: 16)
        CGGetActiveDisplayList(UInt32(ids.count), &ids, &count)
        return ids.prefix(Int(count)).first { !isGlasses($0) } ?? main
    }

    static func pixelSize(of id: CGDirectDisplayID) -> (width: Int, height: Int) {
        guard let mode = CGDisplayCopyDisplayMode(id) else {
            return (CGDisplayPixelsWide(id), CGDisplayPixelsHigh(id))
        }
        return (mode.pixelWidth, mode.pixelHeight)
    }
}

extension NSScreen {
    var displayID: CGDirectDisplayID? {
        (deviceDescription[NSDeviceDescriptionKey("NSScreenNumber")] as? NSNumber)?.uint32Value
    }
}

/// Borderless windows cannot take keyboard focus unless they opt in.
private final class KeyWindow: NSWindow {
    override var canBecomeKey: Bool { true }
    override var canBecomeMain: Bool { true }
}

/// A view backed by an opaque CAMetalLayer. Nothing is drawn over it, so the
/// window server can show the layer without compositing it.
final class MetalView: NSView {
    let metalLayer = CAMetalLayer()
    var onKey: ((NSEvent) -> Void)?

    init(device: MTLDevice) {
        super.init(frame: .zero)
        metalLayer.device = device
        metalLayer.pixelFormat = .bgra8Unorm
        metalLayer.colorspace = CGColorSpace(name: CGColorSpace.sRGB)
        metalLayer.framebufferOnly = true
        metalLayer.displaySyncEnabled = true
        metalLayer.isOpaque = true
        wantsLayer = true
    }

    @available(*, unavailable)
    required init?(coder: NSCoder) { fatalError("not used") }

    override func makeBackingLayer() -> CALayer { metalLayer }
    override var acceptsFirstResponder: Bool { true }
    override var isOpaque: Bool { true }
    override func keyDown(with event: NSEvent) { onKey?(event) }

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

/// Full screen on the glasses, or an ordinary window on the main screen while
/// they are not connected.
@MainActor final class GlassesWindow {
    let view: MetalView
    private let window: NSWindow
    private let displayLink: DisplayLinkThread

    init(device: MTLDevice, displayLink: DisplayLinkThread) {
        self.displayLink = displayLink
        view = MetalView(device: device)
        window = KeyWindow(
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

    func show() {
        place()
        window.makeKeyAndOrderFront(nil)
        window.makeFirstResponder(view)
    }

    /// Moves the window onto the glasses if they are connected.
    func place() {
        if let screen = Displays.glassesScreen() {
            window.styleMask = [.borderless]
            // Above the menu bar and Dock of that screen.
            window.level = .statusBar
            window.setFrame(screen.frame, display: true)
            window.orderFrontRegardless()
        } else {
            window.styleMask = [.titled, .closable, .resizable]
            window.level = .normal
            if let screen = NSScreen.main, !screen.frame.intersects(window.frame) || window.frame.width > screen.frame.width {
                window.setFrame(NSRect(x: 0, y: 0, width: 960, height: 540), display: true)
                window.center()
            }
        }
        // Recreated so the link runs at the refresh rate of this screen.
        displayLink.attach(to: view.metalLayer)
    }
}
