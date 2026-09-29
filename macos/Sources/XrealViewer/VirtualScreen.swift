import CVirtualDisplay
import Foundation
import XrealCore

/// An extra display that exists only inside the glasses. macOS treats it as a
/// real monitor: windows can be moved onto it and it shows up in System
/// Settings > Displays. It disappears when this object is released.
@MainActor final class VirtualScreen {
    /// The size in points, which is what windows are laid out in.
    let width: Int
    let height: Int
    /// Pixels per point: 2 for a HiDPI screen, drawn at twice the detail.
    let scale: Int
    let refreshRate: Double
    private let display: CGVirtualDisplay

    var displayID: CGDirectDisplayID { display.displayID }

    /// `index` tells screens apart, such as the canvas and the ones
    /// `--probe-sizes` creates. `width` and `height` are in points, and
    /// `scale` is 2 for a HiDPI screen with twice as many pixels each way.
    /// `refreshRate` is how often macOS draws it. Returns nil if the
    /// WindowServer refuses to create the display.
    init?(index: Int, width: Int, height: Int, scale: Int = 1, refreshRate: Double) {
        // Much larger displays crash the WindowServer, logging the user out.
        guard RoomScreen.isAllowed(width: width, height: height, scale: scale) else { return nil }
        let (pixelsWide, pixelsHigh) = (width * scale, height * scale)
        let descriptor = CGVirtualDisplayDescriptor()
        descriptor.queue = DispatchQueue(label: "xreal.virtual-screen")
        descriptor.name = "XREAL Virtual Screen \(index + 1)"
        descriptor.maxPixelsWide = UInt32(pixelsWide)
        descriptor.maxPixelsHigh = UInt32(pixelsHigh)
        // A desktop monitor's size for its points, so macOS keeps the UI at
        // its usual size: 1x at 110 pixels per inch, 2x at 220.
        let millimetersPerPoint = 25.4 / 110
        descriptor.sizeInMillimeters = CGSize(
            width: Double(width) * millimetersPerPoint, height: Double(height) * millimetersPerPoint)
        // macOS remembers settings per vendor/product/serial, so keep the
        // identity stable for a given position in the list and size.
        descriptor.vendorID = Displays.virtualScreenVendor
        descriptor.productID = 0x5653  // "VS"
        descriptor.serialNum =
            UInt32(scale == 2 ? 1 : 0) << 31 | UInt32(index & 0x1f) << 26 | UInt32(width & 0x1fff) << 13
            | UInt32(height & 0x1fff)
        descriptor.terminationHandler = { _, _ in eprint("macOS removed the virtual screen") }

        guard let display = CGVirtualDisplay(descriptor: descriptor) else { return nil }
        let settings = CGVirtualDisplaySettings()
        settings.hiDPI = scale == 2 ? 1 : 0
        settings.modes = [CGVirtualDisplayMode(width: UInt(width), height: UInt(height), refreshRate: refreshRate)]
        guard display.apply(settings) else { return nil }

        self.display = display
        self.width = width
        self.height = height
        self.scale = scale
        self.refreshRate = refreshRate
    }
}
