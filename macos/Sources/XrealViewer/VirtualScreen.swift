import CVirtualDisplay
import Foundation
import XrealCore

/// An extra display that exists only inside the glasses. macOS treats it as a
/// real monitor: windows can be moved onto it and it shows up in System
/// Settings > Displays. It disappears when this object is released.
@MainActor final class VirtualScreen {
    let width: Int
    let height: Int
    private let display: CGVirtualDisplay

    var displayID: CGDirectDisplayID { display.displayID }

    /// `index` tells screens apart, such as the canvas and the ones
    /// `--probe-sizes` creates. `refreshRate` matches the glasses, so macOS
    /// draws the screen in step with them. Returns nil if the WindowServer
    /// refuses to create the display.
    init?(index: Int, width: Int, height: Int, refreshRate: Double) {
        // Much larger displays crash the WindowServer, logging the user out.
        guard (1...maxVirtualScreenSide).contains(width), (1...maxVirtualScreenSide).contains(height) else {
            return nil
        }
        let descriptor = CGVirtualDisplayDescriptor()
        descriptor.queue = DispatchQueue(label: "xreal.virtual-screen")
        descriptor.name = "XREAL Virtual Screen \(index + 1)"
        descriptor.maxPixelsWide = UInt32(width)
        descriptor.maxPixelsHigh = UInt32(height)
        // A desktop monitor's pixel density, so macOS keeps 1x UI scaling.
        let millimetersPerPixel = 25.4 / 110
        descriptor.sizeInMillimeters = CGSize(
            width: Double(width) * millimetersPerPixel, height: Double(height) * millimetersPerPixel)
        // macOS remembers settings per vendor/product/serial, so keep the
        // identity stable for a given position in the list and size.
        descriptor.vendorID = Displays.virtualScreenVendor
        descriptor.productID = 0x5653  // "VS"
        descriptor.serialNum = UInt32(index & 0x3f) << 26 | UInt32(width & 0x1fff) << 13 | UInt32(height & 0x1fff)
        descriptor.terminationHandler = { _, _ in eprint("macOS removed the virtual screen") }

        guard let display = CGVirtualDisplay(descriptor: descriptor) else { return nil }
        let settings = CGVirtualDisplaySettings()
        settings.hiDPI = 0
        settings.modes = [CGVirtualDisplayMode(width: UInt(width), height: UInt(height), refreshRate: refreshRate)]
        guard display.apply(settings) else { return nil }

        self.display = display
        self.width = width
        self.height = height
    }
}
