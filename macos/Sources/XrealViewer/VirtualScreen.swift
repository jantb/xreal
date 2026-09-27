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

    /// Returns nil if the WindowServer refuses to create the display.
    init?(width: Int, height: Int) {
        let descriptor = CGVirtualDisplayDescriptor()
        descriptor.queue = DispatchQueue(label: "xreal.virtual-screen")
        descriptor.name = "XREAL Virtual Screen"
        descriptor.maxPixelsWide = UInt32(width)
        descriptor.maxPixelsHigh = UInt32(height)
        // A desktop monitor's pixel density, so macOS keeps 1x UI scaling.
        let millimetersPerPixel = 25.4 / 110
        descriptor.sizeInMillimeters = CGSize(
            width: Double(width) * millimetersPerPixel, height: Double(height) * millimetersPerPixel)
        // macOS remembers arrangement per vendor/product/serial, so keep the
        // identity stable for a given size.
        descriptor.vendorID = 0x5852  // "XR"
        descriptor.productID = 0x5653  // "VS"
        descriptor.serialNum = UInt32(width) << 16 | UInt32(height)
        descriptor.terminationHandler = { _, _ in eprint("macOS removed the virtual screen") }

        guard let display = CGVirtualDisplay(descriptor: descriptor) else { return nil }
        let settings = CGVirtualDisplaySettings()
        settings.hiDPI = 0
        settings.modes = [CGVirtualDisplayMode(width: UInt(width), height: UInt(height), refreshRate: 120)]
        guard display.apply(settings) else { return nil }

        self.display = display
        self.width = width
        self.height = height
    }
}
