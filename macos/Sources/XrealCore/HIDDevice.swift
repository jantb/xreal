import Foundation
import IOKit.hid

/// A HID interface with blocking, timed reads. Input reports are delivered
/// through the run loop of the thread that opened the device, so every
/// method must be called from that thread.
final class HIDDevice {
    // hidapi also keeps about this many unread reports before dropping old ones.
    private static let maxQueuedReports = 30

    private let device: IOHIDDevice
    private let runLoop: CFRunLoop
    private let reportBuffer: UnsafeMutablePointer<UInt8>
    private let reportBufferSize: Int
    private var reports: [[UInt8]] = []
    private var removed = false

    /// All HID interfaces with the given USB vendor ID.
    static func all(vendorID: Int) -> [IOHIDDevice] {
        let manager = IOHIDManagerCreate(kCFAllocatorDefault, IOOptionBits(kIOHIDOptionsTypeNone))
        IOHIDManagerSetDeviceMatching(manager, [kIOHIDVendorIDKey: vendorID] as CFDictionary)
        guard let devices = IOHIDManagerCopyDevices(manager) as NSSet? else { return [] }
        return devices.allObjects.map { $0 as! IOHIDDevice }
    }

    static func productID(of device: IOHIDDevice) -> Int? {
        IOHIDDeviceGetProperty(device, kIOHIDProductIDKey as CFString) as? Int
    }

    /// The USB interface number. Since macOS 11 it lives on the parent USB
    /// interface rather than on the HID device itself.
    static func interfaceNumber(of device: IOHIDDevice) -> Int? {
        if let number = IOHIDDeviceGetProperty(device, "bInterfaceNumber" as CFString) as? Int {
            return number
        }
        let found = IORegistryEntrySearchCFProperty(
            IOHIDDeviceGetService(device), kIOServicePlane, "bInterfaceNumber" as CFString,
            kCFAllocatorDefault, IOOptionBits(kIORegistryIterateRecursively | kIORegistryIterateParents))
        return found as? Int
    }

    init(_ device: IOHIDDevice) throws {
        let result = IOHIDDeviceOpen(device, IOOptionBits(kIOHIDOptionsTypeNone))
        guard result == kIOReturnSuccess else { throw GlassesError.io(result) }

        self.device = device
        runLoop = CFRunLoopGetCurrent()
        let maxReportSize = IOHIDDeviceGetProperty(device, kIOHIDMaxInputReportSizeKey as CFString) as? Int
        reportBufferSize = max(maxReportSize ?? 0, 64)
        reportBuffer = .allocate(capacity: reportBufferSize)

        let context = Unmanaged.passUnretained(self).toOpaque()
        IOHIDDeviceRegisterInputReportCallback(
            device, reportBuffer, reportBufferSize,
            { context, _, _, _, _, report, length in
                guard let context else { return }
                let this = Unmanaged<HIDDevice>.fromOpaque(context).takeUnretainedValue()
                this.enqueue(Array(UnsafeBufferPointer(start: report, count: length)))
            }, context)
        IOHIDDeviceRegisterRemovalCallback(
            device,
            { context, _, _ in
                guard let context else { return }
                Unmanaged<HIDDevice>.fromOpaque(context).takeUnretainedValue().removed = true
            }, context)
        IOHIDDeviceScheduleWithRunLoop(device, runLoop, CFRunLoopMode.defaultMode.rawValue)
    }

    deinit {
        IOHIDDeviceRegisterInputReportCallback(device, reportBuffer, reportBufferSize, nil, nil)
        IOHIDDeviceRegisterRemovalCallback(device, nil, nil)
        IOHIDDeviceUnscheduleFromRunLoop(device, runLoop, CFRunLoopMode.defaultMode.rawValue)
        IOHIDDeviceClose(device, IOOptionBits(kIOHIDOptionsTypeNone))
        reportBuffer.deallocate()
    }

    /// Sends an output report. As with hidapi, the first byte doubles as the
    /// report ID.
    func write(_ bytes: [UInt8]) throws {
        if removed { throw GlassesError.deviceGone }
        let result = IOHIDDeviceSetReport(
            device, kIOHIDReportTypeOutput, CFIndex(bytes[0]), bytes, bytes.count)
        guard result == kIOReturnSuccess else { throw GlassesError.io(result) }
    }

    /// Returns the oldest unread report padded or cut to `size` bytes, or nil
    /// if none arrives within `timeout` seconds. A zero timeout only picks up
    /// reports that are already waiting.
    func read(size: Int, timeout: Double) throws -> [UInt8]? {
        let deadline = monotonicNow() + timeout
        var pumped = false
        while true {
            if !reports.isEmpty {
                var report = reports.removeFirst()
                if report.count < size {
                    report += [UInt8](repeating: 0, count: size - report.count)
                }
                return Array(report.prefix(size))
            }
            if removed { throw GlassesError.deviceGone }
            let remaining = deadline - monotonicNow()
            if pumped && remaining <= 0 { return nil }
            CFRunLoopRunInMode(.defaultMode, max(remaining, 0), true)
            pumped = true
        }
    }

    private func enqueue(_ report: [UInt8]) {
        if reports.count >= Self.maxQueuedReports {
            reports.removeFirst()
        }
        reports.append(report)
    }
}
