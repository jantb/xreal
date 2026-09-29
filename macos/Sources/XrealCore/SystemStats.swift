import Darwin
import Foundation
import IOKit.ps

/// CPU time spent since boot, across all cores, in scheduler ticks.
public struct CpuTicks: Equatable, Sendable {
    public var busy: UInt64
    public var idle: UInt64

    public init(busy: UInt64, idle: UInt64) {
        self.busy = busy
        self.idle = idle
    }

    /// The CPU's ticks now, or nil if the kernel does not say.
    public static func now() -> CpuTicks? {
        var load = host_cpu_load_info_data_t()
        var count = mach_msg_type_number_t(
            MemoryLayout<host_cpu_load_info_data_t>.size / MemoryLayout<integer_t>.size)
        let result = withUnsafeMutablePointer(to: &load) { pointer in
            pointer.withMemoryRebound(to: integer_t.self, capacity: Int(count)) {
                host_statistics(mach_host_self(), HOST_CPU_LOAD_INFO, $0, &count)
            }
        }
        guard result == KERN_SUCCESS else { return nil }
        let ticks = load.cpu_ticks
        return CpuTicks(
            busy: UInt64(ticks.0) + UInt64(ticks.1) + UInt64(ticks.3),  // user, system, nice
            idle: UInt64(ticks.2))
    }

    /// The share of the CPU that was busy between `earlier` and these
    /// ticks, from 0 to 1; nil when no time passed.
    public func load(since earlier: CpuTicks) -> Float? {
        guard busy >= earlier.busy, idle >= earlier.idle else { return nil }
        let (busyTicks, idleTicks) = (busy - earlier.busy, idle - earlier.idle)
        guard busyTicks + idleTicks > 0 else { return nil }
        return Float(busyTicks) / Float(busyTicks + idleTicks)
    }
}

/// Memory in use, as Activity Monitor counts it: apps' own memory, wired
/// and compressed.
public struct MemoryUse: Equatable, Sendable {
    public var used: UInt64
    public var total: UInt64

    public init(used: UInt64, total: UInt64) {
        self.used = used
        self.total = total
    }

    public static func now() -> MemoryUse? {
        var stats = vm_statistics64_data_t()
        var count = mach_msg_type_number_t(
            MemoryLayout<vm_statistics64_data_t>.size / MemoryLayout<integer_t>.size)
        let result = withUnsafeMutablePointer(to: &stats) { pointer in
            pointer.withMemoryRebound(to: integer_t.self, capacity: Int(count)) {
                host_statistics64(mach_host_self(), HOST_VM_INFO64, $0, &count)
            }
        }
        var pageSize: vm_size_t = 0
        guard result == KERN_SUCCESS, host_page_size(mach_host_self(), &pageSize) == KERN_SUCCESS else { return nil }
        let appPages = UInt64(stats.internal_page_count) - min(UInt64(stats.purgeable_count), UInt64(stats.internal_page_count))
        let pages = appPages + UInt64(stats.wire_count) + UInt64(stats.compressor_page_count)
        return MemoryUse(used: pages * UInt64(pageSize), total: ProcessInfo.processInfo.physicalMemory)
    }
}

/// The Mac's battery, when it has one.
public struct Battery: Equatable, Sendable {
    /// From 0 to 100.
    public var percent: Int
    /// On the charger, charging or full.
    public var charging: Bool
    /// Minutes until empty, or until full while charging, when macOS knows.
    public var minutesLeft: Int?

    public init(percent: Int, charging: Bool, minutesLeft: Int? = nil) {
        self.percent = percent
        self.charging = charging
        self.minutesLeft = minutesLeft
    }

    public static func now() -> Battery? {
        guard let info = IOPSCopyPowerSourcesInfo()?.takeRetainedValue(),
            let sources = IOPSCopyPowerSourcesList(info)?.takeRetainedValue() as? [CFTypeRef]
        else { return nil }
        for source in sources {
            guard
                let description = IOPSGetPowerSourceDescription(info, source)?.takeUnretainedValue()
                    as? [String: Any],
                description[kIOPSTypeKey] as? String == kIOPSInternalBatteryType,
                let capacity = description[kIOPSCurrentCapacityKey] as? Int,
                let maximum = description[kIOPSMaxCapacityKey] as? Int, maximum > 0
            else { continue }
            let onCharger = description[kIOPSPowerSourceStateKey] as? String == kIOPSACPowerValue
            let minutes =
                onCharger
                ? description[kIOPSTimeToFullChargeKey] as? Int
                : description[kIOPSTimeToEmptyKey] as? Int
            return Battery(
                percent: min(max(capacity * 100 / maximum, 0), 100), charging: onCharger,
                minutesLeft: minutes.flatMap { $0 > 0 ? $0 : nil })
        }
        return nil
    }
}
