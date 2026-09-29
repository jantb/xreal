import Darwin
import Foundation
import IOKit
import IOKit.ps

/// The latest `capacity` values, oldest first, for a sparkline.
public struct History: Equatable, Sendable {
    public let capacity: Int
    public private(set) var values: [Float] = []

    public init(capacity: Int) {
        self.capacity = max(capacity, 1)
    }

    public mutating func append(_ value: Float) {
        if values.count >= capacity {
            values.removeFirst(values.count - capacity + 1)
        }
        values.append(value)
    }
}

extension CpuTicks {
    /// Each core's ticks now, or nil if the kernel does not say.
    public static func perCore() -> [CpuTicks]? {
        var count: natural_t = 0
        var info: processor_info_array_t?
        var infoCount: mach_msg_type_number_t = 0
        guard host_processor_info(mach_host_self(), PROCESSOR_CPU_LOAD_INFO, &count, &info, &infoCount) == KERN_SUCCESS,
            let info
        else { return nil }
        defer {
            vm_deallocate(
                mach_task_self_, vm_address_t(bitPattern: info), vm_size_t(Int(infoCount) * MemoryLayout<integer_t>.size))
        }
        let states = Int(CPU_STATE_MAX)
        return (0..<Int(count)).map { core in
            let ticks = { (state: Int32) in UInt64(UInt32(bitPattern: info[core * states + Int(state)])) }
            return CpuTicks(
                busy: ticks(CPU_STATE_USER) + ticks(CPU_STATE_SYSTEM) + ticks(CPU_STATE_NICE), idle: ticks(CPU_STATE_IDLE))
        }
    }
}

/// Bytes received and sent over every network interface but loopback,
/// since they came up.
public struct NetworkCounters: Equatable, Sendable {
    public var received: UInt64
    public var sent: UInt64

    public init(received: UInt64, sent: UInt64) {
        self.received = received
        self.sent = sent
    }

    public static func now() -> NetworkCounters? {
        var first: UnsafeMutablePointer<ifaddrs>?
        guard getifaddrs(&first) == 0, let first else { return nil }
        defer { freeifaddrs(first) }
        var counters = NetworkCounters(received: 0, sent: 0)
        var next: UnsafeMutablePointer<ifaddrs>? = first
        while let entry = next {
            defer { next = entry.pointee.ifa_next }
            let flags = Int32(entry.pointee.ifa_flags)
            guard let address = entry.pointee.ifa_addr, address.pointee.sa_family == UInt8(AF_LINK),
                flags & IFF_LOOPBACK == 0, flags & IFF_UP != 0,
                let data = entry.pointee.ifa_data?.assumingMemoryBound(to: if_data.self)
            else { continue }
            counters.received += UInt64(data.pointee.ifi_ibytes)
            counters.sent += UInt64(data.pointee.ifi_obytes)
        }
        return counters
    }

    /// Bytes per second down and up since `earlier`, `seconds` ago; nil
    /// when the counters went back, as when an interface came up again.
    public func rate(since earlier: NetworkCounters, seconds: Double) -> (down: Double, up: Double)? {
        guard seconds > 0, received >= earlier.received, sent >= earlier.sent else { return nil }
        return (Double(received - earlier.received) / seconds, Double(sent - earlier.sent) / seconds)
    }
}

/// CPU time used so far by each of the user's processes, which are the ones
/// macOS lets an app look at.
public struct ProcessTimes: Sendable {
    /// Nanoseconds of CPU time, by process ID.
    public var times: [Int32: UInt64]
    public var names: [Int32: String]

    public init(times: [Int32: UInt64], names: [Int32: String]) {
        self.times = times
        self.names = names
    }

    public static func now() -> ProcessTimes {
        var ids = [pid_t](repeating: 0, count: 4096)
        let count = Int(proc_listallpids(&ids, Int32(ids.count * MemoryLayout<pid_t>.size)))
        var timebase = mach_timebase_info_data_t()
        mach_timebase_info(&timebase)
        var times: [Int32: UInt64] = [:]
        var names: [Int32: String] = [:]
        for id in ids.prefix(max(count, 0)) where id > 0 {
            var usage = rusage_info_v2()
            let result = withUnsafeMutablePointer(to: &usage) { pointer in
                pointer.withMemoryRebound(to: rusage_info_t?.self, capacity: 1) {
                    proc_pid_rusage(id, RUSAGE_INFO_V2, $0)
                }
            }
            guard result == 0 else { continue }
            // In mach time units, not nanoseconds, on Apple silicon.
            let ticks = usage.ri_user_time + usage.ri_system_time
            times[id] = ticks * UInt64(timebase.numer) / UInt64(timebase.denom)
            var name = [UInt8](repeating: 0, count: 256)
            let length = Int(proc_name(id, &name, UInt32(name.count)))
            if length > 0 {
                names[id] = String(decoding: name.prefix(length), as: UTF8.self)
            }
        }
        return ProcessTimes(times: times, names: names)
    }

    /// The `count` processes that used the most CPU since `earlier`,
    /// `seconds` ago, busiest first, with their use as a share of one core
    /// (1 is a core flat out, as Activity Monitor's 100%). Processes of the
    /// same name, such as an app's helpers, count together.
    public func busiest(since earlier: ProcessTimes, seconds: Double, count: Int) -> [(name: String, share: Float)] {
        guard seconds > 0 else { return [] }
        var byName: [String: UInt64] = [:]
        for (id, time) in times {
            guard let before = earlier.times[id], time > before, let name = names[id] ?? earlier.names[id] else {
                continue
            }
            byName[name, default: 0] += time - before
        }
        return byName.map { (name: $0.key, share: Float(Double($0.value) / 1e9 / seconds)) }
            .sorted { $0.share != $1.share ? $0.share > $1.share : $0.name < $1.name }
            .prefix(count).map { $0 }
    }
}

/// How busy the GPU is, from 0 to 1, as its driver reports it.
public func gpuUtilization() -> Float? {
    var iterator: io_iterator_t = 0
    guard IOServiceGetMatchingServices(kIOMainPortDefault, IOServiceMatching("IOAccelerator"), &iterator) == KERN_SUCCESS
    else { return nil }
    defer { IOObjectRelease(iterator) }
    var busiest: Float?
    while case let service = IOIteratorNext(iterator), service != 0 {
        defer { IOObjectRelease(service) }
        guard
            let statistics = IORegistryEntryCreateCFProperty(
                service, "PerformanceStatistics" as CFString, kCFAllocatorDefault, 0)?.takeRetainedValue()
                as? [String: Any],
            let percent = statistics["Device Utilization %"] as? NSNumber
        else { continue }
        busiest = max(busiest ?? 0, percent.floatValue / 100)
    }
    return busiest
}

/// Space on the startup disk, in bytes.
public struct DiskSpace: Equatable, Sendable {
    public var free: UInt64
    public var total: UInt64

    public init(free: UInt64, total: UInt64) {
        self.free = free
        self.total = total
    }

    public static func now() -> DiskSpace? {
        let keys: Set<URLResourceKey> = [.volumeAvailableCapacityForImportantUsageKey, .volumeTotalCapacityKey]
        guard let values = try? URL(fileURLWithPath: "/").resourceValues(forKeys: keys),
            let free = values.volumeAvailableCapacityForImportantUsage, let total = values.volumeTotalCapacity,
            free >= 0, total > 0
        else { return nil }
        return DiskSpace(free: UInt64(free), total: UInt64(total))
    }
}

/// How hot the Mac runs, as macOS judges it.
public enum ThermalLevel: Int, Sendable {
    case nominal, fair, serious, critical

    public static func now() -> ThermalLevel {
        switch ProcessInfo.processInfo.thermalState {
        case .nominal: .nominal
        case .fair: .fair
        case .serious: .serious
        case .critical: .critical
        @unknown default: .fair
        }
    }
}

/// Bytes read from and written to the Mac's disks since they came up.
public struct DiskCounters: Equatable, Sendable {
    public var read: UInt64
    public var written: UInt64

    public init(read: UInt64, written: UInt64) {
        self.read = read
        self.written = written
    }

    public static func now() -> DiskCounters? {
        var iterator: io_iterator_t = 0
        guard
            IOServiceGetMatchingServices(kIOMainPortDefault, IOServiceMatching("IOBlockStorageDriver"), &iterator)
                == KERN_SUCCESS
        else { return nil }
        defer { IOObjectRelease(iterator) }
        var counters = DiskCounters(read: 0, written: 0)
        while case let service = IOIteratorNext(iterator), service != 0 {
            defer { IOObjectRelease(service) }
            guard
                let statistics = IORegistryEntryCreateCFProperty(
                    service, "Statistics" as CFString, kCFAllocatorDefault, 0)?.takeRetainedValue() as? [String: Any]
            else { continue }
            counters.read += (statistics["Bytes (Read)"] as? NSNumber)?.uint64Value ?? 0
            counters.written += (statistics["Bytes (Write)"] as? NSNumber)?.uint64Value ?? 0
        }
        return counters
    }

    /// Bytes per second read and written since `earlier`, `seconds` ago.
    public func rate(since earlier: DiskCounters, seconds: Double) -> (read: Double, write: Double)? {
        guard seconds > 0, read >= earlier.read, written >= earlier.written else { return nil }
        return (Double(read - earlier.read) / seconds, Double(written - earlier.written) / seconds)
    }
}

/// Power the Mac draws, and what its USB-C ports give out, such as to the
/// glasses, in watts, as a laptop's power controller reports them.
public struct PowerUse: Equatable, Sendable {
    public var system: Float
    public var usbOut: Float

    public init(system: Float, usbOut: Float) {
        self.system = system
        self.usbOut = usbOut
    }

    public static func now() -> PowerUse? {
        let service = IOServiceGetMatchingService(kIOMainPortDefault, IOServiceMatching("AppleSmartBattery"))
        guard service != 0 else { return nil }
        defer { IOObjectRelease(service) }
        func property(_ name: String) -> Any? {
            IORegistryEntryCreateCFProperty(service, name as CFString, kCFAllocatorDefault, 0)?.takeRetainedValue()
        }
        let telemetry = property("PowerTelemetryData") as? [String: Any]
        var system = (telemetry?["SystemLoad"] as? NSNumber).map { $0.floatValue / 1000 }
        if system == nil || system == 0,
            let volts = (property("Voltage") as? NSNumber)?.floatValue,
            let amps = (property("InstantAmperage") as? NSNumber)?.int64Value, amps < 0
        {
            // Running on the battery: what leaves it.
            system = volts / 1000 * Float(-amps) / 1000
        }
        guard let system, system > 0 else { return nil }
        let ports = property("PowerOutDetails") as? [[String: Any]] ?? []
        let out = ports.reduce(Float(0)) { $0 + (($1["Watts"] as? NSNumber)?.floatValue ?? 0) / 1000 }
        return PowerUse(system: system, usbOut: out)
    }
}

/// One look at the Mac, with how it went lately.
public struct SystemSample: Sendable {
    public var cpu: Float?
    /// Each core's share busy, 0 to 1, in the kernel's order: efficiency
    /// cores last on Apple silicon.
    public var cores: [Float] = []
    public var cpuHistory: History
    public var memory: MemoryUse?
    public var memoryHistory: History
    public var gpu: Float?
    public var gpuHistory: History
    /// Bytes per second.
    public var network: (down: Double, up: Double)?
    public var downHistory: History
    public var upHistory: History
    /// Bytes per second.
    public var diskTraffic: (read: Double, write: Double)?
    public var readHistory: History
    public var writeHistory: History
    public var power: PowerUse?
    public var powerHistory: History
    public var battery: Battery?
    public var disk: DiskSpace?
    public var thermal: ThermalLevel = .nominal
    /// The user's busiest processes, as shares of one core.
    public var busiest: [(name: String, share: Float)] = []
}

// How long each sparkline reaches back, in samples.
private let historyLength = 300
// Readings that change quickly are smoothed over about this long, so the
// numbers can be read while they update many times a second.
private let smoothingTime: Double = 0.4  // seconds
// Readings that change slowly, or cost more to take, such as ones that ask
// a driver, are taken this often.
private let slowInterval: Double = 1  // seconds

/// Looks at the Mac, as often as asked, and keeps what it saw for the
/// sparklines: the quick readings every time, smoothed, and the slow ones
/// about once a second.
public struct SystemMonitor: Sendable {
    private var previous: (at: Double, cores: [CpuTicks]?, network: NetworkCounters?, disk: DiskCounters?)?
    private var slowAt: (at: Double, processes: ProcessTimes)?
    private var sample = SystemSample(
        cpuHistory: History(capacity: historyLength), memoryHistory: History(capacity: historyLength),
        gpuHistory: History(capacity: historyLength), downHistory: History(capacity: historyLength),
        upHistory: History(capacity: historyLength), readHistory: History(capacity: historyLength),
        writeHistory: History(capacity: historyLength), powerHistory: History(capacity: historyLength))

    public init() {}

    public mutating func sample(now: Double) -> SystemSample {
        let cores = CpuTicks.perCore()
        let network = NetworkCounters.now()
        let disk = DiskCounters.now()
        if let previous {
            let seconds = now - previous.at
            let blend = Float(seconds / (smoothingTime + seconds))
            func smooth(_ old: Float?, _ new: Float) -> Float { old.map { $0 + (new - $0) * blend } ?? new }
            func smooth(_ old: Double?, _ new: Double) -> Double { Double(smooth(old.map(Float.init), Float(new))) }
            if let cores, let before = previous.cores, before.count == cores.count {
                let loads = zip(cores, before).map { $0.load(since: $1) ?? 0 }
                sample.cores =
                    sample.cores.count == loads.count ? zip(sample.cores, loads).map { smooth($0, $1) } : loads
                let total = Self.sum(cores).load(since: Self.sum(before))
                sample.cpu = total.map { smooth(sample.cpu, $0) }
            }
            if let network, let before = previous.network, let rate = network.rate(since: before, seconds: seconds) {
                sample.network = (smooth(sample.network?.down, rate.down), smooth(sample.network?.up, rate.up))
            }
            if let disk, let before = previous.disk, let rate = disk.rate(since: before, seconds: seconds) {
                sample.diskTraffic = (
                    smooth(sample.diskTraffic?.read, rate.read), smooth(sample.diskTraffic?.write, rate.write)
                )
            }
        }
        sample.memory = MemoryUse.now()
        previous = (now, cores, network, disk)

        if slowAt.map({ now - $0.at >= slowInterval }) ?? true {
            let processes = ProcessTimes.now()
            if let slowAt {
                sample.busiest = processes.busiest(since: slowAt.processes, seconds: now - slowAt.at, count: 3)
            }
            slowAt = (now, processes)
            sample.battery = Battery.now()
            sample.disk = DiskSpace.now()
            sample.thermal = ThermalLevel.now()
            // Asking the GPU's driver and the power controller costs more
            // than it seems: the driver's statistics briefly hold up GPU
            // work, which read ten times a second made the glasses drop
            // frames.
            sample.gpu = gpuUtilization()
            sample.power = PowerUse.now()
        }

        if let cpu = sample.cpu { sample.cpuHistory.append(cpu) }
        if let memory = sample.memory, memory.total > 0 {
            sample.memoryHistory.append(Float(memory.used) / Float(memory.total))
        }
        if let gpu = sample.gpu { sample.gpuHistory.append(gpu) }
        if let network = sample.network {
            sample.downHistory.append(Float(network.down))
            sample.upHistory.append(Float(network.up))
        }
        if let traffic = sample.diskTraffic {
            sample.readHistory.append(Float(traffic.read))
            sample.writeHistory.append(Float(traffic.write))
        }
        if let power = sample.power { sample.powerHistory.append(power.system) }
        return sample
    }

    private static func sum(_ ticks: [CpuTicks]) -> CpuTicks {
        ticks.reduce(CpuTicks(busy: 0, idle: 0)) { CpuTicks(busy: $0.busy + $1.busy, idle: $0.idle + $1.idle) }
    }
}
