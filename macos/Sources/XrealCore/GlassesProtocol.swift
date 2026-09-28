// Wire format of XREAL Air glasses, after ar-drivers-rs by Alex Badics (MIT)
// and https://gitlab.com/TheJackiMonster/nrealAirLinuxDriver.

import Foundation

public enum GlassesError: Error, CustomStringConvertible {
    case notFound
    /// The device was unplugged or stopped answering at the USB level.
    case deviceGone
    case io(Int32)
    case timeout
    case malformed(String)

    public var description: String {
        switch self {
        case .notFound: "glasses not found"
        case .deviceGone: "device gone"
        case .io(let code): String(format: "I/O error 0x%08x", code)
        case .timeout: "packet timeout"
        case .malformed(let what): what
        }
    }
}

public enum AirModel: Sendable {
    case air, air2, air2Pro

    public init?(productID: Int) {
        switch productID {
        case 0x0424: self = .air
        case 0x0428: self = .air2
        case 0x0432: self = .air2Pro
        default: return nil
        }
    }

    public var name: String {
        switch self {
        case .air: "XREAL Air"
        case .air2: "XREAL Air 2"
        case .air2Pro: "XREAL Air 2 Pro"
        }
    }
}

public enum DisplayMode: UInt8, Sendable {
    /// Same picture on both eyes at 60 Hz.
    case sameOnBoth = 1
    case stereo = 3
    case halfSBS = 8
    case highRefreshRateSBS = 9
    /// Same picture on both eyes at 120 Hz.
    case highRefreshRate = 11
}

/// CRC-32 (IEEE 802.3), as the glasses' firmware computes it.
func crc32(_ bytes: some Sequence<UInt8>) -> UInt32 {
    var crc: UInt32 = 0xffff_ffff
    for byte in bytes {
        crc = (crc >> 8) ^ crc32Table[Int(UInt8(truncatingIfNeeded: crc) ^ byte)]
    }
    return crc ^ 0xffff_ffff
}

private let crc32Table: [UInt32] = (0..<256).map { index in
    var value = UInt32(index)
    for _ in 0..<8 {
        value = value & 1 == 1 ? (value >> 1) ^ 0xedb8_8320 : value >> 1
    }
    return value
}

/// Commands and events of the MCU interface (display, buttons, serial).
///
/// Layout, little endian: head 0xfd, checksum u32, length u16, request id
/// u32, timestamp u32, command u16, 5 reserved bytes, up to 42 data bytes.
struct McuPacket: Equatable {
    static let size = 0x40
    static let maxData = 42
    private static let headerLength = 17

    var command: UInt16
    var data: [UInt8] = []

    static func deserialize(_ bytes: [UInt8]) -> McuPacket? {
        guard bytes.count >= size, bytes[0] == 0xfd else { return nil }
        let dataLength = Int(bytes.readUInt16(at: 5)) - headerLength
        guard (0...maxData).contains(dataLength) else { return nil }
        return McuPacket(command: bytes.readUInt16(at: 15), data: Array(bytes[22..<22 + dataLength]))
    }

    func serialize() -> [UInt8]? {
        guard data.count <= Self.maxData else { return nil }
        var bytes = [UInt8](repeating: 0, count: Self.size)
        let length = data.count + Self.headerLength
        bytes[0] = 0xfd
        bytes.write(UInt16(length), at: 5)
        bytes.write(UInt32(0x1337), at: 7)
        bytes.write(command, at: 15)
        bytes.replaceSubrange(22..<22 + data.count, with: data)
        bytes.write(crc32(bytes[5..<5 + length]), at: 1)
        return bytes
    }
}

/// Commands to the IMU interface.
///
/// Layout, little endian: head 0xaa, checksum u32, length u16, command u8,
/// up to 56 data bytes.
struct ImuPacket: Equatable {
    static let size = 0x40
    static let maxData = 56
    private static let headerLength = 3

    var command: UInt8
    var data: [UInt8] = []

    static func deserialize(_ bytes: [UInt8]) -> ImuPacket? {
        guard bytes.count >= size, bytes[0] == 0xaa else { return nil }
        let dataLength = Int(bytes.readUInt16(at: 5)) - headerLength
        guard (0...maxData).contains(dataLength) else { return nil }
        return ImuPacket(command: bytes[7], data: Array(bytes[8..<8 + dataLength]))
    }

    func serialize() -> [UInt8]? {
        guard data.count <= Self.maxData else { return nil }
        var bytes = [UInt8](repeating: 0, count: Self.size)
        let length = data.count + Self.headerLength
        bytes[0] = 0xaa
        bytes.write(UInt16(length), at: 5)
        bytes[7] = command
        bytes.replaceSubrange(8..<8 + data.count, with: data)
        bytes.write(crc32(bytes[5..<5 + length]), at: 1)
        return bytes
    }
}

/// Factory calibration offsets stored on the glasses.
struct ImuBiases: Equatable {
    var gyro = SIMD3<Float>.zero
    var accelerometer = SIMD3<Float>.zero

    /// Reads `IMU.device_1.{gyro,accel}_bias` from the glasses' JSON config.
    static func parse(config: Data) throws -> ImuBiases {
        guard
            let root = try? JSONSerialization.jsonObject(with: config) as? [String: Any],
            let imu = root["IMU"] as? [String: Any],
            let device = imu["device_1"] as? [String: Any]
        else {
            throw GlassesError.malformed("invalid glasses config format")
        }
        return ImuBiases(
            gyro: try vector(device["gyro_bias"]),
            accelerometer: try vector(device["accel_bias"]))
    }

    private static func vector(_ value: Any?) throws -> SIMD3<Float> {
        guard let values = value as? [NSNumber], values.count >= 3 else {
            throw GlassesError.malformed("invalid glasses config vector")
        }
        return SIMD3(values[0].floatValue, values[1].floatValue, values[2].floatValue)
    }
}

public struct ImuSample: Equatable, Sendable {
    /// m/s², reads (0, 9.81, 0) when upright. Axes: X right, Y up, Z back.
    public var accelerometer: SIMD3<Float>
    /// Right-handed rad/s; turning left is positive Y.
    public var gyroscope: SIMD3<Float>
    /// Device time in microseconds.
    public var timestamp: UInt64
    /// The IMU chip's temperature in °C, nil when the reading is implausible.
    public var temperature: Float? = nil
}

// The IMU reports its die temperature as a signed 16-bit count. The scale
// is the TDK ICM-42688's (count / 132.48 + 25 °C); the drift model only
// relies on it rising and falling with the real temperature.
private let temperatureCountsPerDegree: Float = 132.48
private let temperatureAtZeroCount: Float = 25
private let plausibleTemperatures: ClosedRange<Float> = -20...100

/// Decodes an IMU stream report (`01 02 ...`). Returns nil for other reports.
func parseImuReport(_ bytes: [UInt8], biases: ImuBiases) -> ImuSample? {
    guard bytes.count >= 42, bytes[0] == 1, bytes[1] == 2 else { return nil }
    let temperatureCount = Float(Int16(bitPattern: bytes.readUInt16(at: 2)))
    let temperature = temperatureCount / temperatureCountsPerDegree + temperatureAtZeroCount
    let timestamp = bytes.readUInt64(at: 4) / 1000

    let gyroScale = Float(bytes.readUInt16(at: 12))
    let gyroDivisor = Float(bytes.readUInt32(at: 14))
    let gyroRaw = SIMD3<Float>(
        Float(bytes.readInt24(at: 18)), Float(bytes.readInt24(at: 21)), Float(bytes.readInt24(at: 24)))
    let gyroDegrees = gyroRaw * gyroScale / gyroDivisor

    let accScale = Float(bytes.readUInt16(at: 27))
    let accDivisor = Float(bytes.readUInt32(at: 29))
    let accRaw = SIMD3<Float>(
        Float(bytes.readInt24(at: 33)), Float(bytes.readInt24(at: 36)), Float(bytes.readInt24(at: 39)))
    let accG = accRaw * accScale / accDivisor

    // The bias fields do not correspond to the raw axes, but this pairing is
    // what reads as zero on still glasses.
    let gyro = SIMD3<Float>(
        gyroDegrees.x * .pi / 180 + biases.gyro.x,
        gyroDegrees.y * .pi / 180 + biases.gyro.z,
        gyroDegrees.z * .pi / 180 + biases.gyro.y)
    let accelerometer = SIMD3<Float>(
        accG.x * 9.81 + biases.accelerometer.x,
        accG.y * 9.81 + biases.accelerometer.z,
        accG.z * 9.81 + biases.accelerometer.y)
    return ImuSample(
        accelerometer: accelerometer, gyroscope: gyro, timestamp: timestamp,
        temperature: plausibleTemperatures.contains(temperature) ? temperature : nil)
}

extension [UInt8] {
    func readUInt16(at offset: Int) -> UInt16 {
        UInt16(self[offset]) | UInt16(self[offset + 1]) << 8
    }

    func readUInt32(at offset: Int) -> UInt32 {
        (0..<4).reduce(0) { $0 | UInt32(self[offset + $1]) << (8 * $1) }
    }

    func readUInt64(at offset: Int) -> UInt64 {
        (0..<8).reduce(0) { $0 | UInt64(self[offset + $1]) << (8 * $1) }
    }

    func readInt24(at offset: Int) -> Int32 {
        let raw = UInt32(self[offset]) | UInt32(self[offset + 1]) << 8 | UInt32(self[offset + 2]) << 16
        // Sign-extend from 24 bits.
        return Int32(bitPattern: raw << 8) >> 8
    }

    mutating func write<T: FixedWidthInteger>(_ value: T, at offset: Int) {
        for index in 0..<MemoryLayout<T>.size {
            self[offset + index] = UInt8(truncatingIfNeeded: value >> (8 * index))
        }
    }
}
