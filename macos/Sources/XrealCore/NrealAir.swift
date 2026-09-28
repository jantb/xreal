// Driver for XREAL Air glasses, ported from ar-drivers-rs by Alex Badics (MIT).

import Foundation
import IOKit.hid

public enum GlassesEvent: Sendable {
    case accGyro(ImuSample)
    /// A button was pressed; IDs start at 0.
    case keyPress(UInt8)
}

/// A connected pair of XREAL Air glasses. They expose two HID interfaces: the
/// MCU (display mode, buttons, serial number) and the IMU sample stream.
/// Must be used from the thread that created it.
public final class NrealAir {
    static let vendorID = 0x3318
    private static let mcuInterface = 4
    private static let imuInterface = 3
    private static let commandTimeout = 1.0
    private static let imuTimeout = 0.25

    public let model: AirModel
    private let mcu: HIDDevice
    private let imu: HIDDevice
    private var pendingPackets: [McuPacket] = []
    private var biases = ImuBiases()
    /// The glasses' factory calibration, as JSON.
    public private(set) var config = Data()

    public init() throws {
        let interfaces = HIDDevice.all(vendorID: Self.vendorID)
        func open(interface: Int) throws -> (AirModel, HIDDevice) {
            for device in interfaces where HIDDevice.interfaceNumber(of: device) == interface {
                if let model = HIDDevice.productID(of: device).flatMap(AirModel.init(productID:)) {
                    return (model, try HIDDevice(device))
                }
            }
            throw GlassesError.notFound
        }
        (model, mcu) = try open(interface: Self.mcuInterface)
        (_, imu) = try open(interface: Self.imuInterface)

        // The IMU stream is paused while the config is read.
        _ = try imuCommand(0x19, [0])
        config = try readConfig()
        biases = try ImuBiases.parse(config: config)
        _ = try imuCommand(0x19, [1])
        // Quick check that the MCU answers.
        _ = try serial()
    }

    public var name: String { model.name }

    public func serial() throws -> String {
        var result = try runCommand(McuPacket(command: 0x15))
        guard !result.isEmpty else { throw GlassesError.malformed("empty serial number") }
        result.removeFirst()
        guard let serial = String(bytes: result, encoding: .utf8) else {
            throw GlassesError.malformed("serial number is not UTF-8")
        }
        return serial
    }

    public func setDisplayMode(_ mode: DisplayMode) throws {
        let result = try runCommand(McuPacket(command: 0x08, data: [mode.rawValue]))
        guard result.first == 0 else { throw GlassesError.malformed("display mode not accepted") }
    }

    /// Blocks for the next event, for at most about 250 ms.
    public func readEvent() throws -> GlassesEvent {
        if let event = try readMcuEvent() {
            return event
        }
        while true {
            guard let report = try imu.read(size: 0x80, timeout: Self.imuTimeout) else {
                throw GlassesError.timeout
            }
            if let sample = parseImuReport(report, biases: biases) {
                return .accGyro(sample)
            }
        }
    }

    private func readMcuEvent() throws -> GlassesEvent? {
        let packet: McuPacket
        if !pendingPackets.isEmpty {
            packet = pendingPackets.removeFirst()
        } else if let report = try mcu.read(size: McuPacket.size, timeout: 0) {
            guard let parsed = McuPacket.deserialize(report) else {
                throw GlassesError.malformed("malformed MCU packet")
            }
            packet = parsed
        } else {
            return nil
        }
        switch packet.command {
        case 0x6c05:
            guard let key = packet.data.first, key > 0 else {
                throw GlassesError.malformed("malformed key press packet")
            }
            return .keyPress(key - 1)
        default:
            // 0x6c09 carries error text from the glasses; nothing to act on.
            return nil
        }
    }

    private func runCommand(_ command: McuPacket) throws -> [UInt8] {
        guard let bytes = command.serialize() else { throw GlassesError.malformed("command too long") }
        try mcu.write(bytes)
        for _ in 0..<64 {
            guard let report = try mcu.read(size: McuPacket.size, timeout: Self.commandTimeout) else {
                throw GlassesError.timeout
            }
            guard let packet = McuPacket.deserialize(report) else {
                throw GlassesError.malformed("malformed MCU packet")
            }
            if packet.command == command.command {
                return packet.data
            }
            pendingPackets.append(packet)
        }
        throw GlassesError.malformed("too many unrelated packets")
    }

    private func imuCommand(_ command: UInt8, _ data: [UInt8]) throws -> [UInt8] {
        guard let bytes = ImuPacket(command: command, data: data).serialize() else {
            throw GlassesError.malformed("command too long")
        }
        try imu.write(bytes)
        for _ in 0..<64 {
            guard let report = try imu.read(size: ImuPacket.size, timeout: Self.imuTimeout) else {
                throw GlassesError.timeout
            }
            if let packet = ImuPacket.deserialize(report) {
                return packet.data
            }
        }
        throw GlassesError.malformed("no acknowledgement to IMU command")
    }

    private func readConfig() throws -> Data {
        let lengthBytes = try imuCommand(0x14, [])
        guard lengthBytes.count == 4 else { throw GlassesError.malformed("invalid config length") }
        let length = Int(lengthBytes.readUInt32(at: 0))
        var config: [UInt8] = []
        while config.count < length {
            let part = try imuCommand(0x15, [])
            guard !part.isEmpty else { throw GlassesError.malformed("empty config chunk") }
            config += part
        }
        return Data(config.prefix(length))
    }
}
