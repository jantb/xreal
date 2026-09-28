import Foundation
import Testing

@testable import XrealCore

private func mcuPacket(length: UInt16) -> [UInt8] {
    var bytes = [UInt8](repeating: 0, count: 0x40)
    bytes[0] = 0xfd
    bytes.write(length, at: 5)
    bytes.write(UInt16(0x1234), at: 15)
    return bytes
}

private func imuPacket(length: UInt16) -> [UInt8] {
    var bytes = [UInt8](repeating: 0, count: 0x40)
    bytes[0] = 0xaa
    bytes.write(length, at: 5)
    bytes[7] = 0x12
    return bytes
}

private func writeInt24(_ value: Int32, into bytes: inout [UInt8], at offset: Int) {
    for index in 0..<3 {
        bytes[offset + index] = UInt8(truncatingIfNeeded: value >> (8 * index))
    }
}

@Test func mcuPacketRejectsInvalidLengths() throws {
    #expect(McuPacket.deserialize(mcuPacket(length: 16)) == nil)
    #expect(McuPacket.deserialize(mcuPacket(length: 60)) == nil)

    let packet = try #require(McuPacket.deserialize(mcuPacket(length: 17)))
    #expect(packet.command == 0x1234)
    #expect(packet.data.isEmpty)
}

@Test func imuPacketRejectsInvalidLengths() throws {
    #expect(ImuPacket.deserialize(imuPacket(length: 2)) == nil)
    #expect(ImuPacket.deserialize(imuPacket(length: 60)) == nil)

    let packet = try #require(ImuPacket.deserialize(imuPacket(length: 3)))
    #expect(packet.command == 0x12)
    #expect(packet.data.isEmpty)
}

@Test func packetSerializersRejectOversizedPayloads() {
    #expect(McuPacket(command: 1, data: [UInt8](repeating: 0, count: 43)).serialize() == nil)
    #expect(ImuPacket(command: 1, data: [UInt8](repeating: 0, count: 57)).serialize() == nil)
}

@Test func serializedCommandsReadBackWithTheirPayload() throws {
    let mcu = McuPacket(command: 0x08, data: [11])
    #expect(McuPacket.deserialize(try #require(mcu.serialize())) == mcu)

    let imu = ImuPacket(command: 0x19, data: [1])
    #expect(ImuPacket.deserialize(try #require(imu.serialize())) == imu)
}

@Test func checksumIsStandardCrc32() {
    #expect(crc32(Array("123456789".utf8)) == 0xcbf4_3926)
}

@Test func configParserReadsFactoryBiases() throws {
    let json = #"{"IMU": {"device_1": {"gyro_bias": [0.1, 0.2, 0.3], "accel_bias": [-1, 0, 1]}}}"#
    let biases = try ImuBiases.parse(config: Data(json.utf8))
    #expect(biases.gyro == SIMD3(0.1, 0.2, 0.3))
    #expect(biases.accelerometer == SIMD3(-1, 0, 1))
}

@Test func configParserRejectsInvalidConfigs() {
    for json in ["null", "{}", #"{"IMU": {"device_1": {"gyro_bias": [1], "accel_bias": [0, 0, 0]}}}"#] {
        #expect(throws: GlassesError.self) { try ImuBiases.parse(config: Data(json.utf8)) }
    }
}

@Test func imuReportDecodesScaledSensorReadings() throws {
    var report = [UInt8](repeating: 0, count: 64)
    report[0] = 1
    report[1] = 2
    report.write(UInt64(5_000_000), at: 4)  // ns
    // Gyro: raw * 1 / 1000 degrees per second.
    report.write(UInt16(1), at: 12)
    report.write(UInt32(1000), at: 14)
    for (index, raw) in [Int32(90_000), -45_000, 0].enumerated() {
        writeInt24(raw, into: &report, at: 18 + 3 * index)
    }
    // Accelerometer: raw * 1 / 1000 g.
    report.write(UInt16(1), at: 27)
    report.write(UInt32(1000), at: 29)
    for (index, raw) in [Int32(0), 1000, -500].enumerated() {
        writeInt24(raw, into: &report, at: 33 + 3 * index)
    }

    let sample = try #require(parseImuReport(report, biases: ImuBiases()))
    #expect(sample.timestamp == 5_000)
    #expect(abs(sample.gyroscope.x - 90 * .pi / 180) < 1e-4)
    #expect(abs(sample.gyroscope.y + 45 * .pi / 180) < 1e-4)
    #expect(abs(sample.accelerometer.y - 9.81) < 1e-4)
    #expect(abs(sample.accelerometer.z + 4.905) < 1e-4)
}

@Test func nonImuReportsAreIgnored() {
    #expect(parseImuReport([UInt8](repeating: 0, count: 64), biases: ImuBiases()) == nil)
}

@Test func imuReportCarriesTheChipTemperature() throws {
    func temperature(count: Int16) -> Float? {
        var report = [UInt8](repeating: 0, count: 64)
        report[0] = 1
        report[1] = 2
        report.write(UInt16(bitPattern: count), at: 2)
        return parseImuReport(report, biases: ImuBiases())?.temperature
    }
    let cool = try #require(temperature(count: 0))
    let warm = try #require(temperature(count: 1_300))
    #expect(warm > cool)
    #expect(temperature(count: .min) == nil, "a reading far outside any real temperature is dropped")
}
