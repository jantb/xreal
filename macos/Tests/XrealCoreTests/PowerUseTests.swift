import Foundation
import Testing

@testable import XrealCore

// As a MacBook Pro on a 140 W charger reported them, charging from 59%.
private func onCharger() -> [String: Any] {
    [
        "PowerTelemetryData": ["SystemLoad": 25122, "SystemPowerIn": 135427] as [String: Any],
        "Voltage": 12360,
        "InstantAmperage": 8951,
        "IsCharging": true,
        "PowerOutDetails": [["Watts": 1717] as [String: Any]],
    ]
}

@Test func aChargingBatteryReportsThePowerGoingIntoIt() throws {
    let power = try #require(PowerUse(properties: onCharger()))
    let charging = try #require(power.charging)
    #expect(abs(charging - 110.6) < 0.1)
    #expect(abs(power.system - 25.1) < 0.1)
    #expect(abs(power.usbOut - 1.7) < 0.1)
}

@Test func aFullBatteryOnTheChargerIsNotCharging() throws {
    var properties = onCharger()
    properties["IsCharging"] = false
    properties["InstantAmperage"] = 0
    let power = try #require(PowerUse(properties: properties))
    #expect(power.charging == nil)
}

@Test func onTheBatteryThePowerLeavingItRunsTheMac() throws {
    let properties: [String: Any] = [
        "PowerTelemetryData": ["SystemLoad": 0] as [String: Any],
        "Voltage": 12000,
        "InstantAmperage": -1500,
        "IsCharging": false,
    ]
    let power = try #require(PowerUse(properties: properties))
    #expect(power.charging == nil)
    #expect(abs(power.system - 18) < 0.1)
}
