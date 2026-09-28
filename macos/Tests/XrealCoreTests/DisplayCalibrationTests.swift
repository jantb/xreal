import Foundation
import Testing
import simd

@testable import XrealCore

/// The display section of an Air 2's factory calibration.
private let air2Calibration = Data(#"{"display": {"resolution": [1920.0, 1080.0], "k_left_display": [2705.48, 0.0, 964.67, 0.0, 2701.73, 571.591, 0.0, 0.0, 1.0], "k_right_display": [2709.87, 0.0, 949.94, 0.0, 2696.04, 549.238, 0.0, 0.0, 1.0], "target_p_left_display": [-0.0592447, 0.0187027, -0.0192912], "target_p_right_display": [0.00436358, 0.0186026, -0.0196755], "target_q_left_display": [-0.00915112, 0.00835657, 0.00262477, 0.99992], "target_q_right_display": [-0.00542599, -0.00240477, 0.00336847, 0.999977]}}"#.utf8)

private let calibrated = DisplayCalibration.parse(config: air2Calibration)

/// Where a point `metres` straight ahead lands in each eye.
private func pixels(_ calibration: DisplayCalibration, metres: Float) throws -> (left: SIMD2<Float>, right: SIMD2<Float>) {
    let ahead = SIMD3<Float>(0, 0, -metres)
    return (try #require(calibration.left.pixel(of: ahead)), try #require(calibration.right.pixel(of: ahead)))
}

@Test func theCalibratedEyesSeeAPointFourMetresAheadInTheSamePlace() throws {
    let calibration = try #require(calibrated)
    let (left, right) = try pixels(calibration, metres: 4)
    // Where both eyes' pictures meet, as a flat picture hangs there.
    #expect(abs(left.x - right.x) < 2, "left \(left) right \(right)")
    // Level in both eyes, so they need not look up or down differently.
    #expect(abs(left.y - right.y) < 3, "left \(left) right \(right)")
}

@Test func nearerThanWhereTheyMeetTheEyesSeeThingsApart() throws {
    let calibration = try #require(calibrated)
    let near = try pixels(calibration, metres: 1)
    let far = try pixels(calibration, metres: 20)
    // Nearer is further right for the left eye, as it is for real eyes.
    #expect(near.left.x - near.right.x > 50)
    #expect(far.left.x - far.right.x < 0)
}

@Test func theCalibratedEyesSitAnEyeDistanceApart() throws {
    let calibration = try #require(calibrated)
    let separation = calibration.right.position - calibration.left.position
    #expect(abs(separation.x - 0.0636) < 0.001, "\(separation)")
    #expect(abs(separation.y) < 0.002 && abs(separation.z) < 0.002)
}

@Test func withoutACalibrationTheEyesStillMeetFourMetresAhead() throws {
    let (left, right) = try pixels(.nominal, metres: 4)
    #expect(abs(left.x - right.x) < 0.5)
    #expect(abs(left.y - right.y) < 0.5)
}

@Test func aCalibrationWithoutUsableDisplayOpticsIsNotUsed() {
    for json in ["{}", #"{"display": {"resolution": [1920, 1080]}}"#, "not json"] {
        #expect(DisplayCalibration.parse(config: Data(json.utf8)) == nil, "\(json)")
    }
}
