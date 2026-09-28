import Foundation
import Testing
import simd

@testable import XrealCore

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

@Test func theLensMapsBackToTheDisplayPixelsItWasCalibratedAt() throws {
    let calibration = try #require(calibrated)
    let root = try #require(try JSONSerialization.jsonObject(with: air2Calibration) as? [String: Any])
    let lenses = try #require(root["display_distortion"] as? [String: Any])
    for (name, eye) in [("left_display", calibration.left), ("right_display", calibration.right)] {
        let lens = try #require(eye.distortion, "\(name) has no distortion")
        let grid = try #require((lenses[name] as? [String: Any])?["data"] as? [NSNumber]).map(\.floatValue)
        // Each calibrated display pixel, and where the lens shows it.
        for index in stride(from: 0, to: grid.count / 4, by: 7) {
            let display = SIMD2(grid[index * 4], grid[index * 4 + 1])
            let shown = SIMD2(grid[index * 4 + 2], grid[index * 4 + 3])
            let found = lens.displayPixel(showing: shown)
            #expect(simd_distance(found, display) < 0.3, "\(name): \(shown) should come from \(display), got \(found)")
        }
    }
}

@Test func throughTheLensTheCornersArePulledInAndTheMiddleBarelyMoves() throws {
    let lens = try #require(calibrated?.left.distortion)
    let middle = SIMD2<Float>(960, 540)
    #expect(simd_distance(lens.displayPixel(showing: middle), middle) < 2)
    // What should appear at the display's corner is drawn further in, as
    // the lens magnifies the edges.
    let corner = lens.displayPixel(showing: SIMD2(0, 0))
    #expect(corner.x > 5 && corner.y > 5, "corner drawn at \(corner)")
}
