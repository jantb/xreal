import Foundation
import Testing

@testable import XrealCore

private let dt: Float = 0.001
private let gravityYUp = SIMD3<Float>(0, 9.81, 0)

private func runStill(
    _ fusion: inout Fusion, _ bias: inout GyroBiasEstimator, gyro: SIMD3<Float>, seconds: Float,
    _ t: inout UInt64
) -> HeadPose {
    var pose = HeadPose()
    for _ in 0..<Int(seconds / dt) {
        t += UInt64(dt * 1_000_000)
        if let update = fusion.push(gyro: gyro, acc: gravityYUp, timestamp: t, bias: &bias) {
            pose = update.pose
        }
    }
    return pose
}

@Test func wrapAngleTakesTheShortWayRound() {
    #expect(abs(wrapAngle(3.0 - -3.0) - (6.0 - 2 * .pi)) < 1e-5)
    #expect(abs(wrapAngle(-3.0 - 3.0) - (2 * .pi - 6.0)) < 1e-5)
    #expect(abs(wrapAngle(0.25) - 0.25) < 1e-6)
}

@Test func firstPoseIsAlreadySettledOnGravity() {
    var fusion = Fusion()
    var bias = GyroBiasEstimator(bias: .zero)
    var t: UInt64 = 1_000
    let first = runStill(&fusion, &bias, gyro: .zero, seconds: 0.01, &t)
    let later = runStill(&fusion, &bias, gyro: .zero, seconds: 2.0, &t)

    #expect(abs(first.pitch - later.pitch) < 0.01, "first=\(first) later=\(later)")
    #expect(abs(first.yaw) < 1e-3)
}

@Test func stillGlassesWithGyroOffsetStopDrifting() {
    let offset = SIMD3<Float>(0.004, 0.012, -0.006)
    var fusion = Fusion()
    var bias = GyroBiasEstimator(bias: .zero)
    var t: UInt64 = 1_000

    _ = runStill(&fusion, &bias, gyro: offset, seconds: 30, &t)
    let before = runStill(&fusion, &bias, gyro: offset, seconds: 0.001, &t)
    let after = runStill(&fusion, &bias, gyro: offset, seconds: 10, &t)

    // Without correction 0.012 rad/s would drift 0.12 rad in 10 s.
    let drift = abs(wrapAngle(after.yaw - before.yaw))
    #expect(drift < 0.01, "yaw drifted \(drift) rad in 10 s")
}

@Test(arguments: [Float(0), .pi / 6])  // Upright, and nodded down 30° while turning.
func turningAboutTheVerticalChangesYawByTheTurnedAngle(tilt: Float) throws {
    let up = SIMD3<Float>(0, cos(tilt), sin(tilt))
    let gravity = up * 9.81
    var fusion = Fusion()
    var bias = GyroBiasEstimator(bias: .zero)
    var t: UInt64 = 1_000
    var start: HeadPose?
    var end = HeadPose()
    for _ in 0..<1000 {
        t += 1_000
        if let update = fusion.push(gyro: .zero, acc: gravity, timestamp: t, bias: &bias) {
            start = start ?? update.pose
        }
    }
    for _ in 0..<1000 {
        t += 1_000
        if let update = fusion.push(gyro: up * 0.5, acc: gravity, timestamp: t, bias: &bias) {
            end = update.pose
        }
    }
    let turned = wrapAngle(end.yaw - (try #require(start)).yaw)
    #expect(abs(turned - 0.5) < 0.01, "tilt \(tilt): turned \(turned) rad")
}

@Test func slowHeadTurnIsNotLearnedAsBias() {
    var bias = GyroBiasEstimator(bias: .zero)
    for _ in 0..<Int(20 / dt) {
        _ = bias.correct(gyro: SIMD3(0, 0.1, 0), dt: dt)
    }
    #expect(abs(bias.bias.y) < 1e-6)
}

@Test func biasIsKeptWhileMoving() {
    var bias = GyroBiasEstimator(bias: SIMD3(0, 0.01, 0))
    for _ in 0..<Int(20 / dt) {
        _ = bias.correct(gyro: SIMD3(0.3, 0.8, -0.2), dt: dt)
    }
    #expect(abs(bias.bias.y - 0.01) < 1e-6)
}

@Test func calibrationMeasuresOffsetOfStillGlasses() {
    let offset = SIMD3<Float>(0.03, -0.045, 0.02)
    var bias = GyroBiasEstimator(bias: .zero)
    bias.startCalibration()
    for i in 0..<Int(3 / dt) {
        let noise: Float = i % 2 == 0 ? 0.002 : -0.002
        _ = bias.correct(gyro: SIMD3(offset.x + noise, offset.y - noise, offset.z), dt: dt)
    }

    #expect(bias.calibrationState == .succeeded)
    #expect(magnitude(bias.bias - offset) < 1e-3, "\(bias.bias)")
}

@Test func calibrationFailsIfGlassesKeepMoving() {
    var bias = GyroBiasEstimator(bias: .zero)
    bias.startCalibration()
    for i in 0..<Int(12 / dt) {
        let swing: Float = (i / 100) % 2 == 0 ? 0.5 : -0.5
        _ = bias.correct(gyro: SIMD3(0, swing, 0), dt: dt)
    }
    #expect(bias.calibrationState == .failed)
    #expect(bias.bias == .zero)
}

@Test func recentersRevealSlowDrift() throws {
    var learner = DriftLearner()
    #expect(learner.observeRecenter(now: 100, yaw: 0.2) == .anchored)

    let observation = learner.observeRecenter(now: 160, yaw: 0.2 + 0.24)
    guard case .learned(let measured, let correction) = observation else {
        Issue.record("expected drift to be learned, got \(observation)")
        return
    }
    #expect(abs(measured - 0.004) < 1e-5)
    #expect(correction > 0 && correction <= measured)
}

@Test func deliberateTurnBetweenRecentersIsNotLearned() {
    var learner = DriftLearner()
    _ = learner.observeRecenter(now: 100, yaw: 0)

    let observation = learner.observeRecenter(now: 130, yaw: 1.2)
    guard case .rejected = observation else {
        Issue.record("expected a rejected turn, got \(observation)")
        return
    }
}

@Test func quickRepeatedRecentersMeasureFromTheOlderOne() {
    var learner = DriftLearner()
    _ = learner.observeRecenter(now: 100, yaw: 0)
    #expect(learner.observeRecenter(now: 105, yaw: 0.02) == .tooSoon)

    let observation = learner.observeRecenter(now: 140, yaw: 0.12)
    guard case .learned(let measured, _) = observation else {
        Issue.record("expected drift to be learned, got \(observation)")
        return
    }
    #expect(abs(measured - 0.003) < 1e-5)
}

@Test func learnedDriftCorrectionRemovesThatDrift() {
    let residual: Float = 0.003
    var fusion = Fusion()
    var bias = GyroBiasEstimator(bias: .zero)
    var t: UInt64 = 1_000
    // Keep moving slightly so automatic learning stays out of the way.
    let gyro = { (i: Int) in SIMD3<Float>(i % 2 == 0 ? 0.05 : -0.05, residual, 0) }

    func yawOver(_ fusion: inout Fusion, _ bias: inout GyroBiasEstimator, _ t: inout UInt64) -> Float {
        var first: Float?
        var last: Float = 0
        for i in 0..<10_000 {
            t += 1_000
            if let update = fusion.push(gyro: gyro(i), acc: gravityYUp, timestamp: t, bias: &bias) {
                first = first ?? update.pose.yaw
                last = update.pose.yaw
            }
        }
        return wrapAngle(last - (first ?? last))
    }

    let before = yawOver(&fusion, &bias, &t)
    bias.correctYawDrift(rate: residual, up: fusion.up)
    let after = yawOver(&fusion, &bias, &t)

    #expect(before > 0.025, "expected visible drift first, got \(before)")
    #expect(abs(after) < 0.003, "still drifting \(after) rad in 10 s")
}

@Test func predictionExtrapolatesAlongHeadMotion() {
    var snapshot = TrackingSnapshot(gyroBias: .zero)
    snapshot.status = .connected
    snapshot.pose = HeadPose(yaw: 0.1, pitch: 0.2)
    snapshot.yawRate = 1.0
    snapshot.pitchRate = -0.5
    snapshot.sampledAt = 50

    let predicted = snapshot.predict(now: 50, lead: 0.01)
    #expect(abs(predicted.yaw - 0.11) < 1e-4)
    #expect(abs(predicted.pitch - 0.195) < 1e-4)
}

@Test func predictionHoldsPoseWhenTrackingIsStale() {
    var snapshot = TrackingSnapshot(gyroBias: .zero)
    snapshot.status = .connected
    snapshot.pose = HeadPose(yaw: 0.1, pitch: 0.2)
    snapshot.yawRate = 1.0
    snapshot.sampledAt = 50

    #expect(snapshot.predict(now: 51, lead: 0.01) == snapshot.pose)
}

@Test func tiltingTheHeadSidewaysChangesRollNotPitch() {
    func settledPose(up: SIMD3<Float>) -> HeadPose {
        var fusion = Fusion()
        var bias = GyroBiasEstimator(bias: .zero)
        var t: UInt64 = 1_000
        var pose = HeadPose()
        for _ in 0..<2000 {
            t += 1_000
            if let update = fusion.push(gyro: .zero, acc: up * 9.81, timestamp: t, bias: &bias) {
                pose = update.pose
            }
        }
        return pose
    }
    let tilt: Float = 0.3
    let level = settledPose(up: SIMD3(0, 1, 0))
    // Right ear down: the glasses' accelerometer sees up lean towards +x.
    let tilted = settledPose(up: SIMD3(sin(tilt), cos(tilt), 0))

    #expect(abs((tilted.roll - level.roll) - tilt) < 0.02, "roll \(tilted.roll - level.roll)")
    #expect(abs(tilted.pitch - level.pitch) < 0.02)
}
