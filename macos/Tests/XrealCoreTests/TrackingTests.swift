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

/// Head motion of someone sitting and reading: small nods and turns that
/// come back to where they started every `period` seconds.
private func calmHead(at time: Float, period: Float = 2) -> SIMD3<Float> {
    let phase = 2 * Float.pi * time / period
    return SIMD3(0.03 * sin(phase), 0.04 * cos(phase), 0.02 * sin(2 * phase))
}

@Test func wornGlassesLearnTheBiasFromCalmHeadMotion() {
    let offset = SIMD3<Float>(0.001, 0.004, -0.002)
    var fusion = Fusion()
    var bias = GyroBiasEstimator(bias: .zero)
    var t: UInt64 = 1_000
    func yaw(after seconds: Float, from start: Int) -> Float {
        var yaw: Float = 0
        for i in start..<start + Int(seconds / dt) {
            t += UInt64(dt * 1_000_000)
            let gyro = calmHead(at: Float(i) * dt) + offset
            if let update = fusion.push(gyro: gyro, acc: gravityYUp, timestamp: t, bias: &bias) {
                yaw = update.pose.yaw
            }
        }
        return yaw
    }

    // Two minutes of wearing them; the head never holds still.
    _ = yaw(after: 120, from: 0)
    let before = yaw(after: 0.001, from: 120_000)
    // A whole number of periods later the head is back where it was.
    let after = yaw(after: 60, from: 120_001)

    // Unlearned, 0.004 rad/s would drift 0.24 rad in the minute.
    let drift = abs(wrapAngle(after - before))
    #expect(drift < 0.03, "yaw drifted \(drift) rad in a minute")
}

@Test func steadySlowPanIsNotLearnedAsBias() {
    var bias = GyroBiasEstimator(bias: .zero)
    // Slowly panning across a wide screen, calm but always one way.
    for i in 0..<Int(30 / dt) {
        _ = bias.correct(gyro: calmHead(at: Float(i) * dt) + SIMD3(0, 0.02, 0), dt: dt)
    }
    #expect(abs(bias.bias.y) < 0.001, "\(bias.bias)")
}

@Test func biasFollowsTheGlassesAsTheyWarmUp() {
    // Bias that grows with temperature, as a MEMS gyro's does.
    let slope = SIMD3<Float>(0.00005, 0.0001, -0.00008)
    let biasAt20 = SIMD3<Float>(0.002, -0.001, 0.003)
    let trueBias = { (temperature: Float) in biasAt20 + slope * (temperature - 20) }
    var bias = GyroBiasEstimator(bias: .zero)

    // Warming from 20 to 35 °C over 15 minutes, lying still now and then,
    // as when put down between uses.
    let warmUp = Int(15 * 60 / dt)
    for i in 0..<warmUp {
        let temperature = 20 + 15 * Float(i) / Float(warmUp)
        let lyingStill = (i / Int(60 / dt)) % 3 == 0
        let motion = lyingStill ? SIMD3<Float>.zero : SIMD3(0.4, i % 2 == 0 ? 0.6 : -0.6, 0.2)
        _ = bias.correct(gyro: trueBias(temperature) + motion, dt: dt, temperature: temperature)
    }
    // Then worn and moving the whole time, no chance to measure, while the
    // glasses warm on to 45 °C.
    for i in 0..<Int(10 * 60 / dt) {
        let temperature = 35 + 10 * Float(i) * dt / 600
        let motion = SIMD3<Float>(0.4, i % 2 == 0 ? 0.6 : -0.6, 0.2)
        _ = bias.correct(gyro: trueBias(temperature) + motion, dt: dt, temperature: temperature)
    }

    // Holding on to the last measured bias would be 0.001 rad/s off about y.
    let error = bias.bias - trueBias(45)
    #expect(abs(error.y) < 0.0004, "bias off by \(error) at 45 °C")
}

@Test func learnedThermalBiasSurvivesARestart() {
    var bias = GyroBiasEstimator(bias: .zero)
    for i in 0..<Int(10 / dt) {
        let temperature: Float = i < Int(5 / dt) ? 25 : 40
        _ = bias.correct(gyro: SIMD3(0, 0.001 + 0.0001 * (temperature - 25), 0), dt: dt, temperature: temperature)
    }
    let saved = bias.thermalBias
    let restarted = GyroBiasEstimator(bias: saved.reference, slope: saved.slope)
    #expect(restarted.thermalBias == saved)
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
    // Keep nodding briskly so automatic learning stays out of the way.
    let gyro = { (i: Int) in SIMD3<Float>(i % 2 == 0 ? 0.3 : -0.3, residual, 0) }

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

@Test func predictionDoesNotOvershootWhenAQuickTurnStops() {
    var fusion = Fusion()
    var bias = GyroBiasEstimator(bias: .zero)
    var t: UInt64 = 1_000
    var snapshot = TrackingSnapshot(gyroBias: .zero)
    snapshot.status = .connected
    func run(rate: Float, seconds: Float) {
        for _ in 0..<Int(seconds / dt) {
            t += 1_000
            if let update = fusion.push(gyro: SIMD3(0, rate, 0), acc: gravityYUp, timestamp: t, bias: &bias) {
                snapshot.pose = update.pose
                (snapshot.yawRate, snapshot.pitchRate, snapshot.rollRate) =
                    (update.yawRate, update.pitchRate, update.rollRate)
            }
        }
    }
    // A quick glance to the side at about 170°/s, then a sudden stop.
    run(rate: 0, seconds: 0.5)
    run(rate: 3, seconds: 0.3)
    run(rate: 0, seconds: 0.015)
    let stoppedAt = snapshot.pose.yaw
    snapshot.sampledAt = 10
    // Predicting as far ahead as frames take to reach the glasses.
    let predicted = snapshot.predict(now: 10, lead: 0.04)
    #expect(abs(wrapAngle(predicted.yaw - stoppedAt)) < 0.02, "overshoot \(predicted.yaw - stoppedAt) rad")
}

@Test func predictionIsCappedForImplausiblyFastRates() {
    var snapshot = TrackingSnapshot(gyroBias: .zero)
    snapshot.status = .connected
    snapshot.yawRate = 50
    snapshot.sampledAt = 5
    let predicted = snapshot.predict(now: 5, lead: 0.04)
    #expect(abs(predicted.yaw) <= 0.15)
}

@Test func aSavedBiasIsNotThrownOffByTheFirstCalmStretchAfterARestart() {
    let saved = SIMD3<Float>(-0.004, 0.0008, 0.0045)
    var bias = GyroBiasEstimator(bias: saved)
    // One calm stretch, worn, whose average picks up a slow drift of the
    // head as well as the bias.
    for i in 0..<Int(4.5 / dt) {
        _ = bias.correct(gyro: saved + SIMD3(0, 0.004, 0) + calmHead(at: Float(i) * dt), dt: dt)
    }
    #expect(abs(bias.bias.y - saved.y) < 0.002, "bias moved to \(bias.bias)")
}
