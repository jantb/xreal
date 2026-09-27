import CDcmImu
import Foundation

// Upper bound for any bias correction; real residual bias is far below this.
let maxBias: Float = 0.1  // rad/s

// Automatic learning only runs while the head is essentially still, and only
// nudges the estimate slowly, so slow deliberate turns are not absorbed.
let autoMotionThreshold: Float = 0.02  // rad/s, relative to the current bias
let autoStillSeconds: Float = 1.5
let autoLearnTimeConstant: Float = 5.0  // seconds

// Explicit calibration: the glasses must lie still for this long.
let calibrationSeconds: Float = 2.0
let calibrationTimeoutSeconds: Float = 10.0
let calibrationMotionThreshold: Float = 0.05  // rad/s, relative to the running mean
let calibrationMinSamples: UInt32 = 50

// Converges the DCM's gravity estimate before the first pose is published, so
// the viewport does not slide while the filter settles.
let warmupSteps = 400
let warmupDt: Float = 0.01

// Longer gaps between IMU samples are skipped instead of integrated.
let maxSampleGap: Float = 0.1  // seconds
let rateTimeConstant: Float = 0.015  // seconds
let maxPredictionAge: Double = 0.05  // seconds

// Learning from manual recenters: yaw drift between two recenters is
// assumed to be leftover gyro bias about the vertical axis.
let driftMinInterval: Double = 20  // seconds
// Faster apparent drift is taken as a deliberate turn (e.g. moving the chair).
let driftMaxRate: Float = 0.01  // rad/s, about 34°/min
// Correct only part of the measured drift each time, so one misjudged
// recenter cannot throw the estimate far off.
let driftGain: Float = 0.5

/// Seconds on the same monotonic clock as `CACurrentMediaTime` and
/// `CADisplayLink` timestamps.
public func monotonicNow() -> Double {
    Double(clock_gettime_nsec_np(CLOCK_UPTIME_RAW)) / 1_000_000_000
}

/// Wraps an angle difference into [-π, π].
public func wrapAngle(_ angle: Float) -> Float {
    let fullTurn = 2 * Float.pi
    var wrapped = (angle + .pi).truncatingRemainder(dividingBy: fullTurn)
    if wrapped < 0 {
        wrapped += fullTurn
    }
    return wrapped - .pi
}

public struct HeadPose: Equatable, Sendable {
    /// Positive turns left.
    public var yaw: Float = 0
    /// Positive nods down.
    public var pitch: Float = 0
    /// Positive tilts the right ear down.
    public var roll: Float = 0

    public init(yaw: Float = 0, pitch: Float = 0, roll: Float = 0) {
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
    }
}

public enum CalibrationState: Equatable, Sendable {
    case idle
    case running(progress: Float)
    case succeeded
    case failed
}

public enum ImuStatus: Equatable, Sendable {
    case searching
    case connected
}

public struct TrackingSnapshot: Sendable {
    public var status = ImuStatus.searching
    /// Increments each time the glasses (re)connect and the pose restarts.
    public var session: UInt64 = 0
    public var pose = HeadPose()
    public var yawRate: Float = 0
    public var pitchRate: Float = 0
    public var rollRate: Float = 0
    /// Monotonic time of the newest IMU sample, see `monotonicNow`.
    public var sampledAt: Double?
    public var sampleRateHz: Float = 0
    public var gyroBias: SIMD3<Float>
    public var still = false
    public var calibration = CalibrationState.idle
    /// Increments whenever the bias changes by calibration or drift correction.
    public var biasRevision: UInt32 = 0

    public init(gyroBias: SIMD3<Float>) {
        self.gyroBias = gyroBias
    }

    /// Extrapolates the pose to `lead` seconds past `now`, covering the time
    /// between the last IMU sample and the moment the frame reaches the display.
    public func predict(now: Double, lead: Double) -> HeadPose {
        guard let sampledAt else { return pose }
        let age = max(0, now - sampledAt)
        if status != .connected || age > maxPredictionAge {
            return pose
        }
        let horizon = Float(age + lead)
        return HeadPose(
            yaw: wrapAngle(pose.yaw + yawRate * horizon),
            pitch: pose.pitch + pitchRate * horizon,
            roll: pose.roll + rollRate * horizon)
    }
}

public enum DriftObservation: Equatable, Sendable {
    /// First recenter: nothing to compare against yet.
    case anchored
    /// Too soon after the reference recenter to tell drift from noise.
    case tooSoon
    /// `measured` rad/s of drift seen; `correction` of it should be removed.
    case learned(measured: Float, correction: Float)
    /// Moved too fast to be drift; treated as a deliberate turn.
    case rejected(measured: Float)
}

/// Estimates leftover yaw drift from manual recenters. Each recenter says
/// "the screen belongs where I am looking now", so the yaw difference between
/// two recenters is drift accumulated over the time between them.
public struct DriftLearner: Sendable {
    private var anchor: (at: Double, yaw: Float)?

    public init() {}

    /// Forgets the reference, e.g. after the pose restarts.
    public mutating func reset() {
        anchor = nil
    }

    public mutating func observeRecenter(now: Double, yaw: Float) -> DriftObservation {
        guard let anchor else {
            self.anchor = (now, yaw)
            return .anchored
        }
        let elapsed = max(0, now - anchor.at)
        if elapsed < driftMinInterval {
            // Keep the older reference: a longer interval measures better.
            return .tooSoon
        }

        self.anchor = (now, yaw)
        let measured = wrapAngle(yaw - anchor.yaw) / Float(elapsed)
        if abs(measured) > driftMaxRate {
            return .rejected(measured: measured)
        }
        return .learned(measured: measured, correction: measured * driftGain)
    }
}

struct FusionUpdate {
    var pose: HeadPose
    var yawRate: Float
    var pitchRate: Float
    var rollRate: Float
}

/// Turns raw IMU samples into a head pose. Every sample is integrated with
/// its own device-time `dt`, so no rotation is lost between samples.
struct Fusion {
    private var dcm = DcmImu()
    private var lastTimestamp: UInt64?
    private var lastPose: HeadPose?
    private var yaw: Float = 0
    private var yawRate: Float = 0
    private var pitchRate: Float = 0
    private var rollRate: Float = 0
    /// Latest body-frame up direction.
    private(set) var up = SIMD3<Float>(0, 1, 0)

    init() {
        dcm_imu_init(&dcm)
    }

    /// `timestamp` is device time in microseconds.
    mutating func push(
        gyro: SIMD3<Float>, acc: SIMD3<Float>, timestamp: UInt64, bias: inout GyroBiasEstimator
    ) -> FusionUpdate? {
        guard let lastTimestamp else {
            self.lastTimestamp = timestamp
            for _ in 0..<warmupSteps {
                _ = dcm_imu_update(&dcm, 0, 0, 0, acc.x, acc.y, acc.z, warmupDt)
            }
            return nil
        }
        if timestamp <= lastTimestamp {
            return nil
        }
        self.lastTimestamp = timestamp
        let dt = Float(timestamp - lastTimestamp) / 1_000_000
        if dt > maxSampleGap {
            return nil
        }

        let corrected = bias.correct(gyro: gyro, dt: dt)
        let angles = dcm_imu_update(
            &dcm, corrected.x, corrected.y, corrected.z, acc.x, acc.y, acc.z, dt)

        // Yaw is integrated here rather than taken from the DCM: its internal
        // bias estimate for the vertical axis is unobservable, wanders, and
        // shows up as yaw drift. Only the DCM's gravity direction is used.
        up = upVector(pitch: angles.pitch, roll: angles.roll)
        let yawRateNow = (corrected * up).sum()
        yaw = wrapAngle(yaw + yawRateNow * dt)

        // The glasses report in a Y-up frame, so the DCM's roll axis is the
        // head's pitch (nodding) and its pitch axis the head's roll. On the
        // glasses, right ear down leans up towards +x, which makes the DCM's
        // pitch negative; roll is positive for right ear down.
        let pose = HeadPose(yaw: yaw, pitch: angles.roll, roll: -angles.pitch)

        if let lastPose {
            let alpha = dt / (rateTimeConstant + dt)
            let measuredYawRate = wrapAngle(pose.yaw - lastPose.yaw) / dt
            let measuredPitchRate = (pose.pitch - lastPose.pitch) / dt
            let measuredRollRate = (pose.roll - lastPose.roll) / dt
            yawRate += (measuredYawRate - yawRate) * alpha
            pitchRate += (measuredPitchRate - pitchRate) * alpha
            rollRate += (measuredRollRate - rollRate) * alpha
        }
        lastPose = pose

        return FusionUpdate(pose: pose, yawRate: yawRate, pitchRate: pitchRate, rollRate: rollRate)
    }
}

/// Estimates the gyro's zero-rate offset. The DCM filter can only learn bias
/// on axes that gravity makes observable; bias about the vertical axis would
/// otherwise integrate straight into yaw drift.
public struct GyroBiasEstimator: Sendable {
    public private(set) var bias: SIMD3<Float>
    private var stillFor: Float = 0
    private var calibration: Calibration?
    private var calibrationResult = CalibrationState.idle
    public private(set) var biasRevision: UInt32 = 0

    private struct Calibration {
        var elapsed: Float = 0
        var stillFor: Float = 0
        var count: UInt32 = 0
        var mean = SIMD3<Double>.zero

        mutating func restartWindow() {
            stillFor = 0
            count = 0
            mean = .zero
        }
    }

    public init(bias: SIMD3<Float>) {
        self.bias = bias.clamped(lowerBound: .init(repeating: -maxBias), upperBound: .init(repeating: maxBias))
    }

    public var isStill: Bool {
        stillFor >= autoStillSeconds
    }

    public var calibrationState: CalibrationState {
        if let calibration {
            return .running(progress: min(calibration.stillFor / calibrationSeconds, 1))
        }
        return calibrationResult
    }

    public mutating func startCalibration() {
        calibration = Calibration()
    }

    /// Adds `rate` (rad/s) of bias about the body-frame `up` axis, which is
    /// what shows up as yaw drift.
    public mutating func correctYawDrift(rate: Float, up: SIMD3<Float>) {
        bias = clampBias(bias + rate * up)
        biasRevision += 1
    }

    /// Returns the bias-corrected gyro reading and updates the estimate.
    public mutating func correct(gyro: SIMD3<Float>, dt: Float) -> SIMD3<Float> {
        let corrected = gyro - bias
        if calibration != nil {
            updateCalibration(gyro: gyro, dt: dt)
        } else {
            updateAuto(gyro: gyro, corrected: corrected, dt: dt)
        }
        return corrected
    }

    private mutating func updateAuto(gyro: SIMD3<Float>, corrected: SIMD3<Float>, dt: Float) {
        if magnitude(corrected) < autoMotionThreshold {
            stillFor += dt
        } else {
            stillFor = 0
        }
        // While moving the estimate is held as-is.
        if isStill {
            let k = dt / autoLearnTimeConstant
            bias = clampBias(bias + (gyro - bias) * k)
        }
    }

    private mutating func updateCalibration(gyro: SIMD3<Float>, dt: Float) {
        guard var calibration else { return }
        calibration.elapsed += dt

        let deviation = gyro - SIMD3<Float>(calibration.mean)
        if calibration.count >= calibrationMinSamples
            && magnitude(deviation) > calibrationMotionThreshold
        {
            calibration.restartWindow()
        }

        calibration.count += 1
        calibration.stillFor += dt
        calibration.mean += (SIMD3<Double>(gyro) - calibration.mean) / Double(calibration.count)

        if calibration.stillFor >= calibrationSeconds {
            bias = clampBias(SIMD3<Float>(calibration.mean))
            self.calibration = nil
            calibrationResult = .succeeded
            biasRevision += 1
            stillFor = 0
        } else if calibration.elapsed >= calibrationTimeoutSeconds {
            self.calibration = nil
            calibrationResult = .failed
        } else {
            self.calibration = calibration
        }
    }

    private func clampBias(_ value: SIMD3<Float>) -> SIMD3<Float> {
        value.clamped(lowerBound: .init(repeating: -maxBias), upperBound: .init(repeating: maxBias))
    }
}

struct SampleRate {
    private var count: UInt32 = 0
    private var since: Double
    private var rate: Float = 0

    init(now: Double) {
        since = now
    }

    mutating func tick(now: Double) -> Float {
        count += 1
        let elapsed = now - since
        if elapsed >= 0.5 {
            rate = Float(Double(count) / elapsed)
            count = 0
            since = now
        }
        return rate
    }
}

/// Body-frame unit vector pointing up, rebuilt from the DCM's angles
/// (roll = atan2(x1, x2), pitch = asin(-x0) of its gravity estimate x).
func upVector(pitch: Float, roll: Float) -> SIMD3<Float> {
    SIMD3(-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll))
}

func magnitude(_ v: SIMD3<Float>) -> Float {
    (v * v).sum().squareRoot()
}
