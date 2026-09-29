import CDcmImu
import Foundation

// Upper bound for any bias correction; real residual bias is far below this.
let maxBias: Float = 0.1  // rad/s

// Glasses count as still after this long without turning faster than this.
let autoMotionThreshold: Float = 0.02  // rad/s, relative to the current bias
let autoStillSeconds: Float = 1.5

// Worn, the head is never that still. Stretches of this long without a
// faster movement are averaged instead, and count unless their mean is a
// steady slow turn.
let calmWindowSeconds: Float = 4
let calmMotionThreshold: Float = 0.08  // rad/s, relative to the current bias
let calmMaxMeanRate: Float = 0.005  // rad/s, about 17°/min
// Lying still, this long measures the bias well.
let stillWindowSeconds: Float = 1
// Standard deviation of each kind of bias measurement.
let stillMeasurementDeviation: Float = 0.0005  // rad/s
let calmMeasurementDeviation: Float = 0.002  // rad/s
let calibrationMeasurementDeviation: Float = 0.0001  // rad/s
// Learned bias is saved at most this often.
let learnedSaveInterval: Float = 60  // seconds

// Thermal model of the bias. MEMS gyro bias typically shifts by around
// 0.0001 rad/s per °C, a few degrees per minute of yaw drift over the
// glasses' warm-up.
let referenceTemperature: Float = 35  // °C
let maxBiasSlope: Float = 0.001  // rad/s per °C
let initialBiasDeviation: Float = 0.005  // rad/s
// A bias saved by an earlier run is already close; trusting it more keeps
// the first measurements after a restart from throwing it off.
let savedBiasDeviation: Float = 0.001  // rad/s
let initialSlopeDeviation: Float = 0.0002  // rad/s per °C
// How fast bias may wander beyond what temperature explains: variance
// growth per second, from 0.0002 rad/s per √minute and 0.00002 rad/s/°C
// per √hour.
let biasWanderPerSecond: Float = 0.0002 * 0.0002 / 60
let slopeWanderPerSecond: Float = 0.00002 * 0.00002 / 3600
let temperatureTimeConstant: Float = 5  // seconds

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
// Head rates are only lightly smoothed: prediction runs 30-40 ms ahead,
// and any lag in the rate makes the view overshoot when a quick turn stops.
let rateTimeConstant: Float = 0.004  // seconds
let maxPredictionAge: Double = 0.05  // seconds
// Prediction never moves the view further than this ahead of the pose.
let maxPredictionAngle: Float = 0.14  // rad, about 8°
// Head motion slower than this is not predicted. Extrapolating a fast
// transient tens of milliseconds ahead multiplies it several times over, and
// the small quick jolts of a heartbeat are just that; a deliberate turn is
// well above it. Above it, prediction is reduced by this much.
let predictionRateFloor: Float = 0.05  // rad/s, about 3°/s

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
    /// Bias at the current temperature.
    public var gyroBias: SIMD3<Float>
    /// The learned bias model, to save.
    public var thermalBias: ThermalBias
    /// IMU temperature, °C, when the glasses report it.
    public var temperature: Float?
    /// Counts bias measurements from still or calm stretches.
    public var learnedWindows: UInt32 = 0
    /// The connected glasses' display optics, when their calibration has
    /// them.
    public var display: DisplayCalibration?
    public var still = false
    public var calibration = CalibrationState.idle
    /// Increments whenever the bias changes by calibration or drift correction.
    public var biasRevision: UInt32 = 0

    public init(gyroBias: SIMD3<Float>, biasSlope: SIMD3<Float> = .zero) {
        self.gyroBias = gyroBias
        thermalBias = ThermalBias(reference: gyroBias, slope: biasSlope)
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
        // Only the speed beyond the floor is predicted, in the direction of
        // the motion, so the prediction grows smoothly from nothing.
        let speed = magnitude(SIMD3(yawRate, pitchRate, rollRate))
        let share = speed > predictionRateFloor ? (speed - predictionRateFloor) / speed : 0
        let ahead = { (rate: Float) in min(max(rate * share * horizon, -maxPredictionAngle), maxPredictionAngle) }
        return HeadPose(
            yaw: wrapAngle(pose.yaw + ahead(yawRate)),
            pitch: pose.pitch + ahead(pitchRate),
            roll: pose.roll + ahead(rollRate))
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

    /// `timestamp` is device time in microseconds, `temperature` the IMU's
    /// in °C.
    mutating func push(
        gyro: SIMD3<Float>, acc: SIMD3<Float>, timestamp: UInt64, temperature: Float? = nil,
        bias: inout GyroBiasEstimator
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

        let corrected = bias.correct(gyro: gyro, dt: dt, temperature: temperature)
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
            // The gyro measures the turn rate directly.
            let measuredYawRate = yawRateNow
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

/// Gyro bias as a function of temperature, per axis:
/// `reference + slope × (temperature − referenceTemperature)`.
public struct ThermalBias: Equatable, Sendable {
    /// Bias at `referenceTemperature`, rad/s.
    public var reference: SIMD3<Float>
    /// Change in bias per °C, rad/s.
    public var slope: SIMD3<Float>

    public init(reference: SIMD3<Float> = .zero, slope: SIMD3<Float> = .zero) {
        self.reference = reference
        self.slope = slope
    }

    public func bias(at temperature: Float?) -> SIMD3<Float> {
        reference + slope * temperatureOffset(temperature)
    }
}

func temperatureOffset(_ temperature: Float?) -> Float {
    temperature.map { $0 - referenceTemperature } ?? 0
}

/// Tracks `ThermalBias` with a Kalman filter per axis. Each bias measurement
/// is taken at some temperature, so measurements at different temperatures
/// teach the slope, and the slope then keeps the bias right as the glasses
/// warm up, between measurements.
struct ThermalBiasFilter: Sendable {
    private(set) var model: ThermalBias
    // Covariance of (reference, slope), per axis.
    private var p00: SIMD3<Float>
    private var p01 = SIMD3<Float>.zero
    private var p11: SIMD3<Float>

    /// `deviation` is how far off `model.reference` may be, rad/s.
    init(_ model: ThermalBias, deviation: Float = initialBiasDeviation) {
        self.model = model
        p00 = SIMD3(repeating: deviation * deviation)
        p11 = SIMD3(repeating: initialSlopeDeviation * initialSlopeDeviation)
        clamp()
    }

    /// Lets the bias wander a little over `dt` seconds, beyond what
    /// temperature explains.
    mutating func elapse(_ dt: Float) {
        p00 += biasWanderPerSecond * dt
        p11 += slopeWanderPerSecond * dt
    }

    /// Folds in a bias `measured` at `temperature`, with the given standard
    /// deviation.
    mutating func measure(_ measured: SIMD3<Float>, deviation: Float, temperature: Float?) {
        let d = temperatureOffset(temperature)
        let innovation = measured - model.bias(at: temperature)
        let a = p00 + d * p01  // P·Hᵀ, first row
        let b = p01 + d * p11  // P·Hᵀ, second row
        let s = a + d * b + deviation * deviation
        let gainReference = a / s
        let gainSlope = b / s
        model.reference += gainReference * innovation
        model.slope += gainSlope * innovation
        p00 -= gainReference * a
        p01 -= gainReference * b
        p11 -= gainSlope * b
        clamp()
    }

    /// Moves the bias by `delta` at every temperature.
    mutating func shift(by delta: SIMD3<Float>) {
        model.reference += delta
        clamp()
    }

    private mutating func clamp() {
        model.reference = model.reference.clamped(
            lowerBound: .init(repeating: -maxBias), upperBound: .init(repeating: maxBias))
        model.slope = model.slope.clamped(
            lowerBound: .init(repeating: -maxBiasSlope), upperBound: .init(repeating: maxBiasSlope))
    }
}

/// Estimates the gyro's zero-rate offset. The DCM filter can only learn bias
/// on axes that gravity makes observable; bias about the vertical axis would
/// otherwise integrate straight into yaw drift. The offset shifts as the
/// glasses warm up, so it is learned against the IMU's temperature.
public struct GyroBiasEstimator: Sendable {
    private var filter: ThermalBiasFilter
    /// Smoothed IMU temperature, °C; nil until the glasses report one.
    public private(set) var temperature: Float?
    private var stillFor: Float = 0
    private var window = Window()
    private var sinceRevision: Float = 0
    private var calibration: Calibration?
    private var calibrationResult = CalibrationState.idle
    public private(set) var biasRevision: UInt32 = 0
    /// Counts bias measurements taken from still or calm stretches.
    public private(set) var learnedWindows: UInt32 = 0

    /// Samples since the last measurement, while the head stayed calm.
    private struct Window {
        var elapsed: Float = 0
        var sum = SIMD3<Double>.zero
        var count: UInt32 = 0
        var stillThroughout = true

        var mean: SIMD3<Float> {
            SIMD3<Float>(sum / Double(max(count, 1)))
        }
    }

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

    /// `bias` and `slope` as saved by an earlier run, or zero for none.
    public init(bias: SIMD3<Float>, slope: SIMD3<Float> = .zero) {
        filter = ThermalBiasFilter(
            ThermalBias(reference: bias, slope: slope),
            deviation: bias == .zero ? initialBiasDeviation : savedBiasDeviation)
    }

    /// The bias at the current temperature, rad/s.
    public var bias: SIMD3<Float> {
        filter.model.bias(at: temperature)
    }

    /// The learned bias and how it changes with temperature, to keep
    /// between runs.
    public var thermalBias: ThermalBias {
        filter.model
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
        filter.shift(by: rate * up)
        biasRevision += 1
    }

    /// Returns the bias-corrected gyro reading and updates the estimate.
    /// `temperature` is the IMU's, in °C, when the glasses report it.
    public mutating func correct(gyro: SIMD3<Float>, dt: Float, temperature: Float? = nil) -> SIMD3<Float> {
        updateTemperature(temperature, dt: dt)
        filter.elapse(dt)
        let corrected = gyro - bias
        if calibration != nil {
            updateCalibration(gyro: gyro, dt: dt)
        } else {
            updateAuto(gyro: gyro, corrected: corrected, dt: dt)
        }
        return corrected
    }

    private mutating func updateTemperature(_ reading: Float?, dt: Float) {
        guard let reading else { return }
        guard let current = temperature else {
            temperature = reading
            return
        }
        temperature = current + (reading - current) * dt / (temperatureTimeConstant + dt)
    }

    /// Learns from stretches without real head motion. Lying still, a
    /// second of samples measures the bias well. Worn, the head is never
    /// that still, but over a few calm seconds its small movements average
    /// out and what is left is bias; those measurements count for less.
    private mutating func updateAuto(gyro: SIMD3<Float>, corrected: SIMD3<Float>, dt: Float) {
        let speed = magnitude(corrected)
        stillFor = speed < autoMotionThreshold ? stillFor + dt : 0
        sinceRevision += dt
        if speed >= calmMotionThreshold {
            window = Window()
            return
        }
        window.elapsed += dt
        window.sum += SIMD3<Double>(gyro)
        window.count += 1
        window.stillThroughout = window.stillThroughout && isStill

        if window.stillThroughout && window.elapsed >= stillWindowSeconds {
            learn(window.mean, deviation: stillMeasurementDeviation)
        } else if window.elapsed >= calmWindowSeconds {
            let mean = window.mean
            // A steady slow turn would read as bias; leave it out.
            if magnitude(mean - bias) < calmMaxMeanRate {
                learn(mean, deviation: calmMeasurementDeviation)
            } else {
                window = Window()
            }
        }
    }

    private mutating func learn(_ measured: SIMD3<Float>, deviation: Float) {
        filter.measure(measured, deviation: deviation, temperature: temperature)
        window = Window()
        learnedWindows += 1
        if sinceRevision >= learnedSaveInterval {
            biasRevision += 1
            sinceRevision = 0
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
            filter.measure(
                SIMD3<Float>(calibration.mean), deviation: calibrationMeasurementDeviation,
                temperature: temperature)
            self.calibration = nil
            calibrationResult = .succeeded
            biasRevision += 1
            stillFor = 0
            window = Window()
        } else if calibration.elapsed >= calibrationTimeoutSeconds {
            self.calibration = nil
            calibrationResult = .failed
        } else {
            self.calibration = calibration
        }
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
