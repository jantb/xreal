import Foundation

public struct RenderStats: Sendable {
    private var frames: UInt32 = 0
    private var capturedSince: UInt64 = 0
    private var lastSample: Double
    public private(set) var fps: Float = 0
    public private(set) var captureFps: Float = 0

    public init(now: Double) {
        lastSample = now
    }

    public mutating func tick(now: Double, captureGeneration: UInt64) {
        frames += 1
        let elapsed = now - lastSample
        if elapsed >= 0.5 {
            fps = Float(Double(frames) / elapsed)
            captureFps = Float(Double(captureGeneration &- capturedSince) / elapsed)
            frames = 0
            capturedSince = captureGeneration
            lastSample = now
        }
    }
}

public struct HudInfo {
    /// The pixel of the canvas looked at, nil when looking away from it.
    public var gaze: SIMD2<Float>?
    public var source: (width: Int, height: Int)?
    /// What the glasses show, e.g. "Canvas 5752×2160".
    public var sourceDescription: String
    public var output: (width: Int, height: Int)
    public var newFrame: Bool
    public var stats: RenderStats
    public var tracking: TrackingSnapshot
    public var pose: HeadPose
    public var prediction: Bool
    public var lastDrift: DriftObservation?
    public var now: Double

    public init(
        gaze: SIMD2<Float>?, source: (width: Int, height: Int)?, sourceDescription: String,
        output: (width: Int, height: Int), newFrame: Bool, stats: RenderStats, tracking: TrackingSnapshot,
        pose: HeadPose, prediction: Bool, lastDrift: DriftObservation?, now: Double
    ) {
        self.gaze = gaze
        self.source = source
        self.sourceDescription = sourceDescription
        self.output = output
        self.newFrame = newFrame
        self.stats = stats
        self.tracking = tracking
        self.pose = pose
        self.prediction = prediction
        self.lastDrift = lastDrift
        self.now = now
    }
}

public func hudLines(_ info: HudInfo) -> [String] {
    let frameState = info.newFrame ? "New frame" : "Held frame"
    let tracking = info.tracking

    let imu: String
    switch tracking.status {
    case .connected:
        let ageMs = tracking.sampledAt.map { (info.now - $0) * 1000 } ?? 0
        imu = String(format: "IMU ok, %.0f Hz, sample age %.0f ms", tracking.sampleRateHz, ageMs)
    case .searching:
        imu = "IMU: no glasses, retrying"
    }
    let calibration: String
    switch tracking.calibration {
    case .idle: calibration = ""
    case .running(let progress): calibration = String(format: "  Calibrating %.0f%%, keep still", progress * 100)
    case .succeeded: calibration = "  Calibrated"
    case .failed: calibration = "  Calibration failed: the glasses moved"
    }
    let degreesPerMinute = { (rate: Float) in rate * 180 / .pi * 60 }
    let drift: String
    switch info.lastDrift {
    case nil: drift = "Recentering teaches drift"
    case .anchored: drift = "Drift reference set"
    case .tooSoon: drift = "Drift: wait 20 s between recenters"
    case .learned(let measured, _):
        drift = String(format: "Drift %+.2f°/min, corrected", degreesPerMinute(measured))
    case .rejected(let measured):
        drift = String(format: "Drift %+.1f°/min, ignored as a turn", degreesPerMinute(measured))
    }

    let source = info.source.map { "Source \($0.width)×\($0.height)" } ?? "Source: no capture yet"
    let view = info.gaze.map { String(format: "Looking at %.0f, %.0f", $0.x, $0.y) } ?? "Looking away from the canvas"
    let bias = tracking.gyroBias
    let temperature = tracking.temperature.map { String(format: "  Temp %.1f °C", $0) } ?? ""

    return [
        String(format: "%@, %.0f fps, capture %.0f fps", frameState, info.stats.fps, info.stats.captureFps),
        imu,
        String(format: "Bias %.4f %.4f %.4f", bias.x, bias.y, bias.z) + temperature
            + "  Learned \(tracking.learnedWindows)  " + (tracking.still ? "still" : "moving") + calibration,
        drift,
        info.sourceDescription,
        "\(source)  Output \(info.output.width)×\(info.output.height)",
        view,
        String(
            format: "Yaw %.3f  Pitch %.3f  Roll %.3f  Prediction %@", info.pose.yaw, info.pose.pitch, info.pose.roll,
            info.prediction ? "on" : "off"),
    ]
}
