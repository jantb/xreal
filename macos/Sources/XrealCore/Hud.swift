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
    public var viewport: ViewportController
    public var geometry: ViewGeometry?
    /// Extra zoom-out while following the cursor, 1 meaning none.
    public var followScale: Float
    public var source: (width: Int, height: Int)?
    /// Where the picture comes from, e.g. "MIRROR MAIN DISPLAY".
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
        viewport: ViewportController, geometry: ViewGeometry?, followScale: Float,
        source: (width: Int, height: Int)?,
        sourceDescription: String, output: (width: Int, height: Int), newFrame: Bool,
        stats: RenderStats, tracking: TrackingSnapshot, pose: HeadPose, prediction: Bool,
        lastDrift: DriftObservation?, now: Double
    ) {
        self.viewport = viewport
        self.geometry = geometry
        self.followScale = followScale
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
    let state = info.viewport.frozen ? "FROZEN" : "LIVE"
    let frameState = info.newFrame ? "NEW" : "HOLD"
    let tracking = info.tracking

    let imu: String
    switch tracking.status {
    case .connected:
        let ageMs = tracking.sampledAt.map { (info.now - $0) * 1000 } ?? 0
        imu = String(format: "IMU OK %.0fHZ  AGE %.0fMS", tracking.sampleRateHz, ageMs)
    case .searching:
        imu = "IMU NO GLASSES - RETRYING"
    }
    let calibration: String
    switch tracking.calibration {
    case .idle: calibration = ""
    case .running(let progress): calibration = String(format: "  CALIBRATING %.0f%% - KEEP STILL", progress * 100)
    case .succeeded: calibration = "  CALIBRATED"
    case .failed: calibration = "  CALIBRATION FAILED - MOVED"
    }
    let degreesPerMinute = { (rate: Float) in rate * 180 / .pi * 60 }
    let drift: String
    switch info.lastDrift {
    case nil: drift = "RECENTER TEACHES DRIFT"
    case .anchored: drift = "DRIFT REFERENCE SET"
    case .tooSoon: drift = "DRIFT: WAIT 20S BETWEEN RECENTERS"
    case .learned(let measured, _):
        drift = String(format: "DRIFT %+.2f DEG/MIN - CORRECTED", degreesPerMinute(measured))
    case .rejected(let measured):
        drift = String(format: "DRIFT %+.1f DEG/MIN - IGNORED AS TURN", degreesPerMinute(measured))
    }

    let source = info.source.map { "SRC \($0.width)X\($0.height)" } ?? "SRC NO CAPTURE YET"
    let view: String
    switch info.geometry {
    case nil: view = "VIEW -"
    case .crop(let rect):
        view = String(format: "VIEW CROP %.0f,%.0f %.0fX%.0f", rect.x, rect.y, rect.width, rect.height)
    case .spatial(let spatial):
        let middle = spatial.sourcePoint(atOutput: .zero)
        view =
            (spatial.curved ? "VIEW CURVED" : "VIEW FLAT")
            + (middle.map { String(format: " LOOKING AT %.0f,%.0f", $0.x, $0.y) } ?? " LOOKING AWAY")
    }
    let bias = tracking.gyroBias

    return [
        String(
            format: "%@ %@  %.0fFPS  CAPTURE %.0fFPS  ZOOM %.2fX", state, frameState, info.stats.fps,
            info.stats.captureFps, info.viewport.zoom * info.followScale),
        imu,
        String(format: "BIAS %.4f %.4f %.4f  ", bias.x, bias.y, bias.z)
            + (tracking.still ? "STILL" : "MOVING") + calibration,
        drift,
        info.sourceDescription,
        "\(source)  OUT \(info.output.width)X\(info.output.height)",
        view,
        String(
            format: "YAW %.3f  PITCH %.3f  ROLL %.3f", info.pose.yaw, info.pose.pitch, info.pose.roll),
        String(
            format: "SENS %.2fX  DEADZONE %.3f  PREDICT %@", info.viewport.sensitivity, info.viewport.deadzone,
            info.prediction ? "ON" : "OFF"),
    ]
}
