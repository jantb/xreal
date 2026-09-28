import Foundation
import Synchronization
import os

private let biasLog = Logger(subsystem: "dev.jantb.xreal.viewer", category: "bias")
// How often the bias log records temperature and bias while connected.
private let biasLogInterval = 30.0  // seconds

private let readErrorsBeforeReconnect = 4

/// Reads the glasses on a dedicated high-priority thread and publishes the
/// latest head pose. Reconnects on its own when the glasses are unplugged.
public final class Tracking: Sendable {
    private enum Command {
        case calibrate
        case correctYawDrift(Float)
        /// Sets the latest `displayMode` on the glasses.
        case applyDisplayMode
    }

    private let shared: Mutex<TrackingSnapshot>
    private let commands = Mutex<[Command]>([])
    /// Set on the glasses whenever they connect.
    private let displayMode: Mutex<DisplayMode>
    /// Counts display modes set on the glasses, to wait for one.
    private let modesApplied = Mutex<UInt64>(0)

    /// `initialBias` is the gyro bias at the reference temperature and
    /// `biasSlope` how it changes per °C, as saved by an earlier run.
    public init(
        initialBias: SIMD3<Float>, biasSlope: SIMD3<Float> = .zero, displayMode: DisplayMode = .highRefreshRate
    ) {
        shared = Mutex(TrackingSnapshot(gyroBias: initialBias, biasSlope: biasSlope))
        self.displayMode = Mutex(displayMode)
        let thread = Thread { [self] in run(initialBias: initialBias, biasSlope: biasSlope) }
        thread.name = "Glasses tracking"
        thread.qualityOfService = .userInteractive
        thread.start()
    }

    public func snapshot() -> TrackingSnapshot {
        shared.withLock { $0 }
    }

    /// Starts measuring gyro bias; the glasses should lie still meanwhile.
    public func calibrate() {
        commands.withLock { $0.append(.calibrate) }
    }

    /// Removes `rate` (rad/s) of yaw drift from the bias estimate.
    public func correctYawDrift(_ rate: Float) {
        commands.withLock { $0.append(.correctYawDrift(rate)) }
    }

    /// Switches the glasses to `mode`, now and whenever they reconnect.
    public func setDisplayMode(_ mode: DisplayMode) {
        let changed = displayMode.withLock { current in
            defer { current = mode }
            return current != mode
        }
        if changed {
            commands.withLock { $0.append(.applyDisplayMode) }
        }
    }

    /// Switches the glasses back to their own picture on both eyes and waits
    /// up to `timeout` seconds for it, so they are not left side by side
    /// once the viewer quits.
    public func restoreDisplayMode(timeout: Double = 1) {
        let before = modesApplied.withLock { $0 }
        displayMode.withLock { $0 = .highRefreshRate }
        commands.withLock { $0.append(.applyDisplayMode) }
        let deadline = monotonicNow() + timeout
        while monotonicNow() < deadline, modesApplied.withLock({ $0 }) == before {
            Thread.sleep(forTimeInterval: 0.01)
        }
    }

    private func run(initialBias: SIMD3<Float>, biasSlope: SIMD3<Float>) {
        var bias = GyroBiasEstimator(bias: initialBias, slope: biasSlope)
        var session: UInt64 = 0
        var reportedMissing = false

        while true {
            shared.withLock { $0.status = .searching }
            let glasses: NrealAir
            do {
                glasses = try NrealAir()
            } catch {
                if !reportedMissing {
                    eprint("Glasses not available (\(error)), retrying")
                    reportedMissing = true
                }
                Thread.sleep(forTimeInterval: 1)
                continue
            }
            reportedMissing = false
            let mode = displayMode.withLock { $0 }
            do {
                try glasses.setDisplayMode(mode)
            } catch {
                eprint("Failed to set display mode \(mode): \(error)")
            }
            eprint("Connected to \(glasses.name)")
            let display = DisplayCalibration.parse(config: glasses.config)
            if display == nil {
                eprint("No display calibration on the glasses; using the nominal optics")
            }
            shared.withLock { $0.display = display }

            session += 1
            track(glasses, session: session, bias: &bias)
            shared.withLock { $0.status = .searching }
            Thread.sleep(forTimeInterval: 0.5)
        }
    }

    /// Runs until the glasses are lost.
    private func track(_ glasses: NrealAir, session: UInt64, bias: inout GyroBiasEstimator) {
        var fusion = Fusion()
        var rate = SampleRate(now: monotonicNow())
        var readErrors = 0
        var loggedAt = -Double.infinity

        while true {
            for command in commands.withLock({ pending in defer { pending = [] }; return pending }) {
                switch command {
                case .calibrate: bias.startCalibration()
                case .correctYawDrift(let rate): bias.correctYawDrift(rate: rate, up: fusion.up)
                case .applyDisplayMode:
                    let mode = displayMode.withLock { $0 }
                    do {
                        try glasses.setDisplayMode(mode)
                    } catch {
                        eprint("Failed to set display mode \(mode): \(error)")
                    }
                    modesApplied.withLock { $0 += 1 }
                }
            }

            let event: GlassesEvent
            do {
                event = try glasses.readEvent()
            } catch {
                readErrors += 1
                let deviceGone: Bool
                switch error as? GlassesError {
                case .deviceGone, .io: deviceGone = true
                default: deviceGone = false
                }
                if deviceGone || readErrors >= readErrorsBeforeReconnect {
                    eprint("Lost glasses (\(error)), reconnecting")
                    return
                }
                continue
            }

            guard case .accGyro(let sample) = event else { continue }
            readErrors = 0
            guard
                let update = fusion.push(
                    gyro: sample.gyroscope, acc: sample.accelerometer, timestamp: sample.timestamp,
                    temperature: sample.temperature, bias: &bias)
            else { continue }
            let now = monotonicNow()
            let sampleRateHz = rate.tick(now: now)
            let bias = bias
            shared.withLock { snapshot in
                snapshot.status = .connected
                snapshot.session = session
                snapshot.pose = update.pose
                snapshot.yawRate = update.yawRate
                snapshot.pitchRate = update.pitchRate
                snapshot.rollRate = update.rollRate
                snapshot.sampledAt = now
                snapshot.sampleRateHz = sampleRateHz
                snapshot.gyroBias = bias.bias
                snapshot.thermalBias = bias.thermalBias
                snapshot.temperature = bias.temperature
                snapshot.learnedWindows = bias.learnedWindows
                snapshot.still = bias.isStill
                snapshot.calibration = bias.calibrationState
                snapshot.biasRevision = bias.biasRevision
            }
            if now - loggedAt >= biasLogInterval {
                loggedAt = now
                logBias(bias, yaw: update.pose.yaw)
            }
        }
    }
}

/// Records temperature, bias and yaw together, to tell drift that follows
/// the glasses warming up from drift that follows head motion. Read with
/// `log stream --predicate 'subsystem == "dev.jantb.xreal.viewer" AND category == "bias"'`.
private func logBias(_ bias: GyroBiasEstimator, yaw: Float) {
    let degreesPerMinute = { (rate: Float) in rate * 180 / .pi * 60 }
    let current = bias.bias
    let slope = bias.thermalBias.slope
    let temperature = bias.temperature.map { String(format: "%.2f", $0) } ?? "-"
    biasLog.notice(
        """
        temp \(temperature, privacy: .public) C  \
        bias \(String(format: "%+.5f %+.5f %+.5f", current.x, current.y, current.z), privacy: .public) rad/s \
        (\(String(format: "%+.2f", degreesPerMinute(current.y)), privacy: .public) deg/min about y)  \
        slope \(String(format: "%+.6f %+.6f %+.6f", slope.x, slope.y, slope.z), privacy: .public) rad/s/C  \
        learned \(bias.learnedWindows, privacy: .public)  \
        still \(bias.isStill, privacy: .public)  \
        yaw \(String(format: "%+.3f", yaw), privacy: .public)
        """)
}

/// Writes a line to stderr, like Rust's `eprintln!`.
public func eprint(_ message: String) {
    FileHandle.standardError.write(Data((message + "\n").utf8))
}
