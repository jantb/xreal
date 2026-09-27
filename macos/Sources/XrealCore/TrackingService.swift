import Foundation
import Synchronization

private let readErrorsBeforeReconnect = 4

/// Reads the glasses on a dedicated high-priority thread and publishes the
/// latest head pose. Reconnects on its own when the glasses are unplugged.
public final class Tracking: Sendable {
    private enum Command {
        case calibrate
        case correctYawDrift(Float)
    }

    private let shared: Mutex<TrackingSnapshot>
    private let commands = Mutex<[Command]>([])

    public init(initialBias: SIMD3<Float>) {
        shared = Mutex(TrackingSnapshot(gyroBias: initialBias))
        let thread = Thread { [self] in run(initialBias: initialBias) }
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

    private func run(initialBias: SIMD3<Float>) {
        var bias = GyroBiasEstimator(bias: initialBias)
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
            do {
                try glasses.setDisplayMode(.highRefreshRate)
            } catch {
                eprint("Failed to set high refresh rate display mode: \(error)")
            }
            eprint("Connected to \(glasses.name)")

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

        while true {
            for command in commands.withLock({ pending in defer { pending = [] }; return pending }) {
                switch command {
                case .calibrate: bias.startCalibration()
                case .correctYawDrift(let rate): bias.correctYawDrift(rate: rate, up: fusion.up)
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
                    bias: &bias)
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
                snapshot.still = bias.isStill
                snapshot.calibration = bias.calibrationState
                snapshot.biasRevision = bias.biasRevision
            }
        }
    }
}

/// Writes a line to stderr, like Rust's `eprintln!`.
public func eprint(_ message: String) {
    FileHandle.standardError.write(Data((message + "\n").utf8))
}
