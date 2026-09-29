import AppKit
import XrealCore

// The rate the Mac's own screen had before it was held at 60 Hz, kept on
// disk so it can be given back even after a crash, at the next start.
private let heldRateKey = "laptopScreenRateBeforeHolding"

/// Holds the Mac's own screen at a steady 60 Hz while the glasses are in use,
/// and gives it back its own rate after. A ProMotion screen changes its rate
/// as it likes; alongside the canvas's virtual display that makes the glasses
/// drop frames every few seconds, and at a fixed 60 Hz, which divides evenly
/// into their 90, about half as often.
@MainActor final class LaptopScreen {
    private let defaults = UserDefaults.standard
    /// Set once the viewer quits: nothing holds the screen after that, not
    /// even a display change arriving while its rate is being given back.
    private var released = false

    /// Gives the screen back its own rate for good, as the viewer quits.
    func release() {
        released = true
        hold(steady: false)
    }

    /// Holds the built-in screen at 60 Hz when `steady`, or gives it back
    /// its own rate. Does nothing without a built-in screen in use, as with
    /// the lid closed; call again when displays change.
    func hold(steady: Bool) {
        guard let display = Displays.active().first(where: { CGDisplayIsBuiltin($0) != 0 }),
            let current = CGDisplayCopyDisplayMode(display)
        else { return }
        let heldFrom = defaults.object(forKey: heldRateKey) as? Double
        if steady && !released {
            guard abs(current.refreshRate - 60) > 0.5, let steadyMode = Self.mode(like: current, at: 60, on: display)
            else { return }
            if heldFrom == nil {
                defaults.set(current.refreshRate, forKey: heldRateKey)
            }
            if apply(steadyMode, to: display) {
                eprint("Holding the laptop screen at 60 Hz")
            }
        } else if !steady || released, let heldFrom {
            // Its own mode at the same size, wherever it went meanwhile, as
            // when the lid was closed and opened.
            guard let own = Self.mode(like: current, at: heldFrom, on: display), apply(own, to: display) else { return }
            // The change takes a moment, and is lost if the viewer quits
            // before it lands: wait for it, but not for long.
            let deadline = monotonicNow() + 1.5
            while monotonicNow() < deadline,
                abs((CGDisplayCopyDisplayMode(display)?.refreshRate ?? heldFrom) - heldFrom) > 0.5
            {
                RunLoop.current.run(until: Date(timeIntervalSinceNow: 0.05))
            }
            if abs((CGDisplayCopyDisplayMode(display)?.refreshRate ?? 0) - heldFrom) <= 0.5 {
                defaults.removeObject(forKey: heldRateKey)
                eprint("Gave the laptop screen back its own refresh rate")
            } else {
                eprint("The laptop screen did not take back its own refresh rate yet")
            }
        }
    }

    /// The mode of `display` with `current`'s size and density at `rate`.
    private static func mode(like current: CGDisplayMode, at rate: Double, on display: CGDirectDisplayID) -> CGDisplayMode? {
        let options = [kCGDisplayShowDuplicateLowResolutionModes: true] as CFDictionary
        let modes = CGDisplayCopyAllDisplayModes(display, options) as? [CGDisplayMode] ?? []
        return modes.first {
            $0.width == current.width && $0.height == current.height && $0.pixelWidth == current.pixelWidth
                && $0.pixelHeight == current.pixelHeight && abs($0.refreshRate - rate) < 0.01
        }
    }

    /// Sets `mode` on `display` until logout, at the latest.
    private func apply(_ mode: CGDisplayMode, to display: CGDirectDisplayID) -> Bool {
        var config: CGDisplayConfigRef?
        guard CGBeginDisplayConfiguration(&config) == .success, let config else { return false }
        CGConfigureDisplayWithDisplayMode(config, display, mode, nil)
        let result = CGCompleteDisplayConfiguration(config, .forSession)
        if result != .success {
            eprint("Could not change the laptop screen's refresh rate: \(result.rawValue)")
        }
        return result == .success
    }
}
