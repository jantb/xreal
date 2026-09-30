import AppKit
import Darwin
import XrealCore

// The rate the Mac's own screen had before it was held at 60 Hz, kept on
// disk so it can be given back even after a crash, at the next start.
private let heldRateKey = "laptopScreenRateBeforeHolding"
// The Mac's own screen's ID while the viewer has it switched off, kept on
// disk so the watchdog, or the next start, can switch it back on.
private let turnedOffKey = "laptopScreenTurnedOff"

/// macOS's private switch for a display, which display utilities use: there
/// is no public way to turn one off. Nil if macOS no longer has it.
private let configureDisplayEnabled:
    (@convention(c) (CGDisplayConfigRef?, CGDirectDisplayID, Bool) -> CGError)? = {
        let handle = dlopen(nil, RTLD_NOW)
        guard let symbol = dlsym(handle, "CGSConfigureDisplayEnabled") ?? dlsym(handle, "SLSConfigureDisplayEnabled")
        else { return nil }
        return unsafeBitCast(
            symbol, to: (@convention(c) (CGDisplayConfigRef?, CGDirectDisplayID, Bool) -> CGError).self)
    }()

/// Switches the Mac's own screen off while the glasses are in use, or, if
/// that is not wanted or not possible, holds it at a steady 60 Hz, and gives
/// it back as it was after. Alongside the canvas's virtual display, the
/// Mac's screen makes the glasses drop frames every few seconds: at 60 Hz
/// about half as often, switched off not at all.
@MainActor final class LaptopScreen {
    private let defaults = UserDefaults.standard
    /// Set once the viewer quits: nothing holds the screen after that, not
    /// even a display change arriving while its rate is being given back.
    private var released = false

    /// Gives the screen back as it was for good, as the viewer quits.
    func release() {
        released = true
        turnBackOn()
        hold(steady: false)
    }

    /// Switches the Mac's screen off when `off`, and otherwise switches it
    /// back on and holds it at 60 Hz when `steady`. Call again when displays
    /// change.
    func update(off: Bool, steady: Bool) {
        if off, !released, configureDisplayEnabled != nil {
            guard defaults.object(forKey: turnedOffKey) == nil,
                let display = Displays.active().first(where: { CGDisplayIsBuiltin($0) != 0 })
            else { return }
            // Back in its own mode first, so that is how it comes back on.
            hold(steady: false)
            // Noted before it goes, so a crash in between still brings it back.
            defaults.set(Int(display), forKey: turnedOffKey)
            if setEnabled(false, display) {
                eprint("Switched the laptop screen off")
            } else {
                defaults.removeObject(forKey: turnedOffKey)
                hold(steady: steady)
            }
            return
        }
        turnBackOn()
        hold(steady: steady)
    }

    /// Switches the Mac's screen back on, if the viewer switched it off.
    func turnBackOn() {
        guard let display = defaults.object(forKey: turnedOffKey) as? Int else { return }
        if setEnabled(true, CGDirectDisplayID(display)) {
            defaults.removeObject(forKey: turnedOffKey)
            eprint("Switched the laptop screen back on")
        }
    }

    /// Everything as it was before the viewer, for the watchdog once the
    /// viewer has gone: the screen on, then its own rate once it is back.
    func recover() {
        released = true
        turnBackOn()
        let deadline = monotonicNow() + 3
        while monotonicNow() < deadline, !Displays.active().contains(where: { CGDisplayIsBuiltin($0) != 0 }) {
            RunLoop.current.run(until: Date(timeIntervalSinceNow: 0.1))
        }
        hold(steady: false)
    }

    private func setEnabled(_ enabled: Bool, _ display: CGDirectDisplayID) -> Bool {
        guard let configureDisplayEnabled else { return false }
        var config: CGDisplayConfigRef?
        guard CGBeginDisplayConfiguration(&config) == .success, let config else { return false }
        guard configureDisplayEnabled(config, display, enabled) == .success else {
            CGCancelDisplayConfiguration(config)
            return false
        }
        // For this login only: logging out always brings it back.
        let result = CGCompleteDisplayConfiguration(config, .forSession)
        if result != .success {
            eprint("Could not switch the laptop screen \(enabled ? "on" : "off"): \(result.rawValue)")
        }
        return result == .success
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
