import AppKit
import Darwin
import XrealCore

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

/// Switches the Mac's own screen off while the glasses are in use, and back
/// on after. Alongside the canvas's virtual display, the Mac's screen makes
/// the glasses drop frames every few seconds; switched off, not at all.
@MainActor final class LaptopScreen {
    private let defaults = UserDefaults.standard
    /// Set once the viewer quits: nothing switches the screen off after
    /// that, not even a display change arriving meanwhile.
    private var released = false

    /// Gives the screen back for good, as the viewer quits, or from the
    /// watchdog once the viewer has gone.
    func release() {
        released = true
        turnBackOn()
    }

    /// Switches the Mac's screen off when `off`, and otherwise back on, if
    /// the viewer switched it off. Call again when displays change.
    func update(off: Bool) {
        guard off, !released, configureDisplayEnabled != nil else {
            turnBackOn()
            return
        }
        guard defaults.object(forKey: turnedOffKey) == nil,
            let display = Displays.active().first(where: { CGDisplayIsBuiltin($0) != 0 })
        else { return }
        // Noted before it goes, so a crash in between still brings it back.
        defaults.set(Int(display), forKey: turnedOffKey)
        if setEnabled(false, display) {
            eprint("Switched the laptop screen off")
        } else {
            defaults.removeObject(forKey: turnedOffKey)
        }
    }

    /// Switches the Mac's screen back on, if the viewer switched it off.
    private func turnBackOn() {
        guard let display = defaults.object(forKey: turnedOffKey) as? Int else { return }
        if setEnabled(true, CGDirectDisplayID(display)) {
            defaults.removeObject(forKey: turnedOffKey)
            eprint("Switched the laptop screen back on")
        }
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
}
