import AppKit
import Darwin
import IOKit
import XrealCore

// The Mac's own screen's ID while the viewer has it switched off, kept on
// disk so the watchdog, or the next start, can switch it back on.
private let turnedOffKey = "laptopScreenTurnedOff"
// How often switching the screen back on is tried, and how long each try
// waits for it to come on.
private let turnOnAttempts = 3
private let turnOnWait = 2.0  // seconds

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
    /// Checks that the screen really came back on, and asks again if not.
    private var turningOn: Task<Void, Never>?
    /// Set once switching it back on did not take, so the display changes
    /// each try causes do not start the tries over and over. Cleared when
    /// the glasses come back.
    private var gaveUp = false
    /// Set once the screen, found off where the canvas belongs, was switched
    /// on to be moved out of the way, so that happens once each time the
    /// glasses are put on.
    private var straightened = false

    /// Gives the screen back for good, as the viewer quits. The screen is
    /// asked to come back on once; the watchdog makes sure it did.
    func release() {
        released = true
        turningOn?.cancel()
        turningOn = nil
        guard let display = turnedOff, !isOn(display), CGDisplayIsOnline(display) != 0 else { return }
        _ = setEnabled(true, display)
    }

    /// From the watchdog once the viewer has gone: brings the screen back
    /// on, waiting for it and asking again until it is.
    func recover() {
        released = true
        for _ in 0..<turnOnAttempts {
            guard let display = turnedOff, !isOn(display), CGDisplayIsOnline(display) != 0 else { return }
            _ = setEnabled(true, display)
            let deadline = monotonicNow() + turnOnWait
            while monotonicNow() < deadline, !isOn(display) {
                RunLoop.current.run(until: Date(timeIntervalSinceNow: 0.1))
            }
        }
    }

    /// Switches the Mac's screen off when `off`, and otherwise back on, if
    /// the viewer switched it off. Call again when displays change.
    func update(off: Bool) {
        guard off, !released, configureDisplayEnabled != nil else {
            turnBackOn()
            return
        }
        // The glasses came back while the screen was coming on.
        turningOn?.cancel()
        turningOn = nil
        gaveUp = false
        if !straightened, let display = Self.builtInSwitchedOff(), CGDisplayIsMain(display) != 0 {
            // Switched off while it was the main display, it keeps the
            // place the canvas needs, and the menu bar and Dock with it,
            // where nothing shows them. On again for a moment, the canvas
            // is arranged above it, and it goes off after.
            straightened = true
            if setEnabled(true, display) {
                defaults.removeObject(forKey: turnedOffKey)
                eprint("The laptop screen was off in the canvas's place; switching it on to move it")
            }
            return
        }
        guard defaults.object(forKey: turnedOffKey) == nil else { return }
        if let display = Self.builtInSwitchedOff() {
            // Left off, as by a switch back on that never took: taken as
            // the viewer's, so it comes back on after.
            defaults.set(Int(display), forKey: turnedOffKey)
            eprint("The laptop screen was already off")
            return
        }
        guard let display = Displays.active().first(where: { CGDisplayIsBuiltin($0) != 0 }) else { return }
        // Noted before it goes, so a crash in between still brings it back.
        defaults.set(Int(display), forKey: turnedOffKey)
        if setEnabled(false, display) {
            eprint("Switched the laptop screen off")
        } else {
            defaults.removeObject(forKey: turnedOffKey)
        }
    }

    /// Switches the Mac's screen back on, if the viewer switched it off. It
    /// is only forgotten once it is really on: a switch reported done can
    /// still not take, as when the glasses go at the same moment.
    private func turnBackOn() {
        straightened = false
        guard turningOn == nil, !gaveUp else { return }
        if turnedOff == nil, let display = Self.builtInSwitchedOff() {
            // Left off with nothing noted, as by an earlier switch back on
            // that never took: without it there may be no screen at all.
            defaults.set(Int(display), forKey: turnedOffKey)
        }
        guard let display = turnedOff else { return }
        turningOn = Task { [weak self] in
            for _ in 0..<turnOnAttempts {
                guard let self else { return }
                // With the lid closed it cannot come on; it is tried again
                // when displays change.
                guard !isOn(display), CGDisplayIsOnline(display) != 0 else { break }
                _ = setEnabled(true, display)
                let deadline = monotonicNow() + turnOnWait
                while monotonicNow() < deadline, !isOn(display) {
                    // Cancelled when the glasses come back or the viewer
                    // quits, which take over from here.
                    do { try await Task.sleep(for: .milliseconds(100)) } catch { return }
                }
            }
            guard let self, !Task.isCancelled else { return }
            turningOn = nil
            if turnedOff == nil {
                eprint("Switched the laptop screen back on")
            } else if CGDisplayIsOnline(display) != 0 {
                gaveUp = true
                eprint("The laptop screen did not come back on")
            }
        }
    }

    /// The screen the viewer switched off, until it is on again.
    private var turnedOff: CGDirectDisplayID? {
        (defaults.object(forKey: turnedOffKey) as? Int).map { CGDirectDisplayID($0) }
    }

    /// Whether `display` is on, forgetting it was switched off once it is.
    private func isOn(_ display: CGDirectDisplayID) -> Bool {
        guard CGDisplayIsActive(display) != 0 else { return false }
        defaults.removeObject(forKey: turnedOffKey)
        return true
    }

    /// The built-in screen, if it is there with the lid open but switched
    /// off, not merely asleep or mirroring another.
    private static func builtInSwitchedOff() -> CGDirectDisplayID? {
        guard !lidClosed() else { return nil }
        return Displays.online().first {
            CGDisplayIsBuiltin($0) != 0 && CGDisplayIsActive($0) == 0 && CGDisplayIsInMirrorSet($0) == 0
        }
    }

    private static func lidClosed() -> Bool {
        let root = IOServiceGetMatchingService(kIOMainPortDefault, IOServiceMatching("IOPMrootDomain"))
        guard root != 0 else { return false }
        defer { IOObjectRelease(root) }
        let state = IORegistryEntryCreateCFProperty(root, "AppleClamshellState" as CFString, kCFAllocatorDefault, 0)
        return (state?.takeRetainedValue() as? Bool) ?? false
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
