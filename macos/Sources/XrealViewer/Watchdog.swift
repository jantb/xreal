import Foundation
import XrealCore

// The viewer's process ID while it runs, cleared as it quits cleanly: still
// set when it has gone means it died without cleaning up.
private let runningKey = "viewerRunning"

/// A second copy of the viewer, started with `--watchdog PID`, that waits for
/// the viewer to go and puts back what it left changed: the Mac's own
/// screen on and at its own rate and, if the viewer died without quitting,
/// the glasses showing their own picture.
enum Watchdog {
    /// Starts the watchdog for this process, in a session of its own so it
    /// outlives the viewer however it ends.
    static func start() {
        let own = ProcessInfo.processInfo.processIdentifier
        UserDefaults.standard.set(Int(own), forKey: runningKey)
        guard let executable = Bundle.main.executablePath else { return }
        var attributes: posix_spawnattr_t?
        posix_spawnattr_init(&attributes)
        defer { posix_spawnattr_destroy(&attributes) }
        // A session of its own, and none of the viewer's open files.
        posix_spawnattr_setflags(&attributes, Int16(POSIX_SPAWN_SETSID | POSIX_SPAWN_CLOEXEC_DEFAULT))
        let arguments = [executable, "--watchdog", String(own)]
        var argv = arguments.map { strdup($0) } + [nil]
        defer { argv.forEach { free($0) } }
        var child: pid_t = 0
        let result = posix_spawn(&child, executable, nil, &attributes, &argv, environ)
        if result != 0 {
            eprint("Could not start the watchdog: \(result)")
        }
    }

    /// Notes that the viewer quit cleanly, having put everything back itself.
    static func quitCleanly() {
        UserDefaults.standard.removeObject(forKey: runningKey)
    }

    /// `--watchdog PID`: waits for process `viewer` to exit, then puts back
    /// what it left.
    @MainActor static func run(watching viewer: pid_t) -> Never {
        let queue = kqueue()
        var watch = kevent(
            ident: UInt(viewer), filter: Int16(EVFILT_PROC), flags: UInt16(EV_ADD | EV_ONESHOT), fflags: NOTE_EXIT,
            data: 0, udata: nil)
        // Fails at once when the viewer has already gone. The wait goes on
        // until the viewer exits, whatever interrupts it.
        if queue >= 0, kevent(queue, &watch, 1, nil, 0, nil) == 0 {
            var event = kevent()
            while kevent(queue, nil, 0, &event, 1, nil) < 0 && errno == EINTR {}
        }
        let died = UserDefaults.standard.integer(forKey: runningKey) == Int(viewer)
        LaptopScreen().release()
        if died {
            UserDefaults.standard.removeObject(forKey: runningKey)
            // The glasses would otherwise stay side by side until replugged.
            if (try? NrealAir())?.setDisplayModeIgnoringErrors(.highRefreshRate) == true {
                eprint("The viewer died; put the glasses back to their own picture")
            }
        }
        exit(0)
    }
}

extension NrealAir {
    /// Sets `mode`, and says whether that worked.
    fileprivate func setDisplayModeIgnoringErrors(_ mode: DisplayMode) -> Bool {
        (try? setDisplayMode(mode)) != nil
    }
}
