import AppKit
import XrealCore

@MainActor final class AppDelegate: NSObject, NSApplicationDelegate {
    private var viewer: Viewer?
    private var menu: StatusMenu?
    private var activity: NSObjectProtocol?

    func applicationDidFinishLaunching(_ notification: Notification) {
        let iconURL = Bundle.main.url(forResource: "AppIcon", withExtension: "icns")
            ?? Bundle.module.url(forResource: "AppIcon", withExtension: "png")
        if let iconURL, let icon = NSImage(contentsOf: iconURL) {
            NSApp.applicationIconImage = icon
        }
        if !CGPreflightScreenCaptureAccess() {
            CGRequestScreenCaptureAccess()
        }
        // Keeps macOS from napping or coalescing timers while other apps
        // have focus, which is most of the time.
        activity = ProcessInfo.processInfo.beginActivity(
            options: [.userInitiated, .latencyCritical], reason: "Rendering head-tracked video")
        do {
            let viewer = try Viewer(settings: Settings.load())
            self.viewer = viewer
            menu = StatusMenu(viewer: viewer)
        } catch {
            eprint("Failed to start: \(error)")
            NSApp.terminate(nil)
        }
    }

    func applicationWillTerminate(_ notification: Notification) {
        viewer?.saveSettings()
    }

    // The menu bar icon stays to quit from or to bring the window back when
    // the glasses connect.
    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool {
        false
    }
}

/// `--probe`: prints what the glasses report for a few seconds, without
/// opening a window. Useful to check the USB side on its own.
func probe() {
    for screen in NSScreen.screens {
        let id = screen.displayID ?? 0
        print(
            String(
                format: "display %u %@ %.0fx%.0f vendor 0x%04x model 0x%04x%@", id, screen.localizedName,
                screen.frame.width, screen.frame.height, CGDisplayVendorNumber(id), CGDisplayModelNumber(id),
                Displays.isGlasses(id) ? " (glasses)" : ""))
    }
    let tracking = Tracking(initialBias: Settings.load().gyroBias)
    for _ in 0..<16 {
        Thread.sleep(forTimeInterval: 0.25)
        let snapshot = tracking.snapshot()
        print(
            String(
                format: "%@ %.0f Hz  yaw %+.3f  pitch %+.3f  %@",
                snapshot.status == .connected ? "connected" : "searching", snapshot.sampleRateHz,
                snapshot.pose.yaw, snapshot.pose.pitch, snapshot.still ? "still" : "moving"))
    }
}

if CommandLine.arguments.contains("--probe") {
    probe()
    exit(0)
}

let delegate = AppDelegate()
NSApplication.shared.delegate = delegate
// Lives in the menu bar only: no Dock icon and not in the app switcher.
NSApplication.shared.setActivationPolicy(.accessory)
NSApplication.shared.run()
