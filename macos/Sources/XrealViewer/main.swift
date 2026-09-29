import AppKit
import Metal
import XrealCore

@MainActor final class AppDelegate: NSObject, NSApplicationDelegate {
    private var viewer: Viewer?
    private var menu: StatusMenu?
    private var controls: ControlPanel?
    private var activity: NSObjectProtocol?
    private var signalSources: [DispatchSourceSignal] = []

    func applicationDidFinishLaunching(_ notification: Notification) {
        let iconURL = Bundle.main.url(forResource: "AppIcon", withExtension: "icns")
            ?? Bundle.module.url(forResource: "AppIcon", withExtension: "png")
        if let iconURL, let icon = NSImage(contentsOf: iconURL) {
            NSApp.applicationIconImage = icon
        }
        let canCapture = CGPreflightScreenCaptureAccess()
        if !canCapture {
            CGRequestScreenCaptureAccess()
        }
        NSApp.mainMenu = mainMenu()
        // A plain `kill` or Ctrl-C quits cleanly too, so the glasses are put
        // back to their own picture instead of being left side by side.
        signalSources = [SIGTERM, SIGINT, SIGHUP].map { number in
            signal(number, SIG_IGN)
            let source = DispatchSource.makeSignalSource(signal: number, queue: .main)
            source.setEventHandler { NSApp.terminate(nil) }
            source.resume()
            return source
        }
        // Puts the Mac's screen and the glasses back should the viewer die.
        Watchdog.start()
        // Keeps macOS from napping or coalescing timers while other apps
        // have focus, which is most of the time.
        activity = ProcessInfo.processInfo.beginActivity(
            options: [.userInitiated, .latencyCritical], reason: "Rendering head-tracked video")
        do {
            let viewer = try Viewer(settings: Settings.load())
            let controls = ControlPanel(viewer: viewer)
            self.viewer = viewer
            self.controls = controls
            menu = StatusMenu(viewer: viewer, controls: controls)
            // Without Screen Recording there is nothing to show; the
            // controls say what to do.
            if !canCapture {
                controls.show()
            }
        } catch {
            eprint("Failed to start: \(error)")
            NSApp.terminate(nil)
        }
    }

    /// Never shown, as the app has no menu bar of its own, but gives the
    /// controls window its standard keys.
    private func mainMenu() -> NSMenu {
        let app = NSMenu()
        app.addItem(withTitle: "Quit XREAL Viewer", action: #selector(NSApplication.terminate(_:)), keyEquivalent: "q")
        let window = NSMenu(title: "Window")
        window.addItem(withTitle: "Close", action: #selector(NSWindow.performClose(_:)), keyEquivalent: "w")
        window.addItem(withTitle: "Minimize", action: #selector(NSWindow.performMiniaturize(_:)), keyEquivalent: "m")
        let main = NSMenu()
        for submenu in [app, window] {
            let item = NSMenuItem()
            item.submenu = submenu
            main.addItem(item)
        }
        return main
    }

    func applicationWillTerminate(_ notification: Notification) {
        viewer?.saveSettings()
        viewer?.restoreGlasses()
        Watchdog.quitCleanly()
    }

    // The menu bar icon stays to quit from or to bring the window back when
    // the glasses connect.
    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool {
        false
    }
}

/// `--probe [seconds]`: prints what the glasses report for a few seconds,
/// without opening a window. Useful to check the USB side on its own.
func probe(seconds: Double) {
    for screen in NSScreen.screens {
        let id = screen.displayID ?? 0
        print(
            String(
                format: "display %u %@ %.0fx%.0f vendor 0x%04x model 0x%04x%@", id, screen.localizedName,
                screen.frame.width, screen.frame.height, CGDisplayVendorNumber(id), CGDisplayModelNumber(id),
                Displays.isGlasses(id) ? " (glasses)" : ""))
    }
    let settings = Settings.load()
    let tracking = Tracking(initialBias: settings.gyroBias, biasSlope: settings.gyroBiasSlope)
    for _ in 0..<max(Int(seconds * 4), 1) {
        Thread.sleep(forTimeInterval: 0.25)
        let snapshot = tracking.snapshot()
        print(
            String(
                format: "%@ %.0f Hz  yaw %+.3f  pitch %+.3f  roll %+.3f  %@",
                snapshot.status == .connected ? "connected" : "searching", snapshot.sampleRateHz,
                snapshot.pose.yaw, snapshot.pose.pitch, snapshot.pose.roll, snapshot.still ? "still" : "moving"))
    }
}

/// `--probe-sizes [--beyond-limit] [WxH[@2x] ...]`: creates a virtual screen
/// of each size in turn and prints the size macOS actually gives it, in
/// points and pixels. macOS refuses or shrinks some sizes, and which ones
/// changes between releases. `@2x` asks for a HiDPI screen of that many
/// points. `--beyond-limit` also tries sizes past `maxVirtualScreenSide`
/// pixels, which has panicked a Mac: save everything first.
@MainActor func probeSizes(_ arguments: [String]) {
    let beyondLimit = arguments.contains("--beyond-limit")
    let asked = arguments.compactMap { argument -> (width: Int, height: Int, scale: Int)? in
        let (size, scale) = argument.hasSuffix("@2x") ? (argument.dropLast(3), 2) : (Substring(argument), 1)
        let parts = size.split(separator: "x").compactMap { Int($0) }
        return parts.count == 2 ? (parts[0], parts[1], scale) : nil
    }
    // Without sizes named, every canvas size within the limit, never the
    // ones past it: several oversize displays in one run panicked a Mac.
    let listed = canvasSizes.filter { $0.width * $0.scale <= maxVirtualScreenSide && $0.height * $0.scale <= maxVirtualScreenSide }
    let sizes = asked.isEmpty ? listed.map { ($0.width, $0.height, $0.scale) } : asked
    for (width, height, scale) in sizes {
        let name = "\(width)x\(height)" + (scale == 2 ? "@2x" : "")
        guard
            let screen = VirtualScreen(
                index: 31, width: width, height: height, scale: scale, refreshRate: glassesRefreshRate,
                beyondLimit: beyondLimit)
        else {
            print("\(name): refused")
            continue
        }
        var got = (points: (0, 0), pixels: (0, 0))
        let deadline = monotonicNow() + 5
        while monotonicNow() < deadline {
            RunLoop.main.run(until: Date(timeIntervalSinceNow: 0.1))
            if let mode = CGDisplayCopyDisplayMode(screen.displayID), mode.pixelWidth > 0 {
                got = ((mode.width, mode.height), (mode.pixelWidth, mode.pixelHeight))
                break
            }
        }
        let expected = (points: (width, height), pixels: (width * scale, height * scale))
        let ok = got.points == expected.points && got.pixels == expected.pixels
        print(
            "\(name): \(ok ? "ok" : "came up as") \(got.points.0)x\(got.points.1) points, "
                + "\(got.pixels.0)x\(got.pixels.1) pixels")
        withExtendedLifetime(screen) {}
    }
}

/// `--probe-mode MODE`: switches the glasses to display mode `MODE` (see
/// `DisplayMode`), prints what macOS then reports for their display each
/// second, and switches back to the 120 Hz mode the glasses show on their
/// own.
func probeMode(_ arguments: [String]) {
    guard let raw = arguments.first.flatMap(UInt8.init), let mode = DisplayMode(rawValue: raw) else {
        print("usage: --probe-mode MODE, one of 1, 3, 8, 9, 11")
        return
    }
    func report() {
        guard let id = Displays.glassesDisplay(), let current = CGDisplayCopyDisplayMode(id) else {
            print("glasses display: not active")
            return
        }
        print(
            String(
                format: "glasses display %u: %dx%d pixels, %dx%d points, %.0f Hz", id, current.pixelWidth,
                current.pixelHeight, current.width, current.height, current.refreshRate))
    }
    do {
        let glasses = try NrealAir()
        report()
        try glasses.setDisplayMode(mode)
        print("set mode \(raw)")
        for _ in 0..<8 {
            Thread.sleep(forTimeInterval: 1)
            report()
        }
        try glasses.setDisplayMode(.highRefreshRate)
        print("back to mode \(DisplayMode.highRefreshRate.rawValue)")
        Thread.sleep(forTimeInterval: 3)
        report()
    } catch {
        print("failed: \(error)")
    }
}

/// `--record SECONDS FILE`: writes every IMU sample the glasses send for
/// `seconds` to `file` as CSV (device time in microseconds, gyro in rad/s,
/// accelerometer in m/s², temperature in °C), to study the raw signal, such
/// as what a heartbeat does to it. Quit the viewer first: it holds the glasses.
@discardableResult
func record(seconds: Double, path: String) -> Bool {
    var lines = ["time_us,gx,gy,gz,ax,ay,az,temp_c"]
    var problem: Error?
    do {
        let glasses = try NrealAir()
        let deadline = monotonicNow() + seconds
        while monotonicNow() < deadline {
            do {
                guard case .accGyro(let sample) = try glasses.readEvent() else { continue }
                let (g, a) = (sample.gyroscope, sample.accelerometer)
                lines.append(
                    "\(sample.timestamp),\(g.x),\(g.y),\(g.z),\(a.x),\(a.y),\(a.z),\(sample.temperature.map { "\($0)" } ?? "")"
                )
            } catch GlassesError.timeout {
                continue
            }
        }
    } catch {
        problem = error
    }
    // What was recorded before an error is still worth keeping.
    if lines.count > 1 {
        do {
            try (lines.joined(separator: "\n") + "\n").write(toFile: path, atomically: true, encoding: .utf8)
            print("recorded \(lines.count - 1) samples to \(path)")
        } catch {
            problem = problem ?? error
        }
    }
    if let problem {
        print("failed: \(problem)")
    }
    return problem == nil
}

/// `--dashboard FILE`: writes the dashboard shown above the canvas, as it
/// looks now, to `FILE` as a PNG, with made-up readings for the glasses.
@MainActor func renderDashboard(to path: String) -> Bool {
    guard let device = MTLCreateSystemDefaultDevice() else { return false }
    let dashboard = Dashboard(device: device)
    let glasses = GlassesReadings(temperature: 31.4, trackingHz: 1000, fps: 90, latency: 0.021, lateFramesPerSecond: 0.5)
    // A few looks a second apart, for rates and a little history.
    for _ in 0..<4 {
        _ = dashboard.png(glasses: glasses)
        Thread.sleep(forTimeInterval: 1)
    }
    // How long a look at the Mac and a redraw take, to judge how often
    // the dashboard can afford to update.
    var monitor = SystemMonitor()
    var started = monotonicNow()
    for _ in 0..<20 { _ = monitor.sample(now: monotonicNow()) }
    let looking = (monotonicNow() - started) / 20
    started = monotonicNow()
    for _ in 0..<20 {
        dashboard.update(glasses: glasses)
        dashboard.settle()
    }
    let updating = (monotonicNow() - started) / 20
    print(String(format: "a look at the Mac takes %.1f ms, a whole update %.1f ms", looking * 1000, updating * 1000))
    guard let png = dashboard.png(glasses: glasses), (try? png.write(to: URL(fileURLWithPath: path))) != nil else {
        print("failed to write \(path)")
        return false
    }
    print("wrote \(path)")
    return true
}

if let index = CommandLine.arguments.firstIndex(of: "--watchdog") {
    guard let viewer = CommandLine.arguments.dropFirst(index + 1).first.flatMap(Int32.init) else { exit(1) }
    MainActor.assumeIsolated { Watchdog.run(watching: viewer) }
}

if let index = CommandLine.arguments.firstIndex(of: "--dashboard") {
    guard let path = CommandLine.arguments.dropFirst(index + 1).first else {
        print("usage: --dashboard FILE")
        exit(1)
    }
    exit(MainActor.assumeIsolated { renderDashboard(to: path) } ? 0 : 1)
}

// `--dump-config`: prints the glasses' factory calibration, as JSON.
if CommandLine.arguments.contains("--dump-config") {
    do {
        FileHandle.standardOutput.write(try NrealAir().config)
    } catch {
        print("failed: \(error)")
    }
    exit(0)
}

if let index = CommandLine.arguments.firstIndex(of: "--record") {
    let rest = Array(CommandLine.arguments[(index + 1)...])
    guard rest.count >= 2, let seconds = Double(rest[0]) else {
        print("usage: --record SECONDS FILE")
        exit(1)
    }
    exit(record(seconds: seconds, path: rest[1]) ? 0 : 1)
}

if let index = CommandLine.arguments.firstIndex(of: "--probe-mode") {
    probeMode(Array(CommandLine.arguments[(index + 1)...]))
    exit(0)
}

if let index = CommandLine.arguments.firstIndex(of: "--probe-sizes") {
    probeSizes(Array(CommandLine.arguments[(index + 1)...]))
    exit(0)
}

if let index = CommandLine.arguments.firstIndex(of: "--probe") {
    let seconds = CommandLine.arguments.dropFirst(index + 1).first.flatMap(Double.init) ?? 4
    probe(seconds: seconds)
    exit(0)
}

let delegate = AppDelegate()
NSApplication.shared.delegate = delegate
// Lives in the menu bar only: no Dock icon and not in the app switcher.
NSApplication.shared.setActivationPolicy(.accessory)
NSApplication.shared.run()
