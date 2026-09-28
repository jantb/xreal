import AppKit
import XrealCore

private let statusRefreshInterval = 0.25

/// A menu item that runs a closure.
private final class ActionItem: NSMenuItem {
    private let handler: () -> Void

    init(_ title: String, checked: Bool? = nil, key: String = "", handler: @escaping () -> Void) {
        self.handler = handler
        super.init(title: title, action: #selector(fire), keyEquivalent: key)
        target = self
        if let checked {
            state = checked ? .on : .off
        }
    }

    @available(*, unavailable)
    required init(coder: NSCoder) { fatalError("not used") }

    @objc private func fire() { handler() }
}

/// A menu item holding a titled slider, for settings that are easier to
/// tune by feel than to pick from a list. `onChange` runs while dragging.
@MainActor private final class SliderItem: NSMenuItem {
    private let onChange: (Float) -> Void
    private let format: (Float) -> String
    private let valueLabel = NSTextField(labelWithString: "")
    private let slider: NSSlider
    // The slider runs over log(value), so each step feels the same size.
    private let logarithmic: Bool

    init(
        _ title: String, value: Float, range: ClosedRange<Float>, logarithmic: Bool = false,
        format: @escaping (Float) -> String, onChange: @escaping (Float) -> Void
    ) {
        self.onChange = onChange
        self.format = format
        self.logarithmic = logarithmic
        let map = { (v: Float) in Double(logarithmic ? log(v) : v) }
        slider = NSSlider(value: map(value), minValue: map(range.lowerBound), maxValue: map(range.upperBound), target: nil, action: nil)
        super.init(title: title, action: nil, keyEquivalent: "")

        let titleLabel = NSTextField(labelWithString: title)
        titleLabel.font = .menuFont(ofSize: 0)
        valueLabel.font = .monospacedDigitSystemFont(ofSize: NSFont.smallSystemFontSize, weight: .regular)
        valueLabel.textColor = .secondaryLabelColor
        valueLabel.alignment = .right
        slider.isContinuous = true
        slider.target = self
        slider.action = #selector(changed)
        let container = NSView(frame: NSRect(x: 0, y: 0, width: 260, height: 48))
        titleLabel.frame = NSRect(x: 20, y: 26, width: 150, height: 18)
        valueLabel.frame = NSRect(x: 170, y: 26, width: 74, height: 18)
        slider.frame = NSRect(x: 18, y: 4, width: 228, height: 22)
        for subview in [titleLabel, valueLabel, slider] {
            container.addSubview(subview)
        }
        view = container
        valueLabel.stringValue = format(value)
    }

    @available(*, unavailable)
    required init(coder: NSCoder) { fatalError("not used") }

    @objc private func changed() {
        let raw = Float(slider.doubleValue)
        let value = logarithmic ? exp(raw) : raw
        valueLabel.stringValue = format(value)
        onChange(value)
    }
}

/// The menu bar icon and its menu, where everything about the view is
/// adjusted. The menu is rebuilt each time it opens, so it always shows the
/// current settings; the status lines refresh while it stays open.
@MainActor final class StatusMenu: NSObject, NSMenuDelegate {
    private let statusItem = NSStatusBar.system.statusItem(withLength: NSStatusItem.squareLength)
    private let viewer: Viewer
    private var statusLines: [NSMenuItem] = []
    private var refresh: Timer?

    init(viewer: Viewer) {
        self.viewer = viewer
        super.init()
        let image = NSImage(systemSymbolName: "eyeglasses", accessibilityDescription: "XREAL Viewer")
        image?.isTemplate = true
        statusItem.button?.image = image
        let menu = NSMenu()
        // Items are enabled by what applies now, not by having a target.
        menu.autoenablesItems = false
        menu.delegate = self
        statusItem.menu = menu
    }

    func menuNeedsUpdate(_ menu: NSMenu) {
        rebuild(menu)
    }

    func menuWillOpen(_ menu: NSMenu) {
        let timer = Timer(timeInterval: statusRefreshInterval, repeats: true) { [weak self] _ in
            MainActor.assumeIsolated { self?.refreshStatus() }
        }
        // `.common` includes the mode the run loop is in while a menu is open.
        RunLoop.main.add(timer, forMode: .common)
        refresh = timer
    }

    func menuDidClose(_ menu: NSMenu) {
        refresh?.invalidate()
        refresh = nil
    }

    private func rebuild(_ menu: NSMenu) {
        menu.removeAllItems()
        let (settings, viewport) = viewer.current
        let canvas = settings.canvas

        let recenter = ActionItem("Recenter", key: "c") { [viewer] in viewer.perform(.recenter) }
        recenter.keyEquivalentModifierMask = [.control, .option, .command]
        menu.addItem(recenter)
        menu.addItem(.separator())

        let curve = submenu(
            "Curve",
            curveRadii.map { curve in
                ActionItem(curve.title, checked: settings.curveRadius == curve.radius) { [viewer] in
                    viewer.perform(.setCurveRadius(curve.radius))
                }
            })
        curve.isEnabled = canvas.curved
        menu.addItem(
            submenu(
                "Canvas",
                [
                    ActionItem("\(canvas.width) × \(canvas.height), Curved", checked: canvas.curved) { [viewer] in
                        viewer.perform(.toggleCurved)
                    },
                    curve,
                    .separator(),
                    hint("Look at the canvas and hold ⌃⌥⌘G to carry it"),
                    hint("⌃⌥⌘= brings it closer, ⌃⌥⌘- pushes it away"),
                    hint("⌃⌥⌘] gives it the next size up, ⌃⌥⌘[ the next down"),
                ]))
        menu.addItem(
            submenu(
                "Windows",
                [
                    shortcut("Move Window to Where You Look", "w") { [viewer] in viewer.perform(.moveWindowToGaze) },
                    shortcut("Fit Window to Zone You Look At", "f") { [viewer] in viewer.perform(.fitWindowToZone) },
                    shortcut("Move Pointer to Where You Look", "m") { [viewer] in viewer.perform(.movePointerToGaze) },
                    ActionItem("Bring Back Windows Hidden Behind the Glasses") { [viewer] in
                        viewer.perform(.gatherWindows)
                    },
                ] + (WindowControl.allowed(prompt: false)
                    ? [] : [.separator(), hint("Moving windows needs Accessibility access")])))
        // How far away the canvas at distance 1 looks: nearer shows more
        // depth between the eyes' views, further less. Its size stays the
        // same.
        let viewingDistance = SliderItem(
            "Viewing Distance", value: settings.metresPerRoomUnit, range: minViewingDistance...maxViewingDistance,
            logarithmic: true, format: { String(format: "%.2f m", $0) }
        ) { [viewer] metres in
            viewer.perform(.setDepthScale(metres))
        }
        menu.addItem(
            submenu(
                "View",
                [
                    ActionItem("Follow Head Tilt", checked: viewport.followsRoll) { [viewer] in
                        viewer.perform(.toggleRoll)
                    },
                    submenu(
                        "3D Depth",
                        [viewingDistance, .separator()]
                            + depthScales.map { scale in
                                ActionItem(scale.title, checked: settings.metresPerRoomUnit == scale.metres) {
                                    [viewer] in
                                    viewer.perform(.setDepthScale(scale.metres))
                                }
                            }),
                    ActionItem("Swap Eyes", checked: settings.swapEyes) { [viewer] in viewer.perform(.toggleSwapEyes) },
                    ActionItem("Correct Lens Distortion", checked: settings.lensCorrection) { [viewer] in
                        viewer.perform(.toggleLensCorrection)
                    },
                ]))
        menu.addItem(
            ActionItem("Zoom Out to Show Cursor", checked: settings.followCursor) { [viewer] in
                viewer.perform(.toggleFollowCursor)
            })
        menu.addItem(
            ActionItem("Predict Head Motion", checked: settings.prediction) { [viewer] in
                viewer.perform(.togglePrediction)
            })
        menu.addItem(.separator())

        menu.addItem(ActionItem("Calibrate Gyro (Keep Glasses Still)") { [viewer] in viewer.perform(.calibrate) })
        menu.addItem(ActionItem("Put Canvas Back Straight Ahead") { [viewer] in viewer.perform(.resetView) })
        menu.addItem(.separator())

        menu.addItem(ActionItem("Show Status", checked: settings.overlayVisible) { [viewer] in
            viewer.perform(.toggleStatus)
        })
        statusLines = []
        if settings.overlayVisible {
            for _ in viewer.statusLines() {
                let item = NSMenuItem()
                item.isEnabled = false
                statusLines.append(item)
                menu.addItem(item)
            }
            refreshStatus()
        }
        menu.addItem(.separator())
        menu.addItem(ActionItem("Quit XREAL Viewer", key: "q") { NSApp.terminate(nil) })
    }

    /// An item showing its global shortcut, ⌃⌥⌘ and `key`.
    private func shortcut(_ title: String, _ key: String, handler: @escaping () -> Void) -> NSMenuItem {
        let item = ActionItem(title, key: key, handler: handler)
        item.keyEquivalentModifierMask = [.control, .option, .command]
        return item
    }

    private func hint(_ text: String) -> NSMenuItem {
        let item = NSMenuItem(title: text, action: nil, keyEquivalent: "")
        item.isEnabled = false
        return item
    }

    private func submenu(_ title: String, _ items: [NSMenuItem]) -> NSMenuItem {
        let item = NSMenuItem(title: title, action: nil, keyEquivalent: "")
        let menu = NSMenu()
        menu.autoenablesItems = false
        items.forEach(menu.addItem)
        item.submenu = menu
        return item
    }

    private func refreshStatus() {
        let font = NSFont.monospacedSystemFont(ofSize: NSFont.smallSystemFontSize, weight: .regular)
        for (item, line) in zip(statusLines, viewer.statusLines()) {
            item.attributedTitle = NSAttributedString(
                string: line, attributes: [.font: font, .foregroundColor: NSColor.secondaryLabelColor])
        }
    }
}
