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

        let recenter = ActionItem("Recenter", key: "c") { [viewer] in viewer.perform(.recenter) }
        recenter.keyEquivalentModifierMask = [.control, .option, .command]
        menu.addItem(recenter)
        menu.addItem(ActionItem("Freeze View", checked: viewport.frozen) { [viewer] in viewer.perform(.toggleFreeze) })
        menu.addItem(.separator())

        menu.addItem(
            submenu(
                "Source",
                SourceChoice.all.map { choice in
                    ActionItem(choice.title, checked: choice.isSelected(in: settings)) { [viewer] in
                        viewer.perform(.setSource(choice))
                    }
                }))
        let projections: [(Projection, String)] = [
            (.crop, "Pixel Exact"), (.flat, "Flat Screen in Room"), (.curved, "Curved Screen in Room"),
        ]
        let room = viewport.projection != .crop
        let roll = ActionItem("Follow Head Tilt", checked: viewport.followsRoll) { [viewer] in
            viewer.perform(.toggleRoll)
        }
        roll.isEnabled = room
        menu.addItem(
            submenu(
                "View",
                projections.map { projection, title in
                    ActionItem(title, checked: viewport.projection == projection) { [viewer] in
                        viewer.perform(.setProjection(projection))
                    }
                } + [.separator(), roll]))
        let edges: [(EdgeMode, String)] = [(.snap, "Stop at Edge"), (.black, "Show Black Beyond Edge")]
        let edge = submenu(
            "At Screen Edge",
            edges.map { edge, title in
                ActionItem(title, checked: viewport.edge == edge) { [viewer] in viewer.perform(.setEdge(edge)) }
            })
        edge.isEnabled = !room
        menu.addItem(edge)
        menu.addItem(
            submenu(
                String(format: "Zoom %.2g×", viewport.zoom),
                zoomLevels.enumerated().map { index, level in
                    ActionItem(String(format: "%.2g×", level), checked: viewport.zoomIndex == index) { [viewer] in
                        viewer.perform(.setZoom(index))
                    }
                }))
        menu.addItem(
            submenu(
                String(format: "Sensitivity %.2f×", viewport.sensitivity),
                [
                    ActionItem("Increase", key: ".") { [viewer] in viewer.perform(.increaseSensitivity) },
                    ActionItem("Decrease", key: ",") { [viewer] in viewer.perform(.decreaseSensitivity) },
                    ActionItem("Reset to 1×") { [viewer] in viewer.perform(.resetSensitivity) },
                ]))
        menu.addItem(
            submenu(
                "Deadzone",
                deadzoneLevels.enumerated().map { index, level in
                    let title = level == 0 ? "Off" : String(format: "%.1f°", level * 180 / .pi)
                    return ActionItem(title, checked: viewport.deadzoneIndex == index) { [viewer] in
                        viewer.perform(.setDeadzone(index))
                    }
                }))
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
        menu.addItem(ActionItem("Reset View Settings") { [viewer] in viewer.perform(.resetView) })
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

    private func submenu(_ title: String, _ items: [NSMenuItem]) -> NSMenuItem {
        let item = NSMenuItem(title: title, action: nil, keyEquivalent: "")
        let menu = NSMenu()
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
