import AppKit
import XrealCore

/// A menu item that runs a closure.
private final class ActionItem: NSMenuItem {
    private let handler: () -> Void

    init(_ title: String, key: String = "", handler: @escaping () -> Void) {
        self.handler = handler
        super.init(title: title, action: #selector(fire), keyEquivalent: key)
        target = self
    }

    @available(*, unavailable)
    required init(coder: NSCoder) { fatalError("not used") }

    @objc private func fire() { handler() }
}

/// The menu bar icon and its menu: how the viewer is doing, the actions
/// used most, and the way to the controls window, where everything else is
/// adjusted. The menu is rebuilt each time it opens, so it is current.
@MainActor final class StatusMenu: NSObject, NSMenuDelegate {
    private let statusItem = NSStatusBar.system.statusItem(withLength: NSStatusItem.squareLength)
    private let viewer: Viewer
    private let controls: ControlPanel

    init(viewer: Viewer, controls: ControlPanel) {
        self.viewer = viewer
        self.controls = controls
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

    private func rebuild(_ menu: NSMenu) {
        menu.removeAllItems()
        let (info, source) = viewer.statusInfo()
        let headline = Headline(source: source, tracking: info.tracking)
        menu.addItem(hint(headline.title))
        menu.addItem(hint(headline.detail))
        menu.addItem(.separator())

        menu.addItem(shortcut("Recenter", "c") { [viewer] in viewer.perform(.recenter) })
        menu.addItem(ActionItem("Calibrate Gyro (Keep Glasses Still)") { [viewer] in viewer.perform(.calibrate) })
        menu.addItem(ActionItem("Put Canvas Back Straight Ahead") { [viewer] in viewer.perform(.resetView) })
        menu.addItem(.separator())

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
        menu.addItem(.separator())
        menu.addItem(ActionItem("Controls…", key: ",") { [controls] in controls.show() })
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
}
