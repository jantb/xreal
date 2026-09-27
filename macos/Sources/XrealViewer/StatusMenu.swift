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
        let (settings, viewport, screenList, glassesOnly) = viewer.current

        if glassesOnly {
            menu.addItem(hint("Glasses Only: Lid Closed"))
        }
        let recenter = ActionItem("Recenter", key: "c") { [viewer] in viewer.perform(.recenter) }
        recenter.keyEquivalentModifierMask = [.control, .option, .command]
        menu.addItem(recenter)
        menu.addItem(ActionItem("Freeze View", checked: viewport.frozen) { [viewer] in viewer.perform(.toggleFreeze) })
        menu.addItem(.separator())

        let source = submenu(
            "Source",
            SourceChoice.all.map { choice in
                ActionItem(choice.title, checked: choice.isSelected(in: settings)) { [viewer] in
                    viewer.perform(.setSource(choice))
                }
            })
        // With the glasses alone there is no display to mirror.
        source.isEnabled = !glassesOnly
        menu.addItem(source)
        let virtual = glassesOnly || settings.source == .virtual
        let screens = submenu(
            "Virtual Screens",
            screenList.enumerated().map { index, screen in
                ActionItem(
                    "Screen \(index + 1): \(screen.width) × \(screen.height), Curved", checked: screen.curved
                ) { [viewer] in viewer.perform(.toggleCurved(index)) }
            } + [
                .separator(),
                submenu(
                    "Add Screen",
                    virtualScreenSizes.map { size in
                        ActionItem("\(size.width) × \(size.height)") { [viewer] in
                            viewer.perform(.addScreen(width: size.width, height: size.height, curved: false))
                        }
                    } + [.separator()]
                        + canvasSizes.map { size in
                            ActionItem("Curved Canvas \(size.width) × \(size.height)") { [viewer] in
                                viewer.perform(.addScreen(width: size.width, height: size.height, curved: true))
                            }
                        }),
                removeScreens(screenList),
                ActionItem(
                    glassesOnly ? "Standard Layout: First Ahead, Others Beside" : "Standard Layout: Wide Above, One Each Side"
                ) { [viewer] in
                    viewer.perform(.standardLayout)
                },
                .separator(),
                hint("Look at a screen and hold ⌃⌥⌘G to carry it"),
                hint("⌃⌥⌘= brings it closer, ⌃⌥⌘- pushes it away"),
            ])
        screens.isEnabled = virtual
        menu.addItem(screens)
        let windows = submenu(
            "Windows",
            [
                shortcut("Move Window to Where You Look", "w") { [viewer] in viewer.perform(.moveWindowToGaze) },
                shortcut("Fit Window to Zone You Look At", "f") { [viewer] in viewer.perform(.fitWindowToZone) },
                shortcut("Move Pointer to Where You Look", "m") { [viewer] in viewer.perform(.movePointerToGaze) },
                ActionItem("Bring Back Windows Hidden Behind the Glasses") { [viewer] in
                    viewer.perform(.gatherWindows)
                },
            ] + (WindowControl.allowed(prompt: false)
                ? [] : [.separator(), hint("Moving windows needs Accessibility access")]))
        windows.isEnabled = virtual
        menu.addItem(windows)
        let projections: [(Projection, String)] = [
            (.crop, "Pixel Exact"), (.flat, "Flat Screen in Room"), (.curved, "Curved Screen in Room"),
        ]
        let room = virtual || viewport.projection != .crop
        let roll = ActionItem("Follow Head Tilt", checked: viewport.followsRoll) { [viewer] in
            viewer.perform(.toggleRoll)
        }
        roll.isEnabled = room
        let view = submenu(
            "View",
            projections.map { projection, title in
                let item = ActionItem(title, checked: viewport.projection == projection) { [viewer] in
                    viewer.perform(.setProjection(projection))
                }
                // Virtual screens always hang flat in the room.
                item.isEnabled = !virtual
                return item
            } + [.separator(), roll])
        menu.addItem(view)
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
        let followCursor = ActionItem("Zoom Out to Show Cursor", checked: settings.followCursor) { [viewer] in
            viewer.perform(.toggleFollowCursor)
        }
        followCursor.isEnabled = !virtual
        menu.addItem(followCursor)
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

    private func removeScreens(_ screens: [RoomScreen]) -> NSMenuItem {
        let item = submenu(
            "Remove Screen",
            screens.enumerated().map { index, screen in
                ActionItem("Screen \(index + 1): \(screen.width) × \(screen.height)") { [viewer] in
                    viewer.perform(.removeScreen(index))
                }
            })
        // At least one screen stays.
        item.isEnabled = screens.count > 1
        return item
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
