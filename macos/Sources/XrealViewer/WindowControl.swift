import AppKit
import ApplicationServices

// An app that does not answer is skipped rather than holding up the rest.
private let answerTimeout: Float = 0.5  // s

/// Moves other apps' windows through the Accessibility API. macOS asks the
/// user to allow this once, in System Settings > Privacy & Security >
/// Accessibility.
@MainActor enum WindowControl {
    /// Whether the app may move windows. `prompt` shows macOS's request
    /// when it may not.
    static func allowed(prompt: Bool) -> Bool {
        AXIsProcessTrustedWithOptions(["AXTrustedCheckOptionPrompt": prompt] as CFDictionary)
    }

    /// The focused window of the app in front.
    static func focusedWindow() -> AXUIElement? {
        guard let app = NSWorkspace.shared.frontmostApplication else { return nil }
        let element = AXUIElementCreateApplication(app.processIdentifier)
        return attribute(kAXFocusedWindowAttribute, of: element).map { $0 as! AXUIElement }
    }

    /// Every standard window of the apps in the Dock, apart from this app's.
    static func allWindows() -> [AXUIElement] {
        let own = ProcessInfo.processInfo.processIdentifier
        return NSWorkspace.shared.runningApplications
            .filter { $0.activationPolicy == .regular && $0.processIdentifier != own }
            .flatMap { app -> [AXUIElement] in
                let element = AXUIElementCreateApplication(app.processIdentifier)
                AXUIElementSetMessagingTimeout(element, answerTimeout)
                return attribute(kAXWindowsAttribute, of: element) as? [AXUIElement] ?? []
            }
    }

    /// The window's frame in global points, top-left origin.
    static func frame(of window: AXUIElement) -> CGRect? {
        guard let position = attribute(kAXPositionAttribute, of: window),
            let size = attribute(kAXSizeAttribute, of: window)
        else { return nil }
        var origin = CGPoint.zero
        var extent = CGSize.zero
        guard AXValueGetValue(position as! AXValue, .cgPoint, &origin),
            AXValueGetValue(size as! AXValue, .cgSize, &extent)
        else { return nil }
        return CGRect(origin: origin, size: extent)
    }

    static func move(_ window: AXUIElement, to origin: CGPoint) {
        var origin = origin
        if let value = AXValueCreate(.cgPoint, &origin) {
            AXUIElementSetAttributeValue(window, kAXPositionAttribute as CFString, value)
        }
    }

    /// Moves and resizes `window` to `frame`. It moves first so the new size
    /// fits on the display it lands on, and again after, for apps that
    /// shift a window while resizing it.
    static func setFrame(_ window: AXUIElement, to frame: CGRect) {
        move(window, to: frame.origin)
        var size = frame.size
        if let value = AXValueCreate(.cgSize, &size) {
            AXUIElementSetAttributeValue(window, kAXSizeAttribute as CFString, value)
        }
        move(window, to: frame.origin)
    }

    private static func attribute(_ name: String, of element: AXUIElement) -> CFTypeRef? {
        var value: CFTypeRef?
        guard AXUIElementCopyAttributeValue(element, name as CFString, &value) == .success else { return nil }
        return value
    }
}
