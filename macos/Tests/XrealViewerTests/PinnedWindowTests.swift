import Testing
import XrealCore
@testable import XrealViewer

private func choice(_ id: UInt32, app: String = "editor", title: String) -> PinnableWindow {
    PinnableWindow(id: id, window: PinnedWindow(bundleID: app, title: title), appName: app, width: 1200, height: 800)
}

@Test func pinningDistinguishesDuplicateTitlesAndFollowsTheSelectedWindowWhenItsTitleChanges() {
    let wanted = PinnedWindow(bundleID: "editor", title: "Code")
    let choices = [choice(1, title: "Code"), choice(2, title: "Code")]
    #expect(pinnedWindowID(for: wanted, preferredID: 2, choices: choices) == 2)
    #expect(pinnedWindowID(for: wanted, preferredID: nil, choices: choices) == nil)
    #expect(pinnedWindowID(for: wanted, preferredID: 2, choices: [choice(2, title: "README.md")]) == 2)
}

@Test func aMissingPinnedWindowNeverSilentlySelectsAnotherDocumentOrApp() {
    let wanted = PinnedWindow(bundleID: "editor", title: "Code")
    #expect(pinnedWindowID(for: wanted, preferredID: 1, choices: [choice(2, title: "Terminal")]) == nil)
    #expect(pinnedWindowID(for: wanted, preferredID: 1, choices: [choice(1, app: "browser", title: "Code")]) == nil)
    #expect(pinnedWindowID(for: wanted, preferredID: 1, choices: [choice(3, title: "Code")]) == 3)
}

@Test func windowSearchFindsAppsAndTitlesWithoutCaseSensitivity() {
    let window = choice(1, app: "Xcode", title: "Renderer.swift")
    #expect(window.matches(""))
    #expect(window.matches("xcode"))
    #expect(window.matches("RENDERER"))
    #expect(!window.matches("Terminal"))
}
