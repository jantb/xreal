# XREAL Viewer improvement notes

Reviewed 2026-09-27. This is a source review of the native macOS app, its build scripts, and the settings shared with the Rust implementation. These are recommendations only; no implementation changes were made for this review. Hardware behavior, latency, sleep/wake, and permission flows have not been tested here. Existing tests were inspected, not rerun.

The current design has useful foundations: capture stays on the GPU, rendering uses the latest frame without waiting for capture, tracking reconnects independently, and the core math and protocol parsing already have tests. I would prioritize recovery and everyday controls before adding more rendering features.

## 1. Make source switching cancellation-safe — high priority

**Evidence:** `macos/Sources/XrealViewer/Viewer.swift`, `startSource()` and `switchSource()`; `ScreenCapture.swift`, `start()`, `stop()`, and `shareableSize()`.

Pressing V cancels the previous task, but the source workflow does not check cancellation after its suspension points. The virtual-display polling loop also suppresses cancellation from `Task.sleep` with `try?`. Older work can continue after a newer source selection, start a stale stream, or overwrite the current status. Main-actor isolation does not prevent this interleaving across `await` calls.

**Proposed change:** Give each source request an identity, check cancellation and identity after asynchronous operations, and stop any stream created by an obsolete request. Let the polling loop exit immediately on cancellation. Filter late capture callbacks by stream/session identity.

**Verify:** Rapidly alternate mirror/virtual while display discovery and capture startup are delayed. Only the final selection should publish frames or status, with one active stream and no polling after cancellation.

## 2. Recover capture after disconnects and display changes — high priority

**Evidence:** `ScreenCapture.swift`, `stream(_:didStopWithError:)`; `Viewer.swift`, screen-parameter observer; `VirtualScreen.swift`, `terminationHandler`.

Capture stopping unexpectedly only prints to stderr; it does not clear the last image or notify the viewer. Display changes only reposition the output window. Virtual-display removal also only logs. This can leave a frozen desktop with an apparently normal source description, even though HID tracking reconnects successfully.

**Proposed change:** Publish capture state and errors to the viewer, visibly distinguish a stopped stream from an unchanged desktop, and restart capture with bounded retries when its source disappears or changes size. Re-evaluate the mirror source after monitor changes. Recreate a removed virtual display when appropriate.

**Verify:** Unplug/reconnect glasses and the mirrored monitor, change resolution/main display, and sleep/wake. Avoid treating ScreenCaptureKit idle frames as failures: an unchanged desktop is valid.

## 3. Validate settings before using them — high priority

**Evidence:** `macos/Sources/XrealCore/Settings.swift`, `parse()`; `macos/Sources/XrealViewer/VirtualScreen.swift`, integer conversions; `macos/Sources/XrealCore/Viewport.swift`, initialization.

Virtual dimensions accept any unsigned value that fits `Int`, including zero and values larger than `UInt32`. `VirtualScreen` then converts them directly to `UInt32`, so sufficiently large persisted values can trap. Floating-point parsing also accepts non-finite values rather than explicitly rejecting them before tracking/viewport math.

**Proposed change:** Validate finite sensitivity/bias values and sensible positive display dimensions at the settings boundary. Use checked conversions, retain documented defaults for invalid fields, and validate programmatically constructed settings too.

**Verify:** Add cases for zero, enormous dimensions, `nan`, infinities, and out-of-range indices. Invalid configuration should recover to usable defaults without a crash or invalid viewport.

## 4. Add controls that work while using the virtual desktop — medium priority

**Evidence:** `Viewer.swift`, `handleKey()` and `handleCharacter()`; `GlobalHotKey.swift`; `StartupPicker.swift`; `main.swift`.

Only recenter has a global shortcut. Zoom, freeze, calibration, and source switching require the viewer to have focus, while normal use involves focusing other apps on the virtual desktop. There is no application menu or settings window exposing these actions.

**Proposed change:** Add a small menu-bar controller with source selection, zoom, freeze, recenter, calibration, settings, and quit. Offer configurable global shortcuts for the actions used most often, report shortcut conflicts visibly, and keep plain letter shortcuts local to the viewer.

**Verify:** Operate essential controls while another app has focus without intercepting ordinary typing. Ensure controls remain accessible with the HUD hidden.

## 5. Make startup and errors actionable — medium priority

**Evidence:** `main.swift`, `applicationDidFinishLaunching`; `Viewer.swift`, `startCapture()`; `Hud.swift`.

Startup requests Screen Recording permission and continues immediately. Initialization failures print and terminate. Capture failures appear in the optional HUD and stderr, so a user who disabled the HUD can get a blank display without an explanation.

**Proposed change:** Show an explicit permission/setup state with a route to System Settings and a retry action. Surface essential errors independently of the diagnostic HUD. Explain calibration before starting it and provide clear completion/failure feedback.

**Verify:** Fresh launch, denied permission, later permission grant, capture failure with HUD hidden, and failed calibration should each have a clear next action.

## 6. Preserve aspect ratio when the source is smaller than the viewport — medium priority

**Evidence:** `Viewport.swift`, `update()`; `Renderer.swift`, full-screen triangle.

The viewport independently clamps width and height to the source dimensions, then stretches that rectangle across the entire output. If only one dimension is constrained, such as a small source or a low zoom level, the source rectangle can have a different aspect ratio from the output and distort the desktop.

**Proposed change:** Define explicit fit/fill behavior. Preserve aspect ratio with letterboxing for fit, or use a proportional crop for fill; preserve the existing crisp 1:1 mode where possible.

**Verify:** Test a square source on a 16:9 output, portrait displays, and zooming out past the source edges. Circles and text should retain their proportions.

## 7. Separate recentering from drift training — medium priority, needs user testing

**Evidence:** `Tracking.swift`, `DriftLearner.observeRecenter()`; `Viewer.swift`, `recenter()`.

Every sufficiently spaced recenter can teach gyro bias. The rate threshold rejects fast apparent turns, but a deliberate change of seating direction over a long interval can still look like slow drift. This is an ambiguity in user intent, not a demonstrated sensor defect.

**Proposed change:** Offer ordinary recenter separately from an explicit drift-learning action, or make learning optional with clear feedback and a way to undo the correction. Evaluate this with realistic desk use before changing the default.

**Verify:** Compare genuine stationary drift against intentional chair/head repositioning over several minutes. Repositioning should not silently introduce a persistent bias correction.

## 8. Measure latency and power before tuning fixed constants — medium priority, exploratory

**Evidence:** `Viewer.swift`, 120 FPS capture and 7 ms display-delay constants; `GlassesWindow.swift`, display-link range; `ScreenCapture.swift`, queue depth; `Hud.swift`, `RenderStats`.

The HUD exposes frame rates and IMU age, but does not measure captured-frame age or GPU completion timing. Fixed capture rate, queue depth, and prediction delay are therefore difficult to evaluate across display modes and glasses models from current diagnostics alone.

**Proposed change:** Record capture timestamps and render timing, then compare 60/90/120 Hz operation, queue depths, and prediction settings. Consider a lower-power mode and validated device-specific defaults only after measurement. Avoid synchronous settings writes from the render callback by scheduling persistence outside that path.

**Verify:** Measure frame-time percentiles, capture age, power use, and perceived stability during head turns on actual hardware. Do not equate FPS alone with motion-to-photon latency.

## 9. Keep shared settings compatible in both directions — medium priority

**Evidence:** `src/settings.rs`, `serialize()`; `macos/Sources/XrealCore/Settings.swift`; `macos/Tests/XrealCoreTests/SettingsTests.swift`.

Both implementations use the same settings file, but Rust does not serialize `source`, `virtual_width`, or `virtual_height`. Running Rust and saving therefore drops those native-app preferences. Existing compatibility coverage checks Rust-to-Swift loading, not a round trip through both implementations.

**Proposed change:** Preserve unknown fields, align the schemas, or separate implementation-specific preferences while retaining intentionally shared values. Use atomic writes in Rust as the Swift implementation already does.

**Verify:** Save native settings, load/save them through Rust, and reopen the native app. Source and virtual-display size should survive.

## 10. Document and verify the distributable app — lower priority

**Evidence:** `macos/scripts/bundle.sh`, `macos/Package.swift`, `macos/Resources/Info.plist`, `macos/Sources/XrealViewer/main.swift`, and `macos/Sources/CVirtualDisplay/include/CVirtualDisplay.h`.

There is no top-level setup guide distinguishing the Rust and native versions. The native package and bundle require macOS 27, virtual displays use private CoreGraphics declarations, and the bundle script explains signing only in a comment. The new SwiftPM icon resource bundle is not copied into the app; the normal bundled icon path works because `AppIcon.icns` exists, but the `Bundle.module` fallback would need that resource bundle if reached in a relocated app.

**Proposed change:** Add concise build/install instructions, supported/tested hardware and OS versions, shortcuts, permission troubleshooting, and the intended role of each implementation. Package SwiftPM resources consistently or deliberately avoid the module fallback inside app bundles. Keep stable-signing instructions visible. Document virtual-display compatibility as tested behavior rather than assuming private interfaces will remain stable.

**Verify:** Copy the finished app away from the repository/build products and launch it there. Check its icon, startup flow, resources, and permissions. Add lifecycle-focused coverage around source switching and capture failures alongside the existing core tests.

## Suggested order

Address source switching, capture recovery, and settings validation first. Then improve everyday controls and error reporting, followed by aspect-ratio behavior. Evaluate drift behavior and latency on hardware before tuning them. Finish with settings compatibility and distribution documentation.
