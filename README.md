# XREAL Viewer

A macOS menu bar app that turns XREAL Air glasses into a head-tracked virtual
monitor. It creates one large virtual display (the canvas), captures it with
ScreenCaptureKit, and draws it in the glasses as a stereo view that follows
your head, corrected for each lens's factory calibration.

## Requirements

- macOS 27 and Xcode with Swift 6.2
- XREAL Air, Air 2 or Air 2 Pro (Air 2 is the one it has been tried on)
- [`just`](https://github.com/casey/just) for the shortcuts below

## Build and run

```sh
just test      # unit tests
just build     # debug build
just run       # release bundle in macos/build, launched from there
just install   # replace /Applications/XREAL Viewer.app and launch it
```

`just bundle` signs with a certificate named "Pace Local". Set `SIGN_IDENTITY`
to another name from `security find-identity -v -p codesigning`. Without a
certificate it signs ad hoc, and macOS then forgets the Screen Recording
permission after every build.

## Permissions

- **Screen Recording**: needed to capture the canvas. Allow it, then quit and
  reopen the app.
- **Accessibility**: only for moving and fitting other apps' windows to where
  you look.

## Using it

The app lives in the menu bar. Everything adjustable is in **Controls…**.
Global shortcuts (⌃⌥⌘ plus a key):

| Key | Action |
| --- | --- |
| C | Recenter |
| G (hold) | Carry the canvas while looking at it |
| = / - | Bring the canvas closer / push it away |
| ] / [ | Next canvas size up / down |
| W | Move the focused window to where you look |
| F | Fit the focused window to the zone you look at |
| M | Move the pointer to where you look |

## Layout

- `macos/Sources/XrealCore`: glasses protocol and IMU, head tracking and gyro
  bias learning, room geometry, calibration, settings. No UI; unit tested.
- `macos/Sources/XrealViewer`: the app: capture, Metal renderer, windows,
  menu, controls, `ViewerState` (canvas placement and per-frame state).
- `macos/Sources/CDcmImu`: the DCM attitude filter, translated from the Rust
  `dcmimu` crate.
- `macos/Sources/CVirtualDisplay`: declarations of CoreGraphics' private
  virtual display API.

Settings are kept in `~/Library/Application Support/xreal/settings.txt`.
Diagnostic commands: `--probe`, `--probe-sizes`, `--probe-mode`, `--dump-config`,
`--record SECONDS FILE` (raw IMU samples as CSV; quit the viewer first).

## Things to know

- **Never create virtual displays wider or taller than 8192 px.** Doing so
  panicked a Mac. `maxVirtualScreenSide` enforces it, and `canvasSizes` lists
  the sizes macOS 27 gives as asked. Other standard sizes are refused or come
  up smaller.
- Quitting (or `kill`) puts the glasses back to their own picture. A crash
  leaves them side by side until they are replugged; `--probe-mode 11`
  switches them back.
- The Air 2's temple buttons never reach the host, so there are no button
  features.
