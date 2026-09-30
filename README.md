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

The canvas can be flat, curved round you like a monitor, or wrapped round
you as part of a sphere (**Wrap Around You**), where every pixel is the same
distance away, faces you and keeps its shape, a little smaller towards the top
and bottom; **Wrap Up and Down** blends from the curved shape to the full
sphere. **Even Out Text Size** shares that shrinking out, a little
larger in the middle and a little smaller at the edges. Edges are drawn
smooth (four samples a pixel) and fade softly into the room (**Soft
Edges**). **Viewing Distance** starts at, and snaps to, the 4 m where the
glasses' optics focus, so the eyes aim and focus at the same distance. **Ambient Light** lights the room round the canvas in the
colours of its edges, like a TV's backlight.

Above the canvas, where you see it by looking up, hangs a row tilted to face
you: a dashboard updated ten times a second (time and thermal state, CPU per
core, memory, GPU, network, disk space and traffic, power draw, battery, the
busiest apps, the glasses and latency), and beside it, if you pick one in the
controls, a pinned window from any app. The pinned window can stay anywhere,
even on the glasses' own display behind the view.

**Pinned Window > Choose…** lists windows by app and title, with their
sizes, so windows with the same title can be told apart. The pinned window
follows its window while its title changes. After a restart it reconnects
only when the saved app and title match exactly one window; otherwise the
controls say it is unavailable rather than showing another one. Over a
wrapped canvas the dashboard and pinned window curve round the eyes like
the canvas and keep clear of it and of each other; otherwise they are flat
panels tilted to face you.

## Latency

The viewer measures when each frame really reaches the glasses and predicts
the head pose for that moment; the latency and late frames per second are in
**Controls… > Diagnostics** and on the dashboard. It also waits to read the
pose until just before each frame's deadline, which measurably drops fewer
frames. Hitches are logged: late and dropped frames,
gaps in the IMU samples, slow dashboard updates and capture rate changes:

```sh
/usr/bin/log stream --predicate 'subsystem == "dev.jantb.xreal.viewer" AND category == "timing"'
```

On an M4 Pro with macOS 27 the glasses drop about two frames a second in bursts
that repeat every 5.055 s (455 of their refreshes). It comes with the canvas's
virtual display: with none there are no drops, with it uncaptured about half,
and neither the canvas's size, rate or place nor the viewer's own settings
and thread priorities change the period. It also needs the Mac's own screen
on: with the lid closed, or the screen switched off, there are none. So the
viewer switches the Mac's screen off while the glasses are in use (with
macOS's private display switch, the one display utilities use). A watchdog, a
second copy of the viewer started with `--watchdog PID`, brings the screen
back, and the glasses to their own picture, if the viewer dies. Environment switches, for measuring:
`XREAL_NO_CANVAS=1` (glasses only), `XREAL_NO_CAPTURE=1` (canvas, not
captured), `XREAL_NO_REALTIME=1`, `XREAL_FRAME_LATENCY=2` (a frame more slack),
as in `open --env XREAL_NO_CANVAS=1 "/Applications/XREAL Viewer.app"`.

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
`--record SECONDS FILE` (raw IMU samples as CSV; quit the viewer first),
`--dashboard FILE` (the dashboard as a PNG, with how long an update takes).

## Things to know

- **Never create virtual displays wider or taller than 8192 px.** Doing so
  panicked a Mac. `maxVirtualScreenSide` enforces it, and `canvasSizes` lists
  the sizes macOS 27 gives as asked, in points; the HiDPI ones (scale 2) have
  twice the pixels each way. Other standard sizes are refused or come up
  smaller. `--probe-sizes 1920x1080@2x` probes a HiDPI size. Two HiDPI sizes,
  5120×1440 and 5120×2160 at 2x (10240 px wide), are allowed past the limit:
  each came up fine when probed on its own. The panic came from many oversize
  displays made in one run. `--probe-sizes --beyond-limit` tries others; save
  everything first.
- Quitting (or `kill`) puts the glasses back to their own picture, and the
  Mac's own screen back on. If the viewer dies, its watchdog does the same.
  `--probe-mode 11` switches the glasses back by hand.
- The Air 2's temple buttons never reach the host, so there are no button
  features.
- The live pointer is drawn from the system's current pointer image; macOS no
  longer says when an app hides it while you type, so it stays visible.
