# XREAL Viewer

See README.md for what the app is, how to build and run it, and its layout.

- Work in `macos/`; tests are `just test` (Swift Testing). `XrealViewerTests`
  reaches the app target with `@testable import XrealViewer`.
- Tests assert behaviour a caller depends on, not implementation shape.
- Never create a virtual display over 8192 px in either dimension, and do not
  run size probes beyond the sizes in `canvasSizes` without asking: larger
  sizes panicked the machine.
- Hardware behaviour (latency, drift, buttons) cannot be checked without the
  glasses; say so instead of claiming it works.
- The old Rust prototype (src/, Cargo.*) was removed; it lives in git history before the "Add a controls window" commit.
