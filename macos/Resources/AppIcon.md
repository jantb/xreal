# App icon

`../Sources/XrealViewer/Resources/AppIcon.png` is the master artwork, packaged by
SwiftPM for direct launches and applied to the running app by its delegate.
Generated with the built-in image generation tool.
`../scripts/build-icon.sh` uses macOS `sips` and `iconutil` to create the standard
16–1024 pixel representations in `build/AppIcon.icns`. The bundle script includes
this file before signing the app.

## Generation prompt

Use case: logo-brand. Create a polished production macOS application icon for XREAL Viewer, a utility that shows a large floating virtual desktop through head-tracked AR glasses. Single icon, square 1024x1024 canvas, actual transparent background outside a rounded-square macOS icon tile with generous standard icon margins. Deep midnight-blue rounded-square tile, beautifully restrained dimensional finish. Central bold recognizable silhouette of sleek AR sunglasses in pearl silver with two dark blue lenses, positioned in the lower middle, with a single luminous cyan curved widescreen floating just above and behind them. Screen has a softly glowing blue-to-teal surface, no UI details. The glasses and floating screen should together communicate spatial computing and a virtual monitor. Elegant precise geometry, subtle material depth, excellent legibility at small Dock sizes, large simple forms, frontal view with subtle perspective only on screen. No text, letters, logos, stars, circuitry, particles, decorative rings, extra objects, or mockup presentation. Deliver only the finished icon asset.
