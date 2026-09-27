#!/bin/sh
# Generate every standard macOS icon size from the master artwork.
set -eu
cd "$(dirname "$0")/.."
iconset="build/AppIcon.iconset"
source="Sources/XrealViewer/Resources/AppIcon.png"
mkdir -p "$iconset"
for size in 16 32 128 256 512; do
    sips -z "$size" "$size" "$source" \
        --out "$iconset/icon_${size}x${size}.png" >/dev/null
    retina=$((size * 2))
    sips -z "$retina" "$retina" "$source" \
        --out "$iconset/icon_${size}x${size}@2x.png" >/dev/null
done
iconutil -c icns "$iconset" -o build/AppIcon.icns
