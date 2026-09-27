#!/bin/sh
# Builds XREAL Viewer.app in macos/build. Screen Recording permission is tied
# to the app's signature. An ad-hoc signature (SIGN_IDENTITY=-) changes with
# every build, so macOS forgets the grant; a certificate keeps it. Set
# SIGN_IDENTITY to another name from `security find-identity -v -p codesigning`
# to sign with a different one.
set -eu
cd "$(dirname "$0")/.."

swift build -c release
./scripts/build-icon.sh
app="build/XREAL Viewer.app"
rm -rf "$app"
mkdir -p "$app/Contents/MacOS"
mkdir -p "$app/Contents/Resources"
cp .build/release/XrealViewer "$app/Contents/MacOS/"
cp Resources/Info.plist "$app/Contents/Info.plist"
cp build/AppIcon.icns "$app/Contents/Resources/"
codesign --force --sign "${SIGN_IDENTITY:-Pace Local}" "$app"
echo "$app"
