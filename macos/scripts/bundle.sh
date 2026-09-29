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
identity="${SIGN_IDENTITY:-Pace Local}"
if [ "$identity" != "-" ] && ! security find-identity -v -p codesigning | grep -qF "\"$identity\""; then
    echo "warning: no signing certificate named \"$identity\"; signing ad hoc, so macOS will ask for Screen Recording again after every build" >&2
    identity="-"
fi
codesign --force --sign "$identity" "$app"
echo "$app"
