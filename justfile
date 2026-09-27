set working-directory := 'macos'

app := "XREAL Viewer.app"
bundle_id := "dev.jantb.xreal.viewer"
install_dir := env('INSTALL_DIR', '/Applications')

default:
    @just --list

# Debug build of the macOS app
build:
    swift build

test:
    swift test

# A certificate signature (not ad-hoc) keeps Screen Recording permission across rebuilds.
# Release build as macos/build/XREAL Viewer.app, signed with "Pace Local" unless SIGN_IDENTITY is set
bundle:
    ./scripts/bundle.sh

# Quit the running copy, replace the installed app and launch it
install: bundle quit
    rm -rf "{{ install_dir }}/{{ app }}"
    ditto "build/{{ app }}" "{{ install_dir }}/{{ app }}"
    open "{{ install_dir }}/{{ app }}"

uninstall: quit
    rm -rf "{{ install_dir }}/{{ app }}"

# Quit the app cleanly so it saves its settings
quit:
    osascript -e 'if application id "{{ bundle_id }}" is running then tell application id "{{ bundle_id }}" to quit'

# Run the release bundle from the build folder without installing
run: bundle quit
    open "build/{{ app }}"
