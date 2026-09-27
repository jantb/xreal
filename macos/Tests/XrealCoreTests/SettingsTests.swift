import Testing
import simd

@testable import XrealCore

@Test func savedSettingsLoadBackUnchanged() throws {
    var settings = Settings()
    settings.zoomIndex = 4
    settings.sensitivity = 0.85
    settings.deadzoneIndex = 0
    settings.prediction = false
    settings.overlayVisible = false
    settings.gyroBias = SIMD3(0.0012, -0.0034, 0.00056)
    settings.source = .virtual
    settings.screens = [
        RoomScreen(width: 5120, height: 1440),
        RoomScreen(
            width: 2880, height: 1620, placement: ScreenPlacement(direction: SIMD3(0.4, 0.3, -1), distance: 1.5)),
    ]
    settings.projection = .curved
    settings.followRoll = false
    settings.edge = .black
    settings.followCursor = false

    var loaded = Settings.parse(settings.serialize())
    #expect(loaded.screens.map(\.width) == [5120, 2880])
    #expect(loaded.screens.map(\.height) == [1440, 1620])
    #expect(loaded.screens[0].placement == nil)
    let placed = try #require(loaded.screens[1].placement)
    let saved = try #require(settings.screens[1].placement)
    #expect(simd_distance(placed.direction, saved.direction) < 1e-5)
    #expect(abs(placed.distance - saved.distance) < 1e-5)

    loaded.screens = settings.screens
    #expect(loaded == settings)
}

@Test func virtualScreenSizeFromBeforeSeveralScreensStillLoads() {
    let settings = Settings.parse("source=virtual\nvirtual_width=5120\nvirtual_height=1440\n")
    #expect(settings.screens == [RoomScreen(width: 5120, height: 1440)])
}

@Test func malformedScreensAreSkipped() {
    let settings = Settings.parse("screen=banana\nscreen=2880x1620\nscreen=0x100\n")
    #expect(settings.screens == [RoomScreen(width: 2880, height: 1620)])
}

@Test func malformedValuesFallBackToDefaults() {
    let settings = Settings.parse("zoom_index=banana\nsensitivity=1.2\nnonsense\ndeadzone_index=-1\nsource=hologram\n")
    #expect(settings.zoomIndex == Settings().zoomIndex)
    #expect(settings.deadzoneIndex == Settings().deadzoneIndex)
    #expect(settings.source == Settings().source)
    #expect(settings.sensitivity == 1.2)
}

@Test func settingsWrittenByTheRustVersionStillLoad() {
    let rust = """
        zoom_index=4
        sensitivity=0.85
        deadzone_index=1
        prediction=false
        overlay_visible=true
        gyro_bias_x=0.0012
        gyro_bias_y=-0.0034
        gyro_bias_z=0.00056

        """
    let settings = Settings.parse(rust)
    #expect(settings.zoomIndex == 4)
    #expect(settings.sensitivity == 0.85)
    #expect(settings.deadzoneIndex == 1)
    #expect(settings.prediction == false)
    #expect(settings.gyroBias == SIMD3(0.0012, -0.0034, 0.00056))
}
