import Testing

@testable import XrealCore

@Test func savedSettingsLoadBackUnchanged() {
    var settings = Settings()
    settings.zoomIndex = 4
    settings.sensitivity = 0.85
    settings.deadzoneIndex = 0
    settings.prediction = false
    settings.overlayVisible = false
    settings.gyroBias = SIMD3(0.0012, -0.0034, 0.00056)
    settings.source = .virtual
    settings.virtualWidth = 5120
    settings.virtualHeight = 1440
    settings.projection = .curved
    settings.followRoll = false
    settings.edge = .black
    settings.followCursor = false

    #expect(Settings.parse(settings.serialize()) == settings)
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
