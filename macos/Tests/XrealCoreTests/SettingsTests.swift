import Testing
import simd

@testable import XrealCore

@Test func savedSettingsLoadBackUnchanged() throws {
    var settings = Settings()
    settings.prediction = false
    settings.overlayVisible = false
    settings.gyroBias = SIMD3(0.0012, -0.0034, 0.00056)
    settings.gyroBiasSlope = SIMD3(0.00005, -0.0001, 0.00002)
    settings.canvas = RoomScreen(
        width: 7672, height: 2160, placement: ScreenPlacement(direction: SIMD3(0.4, 0.3, -1), distance: 1.5, tilt: -0.3),
        curved: true)
    settings.followRoll = false
    settings.followCursor = false

    var loaded = Settings.parse(settings.serialize())
    #expect(loaded.canvas.width == 7672 && loaded.canvas.height == 2160 && loaded.canvas.curved)
    let placed = loaded.canvas.placement
    let saved = settings.canvas.placement
    #expect(simd_distance(placed.direction, saved.direction) < 1e-5)
    #expect(abs(placed.distance - saved.distance) < 1e-5)
    #expect(abs(placed.tilt - saved.tilt) < 1e-6)

    loaded.canvas = settings.canvas
    #expect(loaded == settings)
}

@Test func theCanvasFromBeforeThereWasOnlyOneKeepsItsPlace() throws {
    // As saved when there were several screens and ways to show them.
    let old = """
        zoom_index=2
        sensitivity=1.0
        deadzone_index=3
        prediction=true
        source=virtual_only
        screen=5120x1440@0.0,0.4,-0.9,1.0
        screen=2880x1620
        screen=2880x1620
        glasses_only_screen=5752x2160,curved@-0.0968999,0.0034615872,-0.9952881,1.2099997
        projection=curved
        edge=black
        stereo=true
        curve_radius=1.0

        """
    let canvas = Settings.parse(old).canvas
    #expect(canvas.width == 5752 && canvas.height == 2160 && canvas.curved)
    let expected = simd_normalize(SIMD3<Float>(-0.0968999, 0.0034615872, -0.9952881))
    #expect(simd_distance(canvas.placement.direction, expected) < 1e-5)
    #expect(abs(canvas.placement.distance - 1.2099997) < 1e-5)
    #expect(canvas.placement.tilt == 0)
}

@Test func theCanvasLineWinsOverOlderScreens() {
    let settings = Settings.parse("glasses_only_screen=1920x1080\ncanvas=3832x2160,curved\n")
    #expect(settings.canvas == RoomScreen(width: 3832, height: 2160, curved: true))
}

@Test func malformedCanvasesAreSkipped() {
    #expect(Settings.parse("canvas=banana\n").canvas == Settings().canvas)
    #expect(Settings.parse("canvas=0x100\n").canvas == Settings().canvas)
    #expect(Settings.parse("canvas=banana\ncanvas=2880x1620\n").canvas == RoomScreen(width: 2880, height: 1620))
}

@Test func canvasesTooLargeForMacOSAreSkipped() {
    #expect(Settings.parse("canvas=16384x2160\n").canvas == Settings().canvas)
    #expect(Settings.parse("canvas=99999x100\ncanvas=7672x2160,curved\n").canvas.width == 7672)
}

@Test func malformedValuesFallBackToDefaults() {
    let settings = Settings.parse("prediction=banana\nnonsense\ncurve_radius=-1\nfollow_cursor=false\n")
    #expect(settings.prediction == Settings().prediction)
    #expect(settings.curveRadius == Settings().curveRadius)
    #expect(settings.followCursor == false)
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
    #expect(settings.prediction == false)
    #expect(settings.overlayVisible == true)
    #expect(settings.gyroBias == SIMD3(0.0012, -0.0034, 0.00056))
}

@Test func theCurveStaysChosenAfterARestart() {
    var settings = Settings()
    settings.curveRadius = 2.5
    #expect(Settings.parse(settings.serialize()).curveRadius == 2.5)
    // Saved before it was renamed.
    #expect(Settings.parse("sphere_curve=1.5\n").curveRadius == 1.5)
}

@Test func theDepthScaleStaysChosenAfterARestart() {
    var settings = Settings()
    settings.metresPerRoomUnit = 2
    #expect(Settings.parse(settings.serialize()).metresPerRoomUnit == 2)
}

@Test func aFreshStartShowsAWideCurvedCanvasStraightAhead() {
    let canvas = Settings.parse("").canvas
    #expect(canvas.width == 5752 && canvas.height == 2160 && canvas.curved)
    #expect(canvas.placement == .straightAhead)
}

@Test func aFarViewingDistanceIsKeptAfterARestart() {
    var settings = Settings()
    settings.metresPerRoomUnit = 20
    #expect(Settings.parse(settings.serialize()).metresPerRoomUnit == 20)
}

@Test func lensCorrectionIsOnUntilTurnedOffAndStaysOff() {
    #expect(Settings.parse("").lensCorrection)
    var settings = Settings()
    settings.lensCorrection = false
    #expect(!Settings.parse(settings.serialize()).lensCorrection)
}
