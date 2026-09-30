import Testing
import simd

@testable import XrealCore

@Test func savedSettingsLoadBackUnchanged() throws {
    var settings = Settings()
    settings.gyroBias = SIMD3(0.0012, -0.0034, 0.00056)
    settings.gyroBiasSlope = SIMD3(0.00005, -0.0001, 0.00002)
    settings.canvas = RoomScreen(
        width: 7672, height: 2160, placement: ScreenPlacement(direction: SIMD3(0.4, 0.3, -1), distance: 1.5, tilt: -0.3),
        curved: true)
    settings.followRoll = false

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
    let settings = Settings.parse("follow_roll=banana\nnonsense\ncurve_radius=-1\nsoft_edges=false\n")
    #expect(settings.followRoll == Settings().followRoll)
    #expect(settings.curveRadius == Settings().curveRadius)
    #expect(settings.softEdges == false)
}

@Test func settingsNoLongerUsedAreIgnoredAndTheRestStillLoads() {
    let old = """
        prediction=false
        lens_correction=false
        latency_trim_ms=12
        steady_laptop_screen=false
        gyro_bias_x=0.0012
        gyro_bias_y=-0.0034
        gyro_bias_z=0.00056
        follow_roll=false

        """
    var expected = Settings()
    expected.gyroBias = SIMD3(0.0012, -0.0034, 0.00056)
    expected.followRoll = false
    #expect(Settings.parse(old) == expected)
}

@Test func theCurveStaysChosenAfterARestart() {
    var settings = Settings()
    settings.curveRadius = 2.5
    #expect(Settings.parse(settings.serialize()).curveRadius == 2.5)
}

@Test func savedCurvesAndDistancesOutsideTheSlidersLoadWithinThem() {
    #expect(Settings.parse("curve_radius=50\n").curveRadius == maxCurveRadius)
    #expect(Settings.parse("curve_radius=0.1\n").curveRadius == minCurveRadius)
    #expect(Settings.parse("metres_per_room_unit=0.3\n").metresPerRoomUnit == minViewingDistance)
    #expect(Settings.parse("metres_per_room_unit=90\n").metresPerRoomUnit == maxViewingDistance)
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

@Test func aSavedBiasThatIsNotANumberIsIgnored() {
    let settings = Settings.parse("gyro_bias_x=nan\ngyro_bias_y=inf\ngyro_bias_slope_z=nan\ngyro_bias_z=0.002\n")
    #expect(settings.gyroBias == SIMD3(0, 0, 0.002))
    #expect(settings.gyroBiasSlope == .zero)
}

@Test func theNewerViewingChoicesStayChosenAfterARestart() {
    var settings = Settings()
    settings.canvas = RoomScreen(width: 2880, height: 1620, scale: 2, curved: true, spherical: true, verticalWrap: 0.6)
    settings.canvasRefreshRate = 60
    settings.sharpFiltering = false
    settings.statusStrip = false
    settings.pinnedWindow = PinnedWindow(bundleID: "com.apple.Music", title: "Music | a=b")
    #expect(Settings.parse(settings.serialize()) == settings)
}

@Test func aHiDPICanvasTooLargeForMacOSIsSkipped() {
    let loaded = Settings.parse("canvas=5752x2160,2x,curved@0,0,-1,1\n")
    #expect(loaded.canvas == Settings().canvas)
}

@Test func anUnknownRefreshRateFallsBackToTheDefault() {
    #expect(Settings.parse("canvas_refresh_rate=240\n").canvasRefreshRate == Settings().canvasRefreshRate)
}

@Test func aCanvasTooLargeToCountIsSkippedRatherThanCrashing() {
    let loaded = Settings.parse("canvas=9223372036854775807x1,2x@0,0,-1,1\n")
    #expect(loaded.canvas == Settings().canvas)
}

@Test func onlyTheSizesProbedPastTheLimitLoadPastIt() {
    #expect(Settings.parse("canvas=5120x2160,2x,curved@0,0,-1,1\n").canvas.width == 5120)
    #expect(Settings.parse("canvas=5120x1440,2x@0,0,-1,1\n").canvas.height == 1440)
    for other in ["5120x2880,2x", "6144x2160,2x", "10240x2160", "5120x2161,2x"] {
        #expect(Settings.parse("canvas=\(other)@0,0,-1,1\n").canvas == Settings().canvas, "\(other)")
    }
}

@Test func theViewingDistanceSnapsToTheGlassesFocusWhenCloseAndStartsThere() {
    #expect(snappedViewingDistance(3.8) == glassesFocusDistance)
    #expect(snappedViewingDistance(4.25) == glassesFocusDistance)
    #expect(snappedViewingDistance(2) == 2)
    #expect(snappedViewingDistance(6) == 6)
    #expect(Settings().metresPerRoomUnit == glassesFocusDistance)
}

@Test func softEdgesAndEvenTextSizeStayChosenAfterARestart() {
    var settings = Settings()
    settings.softEdges = false
    settings.canvas.evenSize = false
    settings.laptopScreenOff = false
    settings.ambientLight = true
    let loaded = Settings.parse(settings.serialize())
    #expect(loaded.ambientLight && !Settings().ambientLight)
    #expect(!loaded.laptopScreenOff)
    #expect(Settings().laptopScreenOff)
    #expect(!loaded.softEdges)
    #expect(!loaded.canvas.evenSize)
    #expect(Settings.parse("").softEdges && Settings.parse("").canvas.evenSize)
}
