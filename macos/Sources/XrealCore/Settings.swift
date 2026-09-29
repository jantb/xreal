import Foundation

/// How far, in glasses pixels, the head may move before the canvas follows.
/// Enough to hide the head's tremor and a heartbeat's jolt: a resting pulse
/// moved the head 3-5 pixels per beat when measured with `--record`.
public let defaultSteadiness: Float = 6
public let maxSteadiness: Float = 12

/// The most the prediction lead can be lengthened, in milliseconds.
public let maxLatencyTrimMs: Float = 40

/// The flattest curve kept, as a multiple of a screen's distance.
private let maxCurveRadius: Float = 10

/// User-tunable settings, persisted between runs as `key=value` lines. Files
/// written by the old Rust version of the app still load.
public struct Settings: Equatable, Sendable {
    public var prediction = true
    /// Milliseconds added to how far ahead the head pose is predicted, to
    /// make up for frames reaching the glasses later than assumed.
    public var latencyTrimMs: Float = 0
    /// How many glasses pixels the head can move without the canvas
    /// following, to hide a heartbeat's jolt and other tremor.
    public var steadiness: Float = defaultSteadiness
    /// Whether the diagnostics are expanded in the controls window. Saved as
    /// `overlay_visible`, from when they were drawn over the picture.
    public var diagnosticsVisible = true
    /// Gyro bias at the tracking's reference temperature, rad/s.
    public var gyroBias: SIMD3<Float> = .zero
    /// How the gyro bias changes per °C, rad/s, as learned so far.
    public var gyroBiasSlope: SIMD3<Float> = .zero
    /// The one virtual screen, shown only in the glasses: wide and curved,
    /// straight ahead. macOS 27 refuses some standard sizes, such as
    /// 5760 × 2160; this one works.
    public var canvas = RoomScreen(width: 5752, height: 2160, curved: true)
    /// The canvas tilts with the head so it stays level.
    public var followRoll = true
    /// Zoom out while the mouse moves outside the view.
    public var followCursor = true
    /// The radius the canvas bends with when curved, as a multiple of its
    /// distance: 1 surrounds the viewer evenly, less bends it more, more
    /// bends it less.
    public var curveRadius: Float = 1
    /// Draws each eye through its lens's calibrated distortion, so straight
    /// edges stay straight to the corners of the view.
    public var lensCorrection = true
    /// How many metres a room unit is: the canvas at distance 1 is this far
    /// away. Nearer shows more depth between its parts.
    public var metresPerRoomUnit: Float = 1

    public init() {}

    public static var fileURL: URL {
        FileManager.default.homeDirectoryForCurrentUser
            .appending(path: "Library/Application Support/xreal/settings.txt")
    }

    public static func load() -> Settings {
        guard let text = try? String(contentsOf: fileURL, encoding: .utf8) else {
            return Settings()
        }
        return parse(text)
    }

    public func save() throws {
        let url = Self.fileURL
        try FileManager.default.createDirectory(
            at: url.deletingLastPathComponent(), withIntermediateDirectories: true)
        try serialize().write(to: url, atomically: true, encoding: .utf8)
    }

    /// Unknown keys and malformed values fall back to the defaults.
    public static func parse(_ text: String) -> Settings {
        var settings = Settings()
        var canvas: RoomScreen?
        // Written when there were several screens: the first one used with
        // the glasses alone was the canvas.
        var glassesOnlyScreen: RoomScreen?
        for line in text.split(whereSeparator: \.isNewline) {
            guard let separator = line.firstIndex(of: "=") else { continue }
            let key = line[..<separator].trimmingCharacters(in: .whitespaces)
            let value = line[line.index(after: separator)...].trimmingCharacters(in: .whitespaces)
            switch key {
            case "prediction": parse(value, into: &settings.prediction)
            case "overlay_visible": parse(value, into: &settings.diagnosticsVisible)
            case "gyro_bias_x": parseFinite(value, into: &settings.gyroBias.x)
            case "gyro_bias_y": parseFinite(value, into: &settings.gyroBias.y)
            case "gyro_bias_z": parseFinite(value, into: &settings.gyroBias.z)
            case "gyro_bias_slope_x": parseFinite(value, into: &settings.gyroBiasSlope.x)
            case "gyro_bias_slope_y": parseFinite(value, into: &settings.gyroBiasSlope.y)
            case "gyro_bias_slope_z": parseFinite(value, into: &settings.gyroBiasSlope.z)
            case "latency_trim_ms":
                if let trim = Float(value), trim.isFinite {
                    settings.latencyTrimMs = min(max(trim, 0), maxLatencyTrimMs)
                }
            case "steadiness_px":
                if let pixels = Float(value), pixels.isFinite {
                    settings.steadiness = min(max(pixels, 0), maxSteadiness)
                }
            case "canvas": canvas = canvas ?? parseScreen(value)
            case "glasses_only_screen": glassesOnlyScreen = glassesOnlyScreen ?? parseScreen(value)
            case "follow_roll": parse(value, into: &settings.followRoll)
            case "follow_cursor": parse(value, into: &settings.followCursor)
            case "lens_correction": parse(value, into: &settings.lensCorrection)
            case "metres_per_room_unit":
                if let metres = Float(value), metres.isFinite, metres > 0 {
                    settings.metresPerRoomUnit = min(max(metres, 0.25), 20)
                }
            case "curve_radius", "sphere_curve":
                if let radius = Float(value), radius.isFinite, radius > 0 {
                    settings.curveRadius = min(radius, maxCurveRadius)
                }
            default: break
            }
        }
        if let screen = canvas ?? glassesOnlyScreen {
            settings.canvas = screen
        }
        return settings
    }

    public func serialize() -> String {
        let lines = [
            "prediction=\(prediction)",
            "overlay_visible=\(diagnosticsVisible)",
            "latency_trim_ms=\(latencyTrimMs)",
            "steadiness_px=\(steadiness)",
            "gyro_bias_x=\(gyroBias.x)",
            "gyro_bias_y=\(gyroBias.y)",
            "gyro_bias_z=\(gyroBias.z)",
            "gyro_bias_slope_x=\(gyroBiasSlope.x)",
            "gyro_bias_slope_y=\(gyroBiasSlope.y)",
            "gyro_bias_slope_z=\(gyroBiasSlope.z)",
            "canvas=\(Self.serialize(canvas))",
            "follow_roll=\(followRoll)",
            "follow_cursor=\(followCursor)",
            "curve_radius=\(curveRadius)",
            "lens_correction=\(lensCorrection)",
            "metres_per_room_unit=\(metresPerRoomUnit)",
        ]
        return lines.map { $0 + "\n" }.joined()
    }

    /// `WIDTHxHEIGHT`, `,curved` for a curved screen, then
    /// `@x,y,z,distance` for where it hangs, and `,tilt` if it is tilted.
    private static func serialize(_ screen: RoomScreen) -> String {
        let size = "\(screen.width)x\(screen.height)" + (screen.curved ? ",curved" : "")
        let placement = screen.placement
        let d = placement.direction
        return size + "@\(d.x),\(d.y),\(d.z),\(placement.distance)"
            + (placement.tilt == 0 ? "" : ",\(placement.tilt)")
    }

    private static func parseScreen(_ value: String) -> RoomScreen? {
        let parts = value.split(separator: "@", maxSplits: 1)
        guard let shape = parts.first?.split(separator: ","), let dimensions = shape.first else { return nil }
        let size = dimensions.split(separator: "x")
        guard size.count == 2, let width = UInt(size[0]).flatMap({ Int(exactly: $0) }),
            let height = UInt(size[1]).flatMap({ Int(exactly: $0) }), isScreenSize(width, height)
        else { return nil }
        var screen = RoomScreen(width: width, height: height, curved: shape.dropFirst().contains("curved"))
        if parts.count == 2 {
            let numbers = parts[1].split(separator: ",").compactMap { Float($0) }
            // Written before screens could be tilted: without the tilt.
            if numbers.count == 4 || numbers.count == 5, numbers.allSatisfy(\.isFinite) {
                screen.placement = ScreenPlacement(
                    direction: SIMD3(numbers[0], numbers[1], numbers[2]), distance: numbers[3],
                    tilt: numbers.count == 5 ? numbers[4] : 0)
            }
        }
        return screen
    }

    /// Sizes a virtual screen can be created at without upsetting macOS.
    private static func isScreenSize(_ width: Int, _ height: Int) -> Bool {
        (1...maxVirtualScreenSide).contains(width) && (1...maxVirtualScreenSide).contains(height)
    }

    /// A saved bias that is not a number would otherwise be clamped to the
    /// largest bias and make the view drift.
    private static func parseFinite(_ value: String, into target: inout Float) {
        if let parsed = Float(value), parsed.isFinite {
            target = parsed
        }
    }

    private static func parse<T: LosslessStringConvertible>(_ value: String, into target: inout T) {
        if let parsed = T(value) {
            target = parsed
        }
    }

}
