import Foundation

/// The range of the viewing distance, in metres: how far away the canvas
/// at distance 1 is.
public let minViewingDistance: Float = 0.5
public let maxViewingDistance: Float = 20

/// The range of the curve's radius, as multiples of the canvas's distance:
/// the smaller the radius, the stronger the curve.
public let minCurveRadius: Float = 0.5
public let maxCurveRadius: Float = 5

/// How often macOS can draw the canvas, in frames per second. The glasses
/// still show every one of their own refreshes; this is how often what is on
/// the canvas can change.
public let canvasRefreshRates = [60, 90]

/// How far away, in metres, the glasses' optics show their picture in
/// focus. With the stereo depth set the same, the eyes aim and focus at one
/// distance, which is easiest on them.
public let glassesFocusDistance: Float = 4

/// `metres` as the viewing distance slider leaves it: close to the
/// glasses' focus it snaps onto it.
public func snappedViewingDistance(_ metres: Float) -> Float {
    abs(log(metres / glassesFocusDistance)) < 0.08 ? glassesFocusDistance : metres
}

/// A window to show above the canvas, found by its app and title.
public struct PinnedWindow: Equatable, Hashable, Sendable {
    public var bundleID: String
    public var title: String

    public init(bundleID: String, title: String) {
        self.bundleID = bundleID
        self.title = title
    }
}

/// User-tunable settings, persisted between runs as `key=value` lines.
/// Keys no longer used are ignored.
public struct Settings: Equatable, Sendable {
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
    /// The radius the canvas bends with when curved, as a multiple of its
    /// distance: 1 surrounds the viewer evenly, less bends it more, more
    /// bends it less.
    public var curveRadius: Float = 1
    /// How many metres a room unit is: the canvas at distance 1 is this far
    /// away. Nearer shows more depth between its parts. It starts where the
    /// glasses' optics focus.
    public var metresPerRoomUnit: Float = glassesFocusDistance
    /// Fades the last few pixels of the canvas and what hangs above it, so
    /// they end softly against the room.
    public var softEdges = true
    /// Switches the Mac's own screen off while the glasses are in use: with
    /// it on, the glasses drop frames every few seconds.
    public var laptopScreenOff = true
    /// Lights the room round the canvas in the colours of its edges.
    public var ambientLight = false
    /// How often macOS draws the canvas; one of `canvasRefreshRates`.
    public var canvasRefreshRate = 90
    /// Sharpens the canvas where it shows about one pixel per glasses pixel.
    public var sharpFiltering = true
    /// Shows the dashboard above the canvas. Saved as `status_strip`, from
    /// when it was a line of text.
    public var statusStrip = true
    public var pinnedWindow: PinnedWindow?

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
        var verticalWrap: Float?
        var evenSize: Bool?
        for line in text.split(whereSeparator: \.isNewline) {
            guard let separator = line.firstIndex(of: "=") else { continue }
            let key = line[..<separator].trimmingCharacters(in: .whitespaces)
            let value = line[line.index(after: separator)...].trimmingCharacters(in: .whitespaces)
            switch key {
            case "gyro_bias_x": parseFinite(value, into: &settings.gyroBias.x)
            case "gyro_bias_y": parseFinite(value, into: &settings.gyroBias.y)
            case "gyro_bias_z": parseFinite(value, into: &settings.gyroBias.z)
            case "gyro_bias_slope_x": parseFinite(value, into: &settings.gyroBiasSlope.x)
            case "gyro_bias_slope_y": parseFinite(value, into: &settings.gyroBiasSlope.y)
            case "gyro_bias_slope_z": parseFinite(value, into: &settings.gyroBiasSlope.z)
            case "canvas": canvas = canvas ?? parseScreen(value)
            case "follow_roll": parse(value, into: &settings.followRoll)
            case "metres_per_room_unit":
                if let metres = Float(value), metres.isFinite, metres > 0 {
                    settings.metresPerRoomUnit = min(max(metres, minViewingDistance), maxViewingDistance)
                }
            case "canvas_refresh_rate":
                if let rate = Int(value), canvasRefreshRates.contains(rate) {
                    settings.canvasRefreshRate = rate
                }
            case "sharp_filtering": parse(value, into: &settings.sharpFiltering)
            case "soft_edges": parse(value, into: &settings.softEdges)
            case "laptop_screen_off": parse(value, into: &settings.laptopScreenOff)
            case "ambient_light": parse(value, into: &settings.ambientLight)
            case "even_text_size":
                var even = true
                parse(value, into: &even)
                evenSize = even
            case "status_strip": parse(value, into: &settings.statusStrip)
            case "pinned_window":
                // `BUNDLE_ID|TITLE`; a title may hold anything but a line break.
                let parts = value.split(separator: "|", maxSplits: 1, omittingEmptySubsequences: false)
                if parts.count == 2, !parts[0].isEmpty {
                    settings.pinnedWindow = PinnedWindow(bundleID: String(parts[0]), title: String(parts[1]))
                }
            case "vertical_wrap":
                if let wrap = Float(value), wrap.isFinite {
                    verticalWrap = min(max(wrap, 0), 1)
                }
            case "curve_radius":
                if let radius = Float(value), radius.isFinite, radius > 0 {
                    settings.curveRadius = min(max(radius, minCurveRadius), maxCurveRadius)
                }
            default: break
            }
        }
        if let screen = canvas {
            settings.canvas = screen
        }
        if let verticalWrap {
            settings.canvas.verticalWrap = verticalWrap
        }
        if let evenSize {
            settings.canvas.evenSize = evenSize
        }
        return settings
    }

    public func serialize() -> String {
        let lines = [
            "gyro_bias_x=\(gyroBias.x)",
            "gyro_bias_y=\(gyroBias.y)",
            "gyro_bias_z=\(gyroBias.z)",
            "gyro_bias_slope_x=\(gyroBiasSlope.x)",
            "gyro_bias_slope_y=\(gyroBiasSlope.y)",
            "gyro_bias_slope_z=\(gyroBiasSlope.z)",
            "canvas=\(Self.serialize(canvas))",
            "vertical_wrap=\(canvas.verticalWrap)",
            "even_text_size=\(canvas.evenSize)",
            "follow_roll=\(followRoll)",
            "curve_radius=\(curveRadius)",
            "metres_per_room_unit=\(metresPerRoomUnit)",
            "canvas_refresh_rate=\(canvasRefreshRate)",
            "sharp_filtering=\(sharpFiltering)",
            "soft_edges=\(softEdges)",
            "laptop_screen_off=\(laptopScreenOff)",
            "ambient_light=\(ambientLight)",
            "status_strip=\(statusStrip)",
        ] + (pinnedWindow.map { ["pinned_window=\($0.bundleID)|\($0.title.filter { !$0.isNewline })"] } ?? [])
        return lines.map { $0 + "\n" }.joined()
    }

    /// `WIDTHxHEIGHT`, `,2x` for a HiDPI screen, `,curved` for a curved one,
    /// `,sphere` for a spherical one, then
    /// `@x,y,z,distance` for where it hangs, and `,tilt` if it is tilted.
    private static func serialize(_ screen: RoomScreen) -> String {
        let size =
            "\(screen.width)x\(screen.height)" + (screen.scale == 2 ? ",2x" : "") + (screen.curved ? ",curved" : "")
            + (screen.spherical ? ",sphere" : "")
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
            let height = UInt(size[1]).flatMap({ Int(exactly: $0) })
        else { return nil }
        let scale = shape.dropFirst().contains("2x") ? 2 : 1
        guard RoomScreen.isAllowed(width: width, height: height, scale: scale) else { return nil }
        var screen = RoomScreen(
            width: width, height: height, scale: scale, curved: shape.dropFirst().contains("curved"),
            spherical: shape.dropFirst().contains("sphere"))
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
