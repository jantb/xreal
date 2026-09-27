import Foundation

/// What the glasses show: the main display, or a virtual display that only
/// exists inside the glasses.
public enum CaptureSource: String, Sendable {
    case mirror
    case virtual
}

/// How the source image is laid out in front of the viewer.
public enum Projection: String, Sendable, CaseIterable {
    /// A pixel-exact window onto the source, moved by head turns.
    case crop
    /// A flat virtual monitor fixed in the room.
    case flat
    /// A virtual monitor curved around the viewer, fixed in the room.
    case curved
}

/// What the crop projection shows when looking past the edge of the source.
public enum EdgeMode: String, Sendable {
    /// The view stops at the edge.
    case snap
    /// The view keeps moving and shows black beyond the edge.
    case black
}

/// User-tunable settings, persisted between runs as `key=value` lines. The
/// file is shared with the Rust version of the app.
public struct Settings: Equatable, Sendable {
    public var zoomIndex = 2
    public var sensitivity: Float = 1.0
    public var deadzoneIndex = 3
    public var prediction = true
    /// Whether the status lines show in the menu bar menu.
    public var overlayVisible = true
    public var gyroBias: SIMD3<Float> = .zero
    public var source = CaptureSource.mirror
    // macOS 27 refuses virtual displays exactly 3840 pixels wide (3838 and
    // 3900 both work), so the default stays just below 4K.
    public var virtualWidth = 3832
    public var virtualHeight = 2160
    public var projection = Projection.crop
    /// Room projections tilt with the head so the screen stays level.
    public var followRoll = true
    public var edge = EdgeMode.snap
    /// Zoom out while the mouse moves outside the view.
    public var followCursor = true

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
        for line in text.split(whereSeparator: \.isNewline) {
            guard let separator = line.firstIndex(of: "=") else { continue }
            let key = line[..<separator].trimmingCharacters(in: .whitespaces)
            let value = line[line.index(after: separator)...].trimmingCharacters(in: .whitespaces)
            switch key {
            case "zoom_index": parse(value, into: &settings.zoomIndex, as: UInt.self)
            case "sensitivity": parse(value, into: &settings.sensitivity)
            case "deadzone_index": parse(value, into: &settings.deadzoneIndex, as: UInt.self)
            case "prediction": parse(value, into: &settings.prediction)
            case "overlay_visible": parse(value, into: &settings.overlayVisible)
            case "gyro_bias_x": parse(value, into: &settings.gyroBias.x)
            case "gyro_bias_y": parse(value, into: &settings.gyroBias.y)
            case "gyro_bias_z": parse(value, into: &settings.gyroBias.z)
            case "source": settings.source = CaptureSource(rawValue: value) ?? settings.source
            case "virtual_width": parse(value, into: &settings.virtualWidth, as: UInt.self)
            case "virtual_height": parse(value, into: &settings.virtualHeight, as: UInt.self)
            case "projection": settings.projection = Projection(rawValue: value) ?? settings.projection
            case "follow_roll": parse(value, into: &settings.followRoll)
            case "edge": settings.edge = EdgeMode(rawValue: value) ?? settings.edge
            case "follow_cursor": parse(value, into: &settings.followCursor)
            default: break
            }
        }
        return settings
    }

    public func serialize() -> String {
        """
        zoom_index=\(zoomIndex)
        sensitivity=\(sensitivity)
        deadzone_index=\(deadzoneIndex)
        prediction=\(prediction)
        overlay_visible=\(overlayVisible)
        gyro_bias_x=\(gyroBias.x)
        gyro_bias_y=\(gyroBias.y)
        gyro_bias_z=\(gyroBias.z)
        source=\(source.rawValue)
        virtual_width=\(virtualWidth)
        virtual_height=\(virtualHeight)
        projection=\(projection.rawValue)
        follow_roll=\(followRoll)
        edge=\(edge.rawValue)
        follow_cursor=\(followCursor)

        """
    }

    private static func parse<T: LosslessStringConvertible>(_ value: String, into target: inout T) {
        if let parsed = T(value) {
            target = parsed
        }
    }

    /// Parses through an unsigned type so negative indices are rejected.
    private static func parse(_ value: String, into target: inout Int, as _: UInt.Type) {
        if let parsed = UInt(value), let fits = Int(exactly: parsed) {
            target = fits
        }
    }
}
