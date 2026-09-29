import Foundation
import simd

// Points between the canvas's top edge and the status strip.
private let overheadGap: Float = 48
// Points between the status strip and the pinned window above it.
private let overheadSpacing: Float = 24

/// A part of a screen's surface, in its positions: from -1 to 1 across, and
/// from -1 at the bottom to 1 at the top, beyond 1 above it. The surface
/// carries on past its edges the way it runs: a curved screen's rows keep
/// their arc and its columns stay straight.
public struct SurfaceRect: Equatable, Sendable {
    public var left: Float
    public var right: Float
    public var top: Float
    public var bottom: Float

    public init(left: Float, right: Float, top: Float, bottom: Float) {
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom
    }

    /// The whole screen.
    public static let whole = SurfaceRect(left: -1, right: 1, top: 1, bottom: -1)
}

/// Where the status strip and the pinned window hang, `strip` and `pinned`
/// points in size: above the canvas's top edge, centred on it, the strip
/// nearest and the window above the strip. Either is shrunk to the canvas's
/// width if it is wider.
public func overheadLayout(canvas: RoomScreen, strip: SIMD2<Float>?, pinned: SIMD2<Float>?)
    -> (strip: SurfaceRect?, pinned: SurfaceRect?)
{
    let size = SIMD2(Float(max(canvas.width, 1)), Float(max(canvas.height, 1)))
    var bottom = 1 + overheadGap * 2 / size.y
    func place(_ extent: SIMD2<Float>?) -> SurfaceRect? {
        guard let extent, extent.x > 0, extent.y > 0 else { return nil }
        let fitted = extent * min(1, size.x / extent.x)
        let half = fitted.x / size.x
        let rect = SurfaceRect(left: -half, right: half, top: bottom + fitted.y * 2 / size.y, bottom: bottom)
        bottom = rect.top + overheadSpacing * 2 / size.y
        return rect
    }
    let stripRect = place(strip)
    return (stripRect, place(pinned))
}

/// Where a pointer image `size` points large, with its hot spot `hotSpot`
/// points from its top-left corner, covers the canvas when the mouse is at
/// `point`, in the canvas's points from its top-left corner.
public func pointerRect(canvas: RoomScreen, at point: SIMD2<Float>, size: SIMD2<Float>, hotSpot: SIMD2<Float>)
    -> SurfaceRect
{
    let extent = SIMD2(Float(max(canvas.width, 1)), Float(max(canvas.height, 1)))
    let topLeft = (point - hotSpot) / extent
    let bottomRight = (point - hotSpot + size) / extent
    return SurfaceRect(
        left: topLeft.x * 2 - 1, right: bottomRight.x * 2 - 1, top: 1 - topLeft.y * 2, bottom: 1 - bottomRight.y * 2)
}

/// What the status strip tells, each nil when not known.
public struct StatusReadings: Sendable {
    /// The time of day, already formatted.
    public var clock: String
    public var battery: Battery?
    /// Share of the CPU in use, 0 to 1.
    public var cpuLoad: Float?
    public var memory: MemoryUse?
    /// The glasses' IMU temperature, °C.
    public var glassesTemperature: Float?
    /// Head tracking samples per second, nil while the glasses are not found.
    public var trackingHz: Float?
    /// Frames drawn per second.
    public var fps: Float
    /// Seconds from sampling the head pose to the frame reaching the
    /// glasses.
    public var latency: Double?
    public var lateFramesPerSecond: Float

    public init(
        clock: String, battery: Battery? = nil, cpuLoad: Float? = nil, memory: MemoryUse? = nil,
        glassesTemperature: Float? = nil, trackingHz: Float? = nil, fps: Float = 0, latency: Double? = nil,
        lateFramesPerSecond: Float = 0
    ) {
        self.clock = clock
        self.battery = battery
        self.cpuLoad = cpuLoad
        self.memory = memory
        self.glassesTemperature = glassesTemperature
        self.trackingHz = trackingHz
        self.fps = fps
        self.latency = latency
        self.lateFramesPerSecond = lateFramesPerSecond
    }
}

/// The status strip's items, left to right. Readings that are not known
/// are left out.
public func statusItems(_ readings: StatusReadings) -> [String] {
    var items = [readings.clock]
    if let battery = readings.battery {
        items.append("Battery \(battery.percent)%" + (battery.charging ? ", charging" : ""))
    }
    if let load = readings.cpuLoad {
        items.append(String(format: "CPU %.0f%%", load * 100))
    }
    if let memory = readings.memory {
        let gigabytes = { (bytes: UInt64) in Double(bytes) / 1_073_741_824 }
        items.append(String(format: "RAM %.1f of %.0f GB", gigabytes(memory.used), gigabytes(memory.total)))
    }
    if let temperature = readings.glassesTemperature {
        items.append(String(format: "Glasses %.1f °C", temperature))
    }
    items.append(readings.trackingHz.map { String(format: "Tracking %.0f Hz", $0) } ?? "Tracking lost")
    var frames = String(format: "%.0f fps", readings.fps)
    if let latency = readings.latency {
        frames += String(format: ", %.0f ms", latency * 1000)
    }
    if readings.lateFramesPerSecond > 0 {
        frames += String(format: ", %.1f late/s", readings.lateFramesPerSecond)
    }
    items.append(frames)
    return items
}
