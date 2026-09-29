import AppKit
import Metal
import Synchronization
import XrealCore

// Each tile's size and the space between tiles, in points.
private let tileSize = CGSize(width: 210, height: 172)
private let tileSpacing: CGFloat = 14
private let tilePadding: CGFloat = 14
// How far colour glows around bars and lines, in points.
private let glowRadius: CGFloat = 7

/// What the dashboard tells about the glasses and the viewer.
struct GlassesReadings: Sendable {
    /// The IMU's temperature, °C.
    var temperature: Float?
    /// Head tracking samples per second, nil while the glasses are not found.
    var trackingHz: Float?
    var fps: Float
    /// Seconds from taking the head pose to the frame reaching the glasses.
    var latency: Double?
    var lateFramesPerSecond: Float
}

/// Colours that glow on the glasses, where black shows nothing.
private enum Palette {
    static let time = NSColor(srgbRed: 0.55, green: 0.85, blue: 1, alpha: 1)
    static let cpu = NSColor(srgbRed: 0.25, green: 1, blue: 0.62, alpha: 1)
    static let memory = NSColor(srgbRed: 0.68, green: 0.52, blue: 1, alpha: 1)
    static let gpu = NSColor(srgbRed: 1, green: 0.62, blue: 0.26, alpha: 1)
    static let down = NSColor(srgbRed: 0.24, green: 0.84, blue: 1, alpha: 1)
    static let up = NSColor(srgbRed: 1, green: 0.36, blue: 0.8, alpha: 1)
    static let battery = NSColor(srgbRed: 0.36, green: 1, blue: 0.43, alpha: 1)
    static let disk = NSColor(srgbRed: 0.36, green: 0.62, blue: 1, alpha: 1)
    static let power = NSColor(srgbRed: 1, green: 0.93, blue: 0.35, alpha: 1)
    static let busiest = NSColor(srgbRed: 1, green: 0.82, blue: 0.24, alpha: 1)
    static let glasses = NSColor(srgbRed: 0.25, green: 0.88, blue: 0.82, alpha: 1)
    static let latency = NSColor(srgbRed: 1, green: 0.42, blue: 0.6, alpha: 1)
    static let text = NSColor(white: 0.95, alpha: 1)
    static let detail = NSColor(white: 0.66, alpha: 1)
    static let good = NSColor(srgbRed: 0.3, green: 1, blue: 0.55, alpha: 1)
    static let warm = NSColor(srgbRed: 1, green: 0.8, blue: 0.2, alpha: 1)
    static let hot = NSColor(srgbRed: 1, green: 0.3, blue: 0.3, alpha: 1)

    /// Green when idle, through amber, to red when flat out.
    static func load(_ fraction: Float) -> NSColor {
        let t = CGFloat(min(max(fraction, 0), 1))
        return t < 0.5
            ? good.blended(withFraction: t * 2, of: warm) ?? good
            : warm.blended(withFraction: (t - 0.5) * 2, of: hot) ?? hot
    }
}

/// The row of tiles above the canvas: the time, the Mac's CPU, memory, GPU,
/// network, disk, power and battery, its busiest apps, and how the glasses
/// and the viewer are doing, with half a minute of history where it helps.
/// Looks and draws on a queue of its own, so the main thread never waits
/// for it.
final class Dashboard: @unchecked Sendable {
    let latest = LatestFrame()
    private let device: MTLDevice
    private let queue = DispatchQueue(label: "xreal.dashboard", qos: .utility)
    /// Whether an update is under way; one that is due meanwhile is skipped.
    private let busy = Mutex(false)
    // Touched only on `queue`.
    private var monitor = SystemMonitor()
    /// The textures drawn into in turn, so one is never written while the
    /// GPU may still read it. A new one each time made the GPU's driver
    /// allocate several megabytes ten times a second.
    private var textures: [MTLTexture] = []
    private var nextTexture = 0
    private var latencyHistory = History(capacity: 300)

    init(device: MTLDevice) {
        self.device = device
    }

    /// Looks at the Mac again and redraws, or shows nothing when `glasses`
    /// is nil.
    func update(glasses: GlassesReadings?) {
        guard let glasses else {
            latest.publish(nil)
            return
        }
        guard busy.withLock({ busy in defer { busy = true }; return !busy }) else { return }
        queue.async { [self] in
            let started = monotonicNow()
            let drawn = draw(glasses: glasses, now: .now)
            let frame = drawn.flatMap { upload($0.bitmap, width: $0.width, height: $0.height) }
            let took = monotonicNow() - started
            if took > 0.06 {
                timingLog.notice("Dashboard update took \(String(format: "%.1f", took * 1000), privacy: .public) ms")
            }
            latest.publish(frame)
            busy.withLock { $0 = false }
        }
    }

    /// The dashboard as it looks now, on black as in the glasses, as a PNG,
    /// for `--dashboard`.
    func png(glasses: GlassesReadings) -> Data? {
        guard let drawn = queue.sync(execute: { draw(glasses: glasses, now: .now) }),
            let space = CGColorSpace(name: CGColorSpace.sRGB),
            let provider = CGDataProvider(data: drawn.bitmap as CFData),
            let image = CGImage(
                width: drawn.width, height: drawn.height, bitsPerComponent: 8, bitsPerPixel: 32,
                bytesPerRow: drawn.width * 4, space: space,
                bitmapInfo: CGBitmapInfo(
                    rawValue: CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue),
                provider: provider, decode: nil, shouldInterpolate: false, intent: .defaultIntent),
            let context = CGContext(
                data: nil, width: drawn.width, height: drawn.height, bitsPerComponent: 8, bytesPerRow: 0, space: space,
                bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue)
        else { return nil }
        let bounds = CGRect(x: 0, y: 0, width: drawn.width, height: drawn.height)
        context.setFillColor(CGColor(gray: 0, alpha: 1))
        context.fill(bounds)
        context.draw(image, in: bounds)
        return context.makeImage().flatMap { NSBitmapImageRep(cgImage: $0).representation(using: .png, properties: [:]) }
    }

    /// `bitmap` in the next of the textures drawn into in turn, made again
    /// when the dashboard's size changes.
    private func upload(_ bitmap: Data, width: Int, height: Int) -> CapturedFrame? {
        if textures.first.map({ $0.width != width || $0.height != height }) ?? true {
            let descriptor = MTLTextureDescriptor.texture2DDescriptor(
                pixelFormat: .bgra8Unorm_srgb, width: width, height: height, mipmapped: false)
            descriptor.usage = .shaderRead
            textures = (0..<3).compactMap { _ in device.makeTexture(descriptor: descriptor) }
            guard textures.count == 3 else { return nil }
        }
        let texture = textures[nextTexture]
        nextTexture = (nextTexture + 1) % textures.count
        bitmap.withUnsafeBytes { bytes in
            texture.replace(
                region: MTLRegionMake2D(0, 0, width, height), mipmapLevel: 0, withBytes: bytes.baseAddress!,
                bytesPerRow: width * 4)
        }
        return CapturedFrame(texture: texture, pixelsPerPoint: overlayPixelsPerPoint)
    }

    /// Waits for any update under way, for tests and `--dashboard`.
    func settle() {
        queue.sync {}
    }

    private func draw(glasses: GlassesReadings, now: Date) -> (bitmap: Data, width: Int, height: Int)? {
        let sample = monitor.sample(now: monotonicNow())
        if let latency = glasses.latency {
            latencyHistory.append(Float(latency * 1000))
        }
        let tiles = self.tiles(sample: sample, glasses: glasses, now: now)
        let size = CGSize(
            width: CGFloat(tiles.count) * tileSize.width + CGFloat(max(tiles.count - 1, 0)) * tileSpacing,
            height: tileSize.height)
        let scale = CGFloat(overlayPixelsPerPoint)
        let (width, height) = (Int(size.width * scale), Int(size.height * scale))
        let bitmap = drawBitmap(width: width, height: height) { context in
            // Points, top-left origin, text the right way up.
            context.scaleBy(x: scale, y: scale)
            context.translateBy(x: 0, y: size.height)
            context.scaleBy(x: 1, y: -1)
            NSGraphicsContext.current = NSGraphicsContext(cgContext: context, flipped: true)
            let drawing = TileDrawing(context: context, scale: scale)
            for (index, tile) in tiles.enumerated() {
                let origin = CGPoint(x: CGFloat(index) * (tileSize.width + tileSpacing), y: 0)
                drawing.tile(CGRect(origin: origin, size: tileSize), tile)
            }
        }
        return bitmap.map { ($0, width, height) }
    }

    private func tiles(sample: SystemSample, glasses: GlassesReadings, now: Date) -> [Tile] {
        var tiles: [Tile] = []
        let clock = Calendar.current.dateComponents([.second, .nanosecond], from: now)
        let seconds = Float(clock.second ?? 0) + Float(clock.nanosecond ?? 0) / 1e9
        tiles.append(
            Tile(
                title: "Time", color: Palette.time, value: now.formatted(date: .omitted, time: .shortened),
                detail: now.formatted(.dateTime.weekday(.abbreviated).day().month(.abbreviated)),
                graphs: [
                    .bar(seconds / 60, Palette.time),
                    .label(thermalText(sample.thermal), thermalColor(sample.thermal)),
                ]))
        tiles.append(
            Tile(
                title: "CPU", color: Palette.cpu, value: percent(sample.cpu),
                detail: "\(sample.cores.count) cores",
                graphs: [.cores(sample.cores), .sparkline([(sample.cpuHistory.values, Palette.cpu)], ceiling: 1)]))
        if let memory = sample.memory {
            let fraction = Float(memory.used) / Float(max(memory.total, 1))
            tiles.append(
                Tile(
                    title: "Memory", color: Palette.memory, value: gigabytes(memory.used),
                    detail: "of \(gigabytes(memory.total))",
                    graphs: [
                        .bar(fraction, Palette.memory),
                        .sparkline([(sample.memoryHistory.values, Palette.memory)], ceiling: 1),
                    ]))
        }
        if sample.gpu != nil {
            tiles.append(
                Tile(
                    title: "GPU", color: Palette.gpu, value: percent(sample.gpu), detail: "utilization",
                    graphs: [.sparkline([(sample.gpuHistory.values, Palette.gpu)], ceiling: 1)]))
        }
        if let network = sample.network {
            let peak = max(
                sample.downHistory.values.max() ?? 0, sample.upHistory.values.max() ?? 0, 1024)
            tiles.append(
                Tile(
                    title: "Network", color: Palette.down, value: "↓ " + rate(network.down),
                    detail: "↑ " + rate(network.up),
                    graphs: [
                        .sparkline(
                            [(sample.downHistory.values, Palette.down), (sample.upHistory.values, Palette.up)],
                            ceiling: peak)
                    ]))
        }
        if let battery = sample.battery {
            let color = battery.percent <= 20 && !battery.charging ? Palette.hot : Palette.battery
            tiles.append(
                Tile(
                    title: "Battery", color: color, value: "\(battery.percent)%", detail: batteryText(battery),
                    graphs: [.bar(Float(battery.percent) / 100, color)]))
        }
        if let disk = sample.disk {
            let traffic = sample.diskTraffic.map { "R " + rate($0.read) + "  W " + rate($0.write) }
            let peak = max(sample.readHistory.values.max() ?? 0, sample.writeHistory.values.max() ?? 0, 1_048_576)
            tiles.append(
                Tile(
                    title: "Disk", color: Palette.disk, value: gigabytes(disk.free) + " free",
                    detail: traffic ?? "of \(gigabytes(disk.total))",
                    graphs: [
                        .bar(1 - Float(disk.free) / Float(max(disk.total, 1)), Palette.disk),
                        .sparkline(
                            [(sample.readHistory.values, Palette.disk), (sample.writeHistory.values, Palette.up)],
                            ceiling: peak),
                    ]))
        }
        if let power = sample.power {
            let peak = max(sample.powerHistory.values.max() ?? 0, 10)
            tiles.append(
                Tile(
                    title: "Power", color: Palette.power, value: String(format: "%.1f W", power.system),
                    detail: power.usbOut > 0.05 ? String(format: "USB out %.1f W", power.usbOut) : "system",
                    graphs: [.sparkline([(sample.powerHistory.values, Palette.power)], ceiling: peak)]))
        }
        tiles.append(
            Tile(
                title: "Busiest", color: Palette.busiest, value: nil, detail: nil,
                graphs: [.ranking(sample.busiest, Palette.busiest)]))
        let tracking = glasses.trackingHz.map { String(format: "Tracking %.0f Hz", $0) } ?? "Tracking lost"
        tiles.append(
            Tile(
                title: "Glasses", color: Palette.glasses,
                value: glasses.temperature.map { String(format: "%.1f °C", $0) } ?? "–",
                detail: String(format: "%.0f fps", glasses.fps),
                graphs: [.label(tracking, glasses.trackingHz == nil ? Palette.hot : Palette.good)]))
        let peakLatency = max(latencyHistory.values.max() ?? 0, 30)
        tiles.append(
            Tile(
                title: "Latency", color: Palette.latency,
                value: glasses.latency.map { String(format: "%.0f ms", $0 * 1000) } ?? "–",
                detail: String(format: "%.1f late frames/s", glasses.lateFramesPerSecond),
                graphs: [.sparkline([(latencyHistory.values, Palette.latency)], ceiling: peakLatency)]))
        return tiles
    }

    private func percent(_ fraction: Float?) -> String {
        fraction.map { String(format: "%.0f%%", $0 * 100) } ?? "–"
    }

    private func gigabytes(_ bytes: UInt64) -> String {
        let value = Double(bytes) / 1_073_741_824
        return String(format: value >= 100 ? "%.0f GB" : "%.1f GB", value)
    }

    private func rate(_ bytesPerSecond: Double) -> String {
        switch bytesPerSecond {
        case ..<1024: String(format: "%.0f B/s", bytesPerSecond)
        case ..<1_048_576: String(format: "%.0f KB/s", bytesPerSecond / 1024)
        default: String(format: "%.1f MB/s", bytesPerSecond / 1_048_576)
        }
    }

    private func batteryText(_ battery: Battery) -> String {
        let time = battery.minutesLeft.map { String(format: "%d:%02d", $0 / 60, $0 % 60) }
        if battery.charging {
            return time.map { "charging, \($0) to full" } ?? (battery.percent >= 100 ? "full" : "on power")
        }
        return time.map { "\($0) left" } ?? "on battery"
    }

    private func thermalText(_ level: ThermalLevel) -> String {
        switch level {
        case .nominal: "Thermal normal"
        case .fair: "Thermal warm"
        case .serious: "Thermal hot"
        case .critical: "Thermal critical"
        }
    }

    private func thermalColor(_ level: ThermalLevel) -> NSColor {
        switch level {
        case .nominal: Palette.good
        case .fair: Palette.warm
        case .serious, .critical: Palette.hot
        }
    }
}

/// One tile: a title, a big value, a line of detail, and graphs under them.
private struct Tile {
    enum Graph {
        /// A bar filled to a fraction.
        case bar(Float, NSColor)
        /// A bar for each core, coloured by how busy it is.
        case cores([Float])
        /// Lines over time, scaled so `ceiling` reaches the top.
        case sparkline([(values: [Float], color: NSColor)], ceiling: Float)
        /// Names with how much each uses, as shares of one core.
        case ranking([(name: String, share: Float)], NSColor)
        case label(String, NSColor)
    }

    var title: String
    var color: NSColor
    var value: String?
    var detail: String?
    var graphs: [Graph]
}

/// Draws tiles into a context in points with a top-left origin.
private struct TileDrawing {
    let context: CGContext
    /// Pixels per point, as shadows are measured in pixels.
    let scale: CGFloat

    func tile(_ frame: CGRect, _ tile: Tile) {
        // A faint wash and frame of the tile's colour, so it reads as one.
        let outline = CGPath(roundedRect: frame.insetBy(dx: 1, dy: 1), cornerWidth: 16, cornerHeight: 16, transform: nil)
        context.addPath(outline)
        context.setFillColor(tile.color.withAlphaComponent(0.07).cgColor)
        context.fillPath()
        context.addPath(outline)
        context.setStrokeColor(tile.color.withAlphaComponent(0.4).cgColor)
        context.setLineWidth(1.5)
        context.strokePath()

        let inner = frame.insetBy(dx: tilePadding, dy: tilePadding)
        text(tile.title.uppercased(), at: CGPoint(x: inner.minX, y: inner.minY), size: 12, weight: .bold,
            color: tile.color, kern: 1.6)
        var y = inner.minY + 20
        if let value = tile.value {
            let size: CGFloat = value.count > 8 ? 24 : 32
            text(value, at: CGPoint(x: inner.minX, y: y + (32 - size) / 2), size: size, weight: .semibold,
                color: Palette.text, rounded: true, fitting: inner.width)
            y += 40
        }
        if let detail = tile.detail {
            text(detail, at: CGPoint(x: inner.minX, y: y), size: 13, weight: .medium, color: Palette.detail,
                fitting: inner.width)
            y += 22
        }
        // The graphs share what is left, bars and labels taking a line each.
        let fixed = tile.graphs.filter { if case .bar = $0 { true } else if case .label = $0 { true } else { false } }
        let flexible = CGFloat(tile.graphs.count - fixed.count)
        let spare = inner.maxY - y - CGFloat(fixed.count) * 22 - CGFloat(max(tile.graphs.count - 1, 0)) * 8
        for graph in tile.graphs {
            let height: CGFloat
            switch graph {
            case .bar, .label: height = 22
            default: height = max(spare / max(flexible, 1), 10)
            }
            draw(graph, in: CGRect(x: inner.minX, y: y, width: inner.width, height: height))
            y += height + 8
        }
    }

    private func draw(_ graph: Tile.Graph, in rect: CGRect) {
        switch graph {
        case .bar(let fraction, let color):
            bar(CGRect(x: rect.minX, y: rect.midY - 5, width: rect.width, height: 10), fraction, color)
        case .cores(let loads):
            guard !loads.isEmpty else { return }
            let gap: CGFloat = 3
            let width = (rect.width - gap * CGFloat(loads.count - 1)) / CGFloat(loads.count)
            for (index, load) in loads.enumerated() {
                let column = CGRect(
                    x: rect.minX + CGFloat(index) * (width + gap), y: rect.minY, width: width, height: rect.height)
                track(column, radius: 2)
                let height = max(column.height * CGFloat(min(max(load, 0), 1)), 1.5)
                let filled = CGRect(x: column.minX, y: column.maxY - height, width: width, height: height)
                glowing(Palette.load(load), in: filled) { fill(filled, Palette.load(load), radius: 2) }
            }
        case .sparkline(let lines, let ceiling):
            track(rect, radius: 6)
            for line in lines {
                sparkline(rect.insetBy(dx: 2, dy: 3), line.values, ceiling: ceiling, color: line.color)
            }
        case .ranking(let entries, let color):
            guard !entries.isEmpty else {
                text("Measuring…", at: rect.origin, size: 13, weight: .medium, color: Palette.detail)
                return
            }
            let rowHeight = min(rect.height / 3, 34)
            let most = max(entries.map(\.share).max() ?? 1, 1)
            for (index, entry) in entries.enumerated() {
                let row = CGRect(
                    x: rect.minX, y: rect.minY + CGFloat(index) * rowHeight, width: rect.width, height: rowHeight)
                let share = String(format: "%.0f%%", entry.share * 100)
                text(share, at: CGPoint(x: row.maxX - 44, y: row.minY), size: 13, weight: .semibold,
                    color: Palette.text, rounded: true)
                text(entry.name, at: CGPoint(x: row.minX, y: row.minY), size: 13, weight: .medium, color: Palette.text,
                    fitting: row.width - 50)
                bar(CGRect(x: row.minX, y: row.minY + 19, width: row.width, height: 5), entry.share / most, color)
            }
        case .label(let label, let color):
            let dot = CGRect(x: rect.minX, y: rect.midY - 4, width: 8, height: 8)
            glowing(color, in: dot) {
                context.setFillColor(color.cgColor)
                context.fillEllipse(in: dot)
            }
            text(label, at: CGPoint(x: rect.minX + 16, y: rect.midY - 9), size: 13, weight: .semibold, color: color)
        }
    }

    private func bar(_ rect: CGRect, _ fraction: Float, _ color: NSColor) {
        track(rect, radius: rect.height / 2)
        let filled = CGRect(
            x: rect.minX, y: rect.minY, width: max(rect.width * CGFloat(min(max(fraction, 0), 1)), rect.height),
            height: rect.height)
        glowing(color, in: filled) {
            gradient(filled, from: color.withAlphaComponent(0.55), to: color, radius: rect.height / 2)
        }
    }

    private func sparkline(_ rect: CGRect, _ values: [Float], ceiling: Float, color: NSColor) {
        guard values.count >= 2, ceiling > 0 else { return }
        let step = rect.width / CGFloat(max(values.count - 1, 1))
        let points = values.enumerated().map { index, value in
            CGPoint(
                x: rect.minX + CGFloat(index) * step,
                y: rect.maxY - rect.height * CGFloat(min(max(value / ceiling, 0), 1)))
        }
        let line = CGMutablePath()
        line.addLines(between: points)
        // A fading fill under the line, then the line glowing on top.
        let area = CGMutablePath()
        area.addPath(line)
        area.addLine(to: CGPoint(x: points.last!.x, y: rect.maxY))
        area.addLine(to: CGPoint(x: points[0].x, y: rect.maxY))
        area.closeSubpath()
        context.saveGState()
        context.addPath(area)
        context.clip()
        if let fade = CGGradient(
            colorsSpace: nil, colors: [color.withAlphaComponent(0.4).cgColor, color.withAlphaComponent(0).cgColor] as CFArray,
            locations: [0, 1])
        {
            context.drawLinearGradient(
                fade, start: CGPoint(x: 0, y: rect.minY), end: CGPoint(x: 0, y: rect.maxY), options: [])
        }
        context.restoreGState()
        glowing(color, in: rect) {
            context.addPath(line)
            context.setStrokeColor(color.cgColor)
            context.setLineWidth(2)
            context.setLineJoin(.round)
            context.strokePath()
        }
    }

    /// The dim groove a bar or graph sits in.
    private func track(_ rect: CGRect, radius: CGFloat) {
        fill(rect, NSColor(white: 1, alpha: 0.08), radius: radius)
    }

    private func fill(_ rect: CGRect, _ color: NSColor, radius: CGFloat) {
        context.addPath(CGPath(roundedRect: rect, cornerWidth: radius, cornerHeight: radius, transform: nil))
        context.setFillColor(color.cgColor)
        context.fillPath()
    }

    private func gradient(_ rect: CGRect, from start: NSColor, to end: NSColor, radius: CGFloat) {
        guard let gradient = CGGradient(colorsSpace: nil, colors: [start.cgColor, end.cgColor] as CFArray, locations: [0, 1])
        else { return }
        context.saveGState()
        context.addPath(CGPath(roundedRect: rect, cornerWidth: radius, cornerHeight: radius, transform: nil))
        context.clip()
        context.drawLinearGradient(
            gradient, start: CGPoint(x: rect.minX, y: 0), end: CGPoint(x: rect.maxX, y: 0), options: [])
        context.restoreGState()
    }

    /// Draws with `color` glowing around what `draw` draws inside `bounds`.
    /// The glow is worked out for just that part: across the whole image
    /// it costs a hundred times more.
    private func glowing(_ color: NSColor, in bounds: CGRect, _ draw: () -> Void) {
        context.saveGState()
        context.setShadow(offset: .zero, blur: glowRadius * scale, color: color.withAlphaComponent(0.85).cgColor)
        context.beginTransparencyLayer(in: bounds.insetBy(dx: -2 * glowRadius, dy: -2 * glowRadius), auxiliaryInfo: nil)
        draw()
        context.endTransparencyLayer()
        context.restoreGState()
    }

    private func text(
        _ string: String, at point: CGPoint, size: CGFloat, weight: NSFont.Weight, color: NSColor, rounded: Bool = false,
        kern: CGFloat = 0, fitting width: CGFloat? = nil
    ) {
        var font = NSFont.monospacedDigitSystemFont(ofSize: size, weight: weight)
        if rounded, let descriptor = font.fontDescriptor.withDesign(.rounded) {
            font = NSFont(descriptor: descriptor, size: size) ?? font
        }
        let style = NSMutableParagraphStyle()
        style.lineBreakMode = .byTruncatingTail
        let attributed = NSAttributedString(
            string: string, attributes: [.font: font, .foregroundColor: color, .kern: kern, .paragraphStyle: style])
        if let width {
            attributed.draw(
                with: CGRect(x: point.x, y: point.y, width: width, height: size * 1.4),
                options: [.usesLineFragmentOrigin, .truncatesLastVisibleLine])
        } else {
            attributed.draw(at: point)
        }
    }
}
