import CoreGraphics
import simd

// Room coordinates use the recentered head frame: x right, y up, -z ahead.
// At distance 1 a screen shows one source pixel per glasses pixel when looked
// at straight on.
private let glassesPixelsWide: Float = 1920
let roomUnitsPerPixel = 2 * tan(horizontalFov * 0.5) / glassesPixelsWide
private let worldUp = SIMD3<Float>(0, 1, 0)
// Screens stay off the poles, where "upright" has no meaning.
private let maxElevation: Float = 1.3  // rad, about 75°
private let minDistance: Float = 0.3
private let maxDistance: Float = 5
// Looking this close past a screen's edge still counts as looking at it.
private let gazeSlack: Float = 0.35  // rad
// A curved screen wraps at most this far either side of its middle; one
// brought closer than that allows keeps this curve and stops coming closer.
private let maxHalfArc: Float = 2.9  // rad, about 166°
// Screens wider than this are captured in tiles about one view wide, so
// the parts out of view can update less often.
private let maxUntiledWidth = 3840
private let tileWidth = 1920
// A window zone is about one view wide and one view tall.
private let zoneWidth: Float = 1920
private let zoneHeight: Float = 2160

/// The largest side of a virtual screen. macOS 27's WindowServer crashes when
/// asked for virtual displays much wider than 8K.
public let maxVirtualScreenSide = 8192

/// Where a virtual screen hangs in the room. It always faces the viewer and
/// stays upright.
public struct ScreenPlacement: Equatable, Sendable {
    /// Unit vector from the viewer to the screen's middle.
    public var direction: SIMD3<Float>
    /// 1 shows the screen at the glasses' own pixel density; 2 is twice as
    /// far away and looks half as large.
    public var distance: Float

    public init(direction: SIMD3<Float>, distance: Float = 1) {
        self.direction = Self.upright(direction)
        self.distance = min(max(distance, minDistance), maxDistance)
    }

    /// The placement matching where macOS arranged a screen relative to
    /// `ahead`, both in global points (y down). The middle of `ahead` is
    /// straight ahead, and one point across is one glasses pixel of turn, so
    /// screens side by side in the arrangement sit side by side around the
    /// viewer, however wide they are.
    public init(arrangedAt frame: CGRect, around ahead: CGRect) {
        let angles = SIMD2(Float(frame.midX - ahead.midX), Float(ahead.midY - frame.midY)) * roomUnitsPerPixel
        let (azimuth, elevation) = (angles.x, min(max(angles.y, -maxElevation), maxElevation))
        self.init(direction: SIMD3(sin(azimuth) * cos(elevation), sin(elevation), -cos(azimuth) * cos(elevation)))
    }

    /// Where macOS should arrange a `width` × `height` screen so the mouse
    /// crosses between displays the way they sit in the room. The inverse of
    /// `init(arrangedAt:around:)`.
    public func arrangedOrigin(width: Int, height: Int, around ahead: CGRect) -> CGPoint {
        let azimuth = atan2(direction.x, -direction.z)
        let elevation = asin(min(max(direction.y, -1), 1))
        let offset = SIMD2(azimuth, elevation) / roomUnitsPerPixel
        return CGPoint(
            x: (ahead.midX + CGFloat(offset.x) - CGFloat(width) / 2).rounded(),
            y: (ahead.midY - CGFloat(offset.y) - CGFloat(height) / 2).rounded())
    }

    public func movedAway(by factor: Float) -> ScreenPlacement {
        ScreenPlacement(direction: direction, distance: distance * factor)
    }

    /// The screen's middle and half extents along its right and up edges,
    /// in room coordinates, for a flat screen.
    public func frame(width: Int, height: Int) -> (center: SIMD3<Float>, right: SIMD3<Float>, up: SIMD3<Float>) {
        let right = normalize(cross(direction, worldUp))
        let up = cross(right, direction)
        return (
            direction * distance, right * (Float(width) * roomUnitsPerPixel * 0.5),
            up * (Float(height) * roomUnitsPerPixel * 0.5)
        )
    }

    private static func upright(_ direction: SIMD3<Float>) -> SIMD3<Float> {
        let unit = length(direction) > 1e-6 ? normalize(direction) : SIMD3(0, 0, -1)
        let elevation = min(max(asin(min(max(unit.y, -1), 1)), -maxElevation), maxElevation)
        var flat = SIMD2(unit.x, unit.z)
        flat = length(flat) > 1e-6 ? normalize(flat) : SIMD2(0, -1)
        return SIMD3(flat.x * cos(elevation), sin(elevation), flat.y * cos(elevation))
    }
}

/// A screen's surface in the room: flat, or bent around the viewer like
/// part of a cylinder standing along its up edge. Positions on it run from
/// -1 to 1 across and from -1 at the bottom to 1 at the top.
public struct ScreenSurface: Equatable, Sendable {
    /// Middle of the screen; for a curved one also the cylinder's radius.
    public var center: SIMD3<Float>
    /// Half extents along the right and up edges. A curved screen's right
    /// edge is measured along the curve.
    public var right: SIMD3<Float>
    public var up: SIMD3<Float>
    /// How far a curved screen wraps either side of its middle, in radians;
    /// 0 when flat.
    public var halfArc: Float

    public init(center: SIMD3<Float>, right: SIMD3<Float>, up: SIMD3<Float>, halfArc: Float = 0) {
        self.center = center
        self.right = right
        self.up = up
        self.halfArc = halfArc
    }

    /// The room point at `position` on the screen. The renderer's panel
    /// shader does the same.
    public func point(at position: SIMD2<Float>) -> SIMD3<Float> {
        guard halfArc > 0 else { return center + position.x * right + position.y * up }
        let angle = position.x * halfArc
        return cos(angle) * center + sin(angle) * length(center) * normalize(right) + position.y * up
    }

    /// How far along `gaze` (a unit vector) the screen is hit, where on it
    /// from -1 to 1 across and down, and roughly how far past its edge in
    /// radians (0 on the screen); nil when the gaze misses its surface.
    func hit(gaze: SIMD3<Float>) -> (along: Float, at: SIMD2<Float>, miss: Float)? {
        guard halfArc > 0 else { return flatHit(gaze: gaze) }
        let radius = length(center)
        let (ahead, across, upward) = (center / radius, normalize(right), normalize(up))
        let sideways = SIMD2(dot(gaze, across), dot(gaze, ahead))
        let horizontal = length(sideways)
        guard horizontal > 1e-4 else { return nil }
        let along = radius / horizontal
        let at = SIMD2(atan2(sideways.x, sideways.y) / halfArc, -along * dot(gaze, upward) / length(up))
        let miss = max((abs(at.x) - 1) * halfArc, (abs(at.y) - 1) * length(up) / along, 0)
        return (along, at, miss)
    }

    private func flatHit(gaze: SIMD3<Float>) -> (along: Float, at: SIMD2<Float>, miss: Float)? {
        let facing = normalize(cross(up, right))
        let towards = dot(gaze, facing)
        guard towards > 1e-4 else { return nil }
        let along = dot(center, facing) / towards
        let offset = gaze * along - center
        let at = SIMD2(dot(offset, right) / dot(right, right), -dot(offset, up) / dot(up, up))
        let past = max(abs(at.x), abs(at.y))
        guard past > 1 else { return (along, at, 0) }
        // Angle beyond the edge, roughly: how far past it, over the distance.
        let halfExtent = abs(at.x) > abs(at.y) ? length(right) : length(up)
        return (along, at, (past - 1) * halfExtent / along)
    }
}

/// A virtual screen and where it hangs in the room, nil until it is placed.
public struct RoomScreen: Equatable, Sendable {
    public var width: Int
    public var height: Int
    public var placement: ScreenPlacement?
    /// Bent around the viewer, so every part of it is equally far away.
    public var curved: Bool

    public init(width: Int, height: Int, placement: ScreenPlacement? = nil, curved: Bool = false) {
        self.width = width
        self.height = height
        self.placement = placement
        self.curved = curved
    }

    /// Where the screen is in the room; nil until it is placed.
    public var surface: ScreenSurface? {
        guard let placement else { return nil }
        let (center, right, up) = placement.frame(width: width, height: height)
        guard curved else { return ScreenSurface(center: center, right: right, up: up) }
        let halfWidth = length(right)
        let radius = max(placement.distance, halfWidth / maxHalfArc)
        return ScreenSurface(
            center: placement.direction * radius, right: right, up: up, halfArc: halfWidth / radius)
    }
}

/// The columns of pixels each capture of a `width` pixel wide screen
/// covers: the whole screen, or tiles about one view wide.
public func captureTiles(width: Int) -> [Range<Int>] {
    guard width > maxUntiledWidth else { return [0..<width] }
    let count = (width + tileWidth - 1) / tileWidth
    return (0..<count).map { index in (index * width / count)..<((index + 1) * width / count) }
}

/// The screen the viewer is looking at along `gaze`: the nearest one the
/// gaze passes through, or else the one whose edge is closest to it.
public func screenLooked(at gaze: SIMD3<Float>, among screens: [RoomScreen]) -> Int? {
    gazeTarget(gaze, among: screens)?.index
}

/// The screen looked at along `gaze` and the pixel on it the gaze points at,
/// kept on the screen when the gaze is just past its edge.
public func gazeTarget(_ gaze: SIMD3<Float>, among screens: [RoomScreen]) -> (index: Int, pixel: SIMD2<Float>)? {
    var best: (index: Int, along: Float, at: SIMD2<Float>)?
    var nearest: (index: Int, miss: Float, at: SIMD2<Float>)?
    for (index, screen) in screens.enumerated() {
        guard let hit = screen.surface?.hit(gaze: gaze) else { continue }
        if hit.miss == 0 {
            if best == nil || hit.along < best!.along {
                best = (index, hit.along, hit.at)
            }
        } else if hit.miss < gazeSlack, nearest == nil || hit.miss < nearest!.miss {
            nearest = (index, hit.miss, hit.at)
        }
    }
    guard let (index, at) = best.map({ ($0.index, $0.at) }) ?? nearest.map({ ($0.index, $0.at) }) else {
        return nil
    }
    let size = SIMD2(Float(screens[index].width), Float(screens[index].height))
    return (index, (simd_clamp(at, SIMD2(repeating: -1), SIMD2(repeating: 1)) + 1) * 0.5 * size)
}

/// The zone of a `width` × `height` screen around `pixel`, in its pixels: the
/// screen split into parts about one view wide and one to two views tall.
/// Small screens are one zone.
public func zone(around pixel: SIMD2<Float>, width: Int, height: Int) -> CGRect {
    let size = SIMD2(Float(width), Float(height))
    let counts = simd_max((size / SIMD2(zoneWidth, zoneHeight)).rounded(.toNearestOrAwayFromZero), SIMD2(1, 1))
    let part = size / counts
    let cell = simd_clamp((pixel / part).rounded(.down), .zero, counts - 1)
    let lower = (cell * part).rounded(.toNearestOrAwayFromZero)
    let upper = ((cell + 1) * part).rounded(.toNearestOrAwayFromZero)
    return CGRect(
        x: CGFloat(lower.x), y: CGFloat(lower.y), width: CGFloat(upper.x - lower.x),
        height: CGFloat(upper.y - lower.y))
}

/// Where to put a window of `size` so it is centered on `point` but stays
/// inside `bounds` where it fits, all in global points.
public func windowOrigin(size: CGSize, centeredOn point: CGPoint, within bounds: CGRect) -> CGPoint {
    func axis(_ middle: CGFloat, _ extent: CGFloat, _ low: CGFloat, _ high: CGFloat) -> CGFloat {
        let start = middle - extent / 2
        return extent >= high - low ? low : min(max(start, low), high - extent)
    }
    return CGPoint(
        x: axis(point.x, size.width, bounds.minX, bounds.maxX).rounded(),
        y: axis(point.y, size.height, bounds.minY, bounds.maxY).rounded())
}

/// A screen being carried by the head: it keeps its place in the view while
/// the head turns, and stays where it was when let go.
public struct ScreenGrab: Equatable, Sendable {
    public let index: Int
    private let inView: SIMD3<Float>

    /// `headRotation` turns head directions into room directions.
    public init(index: Int, placement: ScreenPlacement, headRotation: simd_float3x3) {
        self.index = index
        inView = headRotation.transpose * placement.direction
    }

    public func placement(headRotation: simd_float3x3, distance: Float) -> ScreenPlacement {
        ScreenPlacement(direction: headRotation * inView, distance: distance)
    }
}

/// The virtual screens as the glasses see them this frame.
public struct RoomView: Sendable {
    /// One capture of a screen: the whole screen, or a tile of it.
    public struct Panel: Sendable {
        /// Which capture to show.
        public var source: Int
        /// Which screen it is part of.
        public var screen: Int
        public var surface: ScreenSurface
        /// The part of the screen's width it covers, from -1 to 1.
        public var span: SIMD2<Float>
        /// Outlined, because it is the one being looked at or carried.
        public var highlighted: Bool

        public init(
            source: Int, screen: Int, surface: ScreenSurface, span: SIMD2<Float> = SIMD2(-1, 1),
            highlighted: Bool = false
        ) {
            self.source = source
            self.screen = screen
            self.surface = surface
            self.span = span
            self.highlighted = highlighted
        }
    }

    /// Turns head directions into room directions.
    public var headRotation: simd_float3x3
    /// Tangent of half the field of view, horizontally and vertically.
    public var tanHalfFov: SIMD2<Float>
    public var panels: [Panel]

    /// Where the room point `point` appears in the view, from -1 to 1 with y
    /// up, or nil when it is behind the viewer.
    public func outputPoint(ofRoom point: SIMD3<Float>) -> SIMD2<Float>? {
        let head = headRotation.transpose * point
        guard head.z < -1e-6 else { return nil }
        return SIMD2(head.x, head.y) / -head.z / tanHalfFov
    }

    /// Whether any of `panel` is in view or within `margin` (a fraction of
    /// the view) of it, judged from a grid of points on it.
    public func shows(_ panel: Panel, margin: Float) -> Bool {
        let limit = 1 + margin
        for column in 0...4 {
            let x = panel.span.x + (panel.span.y - panel.span.x) * Float(column) / 4
            for row in 0...4 {
                let point = panel.surface.point(at: SIMD2(x, Float(row) / 2 - 1))
                if let output = outputPoint(ofRoom: point), abs(output.x) <= limit, abs(output.y) <= limit {
                    return true
                }
            }
        }
        return false
    }
}

/// Frames, in global points, for the standard layout around the main
/// display: the first screen centered above it, the next to its right, the
/// next to its left, and any more further out on alternating sides. Side
/// screens line up with the main display's top edge, so they stay clear of
/// the one above.
public func standardArrangement(for screens: [RoomScreen], around main: CGRect) -> [CGRect] {
    var rightEdge = main.maxX
    var leftEdge = main.minX
    return screens.enumerated().map { index, screen in
        let width = CGFloat(screen.width)
        let height = CGFloat(screen.height)
        if index == 0 {
            return CGRect(x: (main.midX - width / 2).rounded(), y: main.minY - height, width: width, height: height)
        }
        let x: CGFloat
        if index % 2 == 1 {
            x = rightEdge
            rightEdge += width
        } else {
            leftEdge -= width
            x = leftEdge
        }
        return CGRect(x: x, y: main.minY, width: width, height: height)
    }
}

/// Frames, in global points, for the standard layout when the glasses are the
/// only display: the first screen straight ahead of the point `ahead`, the
/// next to its right, the next to its left, and any more further out on
/// alternating sides, all level with it.
public func glassesOnlyArrangement(for screens: [RoomScreen], ahead: CGPoint = .zero) -> [CGRect] {
    guard let first = screens.first else { return [] }
    let size = { (screen: RoomScreen) in CGSize(width: screen.width, height: screen.height) }
    let middle = CGRect(
        origin: CGPoint(x: ahead.x - CGFloat(first.width / 2), y: ahead.y - CGFloat(first.height / 2)),
        size: size(first))
    var rightEdge = middle.maxX
    var leftEdge = middle.minX
    return [middle]
        + screens.dropFirst().enumerated().map { index, screen in
            let x: CGFloat
            if index % 2 == 0 {
                x = rightEdge
                rightEdge += CGFloat(screen.width)
            } else {
                leftEdge -= CGFloat(screen.width)
                x = leftEdge
            }
            return CGRect(origin: CGPoint(x: x, y: ahead.y - CGFloat(screen.height / 2)), size: size(screen))
        }
}

/// The straight-ahead reference, as a point-sized rect in global points,
/// that has macOS arrange `screen` with its top left corner at `origin`.
/// With the glasses alone there is no real display to measure the room
/// from, so the arrangement is measured from one of the screens instead.
public func aheadReference(for screen: RoomScreen, arrangedAt origin: CGPoint) -> CGRect {
    guard let placement = screen.placement else {
        return CGRect(x: origin.x + CGFloat(screen.width / 2), y: origin.y + CGFloat(screen.height / 2), width: 0, height: 0)
    }
    let offset = placement.arrangedOrigin(width: screen.width, height: screen.height, around: .zero)
    return CGRect(x: origin.x - offset.x, y: origin.y - offset.y, width: 0, height: 0)
}
