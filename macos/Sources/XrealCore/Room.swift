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
// Looking this close past the canvas's edge still counts as looking at it.
private let gazeSlack: Float = 0.35  // rad
// A curved screen wraps at most this far either side of its middle; one
// brought closer than that allows keeps this curve and stops coming closer.
private let maxHalfArc: Float = 2.9  // rad, about 166°
// A spherical screen reaches at most this far above and below its middle,
// leaving room above it for the row that hangs there, short of the pole.
private let maxHalfRise: Float = 1  // rad, about 57°
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

/// The only sizes allowed past `maxVirtualScreenSide`, in points with their
/// scale: each was probed on its own and came up as asked, 10240 × 2880 and
/// 10240 × 4320 pixels. The panic came from many oversize displays made one
/// after another.
public let sizesProbedBeyondLimit: [(width: Int, height: Int, scale: Int)] = [(5120, 1440, 2), (5120, 2160, 2)]

/// Where a virtual screen hangs in the room. It always faces the viewer, and
/// is upright unless tilted.
public struct ScreenPlacement: Equatable, Sendable {
    /// Unit vector from the viewer to the screen's middle.
    public var direction: SIMD3<Float>
    /// 1 shows the screen at the glasses' own pixel density; 2 is twice as
    /// far away and looks half as large.
    public var distance: Float
    /// How far the screen is turned about the line from the viewer to its
    /// middle, in radians; positive leans its top to the viewer's right.
    public var tilt: Float

    public init(direction: SIMD3<Float>, distance: Float = 1, tilt: Float = 0) {
        self.direction = Self.upright(direction)
        self.distance = min(max(distance, minDistance), maxDistance)
        self.tilt = tilt.isFinite ? wrapAngle(tilt) : 0
    }

    /// Level, straight ahead, at the glasses' own pixel density.
    public static let straightAhead = ScreenPlacement(direction: SIMD3(0, 0, -1))

    /// Turns the upright screen by its tilt.
    public var spin: simd_quatf {
        simd_quatf(angle: tilt, axis: direction)
    }

    /// Turns a screen straight ahead and level to where this one hangs,
    /// tilt included.
    public var orientation: simd_quatf {
        let (right, up) = uprightAxes
        return spin * simd_quatf(simd_float3x3(right, up, -direction))
    }

    /// The upright screen's up and right directions.
    private var uprightAxes: (right: SIMD3<Float>, up: SIMD3<Float>) {
        let right = normalize(cross(direction, worldUp))
        return (right, cross(right, direction))
    }

    /// The screen's up direction, tilt included.
    public var up: SIMD3<Float> {
        spin.act(uprightAxes.up)
    }

    /// The same placement tilted so its up points as close to `up` as it
    /// can while facing the viewer.
    public func tilted(toward up: SIMD3<Float>) -> ScreenPlacement {
        let (_, uprightUp) = uprightAxes
        let angle = atan2(dot(up, cross(direction, uprightUp)), dot(up, uprightUp))
        return ScreenPlacement(direction: direction, distance: distance, tilt: angle)
    }

    public func movedAway(by factor: Float) -> ScreenPlacement {
        ScreenPlacement(direction: direction, distance: distance * factor, tilt: tilt)
    }

    /// The screen's middle and half extents along its right and up edges,
    /// in room coordinates, for a flat screen.
    /// Upright: the tilt is applied by the surface.
    public func frame(width: Int, height: Int) -> (center: SIMD3<Float>, right: SIMD3<Float>, up: SIMD3<Float>) {
        let (right, up) = uprightAxes
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

/// A screen's surface in the room: flat, or curved like a monitor, as
/// tightly as asked. Curved, every row is the same arc round the axis along
/// `up` `rowRadius` behind its middle, and every column a straight line, so
/// its top and bottom are as wide as its middle. Positions on it
/// run from -1 to 1 across and from -1 at the bottom to 1 at the top.
public struct ScreenSurface: Equatable, Sendable {
    /// Middle of the screen.
    public var center: SIMD3<Float>
    /// Half extents along the right and up edges. A curved screen's right
    /// edge is measured along the curve.
    public var right: SIMD3<Float>
    public var up: SIMD3<Float>
    /// How far a curved screen wraps either side of its middle, in radians;
    /// 0 when flat.
    public var halfArc: Float
    /// Turns the whole surface as described above about the viewer, to tilt
    /// it; `center`, `right` and `up` describe it before the turn.
    public var spin: simd_quatf
    /// How far every column leans towards the viewer for each unit it rises
    /// from the bottom edge, which stays where it is: 0 upright. A curved
    /// screen leaning so narrows towards its top like a lampshade, facing
    /// the viewer all the way round.
    public var lean: Float
    /// How far a curved screen's columns curve too, from 0, straight, to 1,
    /// round the same middle as its rows: then it is part of a sphere, and
    /// every pixel faces the viewer when that middle is at the eyes. Rows
    /// are spaced out as on a Mercator map, so every pixel keeps its shape
    /// and columns stay straight wherever they are seen from; pixels away
    /// from the middle row are only a little smaller. `up` is then measured
    /// along the middle column as that map lays it out.
    public var wrap: Float

    public init(
        center: SIMD3<Float>, right: SIMD3<Float>, up: SIMD3<Float>, halfArc: Float = 0,
        spin: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1), lean: Float = 0, wrap: Float = 0
    ) {
        self.center = center
        self.right = right
        self.up = up
        self.halfArc = halfArc
        self.spin = spin
        self.lean = lean
        self.wrap = wrap
    }

    /// The radius of each row's arc, for a curved screen.
    private var rowRadius: Float { length(right) / halfArc }

    /// The offset from the middle column to the point `across` radians round
    /// a row, for a curved screen.
    private func rowOffset(_ across: Float) -> SIMD3<Float> {
        rowRadius * (sin(across) * normalize(right) - (1 - cos(across)) * horizontalAhead)
    }

    /// The room point at `position` on the screen. The renderer's panel
    /// shader does the same.
    public func point(at position: SIMD2<Float>) -> SIMD3<Float> {
        spin.act(uprightPoint(at: position))
    }

    private func uprightPoint(at position: SIMD2<Float>) -> SIMD3<Float> {
        guard halfArc > 0 else {
            let tilted = lean == 0 ? .zero : lean * (position.y + 1) * length(up) * horizontalAhead
            return center + position.x * right + position.y * up - tilted
        }
        if wrap > 0 {
            let row = wrappedRow(position.y)
            let across = position.x * halfArc
            let level = sin(across) * normalize(right) + cos(across) * horizontalAhead
            return sphereMiddle + row.radius * level + row.height * normalize(up)
        }
        let across = position.x * halfArc
        // How far towards the viewer the row at this height has come.
        let inward = lean * (position.y + 1) * length(up)
        let radius = rowRadius - inward
        return center + position.y * up - rowRadius * horizontalAhead
            + radius * (sin(across) * normalize(right) + cos(across) * horizontalAhead)
    }

    /// Where the rows' arcs, and a wrapped screen's columns, are centred.
    private var sphereMiddle: SIMD3<Float> {
        center - rowRadius * horizontalAhead
    }

    /// For a wrapped screen, the row at `y`, from -1 at the bottom to 1 at
    /// the top: the radius of its circle and how far above the middle it
    /// is, round its column's arc of radius `rowRadius / wrap`. The rows'
    /// angles up that arc are a Mercator map's, whose rows crowd towards
    /// the poles just as their circles shrink.
    private func wrappedRow(_ y: Float) -> (radius: Float, height: Float) {
        let columnRadius = rowRadius / wrap
        let rise = mercatorRise(y * length(up) / columnRadius)
        return (rowRadius - columnRadius * (1 - cos(rise)), columnRadius * sin(rise))
    }

    /// Level unit vector towards the middle of the screen.
    private var horizontalAhead: SIMD3<Float> {
        normalize(SIMD3(center.x, 0, center.z))
    }

    /// How far along `gaze` (a unit vector) the screen is hit, where on it
    /// from -1 to 1 across and down, and roughly how far past its edge in
    /// radians (0 on the screen); nil when the gaze misses its surface.
    func hit(gaze: SIMD3<Float>) -> (along: Float, at: SIMD2<Float>, miss: Float)? {
        let upright = spin.inverse.act(gaze)
        guard halfArc > 0 else { return flatHit(gaze: upright) }
        return wrap > 0 ? wrappedHit(gaze: upright) : curvedHit(gaze: upright)
    }

    /// `hit(gaze:)` for a wrapped screen: the position whose direction from
    /// the eyes is the gaze, found by Newton's method from where it would
    /// be on a sphere round the eyes.
    private func wrappedHit(gaze: SIMD3<Float>) -> (along: Float, at: SIMD2<Float>, miss: Float)? {
        let helper = abs(gaze.y) < 0.9 ? SIMD3<Float>(0, 1, 0) : SIMD3(1, 0, 0)
        let (first, second) = (normalize(cross(gaze, helper)), normalize(cross(gaze, normalize(cross(gaze, helper)))))
        // How far the position's direction is off the gaze, sideways and up.
        func off(_ position: SIMD2<Float>) -> SIMD2<Float> {
            let seen = normalize(uprightPoint(at: position))
            return SIMD2(dot(seen, first), dot(seen, second))
        }
        let across = atan2(dot(gaze, normalize(right)), dot(gaze, horizontalAhead))
        let rise = asin(min(max(dot(gaze, normalize(up)), -1), 1))
        var position = SIMD2(across / halfArc, mercatorHeight(rise) * rowRadius / (wrap * length(up)))
        let step: Float = 1e-3
        for _ in 0..<20 {
            let miss = off(position)
            if simd_length(miss) < 1e-6 { break }
            let alongX = (off(position + SIMD2(step, 0)) - miss) / step
            let alongY = (off(position + SIMD2(0, step)) - miss) / step
            let determinant = alongX.x * alongY.y - alongY.x * alongX.y
            guard abs(determinant) > 1e-12 else { return nil }
            var move = SIMD2(
                (miss.x * alongY.y - alongY.x * miss.y) / determinant,
                (alongX.x * miss.y - alongX.y * miss.x) / determinant)
            // Small steps, so a far start does not jump past the answer.
            let size = simd_length(move)
            if size > 0.5 { move *= 0.5 / size }
            position -= move
        }
        let point = uprightPoint(at: position)
        guard simd_length(off(position)) < 1e-3, dot(point, gaze) > 0 else { return nil }
        // How far off the nearest point on the screen the gaze is.
        let edge = normalize(uprightPoint(at: simd_clamp(position, SIMD2(repeating: -1), SIMD2(repeating: 1))))
        let past = acos(min(max(dot(edge, gaze), -1), 1))
        return (simd_length(point), SIMD2(position.x, -position.y), past < 1e-4 ? 0 : past)
    }

    /// `hit(gaze:)` for a curved screen. Each angle round the row fixes how
    /// far along the gaze that row is reached; what is left there must lie on
    /// the middle column's line. Stepping round the row finds where it does,
    /// which halving then pins down. A screen wrapping past a quarter turn
    /// can be met more than once; the nearest meeting on the screen is the
    /// one seen, or else the nearest past its edge.
    private func curvedHit(gaze: SIMD3<Float>) -> (along: Float, at: SIMD2<Float>, miss: Float)? {
        let (across, halfHeight) = (normalize(right), length(up))
        let facing = normalize(cross(up, right))
        func result(along: Float, angle: Float) -> (along: Float, at: SIMD2<Float>, miss: Float)? {
            guard along > 1e-4 else { return nil }
            let rest = gaze * along - center - rowOffset(angle)
            let at = SIMD2(angle / halfArc, -dot(rest, up) / (halfHeight * halfHeight))
            return (along, at, max((abs(at.x) - 1) * halfArc, (abs(at.y) - 1) * halfHeight / along, 0))
        }
        let sideways = dot(gaze, across)
        guard abs(sideways) > 1e-5 else {
            // Straight through the middle column's plane.
            let towards = dot(gaze, facing)
            return abs(towards) > 1e-6 ? result(along: dot(center, facing) / towards, angle: 0) : nil
        }
        let side: Float = sideways < 0 ? -1 : 1
        // How far in front of the middle column's line the gaze is there:
        // positive before reaching it, negative past it.
        func offset(_ turn: Float) -> (value: Float, along: Float) {
            let along = rowRadius * sin(turn) / abs(sideways)
            let rest = gaze * along - center - rowOffset(turn * side)
            return (-dot(rest, facing), along)
        }
        let steps = 96
        let limit = min(maxHalfArc, .pi - 1e-3)
        var nearest: (onScreen: Bool, along: Float, turn: Float)?
        var (previous, before) = (Float(1e-4), offset(1e-4).value)
        for step in 1...steps {
            let turn = limit * Float(step) / Float(steps)
            let now = offset(turn).value
            defer { (previous, before) = (turn, now) }
            guard (before > 0) != (now > 0) else { continue }
            var (low, high) = (previous, turn)
            for _ in 0..<30 {
                let middle = (low + high) / 2
                if (offset(middle).value > 0) == (before > 0) { low = middle } else { high = middle }
            }
            let found = (low + high) / 2
            guard let hit = result(along: offset(found).along, angle: found * side) else { continue }
            let onScreen = hit.miss == 0
            if nearest.map({ onScreen != $0.onScreen ? onScreen : hit.along < $0.along }) ?? true {
                nearest = (onScreen, hit.along, found)
            }
        }
        return nearest.flatMap { result(along: $0.along, angle: $0.turn * side) }
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

/// How a screen bends round the viewer.
public enum CanvasShape: String, CaseIterable, Sendable {
    case flat = "Flat"
    /// Round the viewer like a curved monitor.
    case curved = "Curved"
    /// Round the viewer both ways, part of a ball centred on them.
    case wrapped = "Wrap Around You"
}

/// A virtual screen and where it hangs in the room.
public struct RoomScreen: Equatable, Sendable {
    /// The size in points. At distance 1 each point shows on one glasses
    /// pixel.
    public var width: Int
    public var height: Int
    /// Pixels per point: 2 for a HiDPI screen, whose finer pixels are
    /// filtered down onto the glasses' ones.
    public var scale: Int
    public var placement: ScreenPlacement
    /// Bent around the viewer, so every part of a row is equally far away.
    public var curved: Bool
    /// Bent both ways round the viewer, `verticalWrap` of the way to a
    /// sphere, where every pixel is equally far away and faces them. Curved
    /// whether or not `curved` is set.
    public var spherical: Bool
    /// How far a spherical screen's columns bend, from 0, straight, to 1,
    /// part of a sphere round the eyes.
    public var verticalWrap: Float
    /// Shares out how much smaller a spherical screen's pixels are near its
    /// top and bottom: a little larger in the middle, a little smaller at
    /// the edges, instead of full size in the middle and smallest at the
    /// edges.
    public var evenSize: Bool

    public init(
        width: Int, height: Int, scale: Int = 1, placement: ScreenPlacement = .straightAhead, curved: Bool = false,
        spherical: Bool = false, verticalWrap: Float = 1, evenSize: Bool = true
    ) {
        self.width = width
        self.height = height
        self.scale = scale
        self.placement = placement
        self.curved = curved
        self.spherical = spherical
        self.verticalWrap = verticalWrap
        self.evenSize = evenSize
    }

    /// `curved` and `spherical` as one choice.
    public var shape: CanvasShape {
        get { spherical ? .wrapped : curved ? .curved : .flat }
        set {
            curved = newValue != .flat
            spherical = newValue == .wrapped
        }
    }

    /// Whether a virtual screen of this size can be made without upsetting
    /// macOS: within `maxVirtualScreenSide`, or one of the few sizes probed
    /// past it.
    public static func isAllowed(width: Int, height: Int, scale: Int) -> Bool {
        if sizesProbedBeyondLimit.contains(where: { $0 == (width, height, scale) }) {
            return true
        }
        // Checked before multiplying, so no size can overflow.
        return (scale == 1 || scale == 2) && (1...maxVirtualScreenSide / scale).contains(width)
            && (1...maxVirtualScreenSide / scale).contains(height)
    }

    /// Where `pixel` of the screen (x right, y down) is in the room, bent by
    /// `curveRadius` if it is curved.
    public func roomPoint(ofPixel pixel: SIMD2<Float>, curveRadius: Float = 1) -> SIMD3<Float> {
        let size = SIMD2(Float(max(width, 1)), Float(max(height, 1)))
        let unit = pixel / size
        return surface(curveRadius: curveRadius).point(at: SIMD2(unit.x * 2 - 1, 1 - unit.y * 2))
    }

    /// Where the screen is in the room. A curved screen's rows are arcs with
    /// a radius `curveRadius` times its distance: 1 surrounds the viewer
    /// evenly, less bends it more, more bends it less. A spherical one is
    /// always centred on the viewer, bending up and down by `verticalWrap`.
    public func surface(curveRadius: Float = 1) -> ScreenSurface {
        let (center, right, up) = placement.frame(width: width, height: height)
        guard curved || spherical else {
            return ScreenSurface(center: center, right: right, up: up, spin: placement.spin)
        }
        // Made straight ahead and level, where it bends evenly round the
        // viewer, then turned as a whole to where it hangs: raised, lowered
        // or tilted, it looks the same as straight ahead when faced.
        let (halfWidth, halfHeight) = (length(right), length(up))
        let wrap = spherical ? min(max(verticalWrap, 0), 1) : 0
        var rowRadius = max(placement.distance * (spherical ? 1 : curveRadius), halfWidth / maxHalfArc)
        // Pixels at the top and bottom rows shrink with their circle; drawn
        // `grow` times larger all over, the middle comes out as much above
        // its own size as the edges fall below it.
        var grow: Float = 1
        if wrap > 0 {
            // Opened out until the top and bottom rows, grown as they will
            // be, stay short of the poles and the rows short of meeting
            // behind.
            for _ in 0..<80 {
                let columnRadius = rowRadius / wrap
                func edgeRow(_ grow: Float) -> Float {
                    rowRadius - columnRadius * (1 - cos(mercatorRise(halfHeight * grow / columnRadius)))
                }
                grow = 1
                if evenSize {
                    // The edge moves as the growth does: a few rounds settle it.
                    for _ in 0..<6 {
                        grow = 2 / (1 + max(edgeRow(grow), 0) / rowRadius)
                    }
                }
                let rise = mercatorRise(halfHeight * grow / columnRadius)
                if rise <= maxHalfRise, edgeRow(grow) >= 0.35 * rowRadius, halfWidth * grow / rowRadius <= maxHalfArc {
                    break
                }
                rowRadius *= 1.05
            }
        }
        // Keep a wrapped canvas centred on the viewer even when its size
        // would otherwise force a larger radius. Fit its extents uniformly
        // instead of shifting the centre of its sphere behind the eyes.
        let fit = spherical ? placement.distance / rowRadius : 1
        return ScreenSurface(
            center: SIMD3(0, 0, -placement.distance), right: SIMD3(halfWidth * grow * fit, 0, 0),
            up: SIMD3(0, halfHeight * grow * fit, 0), halfArc: halfWidth * grow / rowRadius, spin: placement.orientation,
            wrap: wrap)
    }
}

/// The angle up a Mercator map at `height` along it, both in radians: the
/// map's rows crowd towards the poles by just as much as the circles they
/// lie on shrink, so shapes keep their proportions.
func mercatorRise(_ height: Float) -> Float {
    atan(sinh(height))
}

/// How far along a Mercator map the angle `rise` up lies.
func mercatorHeight(_ rise: Float) -> Float {
    let clamped = min(max(rise, -1.55), 1.55)
    return log(tan(.pi / 4 + clamped / 2))
}

/// The columns of pixels each capture of a `width` pixel wide screen
/// covers: the whole screen, or tiles about one view wide.
public func captureTiles(width: Int) -> [Range<Int>] {
    guard width > maxUntiledWidth else { return [0..<width] }
    let count = (width + tileWidth - 1) / tileWidth
    return (0..<count).map { index in (index * width / count)..<((index + 1) * width / count) }
}

/// The pixel on `screen` the viewer looks at along `gaze`, kept on the
/// screen when the gaze is just past its edge; nil when looking away.
public func gazeTarget(_ gaze: SIMD3<Float>, on screen: RoomScreen, curveRadius: Float = 1) -> SIMD2<Float>? {
    guard let hit = screen.surface(curveRadius: curveRadius).hit(gaze: gaze), hit.miss < gazeSlack else {
        return nil
    }
    let size = SIMD2(Float(screen.width), Float(screen.height))
    return (simd_clamp(hit.at, SIMD2(repeating: -1), SIMD2(repeating: 1)) + 1) * 0.5 * size
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
/// the head turns and tilts, and stays where it was when let go, tilted as
/// the head was.
public struct ScreenGrab: Equatable, Sendable {
    private let inView: SIMD3<Float>
    private let upInView: SIMD3<Float>

    /// `headRotation` turns head directions into room directions.
    public init(placement: ScreenPlacement, headRotation: simd_float3x3) {
        inView = headRotation.transpose * placement.direction
        upInView = headRotation.transpose * placement.up
    }

    public func placement(headRotation: simd_float3x3, distance: Float) -> ScreenPlacement {
        ScreenPlacement(direction: headRotation * inView, distance: distance)
            .tilted(toward: headRotation * upInView)
    }
}

/// The canvas as the glasses see it this frame.
public struct RoomView: Sendable {
    /// Something drawn on the canvas's surface: a capture of the canvas or
    /// of a tile of it, or one of the things shown with it.
    public struct Panel: Sendable {
        public enum Source: Equatable, Sendable {
            /// The capture of this tile of the canvas.
            case canvas(Int)
            /// The dashboard above the canvas.
            case status
            /// The window pinned above the canvas.
            case pinned
            /// The mouse pointer, drawn over the canvas.
            case pointer
            /// The glow round the canvas, in the colours of its edges.
            case ambient
        }

        public var source: Source
        public var surface: ScreenSurface
        /// The part of the surface it covers.
        public var rect: SurfaceRect
        /// Outlined, because the canvas is being carried or was just moved.
        public var highlighted: Bool
        /// For the glow round the canvas: the canvas's size in points, how
        /// far out the glow reaches in points, and how bright it is.
        public var halo: SIMD4<Float>

        public init(
            source: Source, surface: ScreenSurface, rect: SurfaceRect = .whole, highlighted: Bool = false,
            halo: SIMD4<Float> = .zero
        ) {
            self.source = source
            self.surface = surface
            self.rect = rect
            self.highlighted = highlighted
            self.halo = halo
        }

        /// The part of the canvas's width it covers, from -1 to 1.
        public var span: SIMD2<Float> { SIMD2(rect.left, rect.right) }

        /// The canvas tile it shows, if it is one.
        public var tile: Int? {
            if case .canvas(let tile) = source { tile } else { nil }
        }
    }

    /// Turns head directions into room directions, as the head is when the
    /// top row of the glasses lights up.
    public var headRotation: simd_float3x3
    /// The same when the bottom row lights up: the glasses light their rows
    /// top to bottom, so on a quick turn the head has moved on by then. Nil
    /// to draw every row from `headRotation`.
    public var scanEndRotation: simd_float3x3?
    /// Tangent of half the field of view, horizontally and vertically, of
    /// each eye's view.
    public var tanHalfFov: SIMD2<Float>
    public var panels: [Panel]
    /// The eyes whose views the output holds side by side, in that order,
    /// with their positions in room units.
    public var eyes: [EyeOptics] = []

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
        let rect = panel.rect
        for column in 0...4 {
            let x = rect.left + (rect.right - rect.left) * Float(column) / 4
            for row in 0...4 {
                let point = panel.surface.point(at: SIMD2(x, rect.bottom + (rect.top - rect.bottom) * Float(row) / 4))
                if let output = outputPoint(ofRoom: point), abs(output.x) <= limit, abs(output.y) <= limit {
                    return true
                }
            }
        }
        return false
    }
}
