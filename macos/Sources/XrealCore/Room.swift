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

    /// The placement matching where macOS arranged a screen relative to the
    /// main display, both in global points (y down). The main display is
    /// straight ahead, and one point spans one glasses pixel.
    public init(arrangedAt frame: CGRect, around main: CGRect) {
        let offset = SIMD2(Float(frame.midX - main.midX), Float(main.midY - frame.midY)) * roomUnitsPerPixel
        self.init(direction: SIMD3(offset.x, offset.y, -1))
    }

    /// Where macOS should arrange a `width` × `height` screen so the mouse
    /// crosses between displays the way they sit in the room. The inverse of
    /// `init(arrangedAt:around:)`, for screens in front of the viewer.
    public func arrangedOrigin(width: Int, height: Int, around main: CGRect) -> CGPoint {
        // Screens behind the viewer are put at the side they are nearest to.
        let ahead = max(-direction.z, 0.05)
        let offset = SIMD2(direction.x, direction.y) / ahead / roomUnitsPerPixel
        return CGPoint(
            x: (main.midX + CGFloat(offset.x) - CGFloat(width) / 2).rounded(),
            y: (main.midY - CGFloat(offset.y) - CGFloat(height) / 2).rounded())
    }

    public func movedAway(by factor: Float) -> ScreenPlacement {
        ScreenPlacement(direction: direction, distance: distance * factor)
    }

    /// The screen's middle and half extents along its right and up edges,
    /// in room coordinates.
    public func frame(width: Int, height: Int) -> (center: SIMD3<Float>, right: SIMD3<Float>, up: SIMD3<Float>) {
        let right = normalize(cross(direction, worldUp))
        let up = cross(right, direction)
        return (
            direction * distance, right * (Float(width) * roomUnitsPerPixel * 0.5),
            up * (Float(height) * roomUnitsPerPixel * 0.5)
        )
    }

    /// How far along `gaze` (a unit vector) the screen is hit, and where on
    /// it, from -1 to 1 across and down; nil when the gaze misses its plane.
    func hit(gaze: SIMD3<Float>, width: Int, height: Int) -> (along: Float, at: SIMD2<Float>)? {
        let (center, right, up) = frame(width: width, height: height)
        let facing = dot(gaze, direction)
        guard facing > 1e-4 else { return nil }
        let along = dot(center, direction) / facing
        let offset = gaze * along - center
        return (along, SIMD2(dot(offset, right) / dot(right, right), -dot(offset, up) / dot(up, up)))
    }

    private static func upright(_ direction: SIMD3<Float>) -> SIMD3<Float> {
        let unit = length(direction) > 1e-6 ? normalize(direction) : SIMD3(0, 0, -1)
        let elevation = min(max(asin(min(max(unit.y, -1), 1)), -maxElevation), maxElevation)
        var flat = SIMD2(unit.x, unit.z)
        flat = length(flat) > 1e-6 ? normalize(flat) : SIMD2(0, -1)
        return SIMD3(flat.x * cos(elevation), sin(elevation), flat.y * cos(elevation))
    }
}

/// A virtual screen and where it hangs in the room, nil until it is placed.
public struct RoomScreen: Equatable, Sendable {
    public var width: Int
    public var height: Int
    public var placement: ScreenPlacement?

    public init(width: Int, height: Int, placement: ScreenPlacement? = nil) {
        self.width = width
        self.height = height
        self.placement = placement
    }
}

/// The screen the viewer is looking at along `gaze`: the nearest one the
/// gaze passes through, or else the one whose edge is closest to it.
public func screenLooked(at gaze: SIMD3<Float>, among screens: [RoomScreen]) -> Int? {
    var best: (index: Int, along: Float)?
    var nearest: (index: Int, miss: Float)?
    for (index, screen) in screens.enumerated() {
        guard let placement = screen.placement,
            let hit = placement.hit(gaze: gaze, width: screen.width, height: screen.height)
        else { continue }
        let past = max(abs(hit.at.x), abs(hit.at.y))
        if past <= 1 {
            if best == nil || hit.along < best!.along {
                best = (index, hit.along)
            }
        } else {
            // Angle beyond the edge, roughly: how far past it, over the distance.
            let (_, right, up) = placement.frame(width: screen.width, height: screen.height)
            let halfExtent = abs(hit.at.x) > abs(hit.at.y) ? length(right) : length(up)
            let miss = (past - 1) * halfExtent / hit.along
            if miss < gazeSlack, nearest == nil || miss < nearest!.miss {
                nearest = (index, miss)
            }
        }
    }
    return best?.index ?? nearest?.index
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
    public struct Panel: Sendable {
        /// Which captured screen to show.
        public var source: Int
        /// Middle and half extents along the right and up edges, in room
        /// coordinates.
        public var center: SIMD3<Float>
        public var right: SIMD3<Float>
        public var up: SIMD3<Float>
        /// Outlined, because it is the one being looked at or carried.
        public var highlighted: Bool
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
