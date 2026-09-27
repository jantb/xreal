import Foundation
import simd

public let zoomLevels: [Float] = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0]
public let deadzoneLevels: [Float] = [0.0, 0.005, 0.01, 0.018, 0.03]  // rad
private let minSensitivity: Float = 0.25
private let maxSensitivity: Float = 3.0

// Field of view of the Air-series displays: 46° diagonal at 16:9. At
// sensitivity 1.0 content moves exactly as far as the head turns, so the
// screen appears fixed in space.
let horizontalFov: Float = 0.7087  // rad
let verticalFov: Float = 0.4104  // rad

// One Euro filter tuning (angles in rad): heavy smoothing while the head is
// nearly still, almost none during fast turns.
private let filterMinCutoff: Float = 1.0  // Hz
private let filterBeta: Float = 20.0
private let filterDerivativeCutoff: Float = 4.0  // Hz

/// A region of the source image, in source pixels.
public struct ViewportRect: Equatable, Sendable {
    public var x: Float = 0
    public var y: Float = 0
    public var width: Float = 0
    public var height: Float = 0

    public init(x: Float = 0, y: Float = 0, width: Float = 0, height: Float = 0) {
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    }
}

/// Adaptive low-pass filter (Casiez et al., "1€ Filter").
struct OneEuroFilter {
    private var value: Float?
    private var derivative: Float = 0

    mutating func reset(_ value: Float) {
        self.value = value
        derivative = 0
    }

    mutating func filter(_ value: Float, dt: Float) -> Float {
        guard let previous = self.value else {
            reset(value)
            return value
        }
        let measuredDerivative = (value - previous) / dt
        derivative += (measuredDerivative - derivative) * smoothingAlpha(cutoff: filterDerivativeCutoff, dt: dt)
        let cutoff = filterMinCutoff + filterBeta * abs(derivative)
        let filtered = previous + (value - previous) * smoothingAlpha(cutoff: cutoff, dt: dt)
        self.value = filtered
        return filtered
    }

    private func smoothingAlpha(cutoff: Float, dt: Float) -> Float {
        let tau = 1 / (2 * .pi * cutoff)
        return 1 / (1 + tau / dt)
    }
}

/// A virtual monitor placed in the room at unit distance, seen through the
/// glasses. Directions use the head frame x right, y up, -z forward, and
/// output points run from -1 to 1 across the view, y up. The renderer's
/// shader does the same mapping per pixel.
public struct SpatialView: Sendable {
    /// Turns head-frame directions into room directions.
    public var rotation: simd_float3x3
    /// Tangent of half the field of view, horizontally and vertically.
    public var tanHalfFov: SIMD2<Float>
    /// Width and height of the screen at unit distance; for a curved screen
    /// the width is the angle it spans, in radians.
    public var screenSize: SIMD2<Float>
    public var sourceSize: SIMD2<Float>
    public var curved: Bool
    /// Manual pan, in source pixels.
    public var pan: SIMD2<Float> = .zero
    /// Zooming out by `scale` shrinks the screen towards `pivot`, in source
    /// pixels, instead of towards its center.
    public var pivot: SIMD2<Float> = .zero
    public var scale: Float = 1

    /// The source pixel seen at `output`, which may lie outside the source,
    /// or nil when the view there points away from the screen.
    public func sourcePoint(atOutput output: SIMD2<Float>) -> SIMD2<Float>? {
        let room = rotation * SIMD3(output * tanHalfFov, -1)
        let onScreen: SIMD2<Float>
        if curved {
            let distance = (room.x * room.x + room.z * room.z).squareRoot()
            guard distance > 1e-6 else { return nil }
            onScreen = SIMD2(atan2(room.x, -room.z), room.y / distance)
        } else {
            guard room.z < -1e-6 else { return nil }
            onScreen = SIMD2(room.x, room.y) / -room.z
        }
        let unit = SIMD2(onScreen.x / screenSize.x + 0.5, 0.5 - onScreen.y / screenSize.y)
        let unscaled = unit * sourceSize + pan
        return pivot + (unscaled - pivot) / scale
    }

    /// Where the source pixel `point` appears in the view, or nil when it is
    /// behind the viewer.
    public func outputPoint(ofSource point: SIMD2<Float>) -> SIMD2<Float>? {
        let unscaled = pivot + (point - pivot) * scale
        let unit = (unscaled - pan) / sourceSize
        let onScreen = SIMD2((unit.x - 0.5) * screenSize.x, (0.5 - unit.y) * screenSize.y)
        let room =
            curved
            ? SIMD3(sin(onScreen.x), onScreen.y, -cos(onScreen.x))
            : SIMD3(onScreen.x, onScreen.y, -1)
        let head = rotation.transpose * room
        guard head.z < -1e-6 else { return nil }
        return SIMD2(head.x, head.y) / -head.z / tanHalfFov
    }
}

/// What the glasses show of the source this frame.
public enum ViewGeometry: Sendable {
    case crop(ViewportRect)
    case spatial(SpatialView)

    /// Whether `point`, in source pixels, is inside the view shrunk to
    /// `margin` (0 to 1) of its size.
    public func shows(_ point: SIMD2<Float>, margin: Float) -> Bool {
        switch self {
        case .crop(let rect):
            let halfWidth = rect.width * 0.5
            let halfHeight = rect.height * 0.5
            return abs(point.x - (rect.x + halfWidth)) <= halfWidth * margin
                && abs(point.y - (rect.y + halfHeight)) <= halfHeight * margin
        case .spatial(let view):
            guard let output = view.outputPoint(ofSource: point) else { return false }
            return abs(output.x) <= margin && abs(output.y) <= margin
        }
    }
}

/// Turns head pose into what part of the source the glasses show.
public struct ViewportController: Sendable {
    private var center = HeadPose()
    private var manualX: Float = 0
    private var manualY: Float = 0
    private var filterYaw = OneEuroFilter()
    private var filterPitch = OneEuroFilter()
    private var filterRoll = OneEuroFilter()
    private var offsetYaw: Float = 0
    private var offsetPitch: Float = 0
    private var offsetRoll: Float = 0
    private var initialized = false
    public var zoomIndex: Int
    public private(set) var sensitivity: Float
    public private(set) var deadzoneIndex: Int
    public private(set) var frozen = false
    public var projection: Projection
    public var edge: EdgeMode
    public var followsRoll: Bool

    public init(settings: Settings) {
        zoomIndex = min(settings.zoomIndex, zoomLevels.count - 1)
        sensitivity = min(max(settings.sensitivity, minSensitivity), maxSensitivity)
        deadzoneIndex = min(settings.deadzoneIndex, deadzoneLevels.count - 1)
        projection = settings.projection
        edge = settings.edge
        followsRoll = settings.followRoll
    }

    public func store(into settings: inout Settings) {
        settings.zoomIndex = zoomIndex
        settings.sensitivity = sensitivity
        settings.deadzoneIndex = deadzoneIndex
        settings.projection = projection
        settings.edge = edge
        settings.followRoll = followsRoll
    }

    public var zoom: Float { zoomLevels[zoomIndex] }
    public var deadzone: Float { deadzoneLevels[deadzoneIndex] }

    public mutating func recenter(_ pose: HeadPose) {
        center = pose
        manualX = 0
        manualY = 0
        offsetYaw = 0
        offsetPitch = 0
        offsetRoll = 0
        filterYaw.reset(0)
        filterPitch.reset(0)
        filterRoll.reset(0)
        initialized = true
    }

    /// Restores default zoom, sensitivity and deadzone, then recenters. The
    /// projection and edge choices are kept.
    public mutating func reset(_ pose: HeadPose) {
        var defaults = Settings()
        defaults.projection = projection
        defaults.edge = edge
        defaults.followRoll = followsRoll
        self = ViewportController(settings: defaults)
        recenter(pose)
    }

    public mutating func pan(dx: Float, dy: Float) {
        manualX += dx
        manualY += dy
    }

    public mutating func zoomIn() {
        zoomIndex = min(zoomIndex + 1, zoomLevels.count - 1)
    }

    public mutating func zoomOut() {
        zoomIndex = max(zoomIndex - 1, 0)
    }

    public mutating func adjustSensitivity(_ factor: Float) {
        sensitivity = min(max(sensitivity * factor, minSensitivity), maxSensitivity)
    }

    public mutating func resetSensitivity() {
        sensitivity = Settings().sensitivity
    }

    public mutating func cycleDeadzone() {
        deadzoneIndex = (deadzoneIndex + 1) % deadzoneLevels.count
    }

    public mutating func setDeadzone(_ index: Int) {
        deadzoneIndex = min(max(index, 0), deadzoneLevels.count - 1)
    }

    public mutating func toggleFreeze() {
        frozen.toggle()
    }

    /// Follows the head to `pose`; `dt` is the time since the previous
    /// update, in seconds.
    public mutating func track(pose: HeadPose, dt: Float) {
        if !initialized {
            recenter(pose)
        }
        guard !frozen else { return }
        let dt = max(dt, 1e-4)
        offsetYaw = filterYaw.filter(applyDeadzone(wrapAngle(pose.yaw - center.yaw), deadzone), dt: dt)
        offsetPitch = filterPitch.filter(applyDeadzone(pose.pitch - center.pitch, deadzone), dt: dt)
        offsetRoll = filterRoll.filter(applyDeadzone(wrapAngle(pose.roll - center.roll), deadzone), dt: dt)
    }

    /// Tracks `pose` and returns the crop of the source to show.
    public mutating func update(
        pose: HeadPose, dt: Float, outputWidth: Int, outputHeight: Int, sourceWidth: Int,
        sourceHeight: Int
    ) -> ViewportRect {
        track(pose: pose, dt: dt)
        return cropRect(
            outputWidth: outputWidth, outputHeight: outputHeight, sourceWidth: sourceWidth,
            sourceHeight: sourceHeight)
    }

    /// The view for the current projection. `scale` below 1 zooms out
    /// further than the chosen zoom, around the middle of the view.
    public func geometry(
        outputWidth: Int, outputHeight: Int, sourceWidth: Int, sourceHeight: Int, scale: Float = 1
    ) -> ViewGeometry {
        switch projection {
        case .crop:
            .crop(
                cropRect(
                    outputWidth: outputWidth, outputHeight: outputHeight, sourceWidth: sourceWidth,
                    sourceHeight: sourceHeight, scale: scale))
        case .flat, .curved:
            .spatial(
                spatialView(
                    outputWidth: outputWidth, outputHeight: outputHeight, sourceWidth: sourceWidth,
                    sourceHeight: sourceHeight, scale: scale))
        }
    }

    public func cropRect(
        outputWidth: Int, outputHeight: Int, sourceWidth: Int, sourceHeight: Int, scale: Float = 1
    ) -> ViewportRect {
        let sourceWidth = Float(max(sourceWidth, 1))
        let sourceHeight = Float(max(sourceHeight, 1))
        let chosenWidth = max(Float(outputWidth) / zoom, 1)
        let chosenHeight = max(Float(outputHeight) / zoom, 1)

        // Source pixels per radian: one view width per field of view. Taken
        // at the chosen zoom so zooming out by `scale` keeps the middle put.
        let pixelsPerRadX = chosenWidth / horizontalFov * sensitivity
        let pixelsPerRadY = chosenHeight / verticalFov * sensitivity
        let centerX = sourceWidth * 0.5 + manualX - offsetYaw * pixelsPerRadX
        let centerY = sourceHeight * 0.5 + manualY + offsetPitch * pixelsPerRadY

        var width = chosenWidth / scale
        var height = chosenHeight / scale
        switch edge {
        case .snap:
            width = min(width, sourceWidth)
            height = min(height, sourceHeight)
            let x = min(max(centerX - width * 0.5, 0), sourceWidth - width)
            let y = min(max(centerY - height * 0.5, 0), sourceHeight - height)
            return ViewportRect(x: x, y: y, width: width, height: height)
        case .black:
            return ViewportRect(x: centerX - width * 0.5, y: centerY - height * 0.5, width: width, height: height)
        }
    }

    public func spatialView(
        outputWidth: Int, outputHeight: Int, sourceWidth: Int, sourceHeight: Int, scale: Float = 1
    ) -> SpatialView {
        let outputWidth = Float(max(outputWidth, 1))
        let outputHeight = Float(max(outputHeight, 1))
        let sourceSize = SIMD2(Float(max(sourceWidth, 1)), Float(max(sourceHeight, 1)))
        let tanX = tan(horizontalFov * 0.5)
        // At zoom 1 a source pixel covers one output pixel in the middle of
        // the view when looking straight at it.
        let sourcePixelsPerRadian = outputWidth * 0.5 / tanX * zoom

        let yaw = offsetYaw * sensitivity
        let pitch = offsetPitch * sensitivity
        let roll = followsRoll ? offsetRoll : 0
        var view = SpatialView(
            rotation: rotationY(yaw) * rotationX(-pitch) * rotationZ(-roll),
            tanHalfFov: SIMD2(tanX, tanX * outputHeight / outputWidth),
            screenSize: sourceSize / sourcePixelsPerRadian, sourceSize: sourceSize,
            curved: projection == .curved, pan: SIMD2(manualX, manualY))
        let middle = view.sourcePoint(atOutput: .zero) ?? sourceSize * 0.5
        view.pivot = simd_clamp(middle, .zero, sourceSize)
        view.scale = scale
        return view
    }
}

private func rotationX(_ angle: Float) -> simd_float3x3 {
    let c = cos(angle)
    let s = sin(angle)
    return simd_float3x3(SIMD3(1, 0, 0), SIMD3(0, c, s), SIMD3(0, -s, c))
}

private func rotationY(_ angle: Float) -> simd_float3x3 {
    let c = cos(angle)
    let s = sin(angle)
    return simd_float3x3(SIMD3(c, 0, -s), SIMD3(0, 1, 0), SIMD3(s, 0, c))
}

private func rotationZ(_ angle: Float) -> simd_float3x3 {
    let c = cos(angle)
    let s = sin(angle)
    return simd_float3x3(SIMD3(c, s, 0), SIMD3(-s, c, 0), SIMD3(0, 0, 1))
}

private func applyDeadzone(_ value: Float, _ deadzone: Float) -> Float {
    let magnitude = abs(value)
    if magnitude <= deadzone {
        return 0
    }
    return (value < 0 ? -1 : 1) * (magnitude - deadzone)
}
