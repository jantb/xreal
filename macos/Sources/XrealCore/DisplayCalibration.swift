import Foundation
import simd

/// How one eye's display shows the room. Directions use the head frame:
/// x right, y up, -z ahead.
public struct EyeOptics: Equatable, Sendable {
    /// Focal lengths and the optical centre, in the display's pixels from
    /// its top-left corner.
    public var focal: SIMD2<Float>
    public var center: SIMD2<Float>
    /// The display's size in pixels.
    public var size: SIMD2<Float>
    /// Turns directions as this eye looks into head directions.
    public var rotation: simd_float3x3
    /// Where the eye is, from the middle between the two eyes. In metres as
    /// calibrated; the room scales it to room units.
    public var position: SIMD3<Float>
    /// How the display's lens moves its pixels; nil to draw straight.
    public var distortion: LensDistortion?

    public init(
        focal: SIMD2<Float>, center: SIMD2<Float>, size: SIMD2<Float>, rotation: simd_float3x3,
        position: SIMD3<Float>, distortion: LensDistortion? = nil
    ) {
        self.focal = focal
        self.center = center
        self.size = size
        self.rotation = rotation
        self.position = position
        self.distortion = distortion
    }

    /// Where `point`, in the head frame, lands on the display, in its
    /// pixels; nil when it is behind the eye.
    public func pixel(of point: SIMD3<Float>) -> SIMD2<Float>? {
        let seen = rotation.transpose * (point - position)
        guard seen.z < -1e-6 else { return nil }
        return SIMD2(center.x + focal.x * seen.x / -seen.z, center.y - focal.y * seen.y / -seen.z)
    }

    /// The display pixel that shows `point` through the lens: `pixel(of:)`
    /// moved by the lens's distortion. The renderer does the same.
    public func displayPixel(of point: SIMD3<Float>) -> SIMD2<Float>? {
        pixel(of: point).map { distortion?.displayPixel(showing: $0) ?? $0 }
    }
}

/// The glasses' two displays as their factory calibration describes them:
/// each eye's focal length and optical centre, and where each display sits
/// and how it is turned. The optical centres are off the middle, and
/// differently for each eye, and the displays are turned slightly inwards
/// so both eyes' pictures meet some metres ahead.
public struct DisplayCalibration: Equatable, Sendable {
    public var left: EyeOptics
    public var right: EyeOptics

    public init(left: EyeOptics, right: EyeOptics) {
        self.left = left
        self.right = right
    }

    /// Without a calibration: the Air series' nominal optics, both eyes'
    /// pictures meeting 4 m ahead.
    public static let nominal: DisplayCalibration = {
        let size = SIMD2<Float>(1920, 1080)
        // Square pixels: the same focal length both ways.
        let focal = SIMD2(repeating: size.x / 2 / tan(horizontalFov / 2))
        let halfSeparation: Float = 0.0315
        func eye(side: Float) -> EyeOptics {
            // Turned in to look at a point 4 m straight ahead.
            let turn = side * atan(halfSeparation / 4)
            return EyeOptics(
                focal: focal, center: size / 2, size: size,
                rotation: simd_float3x3(simd_quatf(angle: turn, axis: SIMD3(0, 1, 0))),
                position: SIMD3(side * halfSeparation, 0, 0))
        }
        return DisplayCalibration(left: eye(side: -1), right: eye(side: 1))
    }()

    /// Reads the `display` section of the glasses' calibration JSON; nil
    /// when it is missing or malformed.
    public static func parse(config: Data) -> DisplayCalibration? {
        guard
            let root = try? JSONSerialization.jsonObject(with: config) as? [String: Any],
            let display = root["display"] as? [String: Any],
            let resolution = numbers(display["resolution"], count: 2),
            let leftK = numbers(display["k_left_display"], count: 9),
            let rightK = numbers(display["k_right_display"], count: 9),
            let leftQ = numbers(display["target_q_left_display"], count: 4),
            let rightQ = numbers(display["target_q_right_display"], count: 4),
            let leftP = numbers(display["target_p_left_display"], count: 3),
            let rightP = numbers(display["target_p_right_display"], count: 3)
        else { return nil }

        // The calibration uses a camera's axes: x right, y down, z ahead.
        // Each display's turn is given as (x, y, z, w) from the display to
        // the IMU, and its position in the IMU's frame. Only how the eyes
        // sit relative to the two displays together matters here: that is
        // the view straight ahead, and the head tracking has its own idea
        // of how the IMU is turned.
        let leftTurn = simd_quatf(ix: leftQ[0], iy: leftQ[1], iz: leftQ[2], r: leftQ[3])
        let rightTurn = simd_quatf(ix: rightQ[0], iy: rightQ[1], iz: rightQ[2], r: rightQ[3])
        let together = simd_normalize(simd_quatf(vector: leftTurn.vector + rightTurn.vector))
        let middle = (SIMD3(leftP[0], leftP[1], leftP[2]) + SIMD3(rightP[0], rightP[1], rightP[2])) / 2
        let cameraToHead = simd_float3x3(diagonal: SIMD3(1, -1, -1))
        let size = SIMD2(resolution[0], resolution[1])
        // Each lens's distortion, when the calibration has it; without it
        // the eye is drawn straight.
        let lenses = root["display_distortion"] as? [String: Any]
        func distortion(_ name: String) -> LensDistortion? {
            guard let lens = lenses?[name] as? [String: Any], (lens["type"] as? NSNumber)?.intValue == 1,
                let columns = (lens["num_col"] as? NSNumber)?.intValue, let rows = (lens["num_row"] as? NSNumber)?.intValue,
                let data = numbers(lens["data"], count: columns * rows * 4)
            else { return nil }
            return LensDistortion(grid: data, gridColumns: columns, gridRows: rows, size: size)
        }
        func eye(k: [Float], turn: simd_quatf, position: [Float], lens: String) -> EyeOptics {
            let relative = simd_float3x3(together.inverse * turn)
            let offset = simd_float3x3(together.inverse) * (SIMD3(position[0], position[1], position[2]) - middle)
            return EyeOptics(
                focal: SIMD2(k[0], k[4]), center: SIMD2(k[2], k[5]), size: size,
                rotation: cameraToHead * relative * cameraToHead, position: cameraToHead * offset,
                distortion: distortion(lens))
        }
        let calibration = DisplayCalibration(
            left: eye(k: leftK, turn: leftTurn, position: leftP, lens: "left_display"),
            right: eye(k: rightK, turn: rightTurn, position: rightP, lens: "right_display"))
        // A calibration this far off is not one to draw with.
        let separation = calibration.right.position.x - calibration.left.position.x
        guard (0.04...0.09).contains(separation), size.x > 0, size.y > 0,
            [calibration.left, calibration.right].allSatisfy({ $0.focal.x > 0 && $0.focal.y > 0 })
        else { return nil }
        return calibration
    }

    private static func numbers(_ value: Any?, count: Int) -> [Float]? {
        guard let values = value as? [NSNumber], values.count == count else { return nil }
        let floats = values.map(\.floatValue)
        return floats.allSatisfy(\.isFinite) ? floats : nil
    }
}
