import Foundation
import simd

// The display pixels found for the undistorted picture are laid out on an
// even grid this far apart, reaching this far past the display's edges.
private let lookupStep: Float = 24  // pixels
private let lookupMargin: Float = 48  // pixels
// Rounds of refining each display pixel; the lens moves pixels smoothly by
// at most some tens of pixels, so a few rounds settle it.
private let inversionRounds = 16

/// How a display's lens moves its pixels, from the glasses' factory
/// calibration. The lens magnifies the edges more than the middle, so the
/// picture for a display is drawn pulled in towards its corners, by up to
/// about 20 pixels, to look straight through the lens.
public struct LensDistortion: Equatable, Sendable {
    /// Where the lookup grid starts, in pixels of the undistorted picture.
    public let origin: SIMD2<Float>
    public let step: Float
    public let columns: Int
    public let rows: Int
    /// For each point of the lookup grid, row by row, the display pixel
    /// whose light the lens shows there.
    public let displayPixels: [SIMD2<Float>]

    /// `data` holds, row by row over a `gridColumns` × `gridRows` grid of display
    /// pixels, each display pixel followed by where the lens shows it in the
    /// undistorted picture. The grid need not be evenly spaced, but its
    /// columns and rows must line up. Nil when it does not describe a
    /// sensible grid over a display of `size`.
    public init?(grid data: [Float], gridColumns columns: Int, gridRows rows: Int, size: SIMD2<Float>) {
        guard columns >= 2, rows >= 2, data.count == columns * rows * 4, data.allSatisfy(\.isFinite) else {
            return nil
        }
        let points = (0..<columns * rows).map { index in
            (display: SIMD2(data[index * 4], data[index * 4 + 1]), shown: SIMD2(data[index * 4 + 2], data[index * 4 + 3]))
        }
        let across = (0..<columns).map { points[$0].display.x }
        let down = (0..<rows).map { points[$0 * columns].display.y }
        // Every row shares the columns' positions and every column the rows'.
        for row in 0..<rows {
            for column in 0..<columns {
                let display = points[row * columns + column].display
                guard display.x == across[column], display.y == down[row] else { return nil }
            }
        }
        guard zip(across, across.dropFirst()).allSatisfy({ $0 < $1 }),
            zip(down, down.dropFirst()).allSatisfy({ $0 < $1 }),
            points.allSatisfy({ simd_distance($0.display, $0.shown) < size.x * 0.05 })
        else { return nil }
        let grid = Grid(across: across, down: down, shown: points.map(\.shown))

        let lookupColumns = Int(((size.x + 2 * lookupMargin) / lookupStep).rounded(.up)) + 1
        let lookupRows = Int(((size.y + 2 * lookupMargin) / lookupStep).rounded(.up)) + 1
        var displayPixels: [SIMD2<Float>] = []
        displayPixels.reserveCapacity(lookupColumns * lookupRows)
        for row in 0..<lookupRows {
            for column in 0..<lookupColumns {
                let shown = SIMD2(repeating: -lookupMargin) + SIMD2(Float(column), Float(row)) * lookupStep
                // The lens moves pixels by a little, smoothly: start where
                // it would be without the lens and correct by how far off
                // the lens shows that pixel.
                var display = shown
                for _ in 0..<inversionRounds {
                    display -= grid.shown(atDisplay: display) - shown
                }
                displayPixels.append(display)
            }
        }
        origin = SIMD2(repeating: -lookupMargin)
        step = lookupStep
        self.columns = lookupColumns
        self.rows = lookupRows
        self.displayPixels = displayPixels
    }

    /// The display pixel whose light the lens shows at `shown`, in pixels
    /// of the undistorted picture.
    public func displayPixel(showing shown: SIMD2<Float>) -> SIMD2<Float> {
        let cell = (shown - origin) / step
        let x = min(max(cell.x, 0), Float(columns - 1))
        let y = min(max(cell.y, 0), Float(rows - 1))
        let (column, row) = (min(Int(x), columns - 2), min(Int(y), rows - 2))
        let (fx, fy) = (x - Float(column), y - Float(row))
        func at(_ c: Int, _ r: Int) -> SIMD2<Float> { displayPixels[r * columns + c] }
        let top = at(column, row) + (at(column + 1, row) - at(column, row)) * fx
        let bottom = at(column, row + 1) + (at(column + 1, row + 1) - at(column, row + 1)) * fx
        return top + (bottom - top) * fy
    }
}

/// The calibration's own grid: display pixels at `across` × `down`, and
/// where the lens shows each of them.
private struct Grid {
    let across: [Float]
    let down: [Float]
    let shown: [SIMD2<Float>]

    /// Where the lens shows `display`, interpolated between the grid's
    /// points, and carried on past its edges.
    func shown(atDisplay display: SIMD2<Float>) -> SIMD2<Float> {
        let (column, fx) = Self.locate(display.x, in: across)
        let (row, fy) = Self.locate(display.y, in: down)
        func at(_ c: Int, _ r: Int) -> SIMD2<Float> { shown[r * across.count + c] }
        let top = at(column, row) + (at(column + 1, row) - at(column, row)) * fx
        let bottom = at(column, row + 1) + (at(column + 1, row + 1) - at(column, row + 1)) * fx
        return top + (bottom - top) * fy
    }

    /// The segment of `positions` holding `value`, and how far along it
    /// `value` is; the first or last segment, extended, outside them.
    private static func locate(_ value: Float, in positions: [Float]) -> (Int, Float) {
        var index = 0
        while index < positions.count - 2, value > positions[index + 1] {
            index += 1
        }
        return (index, (value - positions[index]) / (positions[index + 1] - positions[index]))
    }
}
