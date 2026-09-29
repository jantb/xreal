import Foundation

// Enough frames to judge by, about a second and a half at 90 Hz.
private let timingWindow = 128
// Until this many frames reached the display, nothing is known yet.
private let minTimedFrames = 16
// Lateness beyond this is a hitch, not how the display runs.
private let maxExtraDelay = 0.05  // seconds
// A frame is sampled this much earlier than its work usually takes, for the
// odd slower one.
private let workMargin = 0.002  // seconds
private let minWorkBudget = 0.002  // seconds
// Late frames are counted over this long.
private let lateWindow = 2.0  // seconds

/// Learns from frames that reached the display how much later than the
/// display link promised they arrive, and how long a frame takes to make.
/// With the first the head pose is predicted for when the frame is really
/// seen; with the second it can be sampled as late as is safe.
public struct FrameTiming: Sendable {
    private var lateness: [Double] = []
    private var work: [Double] = []
    private var lateAt: [Double] = []
    /// Median of `lateness`, kept up to date as frames arrive.
    public private(set) var extraDelay: Double?
    /// How long before its deadline a frame should start: a little more
    /// than nearly every frame took.
    public private(set) var workBudget = minWorkBudget
    /// The time from sampling the pose to the frame reaching the display,
    /// for the latest frame that did.
    public private(set) var lastLead: Double?

    public init() {}

    /// A frame promised for `promised` reached the display at `presented`,
    /// or never did (nil). `sampledAt` is when its pose was taken and
    /// `period` how long each refresh lasts, all in `monotonicNow` time.
    public mutating func presented(promised: Double, at presented: Double?, sampledAt: Double, period: Double) {
        guard let presented else {
            lateAt.append(promised)
            trimLate(now: promised)
            return
        }
        if presented > promised + period / 2 {
            lateAt.append(presented)
        }
        trimLate(now: presented)
        lastLead = presented - sampledAt
        Self.append(min(max(presented - promised, 0), maxExtraDelay), to: &lateness)
        extraDelay = lateness.count >= minTimedFrames ? Self.percentile(lateness, 0.5) : nil
    }

    /// A frame took `seconds` from sampling its pose until the GPU finished
    /// it.
    public mutating func worked(_ seconds: Double, period: Double) {
        Self.append(max(seconds, 0), to: &work)
        let typical = work.count >= minTimedFrames ? Self.percentile(work, 0.95) : 0
        workBudget = min(max(typical + workMargin, minWorkBudget), period * 0.7)
    }

    /// Frames that were late or never shown, per second, lately.
    public func lateFramesPerSecond(now: Double) -> Float {
        Float(Double(lateAt.filter { now - $0 <= lateWindow }.count) / lateWindow)
    }

    private mutating func trimLate(now: Double) {
        lateAt.removeAll { now - $0 > lateWindow }
    }

    private static func append(_ value: Double, to values: inout [Double]) {
        if values.count >= timingWindow {
            values.removeFirst(values.count - timingWindow + 1)
        }
        values.append(value)
    }

    private static func percentile(_ values: [Double], _ fraction: Double) -> Double {
        let sorted = values.sorted()
        return sorted[min(Int(Double(sorted.count - 1) * fraction + 0.5), sorted.count - 1)]
    }
}
