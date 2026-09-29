import CoreGraphics
import Testing
import XrealCore
import simd

@testable import XrealViewer

private typealias Size = (width: Int, height: Int)

/// The default canvas is captured in three tiles.
private let tileSizes: [Size?] = Array(repeating: (width: 1918, height: 2160), count: 3)
private let sideBySide: Size = (width: 3840, height: 1080)

private func headAt(yaw: Float = 0, roll: Float = 0, session: UInt64 = 0, biasRevision: UInt32 = 0)
    -> TrackingSnapshot
{
    var snapshot = TrackingSnapshot(gyroBias: .zero)
    snapshot.pose = HeadPose(yaw: yaw, roll: roll)
    snapshot.session = session
    snapshot.biasRevision = biasRevision
    return snapshot
}

extension ViewerState {
    /// One frame with the head as `snapshot` says.
    @discardableResult
    fileprivate mutating func frame(
        _ head: TrackingSnapshot = headAt(), sizes: [Size?] = tileSizes, output: Size = sideBySide,
        now: Double = monotonicNow(), cursor: CGPoint? = nil, timing: FrameTiming = FrameTiming(),
        extras: ExtraSizes = ExtraSizes()
    ) -> (room: RoomView?, biasChanged: Bool) {
        advance(
            now: now, dt: 1 / 90, presentingAt: now + 1 / 90, snapshot: head, captureGeneration: 0,
            newFrame: false, frameSizes: sizes, output: output, cursor: cursor, timing: timing, extras: extras)
    }
}

private func freshState() -> ViewerState {
    ViewerState(settings: Settings())
}

@Test func theGlassesShowNothingUntilTheyAreSideBySide() {
    var state = freshState()
    let outcome1 = state.frame(output: (width: 1920, height: 1080))
    #expect(outcome1.room == nil)
    let outcome2 = state.frame()
    #expect(outcome2.room != nil)
}

@Test func aCanvasStillStartingUpHasNothingToShow() {
    var state = freshState()
    let starting = state.frame(sizes: [nil, nil, nil]).room
    #expect(starting?.panels.isEmpty == true)
    #expect(state.frame().room?.panels.isEmpty == false)
}

@Test func lookingStraightAheadIsLookingAtTheMiddleOfTheCanvas() throws {
    var state = freshState()
    state.frame()
    let gaze = try #require(state.gaze)
    let canvas = state.settings.canvas
    #expect(abs(gaze.x - Float(canvas.width) / 2) < 100)
    #expect(abs(gaze.y - Float(canvas.height) / 2) < 100)
}

@Test func aNewTrackingSessionStartsWithTheCanvasAheadWhereverTheHeadPoints() throws {
    var state = freshState()
    state.frame(headAt(yaw: 0, session: 0))
    state.frame(headAt(yaw: 1.2, session: 1))
    let gaze = try #require(state.gaze)
    #expect(abs(gaze.x - Float(state.settings.canvas.width) / 2) < 100)
}

@Test func lookingAwayFromTheCanvasFindsNothingAndKeepsItsCapturesAtTheSlowRate() {
    var state = freshState()
    state.frame(headAt(yaw: 0))
    #expect(state.capturesInView.contains(true))
    state.frame(headAt(yaw: 3))
    #expect(state.gaze == nil)
    #expect(state.capturesInView == [false, false, false])
}

@Test func theBiasIsReportedAsChangedOncePerRevision() {
    var state = freshState()
    #expect(state.frame(headAt(biasRevision: 0)).biasChanged == false)
    #expect(state.frame(headAt(biasRevision: 1)).biasChanged == true)
    #expect(state.frame(headAt(biasRevision: 1)).biasChanged == false)
}

@Test func theCanvasCannotBePickedUpWithoutLookingAtIt() {
    var state = freshState()
    let outcome3 = state.startGrab()
    #expect(!outcome3)
    state.frame(headAt(yaw: 0))
    state.frame(headAt(yaw: 3))
    let outcome4 = state.startGrab()
    #expect(!outcome4)
    state.frame(headAt(yaw: 0))
    let outcome5 = state.startGrab()
    #expect(outcome5)
}

@Test func aCarriedCanvasFollowsTheHeadAndStaysWhereItIsLetGo() {
    var state = freshState()
    state.frame()
    let outcome6 = state.startGrab()
    #expect(outcome6)
    state.frame(headAt(yaw: 0.5))
    #expect(abs(state.settings.canvas.placement.direction.x) > 0.3)
    let outcome7 = state.endGrab()
    #expect(outcome7)
    let letGoAt = state.settings.canvas.placement
    state.frame(headAt(yaw: 0))
    #expect(state.settings.canvas.placement == letGoAt)
    let outcome8 = state.endGrab()
    #expect(!outcome8)
}

@Test func aCanvasLetGoNearlyLevelIsSetLevelButATiltedOneKeepsItsTilt() {
    func tiltAfterCarrying(rolling roll: Float) -> Float {
        var state = freshState()
        state.frame()
        _ = state.startGrab()
        state.frame(headAt(roll: roll))
        _ = state.endGrab()
        return abs(state.settings.canvas.placement.tilt)
    }
    #expect(tiltAfterCarrying(rolling: 0.02) == 0)
    #expect(tiltAfterCarrying(rolling: 0.3) > 0.1)
}

@Test func theCanvasComesCloserAndGoesAwayAsAskedEvenWhileCarried() {
    var state = freshState()
    let start = state.settings.canvas.placement.distance
    state.moveCanvas(closer: true, now: 0)
    #expect(state.settings.canvas.placement.distance < start)
    state.moveCanvas(closer: false, now: 0)
    state.moveCanvas(closer: false, now: 0)
    #expect(state.settings.canvas.placement.distance > start)

    state.frame()
    _ = state.startGrab()
    let carried = state.settings.canvas.placement.distance
    state.moveCanvas(closer: false, now: 0)
    state.frame()
    _ = state.endGrab()
    #expect(state.settings.canvas.placement.distance > carried)
}

@Test func steppingBiggerVisitsEveryCanvasSizeThenStops() {
    var state = freshState()
    _ = state.setCanvasSize(width: canvasSizes[0].width, height: canvasSizes[0].height, now: 0)
    var visited = [canvasSizes[0]]
    while state.resizeCanvas(bigger: true, now: 0) {
        let canvas = state.settings.canvas
        visited.append((canvas.width, canvas.height, canvas.scale))
    }
    #expect(visited.map(\.width) == canvasSizes.map(\.width))
    #expect(visited.map(\.height) == canvasSizes.map(\.height))
    #expect(visited.map(\.scale) == canvasSizes.map(\.scale))
    let outcome9 = state.resizeCanvas(bigger: true, now: 0)
    #expect(!outcome9)
}

@Test func steppingSmallerFromTheSmallestSizeChangesNothing() {
    var state = freshState()
    _ = state.setCanvasSize(width: canvasSizes[0].width, height: canvasSizes[0].height, now: 0)
    let outcome10 = state.resizeCanvas(bigger: false, now: 0)
    #expect(!outcome10)
    #expect(state.settings.canvas.width == canvasSizes[0].width)
}

@Test func aNewCanvasSizeLetsGoOfTheCanvasButTheSameSizeChangesNothing() {
    var state = freshState()
    state.frame()
    let outcome11 = state.startGrab()
    #expect(outcome11)
    let canvas = state.settings.canvas
    let outcome12 = state.setCanvasSize(width: canvas.width, height: canvas.height, now: 0)
    #expect(!outcome12)
    #expect(state.grab != nil)
    let outcome13 = state.setCanvasSize(width: 2880, height: 1620, now: 0)
    #expect(outcome13)
    #expect(state.grab == nil)
    #expect(state.gaze == nil)
}

/// How far left the view looks after a quick turn left, predicted with
/// frames lately reaching the display `late` seconds after they were promised.
private func turnSeen(framesLate late: Double) throws -> Float {
    var state = freshState()
    let now = monotonicNow()
    var still = headAt()
    still.status = .connected
    still.sampledAt = now
    state.frame(still, now: now)
    var turning = still
    turning.yawRate = 2
    var timing = FrameTiming()
    for index in 0..<30 {
        let promised = now - 1 + Double(index) / 90
        timing.presented(promised: promised, at: promised + late, sampledAt: promised - 0.01, period: 1 / 90)
    }
    let room = try #require(state.frame(turning, now: now, timing: timing).room)
    return -(room.headRotation * SIMD3(0, 0, -1)).x
}

@Test func thePoseIsPredictedForWhenFramesReallyReachTheGlasses() throws {
    let onTime = try turnSeen(framesLate: 0)
    let late = try turnSeen(framesLate: 0.02)
    #expect(onTime > 0)
    // 20 ms more at 2 rad/s is 0.04 rad further round.
    #expect(abs(asin(late) - asin(onTime) - 0.04) < 0.005)
}

@Test func theStatusStripAndPinnedWindowHangAboveTheCanvas() throws {
    var state = freshState()
    let extras = ExtraSizes(status: SIMD2(1200, 36), pinned: SIMD2(800, 600))
    let room = try #require(state.frame(extras: extras).room)
    let status = try #require(room.panels.first { $0.source == .status })
    let pinned = try #require(room.panels.first { $0.source == .pinned })
    #expect(status.rect.bottom > 1 && pinned.rect.bottom > status.rect.top)

    state.settings.statusStrip = false
    let without = try #require(state.frame(extras: extras).room)
    #expect(!without.panels.contains { $0.source == .status })
    #expect(without.panels.contains { $0.source == .pinned })
}

@Test func thePointerIsDrawnLiveOnlyWhileTheMouseIsOnTheCanvas() throws {
    var state = freshState()
    let canvas = state.settings.canvas
    state.canvasBounds = CGRect(x: 0, y: 0, width: canvas.width, height: canvas.height)
    let extras = ExtraSizes(pointer: (SIMD2(32, 32), SIMD2(4, 4)))
    func drawsPointer(at cursor: CGPoint?) throws -> Bool {
        try #require(state.frame(cursor: cursor, extras: extras).room).panels.contains { $0.source == .pointer }
    }
    #expect(try drawsPointer(at: CGPoint(x: 2800, y: 1000)))
    #expect(try !drawsPointer(at: CGPoint(x: -300, y: 1000)))
    #expect(try !drawsPointer(at: nil))
    state.settings.livePointer = false
    #expect(try !drawsPointer(at: CGPoint(x: 2800, y: 1000)))
}
