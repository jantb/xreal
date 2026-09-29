import Testing

@testable import XrealCore

private let period = 1.0 / 90

/// `count` frames, one per refresh from `start`, each shown `late` seconds
/// after it was promised.
private func frames(_ count: Int, late: Double, start: Double = 100, into timing: inout FrameTiming) {
    for index in 0..<count {
        let promised = start + Double(index) * period
        timing.presented(promised: promised, at: promised + late, sampledAt: promised - 0.012, period: period)
    }
}

@Test func nothingIsKnownAboutTheDisplayUntilFramesHaveReachedIt() {
    var timing = FrameTiming()
    #expect(timing.extraDelay == nil)
    frames(2, late: 0.011, into: &timing)
    #expect(timing.extraDelay == nil)
}

@Test func framesSteadilyShownLaterThanPromisedTeachHowMuchLater() throws {
    var timing = FrameTiming()
    frames(60, late: 0.011, into: &timing)
    let delay = try #require(timing.extraDelay)
    #expect(abs(delay - 0.011) < 0.001)
    // One odd frame does not move it.
    timing.presented(promised: 200, at: 200.04, sampledAt: 199.99, period: period)
    #expect(abs((timing.extraDelay ?? 0) - 0.011) < 0.001)
}

@Test func framesShownLateOrNotAtAllAreCounted() {
    var timing = FrameTiming()
    frames(90, late: 0, into: &timing)
    #expect(timing.lateFramesPerSecond(now: 101) == 0)
    timing.presented(promised: 101, at: 101 + period, sampledAt: 100.99, period: period)
    timing.presented(promised: 101.02, at: nil, sampledAt: 101.01, period: period)
    #expect(timing.lateFramesPerSecond(now: 101.05) > 0)
    // And forgotten a while later.
    #expect(timing.lateFramesPerSecond(now: 110) == 0)
}

@Test func thePoseIsTakenEarlyEnoughForNearlyEveryFramesWork() {
    var timing = FrameTiming()
    for index in 0..<100 {
        timing.worked(index % 10 == 0 ? 0.004 : 0.002, period: period)
    }
    #expect(timing.workBudget > 0.004)
    #expect(timing.workBudget < period)
    // However slow the frames, sampling never waits past most of a refresh.
    for _ in 0..<100 {
        timing.worked(0.05, period: period)
    }
    #expect(timing.workBudget < period)
}
