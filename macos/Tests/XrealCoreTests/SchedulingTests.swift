import Foundation
import Testing

@testable import XrealCore

@Test func aThreadPromotedToRealTimeRunsAsRealTime() async {
    let outcome = await withCheckedContinuation { continuation in
        let thread = Thread {
            let before = currentThreadIsRealTime()
            let accepted = promoteCurrentThreadToRealTime(period: 1.0 / 90, computation: 0.003, constraint: 0.008)
            continuation.resume(returning: (before, accepted, currentThreadIsRealTime()))
        }
        thread.start()
    }
    #expect(outcome.0 == false)
    #expect(outcome.1)
    #expect(outcome.2)
}
