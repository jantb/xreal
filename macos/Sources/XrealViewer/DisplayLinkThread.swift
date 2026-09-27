import Foundation
import QuartzCore

/// Runs a CAMetalDisplayLink on a dedicated high-priority thread and calls
/// `onFrame` there once per refresh, with the drawable to fill and the time
/// it will reach the screen. Frames keep coming while the main thread is
/// busy with menus, window moves or other AppKit work.
final class DisplayLinkThread: NSObject, CAMetalDisplayLinkDelegate, @unchecked Sendable {
    private let onFrame: (CAMetalDrawable, _ presentingAt: Double) -> Void
    private var thread: Thread!
    // Touched only on `thread`.
    private var link: CAMetalDisplayLink?

    init(onFrame: @escaping (CAMetalDrawable, _ presentingAt: Double) -> Void) {
        self.onFrame = onFrame
        super.init()
        thread = Thread {
            // A run loop without sources returns at once; the port keeps it
            // waiting for the display link and `attach` calls.
            RunLoop.current.add(NSMachPort(), forMode: .default)
            while true {
                RunLoop.current.run()
            }
        }
        thread.name = "Glasses render"
        thread.qualityOfService = .userInteractive
        thread.start()
    }

    /// Starts driving `layer`, replacing any earlier link, so frames come at
    /// the refresh rate of the screen the layer is on now.
    func attach(to layer: CAMetalLayer) {
        perform(#selector(attachOnThread(_:)), on: thread, with: layer, waitUntilDone: false)
    }

    @objc private func attachOnThread(_ layer: CAMetalLayer) {
        link?.invalidate()
        let link = CAMetalDisplayLink(metalLayer: layer)
        link.preferredFrameRateRange = CAFrameRateRange(minimum: 60, maximum: 120, preferred: 120)
        // One frame in flight: the pose sampled for a frame is at most one
        // refresh old when it is shown.
        link.preferredFrameLatency = 1
        link.delegate = self
        link.add(to: .current, forMode: .default)
        self.link = link
    }

    func metalDisplayLink(_ link: CAMetalDisplayLink, needsUpdate update: CAMetalDisplayLink.Update) {
        // Secondary threads get no autorelease pool per run loop pass, and
        // every frame autoreleases its drawable.
        autoreleasepool {
            onFrame(update.drawable, update.targetPresentationTimestamp)
        }
    }
}
