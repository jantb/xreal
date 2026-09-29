import Foundation
import QuartzCore
import XrealCore

/// Runs a CAMetalDisplayLink on a dedicated high-priority thread and calls
/// `onFrame` there once per refresh, with the drawable to fill, the time it
/// will reach the screen and the time it must be committed by. Frames keep coming while the main thread is
/// busy with menus, window moves or other AppKit work.
final class DisplayLinkThread: NSObject, CAMetalDisplayLinkDelegate, @unchecked Sendable {
    private let onFrame: (CAMetalDrawable, _ presentingAt: Double, _ deadline: Double) -> Void
    private var thread: Thread!
    // Touched only on `thread`.
    private var link: CAMetalDisplayLink?

    init(onFrame: @escaping (CAMetalDrawable, _ presentingAt: Double, _ deadline: Double) -> Void) {
        self.onFrame = onFrame
        super.init()
        thread = Thread {
            // A frame is due every refresh; the CPU side of one takes a
            // couple of milliseconds. Real time keeps busy apps from making
            // it late.
            let period = 1 / glassesRefreshRate
            if !promoteCurrentThreadToRealTime(period: period, computation: 0.003, constraint: min(0.008, period)) {
                eprint("Could not give the render thread real-time priority")
            }
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

    private final class Attachment: NSObject {
        let layer: CAMetalLayer
        let fps: Float

        init(layer: CAMetalLayer, fps: Float) {
            self.layer = layer
            self.fps = fps
        }
    }

    /// Starts driving `layer`, replacing any earlier link, with frames at
    /// `fps`, the refresh rate of the screen the layer is on now. Asking for
    /// more than the screen shows makes frames come unevenly.
    func attach(to layer: CAMetalLayer, fps: Int) {
        let attachment = Attachment(layer: layer, fps: Float(max(fps, 30)))
        perform(#selector(attachOnThread(_:)), on: thread, with: attachment, waitUntilDone: false)
    }

    @objc private func attachOnThread(_ attachment: Attachment) {
        link?.invalidate()
        let link = CAMetalDisplayLink(metalLayer: attachment.layer)
        let fps = attachment.fps
        link.preferredFrameRateRange = CAFrameRateRange(minimum: fps, maximum: fps, preferred: fps)
        // One frame in flight: the pose sampled for a frame is at most one
        // refresh old when it is shown.
        link.preferredFrameLatency =
            ProcessInfo.processInfo.environment["XREAL_FRAME_LATENCY"].flatMap(Float.init) ?? 1
        link.delegate = self
        link.add(to: .current, forMode: .default)
        self.link = link
    }

    func metalDisplayLink(_ link: CAMetalDisplayLink, needsUpdate update: CAMetalDisplayLink.Update) {
        // Secondary threads get no autorelease pool per run loop pass, and
        // every frame autoreleases its drawable.
        autoreleasepool {
            onFrame(update.drawable, update.targetPresentationTimestamp, update.targetTimestamp)
        }
    }
}
