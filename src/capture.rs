use scap::capturer::{Capturer, Options};
use scap::frame::{BGRAFrame, Frame, FrameType};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use thread_priority::{set_current_thread_priority, ThreadPriority};

/// Holds only the newest captured frame. The render loop never waits on
/// capture: it reuses the last frame until a newer one arrives.
pub struct LatestFrame {
    slot: Mutex<Option<Arc<BGRAFrame>>>,
    generation: AtomicU64,
}

impl LatestFrame {
    fn new() -> Self {
        Self {
            slot: Mutex::new(None),
            generation: AtomicU64::new(0),
        }
    }

    /// Increments every time a new frame is published.
    pub fn generation(&self) -> u64 {
        self.generation.load(Ordering::Acquire)
    }

    pub fn latest(&self) -> Option<Arc<BGRAFrame>> {
        self.slot.lock().unwrap().clone()
    }

    fn publish(&self, frame: BGRAFrame) {
        *self.slot.lock().unwrap() = Some(Arc::new(frame));
        self.generation.fetch_add(1, Ordering::Release);
    }
}

/// Starts screen capture on its own thread. scap's frame queue is unbounded,
/// so draining it continuously here keeps displayed content from going stale.
pub fn spawn_capture(fps: u32) -> Arc<LatestFrame> {
    let latest = Arc::new(LatestFrame::new());
    thread::spawn({
        let latest = Arc::clone(&latest);
        move || {
            if let Err(err) = set_current_thread_priority(ThreadPriority::Max) {
                eprintln!("Failed to set capture thread priority: {err}");
            }
            let mut capturer = match Capturer::build(Options {
                fps,
                show_cursor: true,
                show_highlight: true,
                excluded_targets: None,
                output_type: FrameType::BGRAFrame,
                ..Default::default()
            }) {
                Ok(capturer) => capturer,
                Err(err) => {
                    eprintln!("Failed to start screen capture: {err:?}");
                    return;
                }
            };
            capturer.start_capture();

            while let Ok(frame) = capturer.get_next_frame() {
                // Idle (unchanged screen) frames arrive empty; keep the last one.
                if let Frame::BGRA(frame) = frame {
                    if frame.width > 0 && frame.height > 0 {
                        latest.publish(frame);
                    }
                }
            }
            eprintln!("Screen capture stopped");
        }
    });
    latest
}
