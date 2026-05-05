mod ar_drivers {
    pub mod lib;
}

use ar_drivers::lib::{any_glasses, DisplayMode, GlassesEvent};
use std::sync::{Arc, Mutex};

use crossbeam_channel::{bounded, Receiver, Sender};
use dcmimu::DCMIMU;
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant};
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::{Fullscreen, Window, WindowId};

use pixels::{Pixels, SurfaceTexture};
use scap::capturer::{Capturer, Options};
use scap::frame::{BGRAFrame, Frame};
use thread_priority::{set_current_thread_priority, ThreadPriority};

// For optional bias reset via UI
use once_cell::sync::OnceCell;

struct SharedGlassesStore {
    dcmimu: Arc<Mutex<DCMIMU>>,
}

const TARGET_FPS: u32 = 120;

struct RollingVec3Average<const N: usize> {
    values: [(f32, f32, f32); N],
    next: usize,
    len: usize,
    sum: (f32, f32, f32),
}

impl<const N: usize> RollingVec3Average<N> {
    fn new() -> Self {
        Self {
            values: [(0.0, 0.0, 0.0); N],
            next: 0,
            len: 0,
            sum: (0.0, 0.0, 0.0),
        }
    }

    fn push_average(&mut self, value: (f32, f32, f32)) -> (f32, f32, f32) {
        if self.len == N {
            let old = self.values[self.next];
            self.sum.0 -= old.0;
            self.sum.1 -= old.1;
            self.sum.2 -= old.2;
        } else {
            self.len += 1;
        }

        self.values[self.next] = value;
        self.sum.0 += value.0;
        self.sum.1 += value.1;
        self.sum.2 += value.2;
        self.next += 1;
        if self.next == N {
            self.next = 0;
        }

        let n = self.len as f32;
        (self.sum.0 / n, self.sum.1 / n, self.sum.2 / n)
    }
}

// Global channel to request gyro bias reset from UI
static BIAS_RESET_SENDER: OnceCell<Sender<()>> = OnceCell::new();

fn create_glasses_thread(store: &SharedGlassesStore) {
    let shared_dcmimu_clone = Arc::clone(&store.dcmimu);
    let last_timestamp = Arc::new(AtomicU64::new(0));
    let (sender, receiver): (Sender<_>, Receiver<_>) = bounded(1);

    // Bias reset channel
    let (bias_reset_tx, bias_reset_rx): (Sender<()>, Receiver<()>) = bounded(1);
    // Expose to UI
    let _ = BIAS_RESET_SENDER.set(bias_reset_tx);

    thread::spawn({
        let last_timestamp = Arc::clone(&last_timestamp);
        let sender = sender.clone();
        move || {
            set_current_thread_priority(ThreadPriority::Max)
                .expect("Failed to set thread priority");
            let mut glasses = match any_glasses() {
                Ok(glasses) => glasses,
                Err(_) => return, // Exit if unable to acquire glasses
            };
            let _ = glasses.set_display_mode(DisplayMode::HighRefreshRate);

            loop {
                if let Ok(GlassesEvent::AccGyro {
                    accelerometer,
                    gyroscope,
                    timestamp,
                }) = glasses.read_event()
                {
                    let last_ts = last_timestamp.load(Ordering::Relaxed);
                    if last_ts != 0 {
                        let dt = (timestamp - last_ts) as f32 / 1_000_000.0; // seconds
                        let _ = sender.try_send((
                            (gyroscope.x, gyroscope.y, gyroscope.z),
                            (accelerometer.x, accelerometer.y, accelerometer.z),
                            dt,
                        ));
                    }
                    last_timestamp.store(timestamp, Ordering::Relaxed);
                }
            }
        }
    });

    thread::spawn({
        const BUFFER_SIZE: usize = 32; // smoothing window
        let mut gyro_buffer = RollingVec3Average::<BUFFER_SIZE>::new();
        let mut acc_buffer = RollingVec3Average::<BUFFER_SIZE>::new();

        // Gyro bias estimator state
        let mut gyro_bias = (0.0f32, 0.0f32, 0.0f32);
        let max_bias = 0.2f32; // rad/s clamp
        let alpha = 0.003f32; // learning rate when stationary
        let decay = 0.0005f32; // gentle decay when moving

        // Stationary detection thresholds
        let gyro_thresh = 0.04f32; // rad/s
        let acc_g = 9.81f32;
        let acc_thresh = 0.35f32; // m/s^2 window around |g|

        move || {
            set_current_thread_priority(ThreadPriority::Max)
                .expect("Failed to set thread priority");
            loop {
                // Optional reset request
                while bias_reset_rx.try_recv().is_ok() {
                    gyro_bias = (0.0, 0.0, 0.0);
                }

                if let Ok((gyro, acc, dt)) = receiver.recv() {
                    let average_gyro = gyro_buffer.push_average(gyro);
                    let average_acc = acc_buffer.push_average(acc);

                    // Stationary detection
                    let gyro_mag = (average_gyro.0 * average_gyro.0
                        + average_gyro.1 * average_gyro.1
                        + average_gyro.2 * average_gyro.2)
                        .sqrt();
                    let acc_mag = (average_acc.0 * average_acc.0
                        + average_acc.1 * average_acc.1
                        + average_acc.2 * average_acc.2)
                        .sqrt();
                    let near_gravity = (acc_mag - acc_g).abs() < acc_thresh;
                    let stationary = gyro_mag < gyro_thresh && near_gravity;

                    // Bias update
                    if stationary {
                        gyro_bias.0 = (1.0 - alpha) * gyro_bias.0 + alpha * average_gyro.0;
                        gyro_bias.1 = (1.0 - alpha) * gyro_bias.1 + alpha * average_gyro.1;
                        gyro_bias.2 = (1.0 - alpha) * gyro_bias.2 + alpha * average_gyro.2;

                        // Clamp
                        gyro_bias.0 = gyro_bias.0.clamp(-max_bias, max_bias);
                        gyro_bias.1 = gyro_bias.1.clamp(-max_bias, max_bias);
                        gyro_bias.2 = gyro_bias.2.clamp(-max_bias, max_bias);
                    } else {
                        // Gentle decay toward zero
                        gyro_bias.0 *= 1.0 - decay;
                        gyro_bias.1 *= 1.0 - decay;
                        gyro_bias.2 *= 1.0 - decay;
                    }

                    // Apply correction before DCM update
                    let gyro_corrected = (
                        average_gyro.0 - gyro_bias.0,
                        average_gyro.1 - gyro_bias.1,
                        average_gyro.2 - gyro_bias.2,
                    );

                    let mut dcmimu = shared_dcmimu_clone.lock().unwrap();
                    dcmimu.update(gyro_corrected, average_acc, dt);
                }
            }
        }
    });
}

fn main() -> Result<(), impl std::error::Error> {
    if !scap::is_supported() {
        println!("❌ Platform not supported");
        panic!()
    }

    if !scap::has_permission() {
        println!("❌ Permission not granted. Requesting permission...");
        if !scap::request_permission() {
            panic!("❌ Permission denied");
        }
    }

    let store = SharedGlassesStore {
        dcmimu: Arc::new(Mutex::new(DCMIMU::new())),
    };
    create_glasses_thread(&store);

    let mut recorder = Capturer::new(Options {
        fps: TARGET_FPS,
        show_cursor: true,
        show_highlight: true,
        excluded_targets: None,
        output_type: scap::frame::FrameType::BGRAFrame,
        ..Default::default()
    });
    recorder.start_capture();
    let event_loop = EventLoop::new().unwrap();
    let mut app = ControlFlowDemo::new(recorder, store);
    event_loop.run_app(&mut app)
}

struct ControlFlowDemo {
    close_requested: bool,
    pixels: Option<Pixels>,
    window: Option<Window>,
    recorder: Capturer,
    screen_width: usize,
    store: SharedGlassesStore,
    o_x: f32,
    o_y: f32,
    pred: Option<BGRAFrame>,
    next_frame_at: Instant,
    target_frame_time: Duration,
}

impl ControlFlowDemo {
    fn new(recorder: Capturer, store: SharedGlassesStore) -> Self {
        Self {
            close_requested: false,
            pixels: None,
            window: None,
            screen_width: 0,
            recorder,
            store,
            o_x: -0.5,
            o_y: -0.9,
            pred: None,
            next_frame_at: Instant::now(),
            target_frame_time: Duration::from_nanos(1_000_000_000 / TARGET_FPS as u64),
        }
    }
}

impl ApplicationHandler for ControlFlowDemo {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let window_attributes = Window::default_attributes()
            .with_title("Xreal renderer")
            .with_inner_size(winit::dpi::LogicalSize::new(1920.0, 1080.0));
        let window = event_loop.create_window(window_attributes).unwrap();
        let primary_monitor = window.primary_monitor();
        let desired_monitor = event_loop
            .available_monitors()
            .find(|monitor| {
                monitor
                    .name()
                    .map_or(false, |name| name == "Monitor #12596")
            })
            .or_else(|| Some(primary_monitor.unwrap()));
        window.set_fullscreen(Some(Fullscreen::Borderless(desired_monitor)));
        self.screen_width = window.primary_monitor().unwrap().size().width as usize;
        let size = window.inner_size();

        let surface_texture = SurfaceTexture::new(size.width, size.height, &window);
        let pixels = Pixels::new(size.width, size.height, surface_texture).unwrap();

        self.pixels = Some(pixels);
        self.window = Some(window);
    }

    fn window_event(
        &mut self,
        _event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => {
                self.close_requested = true;
            }
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        logical_key: key,
                        state: ElementState::Pressed,
                        ..
                    },
                ..
            } => match key.as_ref() {
                Key::Named(NamedKey::ArrowRight) => {
                    self.o_x -= 0.008;
                }
                Key::Named(NamedKey::ArrowUp) => {
                    self.o_y += 0.008;
                }
                Key::Named(NamedKey::ArrowDown) => {
                    self.o_y -= 0.008;
                }
                Key::Named(NamedKey::ArrowLeft) => {
                    self.o_x += 0.008;
                }
                // Optional: R to reset learned gyro bias
                Key::Named(NamedKey::Space) => {
                    if let Some(tx) = BIAS_RESET_SENDER.get() {
                        let _ = tx.try_send(());
                    }
                }
                Key::Named(NamedKey::Escape) => {
                    self.close_requested = true;
                }
                _ => (),
            },
            WindowEvent::Resized(size) => {
                if let Some(pixels) = &mut self.pixels {
                    pixels
                        .resize_surface(size.width, size.height)
                        .expect("Resize failed");
                }
            }
            WindowEvent::RedrawRequested => {
                fn calculate_offset(
                    angle: f32,
                    offset: f32,
                    dimension: f32,
                    multiplier: f32,
                ) -> isize {
                    (dimension * ((-angle + offset) + 1.0) * multiplier) as isize
                }
                if let Some(pixels) = &mut self.pixels {
                    if let Ok(Frame::BGRA(data)) = self.recorder.get_next_frame() {
                        let (yaw, roll) = {
                            let dcm = self.store.dcmimu.lock().unwrap().all();
                            (dcm.yaw, dcm.roll)
                        };
                        let current_x_offset = calculate_offset(yaw, self.o_x, 1920.0, 1.5);
                        let current_y_offset = calculate_offset(-roll, self.o_y, 1080.0, 3.0);

                        let frame = pixels.frame_mut();
                        if data.height == 0 || data.width == 0 {
                            if let Some(pred) = &self.pred {
                                process_frame_serial(
                                    &pred.data,
                                    frame,
                                    1920,
                                    self.screen_width,
                                    current_x_offset,
                                    current_y_offset,
                                );
                            }
                        } else {
                            process_frame_serial(
                                &data.data,
                                frame,
                                1920,
                                self.screen_width,
                                current_x_offset,
                                current_y_offset,
                            );
                            self.pred = Some(data);
                        }
                    }
                    pixels.render().unwrap();
                    self.next_frame_at = Instant::now() + self.target_frame_time;
                }
            }
            _ => (),
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        let now = Instant::now();
        if !self.close_requested && now >= self.next_frame_at {
            self.window.as_ref().unwrap().request_redraw();
        }

        let control_flow = if now >= self.next_frame_at {
            ControlFlow::Poll
        } else {
            ControlFlow::WaitUntil(self.next_frame_at)
        };
        event_loop.set_control_flow(control_flow);

        if self.close_requested {
            event_loop.exit();
        }
    }
}

fn process_frame_serial(
    frame: &[u8],
    raw_buffer: &mut [u8],
    width: usize,
    screen_width: usize,
    x_offset: isize,
    y_offset: isize,
) {
    let frame_stride = screen_width * 4;
    let output_stride = width * 4;
    if frame_stride == 0 || output_stride == 0 {
        return;
    }

    let source_height = frame.len() / frame_stride;
    let visible_x_start = 0.max(-x_offset).min(width as isize) as usize;
    let visible_x_end = (screen_width as isize - x_offset)
        .max(0)
        .min(width as isize) as usize;

    if visible_x_start >= visible_x_end || source_height == 0 {
        set_black(raw_buffer);
        return;
    }

    let prefix_len = visible_x_start * 4;
    let visible_len = (visible_x_end - visible_x_start) * 4;
    let visible_offset = prefix_len;
    let suffix_offset = visible_offset + visible_len;

    for (y, row) in raw_buffer.chunks_mut(output_stride).enumerate() {
        let row_y = y as isize + y_offset;

        if row_y < 0 || row_y as usize >= source_height {
            set_black(row);
            continue;
        }

        set_black(&mut row[..prefix_len]);
        set_black(&mut row[suffix_offset..]);

        let source_x = (visible_x_start as isize + x_offset) as usize;
        let source_index = row_y as usize * frame_stride + source_x * 4;
        let source_row = &frame[source_index..source_index + visible_len];
        let visible_row = &mut row[visible_offset..suffix_offset];

        for (source, dest) in source_row
            .chunks_exact(4)
            .zip(visible_row.chunks_exact_mut(4))
        {
            dest[0] = source[2]; // R
            dest[1] = source[1]; // G
            dest[2] = source[0]; // B
            dest[3] = 255; // A
        }
    }
}

fn set_black(buffer: &mut [u8]) {
    for pixel in buffer.chunks_exact_mut(4) {
        pixel[0] = 0;
        pixel[1] = 0;
        pixel[2] = 0;
        pixel[3] = 255;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::hint::black_box;

    fn make_bgra_frame(width: usize, height: usize) -> Vec<u8> {
        let mut frame = vec![0; width * height * 4];
        for y in 0..height {
            for x in 0..width {
                let index = (y * width + x) * 4;
                frame[index] = x as u8;
                frame[index + 1] = y as u8;
                frame[index + 2] = (x + y * 17) as u8;
                frame[index + 3] = 77;
            }
        }
        frame
    }

    fn reference_process_frame(
        frame: &[u8],
        raw_buffer: &mut [u8],
        width: usize,
        screen_width: usize,
        x_offset: isize,
        y_offset: isize,
    ) {
        let frame_stride = screen_width * 4;
        let source_height = frame.len() / frame_stride;

        for (y, row) in raw_buffer.chunks_mut(width * 4).enumerate() {
            let source_y = y as isize + y_offset;
            for (x, pixel) in row.chunks_mut(4).enumerate() {
                let source_x = x as isize + x_offset;
                if source_y < 0
                    || source_y as usize >= source_height
                    || source_x < 0
                    || source_x as usize >= screen_width
                {
                    pixel.copy_from_slice(&[0, 0, 0, 255]);
                    continue;
                }

                let source_index = source_y as usize * frame_stride + source_x as usize * 4;
                pixel[0] = frame[source_index + 2];
                pixel[1] = frame[source_index + 1];
                pixel[2] = frame[source_index];
                pixel[3] = 255;
            }
        }
    }

    #[test]
    fn rolling_average_evicts_old_samples() {
        let mut average = RollingVec3Average::<3>::new();

        assert_eq!(average.push_average((1.0, 2.0, 3.0)), (1.0, 2.0, 3.0));
        assert_eq!(average.push_average((3.0, 4.0, 5.0)), (2.0, 3.0, 4.0));
        assert_eq!(average.push_average((5.0, 6.0, 7.0)), (3.0, 4.0, 5.0));
        assert_eq!(average.push_average((7.0, 8.0, 9.0)), (5.0, 6.0, 7.0));
    }

    #[test]
    fn optimized_frame_processing_matches_reference_across_offsets() {
        let output_width = 4;
        let output_height = 3;
        let screen_width = 6;
        let source_height = 5;
        let frame = make_bgra_frame(screen_width, source_height);

        for y_offset in -4..=6 {
            for x_offset in -5..=7 {
                let mut expected = vec![19; output_width * output_height * 4];
                let mut actual = expected.clone();

                reference_process_frame(
                    &frame,
                    &mut expected,
                    output_width,
                    screen_width,
                    x_offset,
                    y_offset,
                );
                process_frame_serial(
                    &frame,
                    &mut actual,
                    output_width,
                    screen_width,
                    x_offset,
                    y_offset,
                );

                assert_eq!(actual, expected, "x_offset={x_offset}, y_offset={y_offset}");
            }
        }
    }

    #[test]
    #[ignore = "release-mode throughput harness; run with `cargo test --release perf_frame_processing -- --ignored --nocapture`"]
    fn perf_frame_processing_reports_throughput() {
        let output_width = 1920;
        let output_height = 1080;
        let screen_width = 3840;
        let source_height = 2160;
        let iterations = 240;
        let frame = make_bgra_frame(screen_width, source_height);
        let mut output = vec![0; output_width * output_height * 4];

        let start = Instant::now();
        for i in 0..iterations {
            process_frame_serial(
                black_box(&frame),
                black_box(&mut output),
                output_width,
                screen_width,
                640 + (i as isize % 17),
                120 + (i as isize % 11),
            );
        }
        let elapsed = start.elapsed();
        let fps = iterations as f64 / elapsed.as_secs_f64();
        let checksum = output
            .iter()
            .step_by(4096)
            .fold(0u64, |sum, byte| sum.wrapping_add(*byte as u64));

        println!(
            "process_frame_serial: {iterations} frames in {:.3}s = {:.1} fps ({:.3} ms/frame), checksum={checksum}",
            elapsed.as_secs_f64(),
            fps,
            1000.0 / fps
        );

        if let Ok(min_fps) = std::env::var("PERF_MIN_FPS") {
            let min_fps: f64 = min_fps.parse().expect("PERF_MIN_FPS must be numeric");
            assert!(
                fps >= min_fps,
                "throughput {fps:.1} fps is below PERF_MIN_FPS={min_fps}"
            );
        }

        assert_ne!(checksum, 0);
    }

    #[test]
    #[ignore = "release-mode throughput harness; run with `cargo test --release perf_rolling_average -- --ignored --nocapture`"]
    fn perf_rolling_average_reports_throughput() {
        let iterations = 10_000_000;
        let mut average = RollingVec3Average::<32>::new();
        let start = Instant::now();
        let mut checksum = 0.0f32;

        for i in 0..iterations {
            let input = (
                (i & 0xff) as f32,
                ((i >> 8) & 0xff) as f32,
                ((i >> 16) & 0xff) as f32,
            );
            let output = average.push_average(black_box(input));
            checksum += output.0 + output.1 + output.2;
        }

        let elapsed = start.elapsed();
        let samples_per_second = iterations as f64 / elapsed.as_secs_f64();
        println!(
            "RollingVec3Average: {iterations} samples in {:.3}s = {:.1} M samples/s, checksum={checksum:.1}",
            elapsed.as_secs_f64(),
            samples_per_second / 1_000_000.0
        );

        if let Ok(min_samples_per_second) = std::env::var("PERF_MIN_ROLLING_SAMPLES_PER_SEC") {
            let min_samples_per_second: f64 = min_samples_per_second
                .parse()
                .expect("PERF_MIN_ROLLING_SAMPLES_PER_SEC must be numeric");
            assert!(
                samples_per_second >= min_samples_per_second,
                "throughput {samples_per_second:.1} samples/s is below PERF_MIN_ROLLING_SAMPLES_PER_SEC={min_samples_per_second}"
            );
        }

        assert_ne!(checksum, 0.0);
    }
}
