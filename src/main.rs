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
const ZOOM_LEVELS: [f32; 7] = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0];

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

#[derive(Clone, Copy, Debug, Default)]
struct ViewportRect {
    x: f32,
    y: f32,
    width: f32,
    height: f32,
}

#[derive(Clone, Copy, Debug, Default)]
struct ViewportMetrics {
    rect: ViewportRect,
    source_width: usize,
    source_height: usize,
    output_width: usize,
    output_height: usize,
    yaw: f32,
    roll: f32,
    zoom: f32,
    frozen: bool,
    live_frame: bool,
    render_fps: f32,
}

struct ViewportController {
    center_yaw: f32,
    center_roll: f32,
    manual_x: f32,
    manual_y: f32,
    smoothed_x: f32,
    smoothed_y: f32,
    initialized: bool,
    zoom_index: usize,
    deadzone: f32,
    smoothing: f32,
    yaw_sensitivity: f32,
    roll_sensitivity: f32,
    frozen: bool,
    overlay_visible: bool,
    last_metrics: ViewportMetrics,
}

impl ViewportController {
    fn new() -> Self {
        Self {
            center_yaw: 0.0,
            center_roll: 0.0,
            manual_x: 0.0,
            manual_y: 0.0,
            smoothed_x: 0.0,
            smoothed_y: 0.0,
            initialized: false,
            zoom_index: 2,
            deadzone: 0.018,
            smoothing: 0.22,
            yaw_sensitivity: 2100.0,
            roll_sensitivity: 900.0,
            frozen: false,
            overlay_visible: true,
            last_metrics: ViewportMetrics::default(),
        }
    }

    fn recenter(&mut self, yaw: f32, roll: f32) {
        self.center_yaw = yaw;
        self.center_roll = roll;
        self.manual_x = 0.0;
        self.manual_y = 0.0;
        self.smoothed_x = 0.0;
        self.smoothed_y = 0.0;
        self.initialized = true;
    }

    fn reset(&mut self, yaw: f32, roll: f32) {
        let overlay_visible = self.overlay_visible;
        *self = Self::new();
        self.overlay_visible = overlay_visible;
        self.recenter(yaw, roll);
    }

    fn pan(&mut self, dx: f32, dy: f32) {
        self.manual_x += dx;
        self.manual_y += dy;
    }

    fn zoom_in(&mut self) {
        self.zoom_index = (self.zoom_index + 1).min(ZOOM_LEVELS.len() - 1);
    }

    fn zoom_out(&mut self) {
        self.zoom_index = self.zoom_index.saturating_sub(1);
    }

    fn toggle_freeze(&mut self) {
        self.frozen = !self.frozen;
    }

    fn toggle_overlay(&mut self) {
        self.overlay_visible = !self.overlay_visible;
    }

    fn update(
        &mut self,
        yaw: f32,
        roll: f32,
        output_width: usize,
        output_height: usize,
        source_width: usize,
        source_height: usize,
        live_frame: bool,
        render_fps: f32,
    ) -> ViewportMetrics {
        if !self.initialized {
            self.recenter(yaw, roll);
        }

        let zoom = ZOOM_LEVELS[self.zoom_index];
        let view_width = ((output_width as f32) / zoom).clamp(1.0, source_width.max(1) as f32);
        let view_height = ((output_height as f32) / zoom).clamp(1.0, source_height.max(1) as f32);

        if !self.frozen {
            let yaw_delta = apply_deadzone(yaw - self.center_yaw, self.deadzone);
            let roll_delta = apply_deadzone(roll - self.center_roll, self.deadzone);
            let target_x = self.manual_x - yaw_delta * self.yaw_sensitivity;
            let target_y = self.manual_y + roll_delta * self.roll_sensitivity;

            if self.initialized {
                self.smoothed_x += (target_x - self.smoothed_x) * self.smoothing;
                self.smoothed_y += (target_y - self.smoothed_y) * self.smoothing;
            } else {
                self.smoothed_x = target_x;
                self.smoothed_y = target_y;
                self.initialized = true;
            }
        }

        let source_center_x = source_width as f32 * 0.5;
        let source_center_y = source_height as f32 * 0.5;
        let max_x = (source_width as f32 - view_width).max(0.0);
        let max_y = (source_height as f32 - view_height).max(0.0);
        let x = (source_center_x + self.smoothed_x - view_width * 0.5).clamp(0.0, max_x);
        let y = (source_center_y + self.smoothed_y - view_height * 0.5).clamp(0.0, max_y);

        self.last_metrics = ViewportMetrics {
            rect: ViewportRect {
                x,
                y,
                width: view_width,
                height: view_height,
            },
            source_width,
            source_height,
            output_width,
            output_height,
            yaw,
            roll,
            zoom,
            frozen: self.frozen,
            live_frame,
            render_fps,
        };
        self.last_metrics
    }
}

fn apply_deadzone(value: f32, deadzone: f32) -> f32 {
    let magnitude = value.abs();
    if magnitude <= deadzone {
        0.0
    } else {
        value.signum() * (magnitude - deadzone)
    }
}

struct RenderStats {
    frames: u32,
    last_sample: Instant,
    fps: f32,
}

impl RenderStats {
    fn new() -> Self {
        Self {
            frames: 0,
            last_sample: Instant::now(),
            fps: 0.0,
        }
    }

    fn tick(&mut self) -> f32 {
        self.frames += 1;
        let elapsed = self.last_sample.elapsed();
        if elapsed >= Duration::from_millis(500) {
            self.fps = self.frames as f32 / elapsed.as_secs_f32();
            self.frames = 0;
            self.last_sample = Instant::now();
        }
        self.fps
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
            if let Err(err) = set_current_thread_priority(ThreadPriority::Max) {
                eprintln!("Failed to set glasses reader thread priority: {err}");
            }
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
            if let Err(err) = set_current_thread_priority(ThreadPriority::Max) {
                eprintln!("Failed to set IMU update thread priority: {err}");
            }
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
    output_width: usize,
    store: SharedGlassesStore,
    viewport: ViewportController,
    render_stats: RenderStats,
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
            output_width: 0,
            recorder,
            store,
            viewport: ViewportController::new(),
            render_stats: RenderStats::new(),
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
        let size = window.inner_size();
        self.output_width = size.width as usize;

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
                    self.viewport.pan(96.0, 0.0);
                }
                Key::Named(NamedKey::ArrowUp) => {
                    self.viewport.pan(0.0, -72.0);
                }
                Key::Named(NamedKey::ArrowDown) => {
                    self.viewport.pan(0.0, 72.0);
                }
                Key::Named(NamedKey::ArrowLeft) => {
                    self.viewport.pan(-96.0, 0.0);
                }
                Key::Named(NamedKey::Tab) => {
                    self.viewport.toggle_overlay();
                }
                Key::Named(NamedKey::Space) => {
                    if let Some(tx) = BIAS_RESET_SENDER.get() {
                        let _ = tx.try_send(());
                    }
                }
                Key::Named(NamedKey::Escape) => {
                    self.close_requested = true;
                }
                Key::Character(ch) if ch.eq_ignore_ascii_case("c") => {
                    let (yaw, roll) = self.current_pose();
                    self.viewport.recenter(yaw, roll);
                }
                Key::Character(ch) if ch.eq_ignore_ascii_case("r") => {
                    let (yaw, roll) = self.current_pose();
                    self.viewport.reset(yaw, roll);
                }
                Key::Character(ch) if ch.eq_ignore_ascii_case("f") => {
                    self.viewport.toggle_freeze();
                }
                Key::Character("=") | Key::Character("+") => {
                    self.viewport.zoom_in();
                }
                Key::Character("-") | Key::Character("_") => {
                    self.viewport.zoom_out();
                }
                _ => (),
            },
            WindowEvent::Resized(size) => {
                if let Some(pixels) = &mut self.pixels {
                    pixels
                        .resize_surface(size.width, size.height)
                        .expect("Resize failed");
                    pixels
                        .resize_buffer(size.width, size.height)
                        .expect("Buffer resize failed");
                    self.output_width = size.width as usize;
                }
            }
            WindowEvent::RedrawRequested => {
                let render_fps = self.render_stats.tick();
                if let Some(pixels) = &mut self.pixels {
                    if let Ok(Frame::BGRA(data)) = self.recorder.get_next_frame() {
                        let (yaw, roll) = {
                            let dcm = self.store.dcmimu.lock().unwrap().all();
                            (dcm.yaw, dcm.roll)
                        };

                        let frame = pixels.frame_mut();
                        let live_frame = data.height != 0 && data.width != 0;
                        if live_frame {
                            self.pred = Some(data);
                        }

                        if let Some(source) = self.pred.as_ref() {
                            let output_height = output_height(frame, self.output_width);
                            let metrics = self.viewport.update(
                                yaw,
                                roll,
                                self.output_width,
                                output_height,
                                source.width as usize,
                                source.height as usize,
                                live_frame,
                                render_fps,
                            );
                            process_viewport_frame(
                                &source.data,
                                frame,
                                self.output_width,
                                source.width as usize,
                                source.height as usize,
                                metrics.rect,
                            );
                            if self.viewport.overlay_visible {
                                draw_overlay(frame, self.output_width, metrics);
                            }
                        } else {
                            set_black(frame);
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

impl ControlFlowDemo {
    fn current_pose(&self) -> (f32, f32) {
        let dcm = self.store.dcmimu.lock().unwrap().all();
        (dcm.yaw, dcm.roll)
    }
}

#[cfg(test)]
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

fn output_height(raw_buffer: &[u8], output_width: usize) -> usize {
    if output_width == 0 {
        0
    } else {
        raw_buffer.len() / (output_width * 4)
    }
}

fn process_viewport_frame(
    frame: &[u8],
    raw_buffer: &mut [u8],
    output_width: usize,
    source_width: usize,
    source_height: usize,
    viewport: ViewportRect,
) {
    let output_stride = output_width * 4;
    let source_stride = source_width * 4;
    if output_stride == 0 || source_stride == 0 || source_height == 0 {
        set_black(raw_buffer);
        return;
    }

    let available_source_height = (frame.len() / source_stride).min(source_height);
    let output_height = output_height(raw_buffer, output_width);
    if output_height == 0 || available_source_height == 0 {
        set_black(raw_buffer);
        return;
    }

    for (y, row) in raw_buffer.chunks_mut(output_stride).enumerate() {
        let source_y = viewport.y + ((y as f32 + 0.5) * viewport.height / output_height as f32);
        let source_y = source_y
            .floor()
            .clamp(0.0, available_source_height.saturating_sub(1) as f32)
            as usize;
        let source_row_start = source_y * source_stride;

        for (x, dest) in row.chunks_exact_mut(4).enumerate() {
            let source_x = viewport.x + ((x as f32 + 0.5) * viewport.width / output_width as f32);
            let source_x = source_x
                .floor()
                .clamp(0.0, source_width.saturating_sub(1) as f32)
                as usize;
            let source_index = source_row_start + source_x * 4;
            dest[0] = frame[source_index + 2];
            dest[1] = frame[source_index + 1];
            dest[2] = frame[source_index];
            dest[3] = 255;
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

fn draw_overlay(frame: &mut [u8], output_width: usize, metrics: ViewportMetrics) {
    let output_height = output_height(frame, output_width);
    if output_width == 0 || output_height == 0 {
        return;
    }

    let state = if metrics.frozen { "FROZEN" } else { "LIVE" };
    let frame_state = if metrics.live_frame { "CAP" } else { "PRED" };
    let lines = [
        format!(
            "{} {}  {:.0}FPS  ZOOM {:.2}X",
            state, frame_state, metrics.render_fps, metrics.zoom
        ),
        format!(
            "SRC {}X{}  OUT {}X{}",
            metrics.source_width,
            metrics.source_height,
            metrics.output_width,
            metrics.output_height
        ),
        format!(
            "VIEW {:.0},{:.0} {:.0}X{:.0}",
            metrics.rect.x, metrics.rect.y, metrics.rect.width, metrics.rect.height
        ),
        format!("YAW {:.3}  ROLL {:.3}", metrics.yaw, metrics.roll),
        "C CENTER  R RESET  F FREEZE  -/+ ZOOM  TAB HUD".to_string(),
    ];

    let char_w = 6usize;
    let char_h = 8usize;
    let overlay_width = lines
        .iter()
        .map(|line| line.len() * char_w)
        .max()
        .unwrap_or(0)
        + 16;
    let overlay_height = lines.len() * char_h + 14;
    fill_rect(
        frame,
        output_width,
        8,
        8,
        overlay_width.min(output_width.saturating_sub(8)),
        overlay_height.min(output_height.saturating_sub(8)),
        [0, 0, 0, 255],
    );

    for (index, line) in lines.iter().enumerate() {
        draw_text(
            frame,
            output_width,
            16,
            16 + index * char_h,
            line,
            [230, 240, 255, 255],
        );
    }
}

fn fill_rect(
    frame: &mut [u8],
    output_width: usize,
    x: usize,
    y: usize,
    width: usize,
    height: usize,
    color: [u8; 4],
) {
    let output_height = output_height(frame, output_width);
    let max_y = (y + height).min(output_height);
    let max_x = (x + width).min(output_width);
    for row_y in y..max_y {
        let row_start = row_y * output_width * 4;
        for col_x in x..max_x {
            let index = row_start + col_x * 4;
            frame[index..index + 4].copy_from_slice(&color);
        }
    }
}

fn draw_text(
    frame: &mut [u8],
    output_width: usize,
    x: usize,
    y: usize,
    text: &str,
    color: [u8; 4],
) {
    let mut cursor_x = x;
    for ch in text.chars() {
        draw_char(frame, output_width, cursor_x, y, ch, color);
        cursor_x += 6;
    }
}

fn draw_char(frame: &mut [u8], output_width: usize, x: usize, y: usize, ch: char, color: [u8; 4]) {
    let glyph = glyph_5x7(ch);
    let output_height = output_height(frame, output_width);
    for (row, bits) in glyph.iter().enumerate() {
        let py = y + row;
        if py >= output_height {
            break;
        }
        for col in 0..5 {
            if bits & (1 << (4 - col)) == 0 {
                continue;
            }
            let px = x + col;
            if px >= output_width {
                continue;
            }
            let index = (py * output_width + px) * 4;
            frame[index..index + 4].copy_from_slice(&color);
        }
    }
}

fn glyph_5x7(ch: char) -> [u8; 7] {
    match ch.to_ascii_uppercase() {
        'A' => [0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11],
        'B' => [0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e],
        'C' => [0x0e, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0e],
        'D' => [0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e],
        'E' => [0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f],
        'F' => [0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10],
        'G' => [0x0e, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0f],
        'H' => [0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11],
        'I' => [0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1f],
        'J' => [0x01, 0x01, 0x01, 0x01, 0x11, 0x11, 0x0e],
        'K' => [0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11],
        'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f],
        'M' => [0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11],
        'N' => [0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11],
        'O' => [0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e],
        'P' => [0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10],
        'Q' => [0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d],
        'R' => [0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11],
        'S' => [0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e],
        'T' => [0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
        'U' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e],
        'V' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04],
        'W' => [0x11, 0x11, 0x11, 0x15, 0x15, 0x1b, 0x11],
        'X' => [0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11],
        'Y' => [0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04],
        'Z' => [0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f],
        '0' => [0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e],
        '1' => [0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e],
        '2' => [0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f],
        '3' => [0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e],
        '4' => [0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02],
        '5' => [0x1f, 0x10, 0x10, 0x1e, 0x01, 0x01, 0x1e],
        '6' => [0x0e, 0x10, 0x10, 0x1e, 0x11, 0x11, 0x0e],
        '7' => [0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
        '8' => [0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e],
        '9' => [0x0e, 0x11, 0x11, 0x0f, 0x01, 0x01, 0x0e],
        '.' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c],
        ',' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x08],
        ':' => [0x00, 0x0c, 0x0c, 0x00, 0x0c, 0x0c, 0x00],
        '-' => [0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00],
        '+' => [0x00, 0x04, 0x04, 0x1f, 0x04, 0x04, 0x00],
        '/' => [0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10],
        ' ' => [0x00; 7],
        _ => [0x1f, 0x11, 0x02, 0x04, 0x04, 0x00, 0x04],
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
    fn viewport_controller_recenters_and_clamps_to_source() {
        let mut viewport = ViewportController::new();
        viewport.recenter(1.0, 0.5);
        viewport.pan(10_000.0, -10_000.0);

        let metrics = viewport.update(1.0, 0.5, 1920, 1080, 3840, 1080, true, 120.0);
        assert_eq!(metrics.rect.x, 1920.0);
        assert_eq!(metrics.rect.y, 0.0);
        assert_eq!(metrics.rect.width, 1920.0);
        assert_eq!(metrics.rect.height, 1080.0);
    }

    #[test]
    fn viewport_controller_freeze_holds_position() {
        let mut viewport = ViewportController::new();
        let before = viewport.update(0.0, 0.0, 1920, 1080, 3840, 1080, true, 120.0);

        viewport.toggle_freeze();
        let after = viewport.update(1.0, 1.0, 1920, 1080, 3840, 1080, true, 120.0);

        assert_eq!(after.rect.x, before.rect.x);
        assert_eq!(after.rect.y, before.rect.y);
        assert!(after.frozen);
    }

    #[test]
    fn viewport_frame_processing_scales_and_clamps() {
        let source_width = 4;
        let source_height = 2;
        let frame = make_bgra_frame(source_width, source_height);
        let mut output = vec![0; 2 * 1 * 4];

        process_viewport_frame(
            &frame,
            &mut output,
            2,
            source_width,
            source_height,
            ViewportRect {
                x: 1.0,
                y: 0.0,
                width: 2.0,
                height: 1.0,
            },
        );

        assert_eq!(&output[0..4], &[1, 0, 1, 255]);
        assert_eq!(&output[4..8], &[2, 0, 2, 255]);
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
    #[ignore = "release-mode throughput harness; run with `cargo test --release perf_viewport_processing -- --ignored --nocapture`"]
    fn perf_viewport_processing_reports_throughput() {
        let output_width = 1920;
        let output_height = 1080;
        let source_width = 3840;
        let source_height = 2160;
        let iterations = 240;
        let frame = make_bgra_frame(source_width, source_height);
        let mut output = vec![0; output_width * output_height * 4];

        let start = Instant::now();
        for i in 0..iterations {
            process_viewport_frame(
                black_box(&frame),
                black_box(&mut output),
                output_width,
                source_width,
                source_height,
                ViewportRect {
                    x: 640.0 + (i as f32 % 17.0),
                    y: 120.0 + (i as f32 % 11.0),
                    width: 1536.0,
                    height: 864.0,
                },
            );
        }
        let elapsed = start.elapsed();
        let fps = iterations as f64 / elapsed.as_secs_f64();
        let checksum = output
            .iter()
            .step_by(4096)
            .fold(0u64, |sum, byte| sum.wrapping_add(*byte as u64));

        println!(
            "process_viewport_frame: {iterations} frames in {:.3}s = {:.1} fps ({:.3} ms/frame), checksum={checksum}",
            elapsed.as_secs_f64(),
            fps,
            1000.0 / fps
        );

        if let Ok(min_fps) = std::env::var("PERF_MIN_VIEWPORT_FPS") {
            let min_fps: f64 = min_fps
                .parse()
                .expect("PERF_MIN_VIEWPORT_FPS must be numeric");
            assert!(
                fps >= min_fps,
                "throughput {fps:.1} fps is below PERF_MIN_VIEWPORT_FPS={min_fps}"
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
