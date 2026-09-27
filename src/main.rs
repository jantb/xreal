mod ar_drivers {
    pub mod lib;
}
mod capture;
mod render;
mod settings;
mod tracking;
mod viewport;

use std::sync::Arc;
use std::time::{Duration, Instant};
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::monitor::MonitorHandle;
use winit::window::{Fullscreen, Window, WindowId};

use capture::{spawn_capture, LatestFrame};
use pixels::{wgpu, Pixels, PixelsBuilder, SurfaceTexture};
use render::{draw_overlay, output_height, process_viewport_frame, set_black};
use scap::frame::BGRAFrame;
use settings::Settings;
use tracking::{
    CalibrationState, DriftLearner, DriftObservation, HeadPose, ImuStatus, Tracking,
    TrackingSnapshot,
};
use viewport::{ViewportController, ViewportRect};

const TARGET_FPS: u32 = 120;
// Time from sampling the pose to photons: roughly half a frame until the
// next vsync plus the glasses' own display delay (~7 ms).
const PREDICTION_LEAD: Duration = Duration::from_millis(11);
const SENSITIVITY_STEP: f32 = 1.05;

struct RenderStats {
    frames: u32,
    captured_since: u64,
    last_sample: Instant,
    fps: f32,
    capture_fps: f32,
}

impl RenderStats {
    fn new() -> Self {
        Self {
            frames: 0,
            captured_since: 0,
            last_sample: Instant::now(),
            fps: 0.0,
            capture_fps: 0.0,
        }
    }

    fn tick(&mut self, now: Instant, capture_generation: u64) {
        self.frames += 1;
        let elapsed = now.saturating_duration_since(self.last_sample);
        if elapsed >= Duration::from_millis(500) {
            let seconds = elapsed.as_secs_f32();
            self.fps = self.frames as f32 / seconds;
            self.capture_fps =
                capture_generation.saturating_sub(self.captured_since) as f32 / seconds;
            self.frames = 0;
            self.captured_since = capture_generation;
            self.last_sample = now;
        }
    }
}

/// Keeps a fixed frame cadence: render time does not push the next frame
/// later. If a whole period was missed, restart the cadence from `now`.
fn next_frame_deadline(previous: Instant, period: Duration, now: Instant) -> Instant {
    let next = previous + period;
    if next <= now {
        now + period
    } else {
        next
    }
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

    let settings = Settings::load();
    let tracking = Tracking::spawn(settings.gyro_bias);
    let frames = spawn_capture(TARGET_FPS);
    let event_loop = EventLoop::new().unwrap();
    let mut app = App::new(frames, tracking, settings);
    event_loop.run_app(&mut app)
}

struct App {
    close_requested: bool,
    pixels: Option<Pixels>,
    window: Option<Window>,
    output_width: usize,
    frames: Arc<LatestFrame>,
    frame: Option<Arc<BGRAFrame>>,
    frame_generation: u64,
    tracking: Tracking,
    tracking_session: u64,
    bias_revision_saved: u32,
    drift: DriftLearner,
    last_drift: Option<DriftObservation>,
    settings: Settings,
    viewport: ViewportController,
    render_stats: RenderStats,
    source_offsets: Vec<usize>,
    last_pose: HeadPose,
    last_render_at: Instant,
    next_frame_at: Instant,
    frame_period: Duration,
    refresh_checked_at: Instant,
}

impl App {
    fn new(frames: Arc<LatestFrame>, tracking: Tracking, settings: Settings) -> Self {
        Self {
            close_requested: false,
            pixels: None,
            window: None,
            output_width: 0,
            frames,
            frame: None,
            frame_generation: 0,
            tracking,
            tracking_session: 0,
            bias_revision_saved: 0,
            drift: DriftLearner::new(),
            last_drift: None,
            viewport: ViewportController::new(&settings),
            settings,
            render_stats: RenderStats::new(),
            source_offsets: Vec::new(),
            last_pose: HeadPose::default(),
            last_render_at: Instant::now(),
            next_frame_at: Instant::now(),
            frame_period: Duration::from_nanos(1_000_000_000 / TARGET_FPS as u64),
            refresh_checked_at: Instant::now(),
        }
    }

    /// Matches the frame cadence to the display. Re-checked periodically
    /// because the glasses switch to 120 Hz only after they connect.
    fn update_frame_period(&mut self, monitor: Option<MonitorHandle>) {
        if let Some(millihertz) = monitor
            .and_then(|monitor| monitor.refresh_rate_millihertz())
            .filter(|&millihertz| millihertz > 0)
        {
            self.frame_period = Duration::from_nanos(1_000_000_000_000 / millihertz as u64);
        }
    }

    fn save_settings(&mut self) {
        self.viewport.store_into(&mut self.settings);
        self.settings.gyro_bias = self.tracking.snapshot().gyro_bias;
        if let Err(err) = self.settings.save() {
            eprintln!("Failed to save settings: {err}");
        }
    }

    fn handle_key(&mut self, key: Key) {
        match key.as_ref() {
            Key::Named(NamedKey::ArrowRight) => self.viewport.pan(96.0, 0.0),
            Key::Named(NamedKey::ArrowUp) => self.viewport.pan(0.0, -72.0),
            Key::Named(NamedKey::ArrowDown) => self.viewport.pan(0.0, 72.0),
            Key::Named(NamedKey::ArrowLeft) => self.viewport.pan(-96.0, 0.0),
            Key::Named(NamedKey::Tab) => {
                self.settings.overlay_visible = !self.settings.overlay_visible;
                self.save_settings();
            }
            Key::Named(NamedKey::Space) => self.tracking.calibrate(),
            Key::Named(NamedKey::Escape) => self.close_requested = true,
            Key::Character(ch) if ch.eq_ignore_ascii_case("c") => {
                let observation = self.drift.observe_recenter(Instant::now(), self.last_pose.yaw);
                if let DriftObservation::Learned { correction, .. } = observation {
                    self.tracking.correct_yaw_drift(correction);
                }
                self.last_drift = Some(observation);
                self.viewport.recenter(self.last_pose);
            }
            Key::Character(ch) if ch.eq_ignore_ascii_case("r") => {
                self.drift.reset();
                self.viewport.reset(self.last_pose);
                self.save_settings();
            }
            Key::Character(ch) if ch.eq_ignore_ascii_case("f") => self.viewport.toggle_freeze(),
            Key::Character(ch) if ch.eq_ignore_ascii_case("d") => {
                self.viewport.cycle_deadzone();
                self.save_settings();
            }
            Key::Character(ch) if ch.eq_ignore_ascii_case("p") => {
                self.settings.prediction = !self.settings.prediction;
                self.save_settings();
            }
            Key::Character("=") | Key::Character("+") => {
                self.viewport.zoom_in();
                self.save_settings();
            }
            Key::Character("-") | Key::Character("_") => {
                self.viewport.zoom_out();
                self.save_settings();
            }
            Key::Character(".") => {
                self.viewport.adjust_sensitivity(SENSITIVITY_STEP);
                self.save_settings();
            }
            Key::Character(",") => {
                self.viewport.adjust_sensitivity(1.0 / SENSITIVITY_STEP);
                self.save_settings();
            }
            _ => (),
        }
    }

    fn redraw(&mut self) {
        let now = Instant::now();
        let dt = now
            .saturating_duration_since(self.last_render_at)
            .as_secs_f32()
            .min(0.1);
        self.last_render_at = now;

        let generation = self.frames.generation();
        let new_frame = generation != self.frame_generation;
        if new_frame {
            self.frame = self.frames.latest();
            self.frame_generation = generation;
        }
        self.render_stats.tick(now, generation);
        if now.saturating_duration_since(self.refresh_checked_at) >= Duration::from_secs(1) {
            self.refresh_checked_at = now;
            let monitor = self.window.as_ref().and_then(|window| window.current_monitor());
            self.update_frame_period(monitor);
        }

        // Sample the pose as late as possible, just before building the frame.
        let tracking = self.tracking.snapshot();
        if tracking.bias_revision != self.bias_revision_saved {
            self.bias_revision_saved = tracking.bias_revision;
            self.save_settings();
        }
        let pose = if self.settings.prediction {
            tracking.predict(Instant::now(), PREDICTION_LEAD)
        } else {
            tracking.pose
        };
        self.last_pose = pose;
        if tracking.session != self.tracking_session {
            self.tracking_session = tracking.session;
            self.drift.reset();
            self.viewport.recenter(pose);
        }

        let Some(pixels) = &mut self.pixels else {
            return;
        };
        let output_width = self.output_width;
        let buffer = pixels.frame_mut();
        let output_height = output_height(buffer, output_width);

        let rect = match self.frame.as_deref() {
            Some(source) => {
                let rect = self.viewport.update(
                    pose,
                    dt,
                    output_width,
                    output_height,
                    source.width as usize,
                    source.height as usize,
                );
                process_viewport_frame(
                    &source.data,
                    buffer,
                    output_width,
                    source.width as usize,
                    source.height as usize,
                    rect,
                    &mut self.source_offsets,
                );
                Some(rect)
            }
            None => {
                set_black(buffer);
                None
            }
        };

        if self.settings.overlay_visible {
            let lines = hud_lines(&HudInfo {
                viewport: &self.viewport,
                rect,
                source: self.frame.as_deref().map(|f| (f.width, f.height)),
                output: (output_width, output_height),
                new_frame,
                stats: &self.render_stats,
                tracking: &tracking,
                pose,
                prediction: self.settings.prediction,
                last_drift: self.last_drift,
            });
            draw_overlay(buffer, output_width, &lines);
        }

        if let Err(err) = pixels.render() {
            eprintln!("Render failed: {err}");
        }
        self.next_frame_at = next_frame_deadline(self.next_frame_at, self.frame_period, Instant::now());
    }
}

impl ApplicationHandler for App {
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
            .or(primary_monitor);
        self.update_frame_period(desired_monitor.clone());
        window.set_fullscreen(Some(Fullscreen::Borderless(desired_monitor)));
        let size = window.inner_size();
        self.output_width = size.width as usize;

        let surface_texture = SurfaceTexture::new(size.width, size.height, &window);
        let pixels = PixelsBuilder::new(size.width, size.height, surface_texture)
            .texture_format(wgpu::TextureFormat::Bgra8UnormSrgb)
            .build()
            .unwrap();

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
            } => self.handle_key(key),
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
            WindowEvent::RedrawRequested => self.redraw(),
            _ => (),
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        if self.close_requested {
            self.save_settings();
            event_loop.exit();
            return;
        }
        if Instant::now() >= self.next_frame_at {
            self.window.as_ref().unwrap().request_redraw();
        }
        event_loop.set_control_flow(ControlFlow::WaitUntil(self.next_frame_at));
    }
}

struct HudInfo<'a> {
    viewport: &'a ViewportController,
    rect: Option<ViewportRect>,
    source: Option<(i32, i32)>,
    output: (usize, usize),
    new_frame: bool,
    stats: &'a RenderStats,
    tracking: &'a TrackingSnapshot,
    pose: HeadPose,
    prediction: bool,
    last_drift: Option<DriftObservation>,
}

fn hud_lines(info: &HudInfo) -> Vec<String> {
    let state = if info.viewport.frozen { "FROZEN" } else { "LIVE" };
    let frame_state = if info.new_frame { "NEW" } else { "HOLD" };
    let tracking = info.tracking;

    let imu = match tracking.status {
        ImuStatus::Connected => {
            let age_ms = tracking
                .sampled_at
                .map_or(0.0, |at| at.elapsed().as_secs_f32() * 1000.0);
            format!("IMU OK {:.0}HZ  AGE {:.0}MS", tracking.sample_rate_hz, age_ms)
        }
        ImuStatus::Searching => "IMU NO GLASSES - RETRYING".to_string(),
    };
    let calibration = match tracking.calibration {
        CalibrationState::Idle => String::new(),
        CalibrationState::Running { progress } => {
            format!("  CALIBRATING {:.0}% - KEEP STILL", progress * 100.0)
        }
        CalibrationState::Succeeded => "  CALIBRATED".to_string(),
        CalibrationState::Failed => "  CALIBRATION FAILED - MOVED".to_string(),
    };
    let bias = tracking.gyro_bias;
    let to_degrees_per_minute = |rate: f32| rate.to_degrees() * 60.0;
    let drift = match info.last_drift {
        None => "C RECENTER TEACHES DRIFT".to_string(),
        Some(DriftObservation::Anchored) => "DRIFT REFERENCE SET".to_string(),
        Some(DriftObservation::TooSoon) => "DRIFT: WAIT 20S BETWEEN RECENTERS".to_string(),
        Some(DriftObservation::Learned { measured, .. }) => format!(
            "DRIFT {:+.2} DEG/MIN - CORRECTED",
            to_degrees_per_minute(measured)
        ),
        Some(DriftObservation::Rejected { measured }) => format!(
            "DRIFT {:+.1} DEG/MIN - IGNORED AS TURN",
            to_degrees_per_minute(measured)
        ),
    };

    let source = match info.source {
        Some((width, height)) => format!("SRC {width}X{height}"),
        None => "SRC NO CAPTURE YET".to_string(),
    };
    let view = match info.rect {
        Some(rect) => format!(
            "VIEW {:.0},{:.0} {:.0}X{:.0}",
            rect.x, rect.y, rect.width, rect.height
        ),
        None => "VIEW -".to_string(),
    };

    vec![
        format!(
            "{} {}  {:.0}FPS  CAPTURE {:.0}FPS  ZOOM {:.2}X",
            state,
            frame_state,
            info.stats.fps,
            info.stats.capture_fps,
            info.viewport.zoom()
        ),
        imu,
        format!(
            "BIAS {:.4} {:.4} {:.4}  {}{}",
            bias[0],
            bias[1],
            bias[2],
            if tracking.still { "STILL" } else { "MOVING" },
            calibration
        ),
        drift,
        format!("{}  OUT {}X{}", source, info.output.0, info.output.1),
        view,
        format!(
            "YAW {:.3}  PITCH {:.3}  SENS {:.2}X  DEADZONE {:.3}  PREDICT {}",
            info.pose.yaw,
            info.pose.pitch,
            info.viewport.sensitivity,
            info.viewport.deadzone(),
            if info.prediction { "ON" } else { "OFF" }
        ),
        "C CENTER  R RESET  F FREEZE  -/+ ZOOM  TAB HUD  ARROWS PAN".to_string(),
        "SPACE CALIBRATE  ,/. SENSITIVITY  D DEADZONE  P PREDICT".to_string(),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_cadence_does_not_absorb_render_time() {
        let start = Instant::now();
        let period = Duration::from_micros(8_333);
        let render_finished = start + Duration::from_millis(3);

        assert_eq!(next_frame_deadline(start, period, render_finished), start + period);
    }

    #[test]
    fn frame_cadence_restarts_after_a_stall() {
        let start = Instant::now();
        let period = Duration::from_micros(8_333);
        let after_stall = start + Duration::from_millis(50);

        assert_eq!(next_frame_deadline(start, period, after_stall), after_stall + period);
    }
}
