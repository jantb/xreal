use crate::settings::Settings;
use crate::tracking::{wrap_angle, HeadPose};
use std::f32::consts::PI;

pub const ZOOM_LEVELS: [f32; 7] = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0];
pub const DEADZONE_LEVELS: [f32; 5] = [0.0, 0.005, 0.01, 0.018, 0.03]; // rad
const MIN_SENSITIVITY: f32 = 0.25;
const MAX_SENSITIVITY: f32 = 3.0;

// Field of view of the Air-series displays: 46° diagonal at 16:9. At
// sensitivity 1.0 content moves exactly as far as the head turns, so the
// screen appears fixed in space.
const HORIZONTAL_FOV: f32 = 0.7087; // rad
const VERTICAL_FOV: f32 = 0.4104; // rad

// One Euro filter tuning (angles in rad): heavy smoothing while the head is
// nearly still, almost none during fast turns.
const FILTER_MIN_CUTOFF: f32 = 1.0; // Hz
const FILTER_BETA: f32 = 20.0;
const FILTER_DERIVATIVE_CUTOFF: f32 = 4.0; // Hz

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ViewportRect {
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
}

/// Adaptive low-pass filter (Casiez et al., "1€ Filter").
struct OneEuroFilter {
    min_cutoff: f32,
    beta: f32,
    derivative_cutoff: f32,
    value: Option<f32>,
    derivative: f32,
}

impl OneEuroFilter {
    fn new(min_cutoff: f32, beta: f32, derivative_cutoff: f32) -> Self {
        Self {
            min_cutoff,
            beta,
            derivative_cutoff,
            value: None,
            derivative: 0.0,
        }
    }

    fn reset(&mut self, value: f32) {
        self.value = Some(value);
        self.derivative = 0.0;
    }

    fn filter(&mut self, value: f32, dt: f32) -> f32 {
        let Some(previous) = self.value else {
            self.reset(value);
            return value;
        };
        let derivative = (value - previous) / dt;
        self.derivative += (derivative - self.derivative) * smoothing_alpha(self.derivative_cutoff, dt);
        let cutoff = self.min_cutoff + self.beta * self.derivative.abs();
        let filtered = previous + (value - previous) * smoothing_alpha(cutoff, dt);
        self.value = Some(filtered);
        filtered
    }
}

fn smoothing_alpha(cutoff: f32, dt: f32) -> f32 {
    let tau = 1.0 / (2.0 * PI * cutoff);
    1.0 / (1.0 + tau / dt)
}

pub struct ViewportController {
    center: HeadPose,
    manual_x: f32,
    manual_y: f32,
    filter_yaw: OneEuroFilter,
    filter_pitch: OneEuroFilter,
    offset_yaw: f32,
    offset_pitch: f32,
    initialized: bool,
    pub zoom_index: usize,
    pub sensitivity: f32,
    pub deadzone_index: usize,
    pub frozen: bool,
}

impl ViewportController {
    pub fn new(settings: &Settings) -> Self {
        Self {
            center: HeadPose::default(),
            manual_x: 0.0,
            manual_y: 0.0,
            filter_yaw: OneEuroFilter::new(FILTER_MIN_CUTOFF, FILTER_BETA, FILTER_DERIVATIVE_CUTOFF),
            filter_pitch: OneEuroFilter::new(FILTER_MIN_CUTOFF, FILTER_BETA, FILTER_DERIVATIVE_CUTOFF),
            offset_yaw: 0.0,
            offset_pitch: 0.0,
            initialized: false,
            zoom_index: settings.zoom_index.min(ZOOM_LEVELS.len() - 1),
            sensitivity: settings.sensitivity.clamp(MIN_SENSITIVITY, MAX_SENSITIVITY),
            deadzone_index: settings.deadzone_index.min(DEADZONE_LEVELS.len() - 1),
            frozen: false,
        }
    }

    pub fn store_into(&self, settings: &mut Settings) {
        settings.zoom_index = self.zoom_index;
        settings.sensitivity = self.sensitivity;
        settings.deadzone_index = self.deadzone_index;
    }

    pub fn zoom(&self) -> f32 {
        ZOOM_LEVELS[self.zoom_index]
    }

    pub fn deadzone(&self) -> f32 {
        DEADZONE_LEVELS[self.deadzone_index]
    }

    pub fn recenter(&mut self, pose: HeadPose) {
        self.center = pose;
        self.manual_x = 0.0;
        self.manual_y = 0.0;
        self.offset_yaw = 0.0;
        self.offset_pitch = 0.0;
        self.filter_yaw.reset(0.0);
        self.filter_pitch.reset(0.0);
        self.initialized = true;
    }

    /// Restores default zoom, sensitivity and deadzone, then recenters.
    pub fn reset(&mut self, pose: HeadPose) {
        *self = Self::new(&Settings::default());
        self.recenter(pose);
    }

    pub fn pan(&mut self, dx: f32, dy: f32) {
        self.manual_x += dx;
        self.manual_y += dy;
    }

    pub fn zoom_in(&mut self) {
        self.zoom_index = (self.zoom_index + 1).min(ZOOM_LEVELS.len() - 1);
    }

    pub fn zoom_out(&mut self) {
        self.zoom_index = self.zoom_index.saturating_sub(1);
    }

    pub fn adjust_sensitivity(&mut self, factor: f32) {
        self.sensitivity = (self.sensitivity * factor).clamp(MIN_SENSITIVITY, MAX_SENSITIVITY);
    }

    pub fn cycle_deadzone(&mut self) {
        self.deadzone_index = (self.deadzone_index + 1) % DEADZONE_LEVELS.len();
    }

    pub fn toggle_freeze(&mut self) {
        self.frozen = !self.frozen;
    }

    /// `dt` is the time since the previous update, in seconds.
    pub fn update(
        &mut self,
        pose: HeadPose,
        dt: f32,
        output_width: usize,
        output_height: usize,
        source_width: usize,
        source_height: usize,
    ) -> ViewportRect {
        if !self.initialized {
            self.recenter(pose);
        }

        let zoom = self.zoom();
        let view_width = ((output_width as f32) / zoom).clamp(1.0, source_width.max(1) as f32);
        let view_height = ((output_height as f32) / zoom).clamp(1.0, source_height.max(1) as f32);

        if !self.frozen {
            let deadzone = self.deadzone();
            let yaw_delta = apply_deadzone(wrap_angle(pose.yaw - self.center.yaw), deadzone);
            let pitch_delta = apply_deadzone(pose.pitch - self.center.pitch, deadzone);
            let dt = dt.max(1e-4);
            self.offset_yaw = self.filter_yaw.filter(yaw_delta, dt);
            self.offset_pitch = self.filter_pitch.filter(pitch_delta, dt);
        }

        // Source pixels per radian: one view width per field of view.
        let pixels_per_rad_x = view_width / HORIZONTAL_FOV * self.sensitivity;
        let pixels_per_rad_y = view_height / VERTICAL_FOV * self.sensitivity;
        let offset_x = self.manual_x - self.offset_yaw * pixels_per_rad_x;
        let offset_y = self.manual_y + self.offset_pitch * pixels_per_rad_y;

        let max_x = (source_width as f32 - view_width).max(0.0);
        let max_y = (source_height as f32 - view_height).max(0.0);
        let x = (source_width as f32 * 0.5 + offset_x - view_width * 0.5).clamp(0.0, max_x);
        let y = (source_height as f32 * 0.5 + offset_y - view_height * 0.5).clamp(0.0, max_y);

        ViewportRect {
            x,
            y,
            width: view_width,
            height: view_height,
        }
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

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 120.0;

    fn no_deadzone() -> ViewportController {
        ViewportController::new(&Settings {
            deadzone_index: 0,
            ..Settings::default()
        })
    }

    fn pose(yaw: f32, pitch: f32) -> HeadPose {
        HeadPose { yaw, pitch }
    }

    fn settle(viewport: &mut ViewportController, pose: HeadPose) -> ViewportRect {
        let mut rect = ViewportRect::default();
        for _ in 0..240 {
            rect = viewport.update(pose, DT, 1920, 1080, 7680, 4320);
        }
        rect
    }

    #[test]
    fn viewport_controller_recenters_and_clamps_to_source() {
        let mut viewport = ViewportController::new(&Settings::default());
        viewport.recenter(pose(1.0, 0.5));
        viewport.pan(10_000.0, -10_000.0);

        let rect = viewport.update(pose(1.0, 0.5), DT, 1920, 1080, 3840, 1080);
        assert_eq!(rect.x, 1920.0);
        assert_eq!(rect.y, 0.0);
        assert_eq!(rect.width, 1920.0);
        assert_eq!(rect.height, 1080.0);
    }

    #[test]
    fn viewport_controller_freeze_holds_position() {
        let mut viewport = ViewportController::new(&Settings::default());
        let before = viewport.update(pose(0.0, 0.0), DT, 1920, 1080, 3840, 1080);

        viewport.toggle_freeze();
        let after = viewport.update(pose(1.0, 1.0), DT, 1920, 1080, 3840, 1080);

        assert_eq!(after.x, before.x);
        assert_eq!(after.y, before.y);
    }

    #[test]
    fn head_turn_moves_content_by_the_same_angle_at_any_zoom() {
        let turn = 0.1;
        let expected_output_px = turn * 1920.0 / HORIZONTAL_FOV;

        for zoom_index in [2, 5] {
            let mut viewport = no_deadzone();
            viewport.zoom_index = zoom_index;
            let start = settle(&mut viewport, pose(0.0, 0.0));
            let turned = settle(&mut viewport, pose(turn, 0.0));

            let shift_output_px = (start.x - turned.x) * viewport.zoom();
            assert!(
                (shift_output_px - expected_output_px).abs() < 1.0,
                "zoom {}: shifted {shift_output_px} px, expected {expected_output_px}",
                viewport.zoom()
            );
        }
    }

    #[test]
    fn turning_across_the_yaw_seam_is_a_small_move() {
        let mut viewport = no_deadzone();
        viewport.recenter(pose(3.1, 0.0));
        let start = settle(&mut viewport, pose(3.1, 0.0));
        let crossed = settle(&mut viewport, pose(-3.1, 0.0));

        let expected = (2.0 * PI - 6.2) * 1920.0 / HORIZONTAL_FOV;
        assert!(((start.x - crossed.x) - expected).abs() < 1.0);
    }

    #[test]
    fn fast_head_turn_is_followed_within_a_few_frames() {
        let mut viewport = no_deadzone();
        let start = settle(&mut viewport, pose(0.0, 0.0));
        let mut reference = no_deadzone();
        settle(&mut reference, pose(0.0, 0.0));
        let target = settle(&mut reference, pose(0.2, 0.0));

        let mut rect = start;
        for _ in 0..3 {
            rect = viewport.update(pose(0.2, 0.0), DT, 1920, 1080, 7680, 4320);
        }
        let progress = (start.x - rect.x) / (start.x - target.x);
        assert!(progress > 0.9, "only {progress:.2} of the way after 25 ms");
    }

    #[test]
    fn small_jitter_while_still_is_damped() {
        let mut viewport = no_deadzone();
        let jitter = 0.002;
        let center = settle(&mut viewport, pose(0.0, 0.0));

        let mut max_offset: f32 = 0.0;
        for i in 0..240 {
            let yaw = if i % 2 == 0 { jitter } else { -jitter };
            let rect = viewport.update(pose(yaw, 0.0), DT, 1920, 1080, 7680, 4320);
            max_offset = max_offset.max((rect.x - center.x).abs());
        }
        let unfiltered = jitter * 1920.0 / HORIZONTAL_FOV;
        assert!(max_offset < unfiltered * 0.3, "jitter {max_offset} px vs raw {unfiltered} px");
    }
}
