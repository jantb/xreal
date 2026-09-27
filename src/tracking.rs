use crate::ar_drivers::lib::{any_glasses, DisplayMode, Error, GlassesEvent};
use dcmimu::DCMIMU;
use std::f32::consts::PI;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use thread_priority::{set_current_thread_priority, ThreadPriority};

// Upper bound for any bias correction; real residual bias is far below this.
const MAX_BIAS: f32 = 0.1; // rad/s

// Automatic learning only runs while the head is essentially still, and only
// nudges the estimate slowly, so slow deliberate turns are not absorbed.
const AUTO_MOTION_THRESHOLD: f32 = 0.02; // rad/s, relative to the current bias
const AUTO_STILL_SECONDS: f32 = 1.5;
const AUTO_LEARN_TIME_CONSTANT: f32 = 5.0; // seconds

// Explicit calibration: the glasses must lie still for this long.
const CALIBRATION_SECONDS: f32 = 2.0;
const CALIBRATION_TIMEOUT_SECONDS: f32 = 10.0;
const CALIBRATION_MOTION_THRESHOLD: f32 = 0.05; // rad/s, relative to the running mean
const CALIBRATION_MIN_SAMPLES: u32 = 50;

// Converges the DCM's gravity estimate before the first pose is published, so
// the viewport does not slide while the filter settles.
const WARMUP_STEPS: usize = 400;
const WARMUP_DT: f32 = 0.01;

// Longer gaps between IMU samples are skipped instead of integrated.
const MAX_SAMPLE_GAP: f32 = 0.1; // seconds
const RATE_TIME_CONSTANT: f32 = 0.015; // seconds
const MAX_PREDICTION_AGE: Duration = Duration::from_millis(50);
const READ_ERRORS_BEFORE_RECONNECT: u32 = 4;

// Learning from manual recenters: yaw drift between two recenters is
// assumed to be leftover gyro bias about the vertical axis.
const DRIFT_MIN_INTERVAL: Duration = Duration::from_secs(20);
// Faster apparent drift is taken as a deliberate turn (e.g. moving the chair).
const DRIFT_MAX_RATE: f32 = 0.01; // rad/s, about 34°/min
// Correct only part of the measured drift each time, so one misjudged
// recenter cannot throw the estimate far off.
const DRIFT_GAIN: f32 = 0.5;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CalibrationState {
    Idle,
    Running { progress: f32 },
    Succeeded,
    Failed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImuStatus {
    Searching,
    Connected,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HeadPose {
    pub yaw: f32,
    pub pitch: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct TrackingSnapshot {
    pub status: ImuStatus,
    /// Increments each time the glasses (re)connect and the pose restarts.
    pub session: u64,
    pub pose: HeadPose,
    pub yaw_rate: f32,
    pub pitch_rate: f32,
    pub sampled_at: Option<Instant>,
    pub sample_rate_hz: f32,
    pub gyro_bias: [f32; 3],
    pub still: bool,
    pub calibration: CalibrationState,
    /// Increments whenever the bias changes by calibration or drift correction.
    pub bias_revision: u32,
}

impl TrackingSnapshot {
    fn new(gyro_bias: [f32; 3]) -> Self {
        Self {
            status: ImuStatus::Searching,
            session: 0,
            pose: HeadPose::default(),
            yaw_rate: 0.0,
            pitch_rate: 0.0,
            sampled_at: None,
            sample_rate_hz: 0.0,
            gyro_bias,
            still: false,
            calibration: CalibrationState::Idle,
            bias_revision: 0,
        }
    }

    /// Extrapolates the pose to `lead` past `now`, covering the time between
    /// the last IMU sample and the moment the frame reaches the display.
    pub fn predict(&self, now: Instant, lead: Duration) -> HeadPose {
        let Some(sampled_at) = self.sampled_at else {
            return self.pose;
        };
        let age = now.saturating_duration_since(sampled_at);
        if self.status != ImuStatus::Connected || age > MAX_PREDICTION_AGE {
            return self.pose;
        }
        let horizon = (age + lead).as_secs_f32();
        HeadPose {
            yaw: wrap_angle(self.pose.yaw + self.yaw_rate * horizon),
            pitch: self.pose.pitch + self.pitch_rate * horizon,
        }
    }
}

enum TrackingCommand {
    Calibrate,
    CorrectYawDrift(f32),
}

pub struct Tracking {
    shared: Arc<Mutex<TrackingSnapshot>>,
    commands: Sender<TrackingCommand>,
}

impl Tracking {
    pub fn spawn(initial_bias: [f32; 3]) -> Self {
        let shared = Arc::new(Mutex::new(TrackingSnapshot::new(initial_bias)));
        let (commands, command_rx) = mpsc::channel();
        thread::spawn({
            let shared = Arc::clone(&shared);
            move || run_tracking(shared, command_rx, initial_bias)
        });
        Self { shared, commands }
    }

    pub fn snapshot(&self) -> TrackingSnapshot {
        *self.shared.lock().unwrap()
    }

    /// Starts measuring gyro bias; the glasses should lie still meanwhile.
    pub fn calibrate(&self) {
        let _ = self.commands.send(TrackingCommand::Calibrate);
    }

    /// Removes `rate` (rad/s) of yaw drift from the bias estimate.
    pub fn correct_yaw_drift(&self, rate: f32) {
        let _ = self.commands.send(TrackingCommand::CorrectYawDrift(rate));
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DriftObservation {
    /// First recenter: nothing to compare against yet.
    Anchored,
    /// Too soon after the reference recenter to tell drift from noise.
    TooSoon,
    /// `measured` rad/s of drift seen; `correction` of it should be removed.
    Learned { measured: f32, correction: f32 },
    /// Moved too fast to be drift; treated as a deliberate turn.
    Rejected { measured: f32 },
}

/// Estimates leftover yaw drift from manual recenters. Each recenter says
/// "the screen belongs where I am looking now", so the yaw difference between
/// two recenters is drift accumulated over the time between them.
pub struct DriftLearner {
    anchor: Option<(Instant, f32)>,
}

impl DriftLearner {
    pub fn new() -> Self {
        Self { anchor: None }
    }

    /// Forgets the reference, e.g. after the pose restarts.
    pub fn reset(&mut self) {
        self.anchor = None;
    }

    pub fn observe_recenter(&mut self, now: Instant, yaw: f32) -> DriftObservation {
        let Some((anchored_at, anchor_yaw)) = self.anchor else {
            self.anchor = Some((now, yaw));
            return DriftObservation::Anchored;
        };
        let elapsed = now.saturating_duration_since(anchored_at);
        if elapsed < DRIFT_MIN_INTERVAL {
            // Keep the older reference: a longer interval measures better.
            return DriftObservation::TooSoon;
        }

        self.anchor = Some((now, yaw));
        let measured = wrap_angle(yaw - anchor_yaw) / elapsed.as_secs_f32();
        if measured.abs() > DRIFT_MAX_RATE {
            DriftObservation::Rejected { measured }
        } else {
            DriftObservation::Learned {
                measured,
                correction: measured * DRIFT_GAIN,
            }
        }
    }
}

fn run_tracking(
    shared: Arc<Mutex<TrackingSnapshot>>,
    commands: Receiver<TrackingCommand>,
    initial_bias: [f32; 3],
) {
    if let Err(err) = set_current_thread_priority(ThreadPriority::Max) {
        eprintln!("Failed to set glasses tracking thread priority: {err}");
    }

    let mut bias = GyroBiasEstimator::new(initial_bias);
    let mut session = 0;
    let mut reported_missing = false;

    loop {
        shared.lock().unwrap().status = ImuStatus::Searching;
        let mut glasses = match any_glasses() {
            Ok(glasses) => glasses,
            Err(err) => {
                if !reported_missing {
                    eprintln!("Glasses not available ({err}), retrying");
                    reported_missing = true;
                }
                thread::sleep(Duration::from_secs(1));
                continue;
            }
        };
        reported_missing = false;
        if let Err(err) = glasses.set_display_mode(DisplayMode::HighRefreshRate) {
            eprintln!("Failed to set high refresh rate display mode: {err}");
        }
        eprintln!("Connected to {}", glasses.name());

        session += 1;
        let mut fusion = Fusion::new();
        let mut rate = SampleRate::new();
        let mut read_errors = 0;

        loop {
            while let Ok(command) = commands.try_recv() {
                match command {
                    TrackingCommand::Calibrate => bias.start_calibration(),
                    TrackingCommand::CorrectYawDrift(rate) => {
                        bias.correct_yaw_drift(rate, fusion.up)
                    }
                }
            }

            match glasses.read_event() {
                Ok(GlassesEvent::AccGyro {
                    accelerometer,
                    gyroscope,
                    timestamp,
                }) => {
                    read_errors = 0;
                    let gyro = [gyroscope.x, gyroscope.y, gyroscope.z];
                    let acc = [accelerometer.x, accelerometer.y, accelerometer.z];
                    let Some(update) = fusion.push(gyro, acc, timestamp, &mut bias) else {
                        continue;
                    };
                    let now = Instant::now();
                    let sample_rate_hz = rate.tick(now);

                    let mut snapshot = shared.lock().unwrap();
                    snapshot.status = ImuStatus::Connected;
                    snapshot.session = session;
                    snapshot.pose = update.pose;
                    snapshot.yaw_rate = update.yaw_rate;
                    snapshot.pitch_rate = update.pitch_rate;
                    snapshot.sampled_at = Some(now);
                    snapshot.sample_rate_hz = sample_rate_hz;
                    snapshot.gyro_bias = bias.bias();
                    snapshot.still = bias.is_still();
                    snapshot.calibration = bias.calibration_state();
                    snapshot.bias_revision = bias.bias_revision();
                }
                Ok(_) => {}
                Err(err) => {
                    read_errors += 1;
                    let device_gone = matches!(err, Error::HidError(_) | Error::IoError(_));
                    if device_gone || read_errors >= READ_ERRORS_BEFORE_RECONNECT {
                        eprintln!("Lost glasses ({err}), reconnecting");
                        break;
                    }
                }
            }
        }

        drop(glasses);
        shared.lock().unwrap().status = ImuStatus::Searching;
        thread::sleep(Duration::from_millis(500));
    }
}

struct FusionUpdate {
    pose: HeadPose,
    yaw_rate: f32,
    pitch_rate: f32,
}

/// Turns raw IMU samples into a head pose. Every sample is integrated with
/// its own device-time `dt`, so no rotation is lost between samples.
struct Fusion {
    dcm: DCMIMU,
    last_timestamp: Option<u64>,
    last_pose: Option<HeadPose>,
    yaw: f32,
    yaw_rate: f32,
    pitch_rate: f32,
    /// Latest body-frame up direction.
    up: [f32; 3],
}

impl Fusion {
    fn new() -> Self {
        Self {
            dcm: DCMIMU::new(),
            last_timestamp: None,
            last_pose: None,
            yaw: 0.0,
            up: [0.0, 1.0, 0.0],
            yaw_rate: 0.0,
            pitch_rate: 0.0,
        }
    }

    /// `timestamp` is device time in microseconds.
    fn push(
        &mut self,
        gyro: [f32; 3],
        acc: [f32; 3],
        timestamp: u64,
        bias: &mut GyroBiasEstimator,
    ) -> Option<FusionUpdate> {
        let Some(last_timestamp) = self.last_timestamp else {
            self.last_timestamp = Some(timestamp);
            for _ in 0..WARMUP_STEPS {
                self.dcm
                    .update((0.0, 0.0, 0.0), (acc[0], acc[1], acc[2]), WARMUP_DT);
            }
            return None;
        };
        if timestamp <= last_timestamp {
            return None;
        }
        self.last_timestamp = Some(timestamp);
        let dt = (timestamp - last_timestamp) as f32 / 1_000_000.0;
        if dt > MAX_SAMPLE_GAP {
            return None;
        }

        let gyro = bias.correct(gyro, dt);
        let (angles, _) = self
            .dcm
            .update((gyro[0], gyro[1], gyro[2]), (acc[0], acc[1], acc[2]), dt);

        // Yaw is integrated here rather than taken from the DCM: its internal
        // bias estimate for the vertical axis is unobservable, wanders, and
        // shows up as yaw drift. Only the DCM's gravity direction is used.
        let up = up_vector(angles.pitch, angles.roll);
        self.up = up;
        let yaw_rate = dot(gyro, up);
        self.yaw = wrap_angle(self.yaw + yaw_rate * dt);

        // The glasses report in a Y-up frame, so the DCM's roll axis is the
        // head's pitch (nodding).
        let pose = HeadPose {
            yaw: self.yaw,
            pitch: angles.roll,
        };

        if let Some(last) = self.last_pose {
            let alpha = dt / (RATE_TIME_CONSTANT + dt);
            let yaw_rate = wrap_angle(pose.yaw - last.yaw) / dt;
            let pitch_rate = (pose.pitch - last.pitch) / dt;
            self.yaw_rate += (yaw_rate - self.yaw_rate) * alpha;
            self.pitch_rate += (pitch_rate - self.pitch_rate) * alpha;
        }
        self.last_pose = Some(pose);

        Some(FusionUpdate {
            pose,
            yaw_rate: self.yaw_rate,
            pitch_rate: self.pitch_rate,
        })
    }
}

/// Estimates the gyro's zero-rate offset. The DCM filter can only learn bias
/// on axes that gravity makes observable; bias about the vertical axis would
/// otherwise integrate straight into yaw drift.
pub struct GyroBiasEstimator {
    bias: [f32; 3],
    still_for: f32,
    calibration: Option<Calibration>,
    calibration_result: CalibrationState,
    bias_revision: u32,
}

struct Calibration {
    elapsed: f32,
    still_for: f32,
    count: u32,
    mean: [f64; 3],
}

impl Calibration {
    fn new() -> Self {
        Self {
            elapsed: 0.0,
            still_for: 0.0,
            count: 0,
            mean: [0.0; 3],
        }
    }

    fn restart_window(&mut self) {
        self.still_for = 0.0;
        self.count = 0;
        self.mean = [0.0; 3];
    }
}

impl GyroBiasEstimator {
    pub fn new(bias: [f32; 3]) -> Self {
        Self {
            bias: bias.map(|b| b.clamp(-MAX_BIAS, MAX_BIAS)),
            still_for: 0.0,
            calibration: None,
            calibration_result: CalibrationState::Idle,
            bias_revision: 0,
        }
    }

    pub fn bias(&self) -> [f32; 3] {
        self.bias
    }

    pub fn is_still(&self) -> bool {
        self.still_for >= AUTO_STILL_SECONDS
    }

    pub fn calibration_state(&self) -> CalibrationState {
        match &self.calibration {
            Some(calibration) => CalibrationState::Running {
                progress: (calibration.still_for / CALIBRATION_SECONDS).min(1.0),
            },
            None => self.calibration_result,
        }
    }

    pub fn bias_revision(&self) -> u32 {
        self.bias_revision
    }

    pub fn start_calibration(&mut self) {
        self.calibration = Some(Calibration::new());
    }

    /// Adds `rate` (rad/s) of bias about the body-frame `up` axis, which is
    /// what shows up as yaw drift.
    pub fn correct_yaw_drift(&mut self, rate: f32, up: [f32; 3]) {
        for (bias, up) in self.bias.iter_mut().zip(up) {
            *bias = (*bias + rate * up).clamp(-MAX_BIAS, MAX_BIAS);
        }
        self.bias_revision += 1;
    }

    /// Returns the bias-corrected gyro reading and updates the estimate.
    pub fn correct(&mut self, gyro: [f32; 3], dt: f32) -> [f32; 3] {
        let corrected = [
            gyro[0] - self.bias[0],
            gyro[1] - self.bias[1],
            gyro[2] - self.bias[2],
        ];

        if self.calibration.is_some() {
            self.update_calibration(gyro, dt);
        } else {
            self.update_auto(gyro, corrected, dt);
        }
        corrected
    }

    fn update_auto(&mut self, gyro: [f32; 3], corrected: [f32; 3], dt: f32) {
        if magnitude(corrected) < AUTO_MOTION_THRESHOLD {
            self.still_for += dt;
        } else {
            self.still_for = 0.0;
        }
        // While moving the estimate is held as-is.
        if self.is_still() {
            let k = dt / AUTO_LEARN_TIME_CONSTANT;
            for (bias, gyro) in self.bias.iter_mut().zip(gyro) {
                *bias = (*bias + (gyro - *bias) * k).clamp(-MAX_BIAS, MAX_BIAS);
            }
        }
    }

    fn update_calibration(&mut self, gyro: [f32; 3], dt: f32) {
        let Some(calibration) = self.calibration.as_mut() else {
            return;
        };
        calibration.elapsed += dt;

        let deviation = [
            gyro[0] - calibration.mean[0] as f32,
            gyro[1] - calibration.mean[1] as f32,
            gyro[2] - calibration.mean[2] as f32,
        ];
        if calibration.count >= CALIBRATION_MIN_SAMPLES
            && magnitude(deviation) > CALIBRATION_MOTION_THRESHOLD
        {
            calibration.restart_window();
        }

        calibration.count += 1;
        calibration.still_for += dt;
        let n = calibration.count as f64;
        for (mean, gyro) in calibration.mean.iter_mut().zip(gyro) {
            *mean += (gyro as f64 - *mean) / n;
        }

        if calibration.still_for >= CALIBRATION_SECONDS {
            self.bias = calibration
                .mean
                .map(|m| (m as f32).clamp(-MAX_BIAS, MAX_BIAS));
            self.calibration = None;
            self.calibration_result = CalibrationState::Succeeded;
            self.bias_revision += 1;
            self.still_for = 0.0;
        } else if calibration.elapsed >= CALIBRATION_TIMEOUT_SECONDS {
            self.calibration = None;
            self.calibration_result = CalibrationState::Failed;
        }
    }
}

struct SampleRate {
    count: u32,
    since: Instant,
    rate: f32,
}

impl SampleRate {
    fn new() -> Self {
        Self {
            count: 0,
            since: Instant::now(),
            rate: 0.0,
        }
    }

    fn tick(&mut self, now: Instant) -> f32 {
        self.count += 1;
        let elapsed = now.saturating_duration_since(self.since);
        if elapsed >= Duration::from_millis(500) {
            self.rate = self.count as f32 / elapsed.as_secs_f32();
            self.count = 0;
            self.since = now;
        }
        self.rate
    }
}

/// Body-frame unit vector pointing up, rebuilt from the DCM's angles
/// (roll = atan2(x1, x2), pitch = asin(-x0) of its gravity estimate x).
fn up_vector(pitch: f32, roll: f32) -> [f32; 3] {
    [-pitch.sin(), pitch.cos() * roll.sin(), pitch.cos() * roll.cos()]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn magnitude(v: [f32; 3]) -> f32 {
    dot(v, v).sqrt()
}

/// Wraps an angle difference into [-PI, PI].
pub fn wrap_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;
    const GRAVITY_Y_UP: [f32; 3] = [0.0, 9.81, 0.0];

    fn run_still(fusion: &mut Fusion, bias: &mut GyroBiasEstimator, gyro: [f32; 3], seconds: f32, t: &mut u64) -> HeadPose {
        let mut pose = HeadPose::default();
        for _ in 0..(seconds / DT) as usize {
            *t += (DT * 1_000_000.0) as u64;
            if let Some(update) = fusion.push(gyro, GRAVITY_Y_UP, *t, bias) {
                pose = update.pose;
            }
        }
        pose
    }

    #[test]
    fn wrap_angle_takes_the_short_way_round() {
        assert!((wrap_angle(3.0 - (-3.0)) - (6.0 - 2.0 * PI)).abs() < 1e-5);
        assert!((wrap_angle(-3.0 - 3.0) - (2.0 * PI - 6.0)).abs() < 1e-5);
        assert!((wrap_angle(0.25) - 0.25).abs() < 1e-6);
    }

    #[test]
    fn first_pose_is_already_settled_on_gravity() {
        let mut fusion = Fusion::new();
        let mut bias = GyroBiasEstimator::new([0.0; 3]);
        let mut t = 1_000;
        let first = run_still(&mut fusion, &mut bias, [0.0; 3], 0.01, &mut t);
        let later = run_still(&mut fusion, &mut bias, [0.0; 3], 2.0, &mut t);

        assert!((first.pitch - later.pitch).abs() < 0.01, "first={first:?} later={later:?}");
        assert!(first.yaw.abs() < 1e-3);
    }

    #[test]
    fn still_glasses_with_gyro_offset_stop_drifting() {
        let offset = [0.004, 0.012, -0.006];
        let mut fusion = Fusion::new();
        let mut bias = GyroBiasEstimator::new([0.0; 3]);
        let mut t = 1_000;

        run_still(&mut fusion, &mut bias, offset, 30.0, &mut t);
        let before = run_still(&mut fusion, &mut bias, offset, 0.001, &mut t);
        let after = run_still(&mut fusion, &mut bias, offset, 10.0, &mut t);

        // Without correction 0.012 rad/s would drift 0.12 rad in 10 s.
        let drift = wrap_angle(after.yaw - before.yaw).abs();
        assert!(drift < 0.01, "yaw drifted {drift} rad in 10 s");
    }

    #[test]
    fn turning_about_the_vertical_changes_yaw_by_the_turned_angle() {
        // Upright, and nodded down 30° while turning.
        for tilt in [0.0f32, std::f32::consts::FRAC_PI_6] {
            let up = [0.0, tilt.cos(), tilt.sin()];
            let gravity = up.map(|c| c * 9.81);
            let mut fusion = Fusion::new();
            let mut bias = GyroBiasEstimator::new([0.0; 3]);
            let mut t = 1_000;
            let mut start = None;
            let mut end = HeadPose::default();
            for _ in 0..1000 {
                t += 1_000;
                if let Some(update) = fusion.push([0.0; 3], gravity, t, &mut bias) {
                    start.get_or_insert(update.pose);
                }
            }
            for _ in 0..1000 {
                t += 1_000;
                let gyro = up.map(|c| c * 0.5);
                if let Some(update) = fusion.push(gyro, gravity, t, &mut bias) {
                    end = update.pose;
                }
            }
            let turned = wrap_angle(end.yaw - start.unwrap().yaw);
            assert!((turned - 0.5).abs() < 0.01, "tilt {tilt}: turned {turned} rad");
        }
    }

    #[test]
    fn slow_head_turn_is_not_learned_as_bias() {
        let mut bias = GyroBiasEstimator::new([0.0; 3]);
        for _ in 0..(20.0 / DT) as usize {
            bias.correct([0.0, 0.1, 0.0], DT);
        }
        assert!(bias.bias()[1].abs() < 1e-6);
    }

    #[test]
    fn bias_is_kept_while_moving() {
        let mut bias = GyroBiasEstimator::new([0.0, 0.01, 0.0]);
        for _ in 0..(20.0 / DT) as usize {
            bias.correct([0.3, 0.8, -0.2], DT);
        }
        assert!((bias.bias()[1] - 0.01).abs() < 1e-6);
    }

    #[test]
    fn calibration_measures_offset_of_still_glasses() {
        let offset = [0.03, -0.045, 0.02];
        let mut bias = GyroBiasEstimator::new([0.0; 3]);
        bias.start_calibration();
        for i in 0..(3.0 / DT) as usize {
            let noise = if i % 2 == 0 { 0.002 } else { -0.002 };
            bias.correct([offset[0] + noise, offset[1] - noise, offset[2]], DT);
        }

        assert_eq!(bias.calibration_state(), CalibrationState::Succeeded);
        for (learned, offset) in bias.bias().iter().zip(offset) {
            assert!((learned - offset).abs() < 1e-3, "{:?}", bias.bias());
        }
    }

    #[test]
    fn calibration_fails_if_glasses_keep_moving() {
        let mut bias = GyroBiasEstimator::new([0.0; 3]);
        bias.start_calibration();
        for i in 0..(12.0 / DT) as usize {
            let swing = if (i / 100) % 2 == 0 { 0.5 } else { -0.5 };
            bias.correct([0.0, swing, 0.0], DT);
        }
        assert_eq!(bias.calibration_state(), CalibrationState::Failed);
        assert_eq!(bias.bias(), [0.0; 3]);
    }

    #[test]
    fn recenters_reveal_slow_drift() {
        let start = Instant::now();
        let mut learner = DriftLearner::new();
        assert_eq!(learner.observe_recenter(start, 0.2), DriftObservation::Anchored);

        let observation = learner.observe_recenter(start + Duration::from_secs(60), 0.2 + 0.24);
        let DriftObservation::Learned { measured, correction } = observation else {
            panic!("expected drift to be learned, got {observation:?}");
        };
        assert!((measured - 0.004).abs() < 1e-5);
        assert!(correction > 0.0 && correction <= measured);
    }

    #[test]
    fn deliberate_turn_between_recenters_is_not_learned() {
        let start = Instant::now();
        let mut learner = DriftLearner::new();
        learner.observe_recenter(start, 0.0);

        let observation = learner.observe_recenter(start + Duration::from_secs(30), 1.2);
        assert!(matches!(observation, DriftObservation::Rejected { .. }), "{observation:?}");
    }

    #[test]
    fn quick_repeated_recenters_measure_from_the_older_one() {
        let start = Instant::now();
        let mut learner = DriftLearner::new();
        learner.observe_recenter(start, 0.0);
        assert_eq!(
            learner.observe_recenter(start + Duration::from_secs(5), 0.02),
            DriftObservation::TooSoon
        );

        let observation = learner.observe_recenter(start + Duration::from_secs(40), 0.12);
        let DriftObservation::Learned { measured, .. } = observation else {
            panic!("expected drift to be learned, got {observation:?}");
        };
        assert!((measured - 0.003).abs() < 1e-5);
    }

    #[test]
    fn learned_drift_correction_removes_that_drift() {
        let residual = 0.003;
        let mut fusion = Fusion::new();
        let mut bias = GyroBiasEstimator::new([0.0; 3]);
        let mut t = 1_000;
        // Keep moving slightly so automatic learning stays out of the way.
        let gyro = |i: usize| [if i.is_multiple_of(2) { 0.05 } else { -0.05 }, residual, 0.0];

        let yaw_over = |fusion: &mut Fusion, bias: &mut GyroBiasEstimator, t: &mut u64| {
            let mut first = None;
            let mut last = 0.0;
            for i in 0..10_000 {
                *t += 1_000;
                if let Some(update) = fusion.push(gyro(i), GRAVITY_Y_UP, *t, bias) {
                    first.get_or_insert(update.pose.yaw);
                    last = update.pose.yaw;
                }
            }
            wrap_angle(last - first.unwrap_or(last))
        };

        let before = yaw_over(&mut fusion, &mut bias, &mut t);
        bias.correct_yaw_drift(residual, fusion.up);
        let after = yaw_over(&mut fusion, &mut bias, &mut t);

        assert!(before > 0.025, "expected visible drift first, got {before}");
        assert!(after.abs() < 0.003, "still drifting {after} rad in 10 s");
    }

    #[test]
    fn prediction_extrapolates_along_head_motion() {
        let now = Instant::now();
        let snapshot = TrackingSnapshot {
            status: ImuStatus::Connected,
            pose: HeadPose { yaw: 0.1, pitch: 0.2 },
            yaw_rate: 1.0,
            pitch_rate: -0.5,
            sampled_at: Some(now),
            ..TrackingSnapshot::new([0.0; 3])
        };
        let predicted = snapshot.predict(now, Duration::from_millis(10));
        assert!((predicted.yaw - 0.11).abs() < 1e-4);
        assert!((predicted.pitch - 0.195).abs() < 1e-4);
    }

    #[test]
    fn prediction_holds_pose_when_tracking_is_stale() {
        let sampled_at = Instant::now();
        let snapshot = TrackingSnapshot {
            status: ImuStatus::Connected,
            pose: HeadPose { yaw: 0.1, pitch: 0.2 },
            yaw_rate: 1.0,
            sampled_at: Some(sampled_at),
            ..TrackingSnapshot::new([0.0; 3])
        };
        let predicted = snapshot.predict(sampled_at + Duration::from_secs(1), Duration::from_millis(10));
        assert_eq!(predicted, snapshot.pose);
    }
}
