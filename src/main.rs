mod ar_drivers {
    pub mod lib;
}

use ar_drivers::lib::{any_glasses, GlassesEvent};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use crossbeam_channel::{bounded, Receiver, Sender};
use dcmimu::DCMIMU;
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;
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
        set_current_thread_priority(ThreadPriority::Max).expect("Failed to set thread priority");
        let last_timestamp = Arc::clone(&last_timestamp);
        let sender = sender.clone();
        move || {
            let mut glasses = match any_glasses() {
                Ok(glasses) => glasses,
                Err(_) => return, // Exit if unable to acquire glasses
            };

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
                thread::yield_now();
            }
        }
    });

    thread::spawn({
        let buffer_size = 32; // smoothing window
        let gyro_buffer = Arc::new(Mutex::new(VecDeque::with_capacity(buffer_size)));
        let acc_buffer = Arc::new(Mutex::new(VecDeque::with_capacity(buffer_size)));

        // Gyro bias estimator state
        let mut gyro_bias = (0.0f32, 0.0f32, 0.0f32);
        let max_bias = 0.2f32;     // rad/s clamp
        let alpha = 0.003f32;      // learning rate when stationary
        let decay = 0.0005f32;     // gentle decay when moving

        // Stationary detection thresholds
        let gyro_thresh = 0.04f32; // rad/s
        let acc_g = 9.81f32;
        let acc_thresh = 0.35f32;  // m/s^2 window around |g|

        move || {
            set_current_thread_priority(ThreadPriority::Max).expect("Failed to set thread priority");
            loop {
                // Optional reset request
                while bias_reset_rx.try_recv().is_ok() {
                    gyro_bias = (0.0, 0.0, 0.0);
                }

                if let Ok((gyro, acc, dt)) = receiver.recv() {
                    let mut gyro_buffer = gyro_buffer.lock().unwrap();
                    let mut acc_buffer = acc_buffer.lock().unwrap();

                    if gyro_buffer.len() == buffer_size {
                        gyro_buffer.pop_front();
                    }
                    gyro_buffer.push_back(gyro);

                    if acc_buffer.len() == buffer_size {
                        acc_buffer.pop_front();
                    }
                    acc_buffer.push_back(acc);

                    // Moving averages
                    let average_gyro_sum = gyro_buffer.iter().fold((0.0, 0.0, 0.0), |a, &(x, y, z)| {
                        (a.0 + x, a.1 + y, a.2 + z)
                    });
                    let n_g = gyro_buffer.len() as f32;
                    let average_gyro = (
                        average_gyro_sum.0 / n_g,
                        average_gyro_sum.1 / n_g,
                        average_gyro_sum.2 / n_g,
                    );

                    let average_acc_sum = acc_buffer.iter().fold((0.0, 0.0, 0.0), |a, &(x, y, z)| {
                        (a.0 + x, a.1 + y, a.2 + z)
                    });
                    let n_a = acc_buffer.len() as f32;
                    let average_acc = (
                        average_acc_sum.0 / n_a,
                        average_acc_sum.1 / n_a,
                        average_acc_sum.2 / n_a,
                    );

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

    let store = SharedGlassesStore { dcmimu: Arc::new(Mutex::new(DCMIMU::new())) };
    create_glasses_thread(&store);

    let mut recorder = Capturer::new(Options {
        fps: 60,
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
    x_offset: f64,
    y_offset: f64,
    screen_width: usize,
    store: SharedGlassesStore,
    o_x: f32,
    o_y: f32,
    pred: Option<BGRAFrame>,
}

impl ControlFlowDemo {
    fn new(recorder: Capturer, store: SharedGlassesStore) -> Self {
        Self {
            close_requested: false,
            pixels: None,
            window: None,
            screen_width: 0,
            recorder,
            x_offset: 0.,
            y_offset: 0.,
            store,
            o_x: -0.5,
            o_y: -0.9,
            pred: None,
        }
    }
}

impl ApplicationHandler for ControlFlowDemo {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let window_attributes = Window::default_attributes().with_title(
            "Xreal renderer",
        ).with_inner_size(winit::dpi::LogicalSize::new(1920.0, 1080.0));
        let window = event_loop.create_window(window_attributes).unwrap();
        let primary_monitor = window.primary_monitor();
        let desired_monitor = event_loop
            .available_monitors()
            .find(|monitor| monitor.name().map_or(false, |name| name == "Monitor #12596"))
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
                event: KeyEvent { logical_key: key, state: ElementState::Pressed, .. },
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
                    pixels.resize_surface(size.width, size.height).expect("Resize failed");
                }
            }
            WindowEvent::RedrawRequested => {
                fn calculate_offset(angle: f32, offset: f32, dimension: f32, multiplier: f32) -> isize {
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
                }
            }
            _ => (),
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        if !self.close_requested {
            self.window.as_ref().unwrap().request_redraw();
        }

        event_loop.set_control_flow(ControlFlow::Poll);

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

    for (y, row) in raw_buffer.chunks_mut(width * 4).enumerate() {
        let row_y = y as isize + y_offset;

        let row_out_of_bounds =
            row_y < 0 || row_y >= (frame.len() as isize / frame_stride as isize);

        for (x, chunk) in row.chunks_mut(4).enumerate() {
            let col_x = x as isize + x_offset;

            let pixel_out_of_bounds =
                col_x < 0 || col_x >= screen_width as isize || row_out_of_bounds;

            if pixel_out_of_bounds {
                // Set pixel to black
                chunk[0] = 0; // R
                chunk[1] = 0; // G
                chunk[2] = 0; // B
                chunk[3] = 255; // A
            } else {
                let frame_offset = row_y * frame_stride as isize + col_x * 4;
                let frame_index = frame_offset as usize;

                if frame_index + 3 < frame.len() {
                    chunk[0] = frame[frame_index + 2]; // R
                    chunk[1] = frame[frame_index + 1]; // G
                    chunk[2] = frame[frame_index]; // B
                    chunk[3] = 255; // A
                } else {
                    chunk[0] = 0;
                    chunk[1] = 0;
                    chunk[2] = 0;
                    chunk[3] = 255;
                }
            }
        }
    }
}