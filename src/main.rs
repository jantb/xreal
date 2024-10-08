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

struct SharedGlassesStore {
    dcmimu: Arc<Mutex<DCMIMU>>,
}

fn create_glasses_thread(store: &SharedGlassesStore) {
    let shared_dcmimu_clone = Arc::clone(&store.dcmimu);
    let last_timestamp = Arc::new(AtomicU64::new(0));
    let (sender, receiver): (Sender<_>, Receiver<_>) = bounded(1);

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
                        let dt = (timestamp - last_ts) as f32 / 1_000_000.0; // in seconds
                        match sender.try_send((
                            (gyroscope.x, gyroscope.y, gyroscope.z),
                            (accelerometer.x, accelerometer.y, accelerometer.z),
                            dt,
                        )) {
                            Ok(_) => {}
                            Err(_) => {}
                        };
                    }
                    last_timestamp.store(timestamp, Ordering::Relaxed);
                }
                thread::yield_now();
            }
        }
    });

    thread::spawn({
        let buffer_size = 32; // Larger buffer size for smoother response
        let gyro_buffer = Arc::new(Mutex::new(VecDeque::with_capacity(buffer_size)));
        let acc_buffer = Arc::new(Mutex::new(VecDeque::with_capacity(buffer_size)));

        move || {
            set_current_thread_priority(ThreadPriority::Max).expect("Failed to set thread priority");
            loop {
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

                    // Calculating moving average
                    let average_gyro = gyro_buffer.iter().fold((0.0, 0.0, 0.0), |acc, &(x, y, z)| {
                        (acc.0 + x, acc.1 + y, acc.2 + z)
                    });
                    let average_gyro = (
                        average_gyro.0 / gyro_buffer.len() as f32,
                        average_gyro.1 / gyro_buffer.len() as f32,
                        average_gyro.2 / gyro_buffer.len() as f32,
                    );

                    let average_acc = acc_buffer.iter().fold((0.0, 0.0, 0.0), |acc, &(x, y, z)| {
                        (acc.0 + x, acc.1 + y, acc.2 + z)
                    });
                    let average_acc = (
                        average_acc.0 / acc_buffer.len() as f32,
                        average_acc.1 / acc_buffer.len() as f32,
                        average_acc.2 / acc_buffer.len() as f32,
                    );

                    let mut dcmimu = shared_dcmimu_clone.lock().unwrap();
                    dcmimu.update(average_gyro, average_acc, dt);
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
        ).with_inner_size(winit::dpi::LogicalSize::new(1920.0, 1080.0))
            ;;
        let window = event_loop.create_window(window_attributes).unwrap();
        let primary_monitor = window.primary_monitor();
        let desired_monitor = event_loop.available_monitors()
            .find(|monitor| monitor.name().map_or(false, |name| name == "Monitor #12596"))
            .or_else(|| Some(primary_monitor.unwrap()));
        window.set_fullscreen(Some(Fullscreen::Borderless(desired_monitor)));
        self.screen_width = window.primary_monitor().unwrap().size().width as usize;
        let size = window.inner_size();

        // Create the pixel buffer using `pixels` crate
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
                Key::Named(NamedKey::Space) => {
                    let (yaw, roll) = {
                        let dcm = self.store.dcmimu.lock().unwrap().all();
                        (dcm.yaw, dcm.roll)
                    };

                    self.o_x = -yaw;
                    self.o_y = roll;
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

        let row_out_of_bounds = row_y < 0 || row_y >= (frame.len() as isize / frame_stride as isize);

        for (x, chunk) in row.chunks_mut(4).enumerate() {
            let col_x = x as isize + x_offset;

            let pixel_out_of_bounds = col_x < 0 || col_x >= screen_width as isize || row_out_of_bounds;

            if pixel_out_of_bounds {
                // Set pixel to black
                chunk[0] = 0;   // R
                chunk[1] = 0;   // G
                chunk[2] = 0;   // B
                chunk[3] = 255; // A (fully opaque)
            } else {
                // Calculate the position in the frame buffer
                let frame_offset = row_y * frame_stride as isize + col_x * 4;
                let frame_index = frame_offset as usize;

                // Ensure the index is within the valid frame data
                if frame_index + 3 < frame.len() {
                    chunk[0] = frame[frame_index + 2]; // R
                    chunk[1] = frame[frame_index + 1]; // G
                    chunk[2] = frame[frame_index];     // B
                    chunk[3] = 255;                    // A (fully opaque)
                } else {
                    // Set pixel to black if the index is out of bounds
                    chunk[0] = 0;   // R
                    chunk[1] = 0;   // G
                    chunk[2] = 0;   // B
                    chunk[3] = 255; // A (fully opaque)
                }
            }
        }
    }
}