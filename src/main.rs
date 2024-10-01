mod ar_drivers {
    pub mod lib;
}

use ar_drivers::lib::{any_glasses, GlassesEvent};
use std::sync::{Arc, Mutex};

use dcmimu::DCMIMU;
use std::time::Duration;
use std::{thread, time};
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, StartCause, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::{Window, WindowId, WindowLevel};

use pixels::{Pixels, SurfaceTexture};
use scap::capturer::{Capturer, Options};
use scap::frame::{BGRAFrame, Frame};

struct SharedGlassesStore {
    dcmimu: Arc<Mutex<DCMIMU>>,
}

fn create_glasses_thread(store: &SharedGlassesStore) {
    let shared_dcmimu_clone = Arc::clone(&store.dcmimu);
    thread::spawn(move || {
        let mut glasses = match any_glasses() {
            Ok(glasses) => glasses,
            Err(_) => return, // Exit if unable to acquire glasses
        };
        let mut last_timestamp: Option<u64> = None;

        loop {
            match glasses.read_event() {
                Ok(GlassesEvent::AccGyro {
                       accelerometer,
                       gyroscope,
                       timestamp
                   }) => {
                    if let Some(last_timestamp) = last_timestamp {
                        let dt = (timestamp - last_timestamp) as f32 / 1_000_000.0; // in seconds
                        {
                            let mut dcmimu = shared_dcmimu_clone.lock().unwrap();
                            dcmimu.update(
                                (gyroscope.x, gyroscope.y, gyroscope.z),
                                (accelerometer.x, accelerometer.y, accelerometer.z),
                                // (0., 0., 0.), // set accel to 0 to disable prediction
                                dt,
                            );
                        }
                    }
                    last_timestamp = Some(timestamp);
                }
                _ => thread::sleep(Duration::from_millis(10)), // Sleep to avoid busy waiting
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
        ).with_resizable(false)
            .with_inner_size(winit::dpi::LogicalSize::new(1920.0, 1080.0))
            ;
        let window = event_loop.create_window(window_attributes).unwrap();
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
                    ((dimension * ((-angle + offset) + 1.0) * multiplier) as isize)
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
                            self.pred = Some(data.clone());  // Only clone if necessary
                            process_frame_serial(
                                &data.data,
                                frame,
                                1920,
                                self.screen_width,
                                current_x_offset,
                                current_y_offset,
                            );
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