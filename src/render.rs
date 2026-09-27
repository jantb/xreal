use crate::viewport::ViewportRect;

// The pixel buffer uses a BGRA texture, the same byte order scap captures in,
// so frames are copied without swizzling. Colors below are [B, G, R, A].
const BLACK: [u8; 4] = [0, 0, 0, 255];
const HUD_BACKGROUND: [u8; 4] = [0, 0, 0, 255];
const HUD_TEXT: [u8; 4] = [255, 240, 230, 255];
const OPAQUE: u32 = u32::from_ne_bytes([0, 0, 0, 255]);

const CHAR_WIDTH: usize = 6;
const CHAR_HEIGHT: usize = 8;

pub fn output_height(raw_buffer: &[u8], output_width: usize) -> usize {
    if output_width == 0 {
        0
    } else {
        raw_buffer.len() / (output_width * 4)
    }
}

/// Scales the `viewport` region of a BGRA `frame` into `raw_buffer` with
/// nearest-neighbour sampling. `source_offsets` is scratch space reused
/// between frames to avoid reallocating.
pub fn process_viewport_frame(
    frame: &[u8],
    raw_buffer: &mut [u8],
    output_width: usize,
    source_width: usize,
    source_height: usize,
    viewport: ViewportRect,
    source_offsets: &mut Vec<usize>,
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

    // The source column depends only on the output column, so compute it once
    // per frame instead of once per pixel.
    let max_source_x = source_width.saturating_sub(1) as f32;
    source_offsets.clear();
    source_offsets.extend((0..output_width).map(|x| {
        let source_x = viewport.x + ((x as f32 + 0.5) * viewport.width / output_width as f32);
        source_x.floor().clamp(0.0, max_source_x) as usize * 4
    }));

    let max_source_y = available_source_height.saturating_sub(1) as f32;
    for (y, row) in raw_buffer.chunks_exact_mut(output_stride).enumerate() {
        let source_y = viewport.y + ((y as f32 + 0.5) * viewport.height / output_height as f32);
        let source_y = source_y.floor().clamp(0.0, max_source_y) as usize;
        let source_row = &frame[source_y * source_stride..(source_y + 1) * source_stride];

        for (dest, &offset) in row.chunks_exact_mut(4).zip(source_offsets.iter()) {
            let pixel: [u8; 4] = source_row[offset..offset + 4].try_into().unwrap();
            let pixel = u32::from_ne_bytes(pixel) | OPAQUE;
            dest.copy_from_slice(&pixel.to_ne_bytes());
        }
    }
}

pub fn set_black(buffer: &mut [u8]) {
    for pixel in buffer.chunks_exact_mut(4) {
        pixel.copy_from_slice(&BLACK);
    }
}

pub fn draw_overlay(frame: &mut [u8], output_width: usize, lines: &[String]) {
    let output_height = output_height(frame, output_width);
    if output_width == 0 || output_height == 0 {
        return;
    }

    let overlay_width = lines
        .iter()
        .map(|line| line.len() * CHAR_WIDTH)
        .max()
        .unwrap_or(0)
        + 16;
    let overlay_height = lines.len() * CHAR_HEIGHT + 14;
    fill_rect(
        frame,
        output_width,
        8,
        8,
        overlay_width.min(output_width.saturating_sub(8)),
        overlay_height.min(output_height.saturating_sub(8)),
        HUD_BACKGROUND,
    );

    for (index, line) in lines.iter().enumerate() {
        draw_text(frame, output_width, 16, 16 + index * CHAR_HEIGHT, line, HUD_TEXT);
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
        cursor_x += CHAR_WIDTH;
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
        '%' => [0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03],
        ' ' => [0x00; 7],
        _ => [0x1f, 0x11, 0x02, 0x04, 0x04, 0x00, 0x04],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::hint::black_box;
    use std::time::Instant;

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

    fn pixel(frame: &[u8], width: usize, x: usize, y: usize) -> [u8; 3] {
        let index = (y * width + x) * 4;
        [frame[index], frame[index + 1], frame[index + 2]]
    }

    #[test]
    fn viewport_at_native_scale_copies_the_region_opaque() {
        let (source_width, source_height) = (8, 6);
        let frame = make_bgra_frame(source_width, source_height);
        let (output_width, output_height) = (3, 2);
        let mut output = vec![0; output_width * output_height * 4];

        process_viewport_frame(
            &frame,
            &mut output,
            output_width,
            source_width,
            source_height,
            ViewportRect {
                x: 2.0,
                y: 3.0,
                width: 3.0,
                height: 2.0,
            },
            &mut Vec::new(),
        );

        for y in 0..output_height {
            for x in 0..output_width {
                let index = (y * output_width + x) * 4;
                assert_eq!(
                    [output[index], output[index + 1], output[index + 2]],
                    pixel(&frame, source_width, x + 2, y + 3)
                );
                assert_eq!(output[index + 3], 255);
            }
        }
    }

    #[test]
    fn viewport_zoomed_out_samples_across_the_region() {
        let (source_width, source_height) = (4, 2);
        let frame = make_bgra_frame(source_width, source_height);
        let mut output = vec![0; 2 * 4];

        process_viewport_frame(
            &frame,
            &mut output,
            2,
            source_width,
            source_height,
            ViewportRect {
                x: 0.0,
                y: 0.0,
                width: 4.0,
                height: 2.0,
            },
            &mut Vec::new(),
        );

        assert_eq!(&output[0..3], &pixel(&frame, source_width, 1, 1));
        assert_eq!(&output[4..7], &pixel(&frame, source_width, 3, 1));
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
        let mut source_offsets = Vec::new();

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
                &mut source_offsets,
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
}
