use std::fs;
use std::io;
use std::path::PathBuf;

/// User-tunable settings, persisted between runs as `key=value` lines.
#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    pub zoom_index: usize,
    pub sensitivity: f32,
    pub deadzone_index: usize,
    pub prediction: bool,
    pub overlay_visible: bool,
    pub gyro_bias: [f32; 3],
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            zoom_index: 2,
            sensitivity: 1.0,
            deadzone_index: 3,
            prediction: true,
            overlay_visible: true,
            gyro_bias: [0.0; 3],
        }
    }
}

impl Settings {
    pub fn path() -> Option<PathBuf> {
        let home = std::env::var_os("HOME")?;
        Some(PathBuf::from(home).join("Library/Application Support/xreal/settings.txt"))
    }

    pub fn load() -> Self {
        Self::path()
            .and_then(|path| fs::read_to_string(path).ok())
            .map(|text| Self::parse(&text))
            .unwrap_or_default()
    }

    pub fn save(&self) -> io::Result<()> {
        let Some(path) = Self::path() else {
            return Ok(());
        };
        if let Some(dir) = path.parent() {
            fs::create_dir_all(dir)?;
        }
        fs::write(path, self.serialize())
    }

    /// Unknown keys and malformed values fall back to the defaults.
    pub fn parse(text: &str) -> Self {
        let mut settings = Self::default();
        for line in text.lines() {
            let Some((key, value)) = line.split_once('=') else {
                continue;
            };
            let value = value.trim();
            match key.trim() {
                "zoom_index" => parse_into(value, &mut settings.zoom_index),
                "sensitivity" => parse_into(value, &mut settings.sensitivity),
                "deadzone_index" => parse_into(value, &mut settings.deadzone_index),
                "prediction" => parse_into(value, &mut settings.prediction),
                "overlay_visible" => parse_into(value, &mut settings.overlay_visible),
                "gyro_bias_x" => parse_into(value, &mut settings.gyro_bias[0]),
                "gyro_bias_y" => parse_into(value, &mut settings.gyro_bias[1]),
                "gyro_bias_z" => parse_into(value, &mut settings.gyro_bias[2]),
                _ => {}
            }
        }
        settings
    }

    pub fn serialize(&self) -> String {
        format!(
            "zoom_index={}\nsensitivity={}\ndeadzone_index={}\nprediction={}\noverlay_visible={}\ngyro_bias_x={}\ngyro_bias_y={}\ngyro_bias_z={}\n",
            self.zoom_index,
            self.sensitivity,
            self.deadzone_index,
            self.prediction,
            self.overlay_visible,
            self.gyro_bias[0],
            self.gyro_bias[1],
            self.gyro_bias[2],
        )
    }
}

fn parse_into<T: std::str::FromStr>(value: &str, target: &mut T) {
    if let Ok(parsed) = value.parse() {
        *target = parsed;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn saved_settings_load_back_unchanged() {
        let settings = Settings {
            zoom_index: 4,
            sensitivity: 0.85,
            deadzone_index: 0,
            prediction: false,
            overlay_visible: false,
            gyro_bias: [0.0012, -0.0034, 0.00056],
        };
        assert_eq!(Settings::parse(&settings.serialize()), settings);
    }

    #[test]
    fn malformed_values_fall_back_to_defaults() {
        let settings = Settings::parse("zoom_index=banana\nsensitivity=1.2\nnonsense\n");
        assert_eq!(settings.zoom_index, Settings::default().zoom_index);
        assert_eq!(settings.sensitivity, 1.2);
    }
}
