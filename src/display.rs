//! SonarDisplay — ASCII radar rendering for sonar data.

use crate::map::{CellState, SpatialMap};
use crate::tracker::ObjectTracker;

const RAMP: &[u8] = b" .:-=+*#%@";
const RAMP_LEN: usize = 10;

/// ASCII-based sonar radar display.
pub struct SonarDisplay {
    pub width: usize,
    pub height: usize,
    pub range_m: f64,
}

impl Default for SonarDisplay {
    fn default() -> Self {
        Self { width: 41, height: 21, range_m: 50.0 }
    }
}

impl SonarDisplay {
    pub fn new(width: usize, height: usize, range_m: f64) -> Self {
        Self { width, height, range_m }
    }

    /// Render a radar sweep from bearing/distance readings.
    pub fn render_sweep(&self, bearing_angles: &[f64], distances: &[f64], max_range: Option<f64>) -> String {
        let rng = max_range.unwrap_or(self.range_m);
        let mut grid = vec![vec![b' '; self.width]; self.height];
        let cx = self.width / 2;
        let cy = self.height / 2;

        // Draw crosshairs
        for x in 0..self.width {
            if grid[cy][x] == b' ' { grid[cy][x] = b'.'; }
        }
        for y in 0..self.height {
            if grid[y][cx] == b' ' { grid[y][cx] = b'.'; }
        }

        // Range rings
        for &frac in &[0.25, 0.5, 0.75] {
            let r_cells = (frac * cx.min(cy) as f64) as usize;
            for angle_deg in (0..360).step_by(5) {
                let ar = (angle_deg as f64).to_radians();
                let px = cx as isize + (r_cells as f64 * ar.cos()) as isize;
                let py = cy as isize - (r_cells as f64 * ar.sin()) as isize;
                if px >= 0 && px < self.width as isize && py >= 0 && py < self.height as isize {
                    grid[py as usize][px as usize] = b'.';
                }
            }
        }

        // Center marker
        grid[cy][cx] = b'+';

        // Plot detections
        for (bearing, dist) in bearing_angles.iter().zip(distances.iter()) {
            if *dist > rng || *dist <= 0.0 { continue; }
            let ar = bearing.to_radians();
            let r_norm = dist / rng;
            let px = cx as isize + (r_norm * cx as f64 * ar.cos()) as isize;
            let py = cy as isize - (r_norm * cy as f64 * ar.sin()) as isize;
            if px >= 0 && px < self.width as isize && py >= 0 && py < self.height as isize {
                let intensity = ((1.0 - r_norm) * (RAMP_LEN - 1) as f64) as usize;
                let intensity = intensity.min(RAMP_LEN - 1);
                grid[py as usize][px as usize] = RAMP[intensity];
            }
        }

        grid.iter()
            .map(|row| String::from_utf8_lossy(row).into_owned())
            .collect::<Vec<_>>()
            .join("\n")
    }

    /// Render a SpatialMap as ASCII.
    pub fn render_map(&self, smap: &SpatialMap) -> String {
        let symbols = |s: CellState| -> char {
            match s {
                CellState::Unknown => ' ',
                CellState::Free => '.',
                CellState::Occupied => '#',
            }
        };

        let row_scale = smap.rows() as f64 / self.height as f64;
        let col_scale = smap.cols() as f64 / self.width as f64;

        let mut lines = Vec::new();
        for dr in 0..self.height {
            let row_start = (dr as f64 * row_scale) as usize;
            let row_end = ((dr + 1) as f64 * row_scale).min(smap.rows() as f64) as usize;
            let mut line_chars = Vec::new();
            for dc in 0..self.width {
                let col_start = (dc as f64 * col_scale) as usize;
                let col_end = ((dc + 1) as f64 * col_scale).min(smap.cols() as f64) as usize;
                let mut cell = CellState::Unknown;
                for r in row_start..row_end.min(smap.rows()) {
                    for c in col_start..col_end.min(smap.cols()) {
                        let s = smap.get_cell(
                            (c as f64 + 0.5) * smap.resolution - smap.width / 2.0,
                            (r as f64 + 0.5) * smap.resolution - smap.height / 2.0,
                        );
                        if s == CellState::Occupied {
                            cell = s;
                        } else if s == CellState::Free && cell == CellState::Unknown {
                            cell = s;
                        }
                    }
                }
                line_chars.push(symbols(cell));
            }
            lines.push(line_chars.into_iter().collect::<String>());
        }
        lines.join("\n")
    }

    /// Render active tracks relative to a centre point.
    pub fn render_tracks(&self, tracker: &ObjectTracker, cx_m: f64, cy_m: f64, now: f64) -> String {
        let mut grid = vec![vec![b' '; self.width]; self.height];
        let cx = self.width / 2;
        let cy = self.height / 2;

        // Crosshairs
        for x in 0..self.width { grid[cy][x] = b'.'; }
        for y in 0..self.height { grid[y][cx] = b'.'; }
        grid[cy][cx] = b'+';

        let active = tracker.active_tracks(now);
        for t in &active {
            let dx = t.x - cx_m;
            let dy = t.y - cy_m;
            let px = cx as isize + (dx / self.range_m * cx as f64) as isize;
            let py = cy as isize - (dy / self.range_m * cy as f64) as isize;
            if px >= 0 && px < self.width as isize && py >= 0 && py < self.height as isize {
                let marker = if t.track_id < 10 {
                    (b'0' + t.track_id as u8) as char
                } else {
                    '*'
                };
                grid[py as usize][px as usize] = marker as u8;
            }
        }

        grid.iter()
            .map(|row| String::from_utf8_lossy(row).into_owned())
            .collect::<Vec<_>>()
            .join("\n")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_render_sweep() {
        let display = SonarDisplay::default();
        let result = display.render_sweep(&[0.0, 90.0], &[10.0, 20.0], None);
        assert!(result.contains('+'));
        assert!(!result.is_empty());
    }

    #[test]
    fn test_render_sweep_with_detections() {
        let display = SonarDisplay::default();
        let result = display.render_sweep(&[0.0], &[25.0], None);
        assert!(!result.is_empty());
    }

    #[test]
    fn test_render_map() {
        let mut map = SpatialMap::new(100.0, 100.0, 1.0);
        map.add_obstacle(crate::map::Obstacle::new(10.0, 10.0));
        let display = SonarDisplay::new(20, 10, 50.0);
        let result = display.render_map(&map);
        assert!(!result.is_empty());
    }

    #[test]
    fn test_render_tracks() {
        let mut tracker = ObjectTracker::new();
        let det = crate::tracker::Detection {
            x: 5.0, y: 5.0, timestamp: 100.0, confidence: 1.0, label: String::new(),
        };
        tracker.update(&[det]);
        let display = SonarDisplay::default();
        let result = display.render_tracks(&tracker, 0.0, 0.0, 100.0);
        assert!(result.contains('+'));
    }
}
