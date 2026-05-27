//! SpatialMap — obstacle detection, mapping, and spatial queries.


/// State of a grid cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CellState {
    Unknown,
    Free,
    Occupied,
}

/// Detected obstacle.
#[derive(Debug, Clone)]
pub struct Obstacle {
    pub x: f64,
    pub y: f64,
    pub radius: f64,
    pub confidence: f64,
    pub label: String,
}

impl Obstacle {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y, radius: 0.0, confidence: 1.0, label: String::new() }
    }

    pub fn with_radius(mut self, radius: f64) -> Self {
        self.radius = radius;
        self
    }

    pub fn with_label(mut self, label: &str) -> Self {
        self.label = label.to_string();
        self
    }
}

/// 2D occupancy grid map with obstacle management.
pub struct SpatialMap {
    pub width: f64,
    pub height: f64,
    pub resolution: f64,
    grid: Vec<Vec<CellState>>,
    obstacles: Vec<Obstacle>,
}

impl SpatialMap {
    pub fn new(width: f64, height: f64, resolution: f64) -> Self {
        let cols = (width / resolution).max(1.0) as usize;
        let rows = (height / resolution).max(1.0) as usize;
        let grid = vec![vec![CellState::Unknown; cols]; rows];
        Self { width, height, resolution, grid, obstacles: Vec::new() }
    }

    pub fn cols(&self) -> usize {
        self.grid.first().map_or(0, |r| r.len())
    }

    pub fn rows(&self) -> usize {
        self.grid.len()
    }

    fn world_to_cell(&self, x: f64, y: f64) -> (usize, usize) {
        let col = ((x + self.width / 2.0) / self.resolution) as usize;
        let row = ((y + self.height / 2.0) / self.resolution) as usize;
        (row, col)
    }

    fn cell_to_world(&self, row: usize, col: usize) -> (f64, f64) {
        let x = (col as f64 + 0.5) * self.resolution - self.width / 2.0;
        let y = (row as f64 + 0.5) * self.resolution - self.height / 2.0;
        (x, y)
    }

    fn in_bounds(&self, row: usize, col: usize) -> bool {
        row < self.rows() && col < self.cols()
    }

    /// Get the state of the cell at world coordinates (x, y).
    pub fn get_cell(&self, x: f64, y: f64) -> CellState {
        let (row, col) = self.world_to_cell(x, y);
        if self.in_bounds(row, col) {
            self.grid[row][col]
        } else {
            CellState::Unknown
        }
    }

    /// Set the state of the cell at world coordinates (x, y).
    pub fn set_cell(&mut self, x: f64, y: f64, state: CellState) {
        let (row, col) = self.world_to_cell(x, y);
        if self.in_bounds(row, col) {
            self.grid[row][col] = state;
        }
    }

    /// Add an obstacle and mark its cells as occupied.
    pub fn add_obstacle(&mut self, obstacle: Obstacle) {
        let (center_row, center_col) = self.world_to_cell(obstacle.x, obstacle.y);
        if self.in_bounds(center_row, center_col) {
            self.grid[center_row][center_col] = CellState::Occupied;
        }
        // Iterate over bounding box
        let r_cells = if obstacle.radius > 0.0 {
            (obstacle.radius / self.resolution).ceil() as usize
        } else {
            1
        };
        let half = r_cells as i32;
        for dr in -half..=half {
            for dc in -half..=half {
                let nr = (center_row as i32 + dr) as usize;
                let nc = (center_col as i32 + dc) as usize;
                if self.in_bounds(nr, nc) {
                    let (wx, wy) = self.cell_to_world(nr, nc);
                    let dist = ((wx - obstacle.x).powi(2) + (wy - obstacle.y).powi(2)).sqrt();
                    if obstacle.radius <= 0.0 || dist <= obstacle.radius {
                        self.grid[nr][nc] = CellState::Occupied;
                    }
                }
            }
        }
        self.obstacles.push(obstacle);
    }

    /// Remove obstacle by index and clear its cells.
    pub fn remove_obstacle(&mut self, index: usize) -> Option<Obstacle> {
        if index < self.obstacles.len() {
            let obs = self.obstacles.remove(index);
            let (center_row, center_col) = self.world_to_cell(obs.x, obs.y);
            let r_cells = if obs.radius > 0.0 {
                (obs.radius / self.resolution).ceil() as usize
            } else {
                1
            };
            let half = r_cells as i32;
            for dr in -half..=half {
                for dc in -half..=half {
                    let nr = (center_row as i32 + dr) as usize;
                    let nc = (center_col as i32 + dc) as usize;
                    if self.in_bounds(nr, nc) {
                        let (wx, wy) = self.cell_to_world(nr, nc);
                        let dist = ((wx - obs.x).powi(2) + (wy - obs.y).powi(2)).sqrt();
                        if obs.radius <= 0.0 || dist <= obs.radius {
                            self.grid[nr][nc] = CellState::Free;
                        }
                    }
                }
            }
            Some(obs)
        } else {
            None
        }
    }

    /// Get all obstacles.
    pub fn obstacles(&self) -> &[Obstacle] {
        &self.obstacles
    }

    /// Clear all obstacles and reset grid.
    pub fn clear(&mut self) {
        self.obstacles.clear();
        for row in &mut self.grid {
            for cell in row.iter_mut() {
                *cell = CellState::Unknown;
            }
        }
    }

    /// Check if position (x, y) is occupied.
    pub fn is_occupied(&self, x: f64, y: f64) -> bool {
        self.get_cell(x, y) == CellState::Occupied
    }

    /// Check if position (x, y) is known free.
    pub fn is_free(&self, x: f64, y: f64) -> bool {
        self.get_cell(x, y) == CellState::Free
    }

    /// Return the nearest obstacle to (x, y), or None.
    pub fn nearest_obstacle(&self, x: f64, y: f64) -> Option<&Obstacle> {
        self.obstacles.iter().min_by(|a, b| {
            let da = ((a.x - x).powi(2) + (a.y - y).powi(2)).sqrt();
            let db = ((b.x - x).powi(2) + (b.y - y).powi(2)).sqrt();
            da.partial_cmp(&db).unwrap()
        })
    }

    /// Return obstacles within `radius` meters of (x, y).
    pub fn obstacles_in_radius(&self, x: f64, y: f64, radius: f64) -> Vec<&Obstacle> {
        self.obstacles.iter().filter(|o| {
            ((o.x - x).powi(2) + (o.y - y).powi(2)).sqrt() <= radius
        }).collect()
    }

    /// Distance to the nearest occupied cell or obstacle.
    pub fn distance_to_nearest(&self, x: f64, y: f64) -> f64 {
        self.nearest_obstacle(x, y).map_or(f64::INFINITY, |o| {
            ((o.x - x).powi(2) + (o.y - y).powi(2)).sqrt()
        })
    }

    /// Cast a ray from (ox, oy) at `angle_deg` and return first hit (x, y).
    pub fn ray_cast(&self, ox: f64, oy: f64, angle_deg: f64, max_dist: f64) -> Option<(f64, f64)> {
        let angle_rad = angle_deg.to_radians();
        let dx = angle_rad.cos() * self.resolution * 0.5;
        let dy = angle_rad.sin() * self.resolution * 0.5;
        let steps = (max_dist / (self.resolution * 0.5)) as usize;
        let mut cx = ox;
        let mut cy = oy;
        for _ in 0..steps {
            if self.is_occupied(cx, cy) {
                return Some((cx, cy));
            }
            let (row, col) = self.world_to_cell(cx, cy);
            if !self.in_bounds(row, col) {
                return None;
            }
            cx += dx;
            cy += dy;
        }
        None
    }

    /// Mark cells along a ray as FREE up to `distance` meters.
    pub fn mark_free_ray(&mut self, ox: f64, oy: f64, angle_deg: f64, distance: f64) {
        let angle_rad = angle_deg.to_radians();
        let steps = (distance / self.resolution).max(1.0) as usize;
        for i in 0..steps {
            let t = (i as f64 + 0.5) * self.resolution;
            if t > distance { break; }
            let wx = ox + t * angle_rad.cos();
            let wy = oy + t * angle_rad.sin();
            if self.get_cell(wx, wy) != CellState::Occupied {
                self.set_cell(wx, wy, CellState::Free);
            }
        }
    }

    /// Fraction of map that is explored (FREE + OCCUPIED).
    pub fn coverage(&self) -> f64 {
        let total = self.rows() * self.cols();
        if total == 0 { return 0.0; }
        let explored: usize = self.grid.iter()
            .flat_map(|row| row.iter())
            .filter(|&&c| c == CellState::Free || c == CellState::Occupied)
            .count();
        explored as f64 / total as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_map() {
        let map = SpatialMap::new(100.0, 100.0, 1.0);
        assert_eq!(map.cols(), 100);
        assert_eq!(map.rows(), 100);
    }

    #[test]
    fn test_add_obstacle() {
        let mut map = SpatialMap::new(100.0, 100.0, 1.0);
        map.add_obstacle(Obstacle::new(10.0, 10.0));
        assert!(map.is_occupied(10.0, 10.0));
        assert_eq!(map.obstacles().len(), 1);
    }

    #[test]
    fn test_remove_obstacle() {
        let mut map = SpatialMap::new(100.0, 100.0, 1.0);
        map.add_obstacle(Obstacle::new(10.0, 10.0));
        let obs = map.remove_obstacle(0);
        assert!(obs.is_some());
        assert!(map.is_free(10.0, 10.0));
    }

    #[test]
    fn test_nearest_obstacle() {
        let mut map = SpatialMap::new(100.0, 100.0, 1.0);
        map.add_obstacle(Obstacle::new(5.0, 5.0));
        map.add_obstacle(Obstacle::new(20.0, 20.0));
        let nearest = map.nearest_obstacle(6.0, 6.0).unwrap();
        assert!((nearest.x - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_obstacles_in_radius() {
        let mut map = SpatialMap::new(100.0, 100.0, 1.0);
        map.add_obstacle(Obstacle::new(5.0, 5.0));
        map.add_obstacle(Obstacle::new(20.0, 20.0));
        let in_range = map.obstacles_in_radius(5.0, 5.0, 10.0);
        assert_eq!(in_range.len(), 1);
    }

    #[test]
    fn test_distance_to_nearest() {
        let mut map = SpatialMap::new(100.0, 100.0, 1.0);
        map.add_obstacle(Obstacle::new(5.0, 5.0));
        let dist = map.distance_to_nearest(8.0, 5.0);
        assert!((dist - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_ray_cast() {
        let mut map = SpatialMap::new(100.0, 100.0, 1.0);
        map.add_obstacle(Obstacle::new(10.0, 0.0));
        let hit = map.ray_cast(0.0, 0.0, 0.0, 50.0);
        assert!(hit.is_some());
        let (hx, _hy) = hit.unwrap();
        assert!(hx > 0.0 && hx <= 15.0);
    }

    #[test]
    fn test_coverage() {
        let mut map = SpatialMap::new(10.0, 10.0, 1.0);
        assert!((map.coverage() - 0.0).abs() < 1e-10);
        map.add_obstacle(Obstacle::new(0.0, 0.0));
        assert!(map.coverage() > 0.0);
    }

    #[test]
    fn test_mark_free_ray() {
        let mut map = SpatialMap::new(100.0, 100.0, 1.0);
        map.mark_free_ray(0.0, 0.0, 0.0, 10.0);
        assert!(map.is_free(5.0, 0.0));
    }

    #[test]
    fn test_clear() {
        let mut map = SpatialMap::new(100.0, 100.0, 1.0);
        map.add_obstacle(Obstacle::new(5.0, 5.0));
        map.clear();
        assert!(map.obstacles().is_empty());
        assert!(!map.is_occupied(5.0, 5.0));
    }
}
