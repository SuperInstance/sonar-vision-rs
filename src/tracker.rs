//! ObjectTracker — multi-object tracking with motion prediction.

use std::collections::HashMap;

/// A tracked object.
#[derive(Debug, Clone)]
pub struct Track {
    pub track_id: usize,
    pub x: f64,
    pub y: f64,
    pub vx: f64,
    pub vy: f64,
    pub last_seen: f64,
    pub detections: usize,
    pub label: String,
}

/// A single detection observation.
#[derive(Debug, Clone)]
pub struct Detection {
    pub x: f64,
    pub y: f64,
    pub timestamp: f64,
    pub confidence: f64,
    pub label: String,
}

/// Simple multi-object tracker with nearest-neighbour association.
pub struct ObjectTracker {
    pub association_gate: f64,
    pub max_velocity: f64,
    pub lost_timeout: f64,
    next_id: usize,
    tracks: HashMap<usize, Track>,
}

impl Default for ObjectTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl ObjectTracker {
    pub fn new() -> Self {
        Self {
            association_gate: 5.0,
            max_velocity: 10.0,
            lost_timeout: 5.0,
            next_id: 1,
            tracks: HashMap::new(),
        }
    }

    /// Return all active tracks.
    pub fn tracks(&self) -> Vec<&Track> {
        self.tracks.values().collect()
    }

    /// Get a specific track by ID.
    pub fn get_track(&self, track_id: usize) -> Option<&Track> {
        self.tracks.get(&track_id)
    }

    /// Return tracks that have been seen within `lost_timeout`.
    pub fn active_tracks(&self, now: f64) -> Vec<&Track> {
        self.tracks.values().filter(|t| (now - t.last_seen) <= self.lost_timeout).collect()
    }

    /// Return tracks that have timed out.
    pub fn lost_tracks(&self, now: f64) -> Vec<&Track> {
        self.tracks.values().filter(|t| (now - t.last_seen) > self.lost_timeout).collect()
    }

    /// Remove all lost tracks. Returns count removed.
    pub fn prune_lost(&mut self, now: f64) -> usize {
        let lost: Vec<usize> = self.tracks.iter()
            .filter(|(_, t)| (now - t.last_seen) > self.lost_timeout)
            .map(|(id, _)| *id)
            .collect();
        let count = lost.len();
        for id in lost {
            self.tracks.remove(&id);
        }
        count
    }

    /// Process a batch of detections. Returns track IDs that were updated or created.
    pub fn update(&mut self, detections: &[Detection]) -> Vec<usize> {
        let mut updated_ids = Vec::new();
        let mut used_tracks: Vec<usize> = Vec::new();

        // Sort by confidence descending
        let mut sorted_dets: Vec<&Detection> = detections.iter().collect();
        sorted_dets.sort_by(|a, b| b.confidence.partial_cmp(&a.confidence).unwrap());

        for det in &sorted_dets {
            let mut best_id: Option<usize> = None;
            let mut best_dist = self.association_gate;

            for (&tid, t) in &self.tracks {
                if used_tracks.contains(&tid) { continue; }
                let dt = det.timestamp - t.last_seen;
                let px = t.x + t.vx * dt;
                let py = t.y + t.vy * dt;
                let d = ((px - det.x).powi(2) + (py - det.y).powi(2)).sqrt();
                if d < best_dist {
                    best_dist = d;
                    best_id = Some(tid);
                }
            }

            if let Some(tid) = best_id {
                self.update_track(tid, det);
                used_tracks.push(tid);
                updated_ids.push(tid);
            } else {
                let tid = self.create_track(det);
                updated_ids.push(tid);
            }
        }

        updated_ids
    }

    fn create_track(&mut self, det: &Detection) -> usize {
        let tid = self.next_id;
        self.next_id += 1;
        self.tracks.insert(tid, Track {
            track_id: tid,
            x: det.x,
            y: det.y,
            vx: 0.0,
            vy: 0.0,
            last_seen: det.timestamp,
            detections: 1,
            label: det.label.clone(),
        });
        tid
    }

    fn update_track(&mut self, track_id: usize, det: &Detection) {
        if let Some(t) = self.tracks.get_mut(&track_id) {
            let dt = det.timestamp - t.last_seen;
            if dt > 0.0 {
                let new_vx = (det.x - t.x) / dt;
                let new_vy = (det.y - t.y) / dt;
                let speed = (new_vx.powi(2) + new_vy.powi(2)).sqrt();
                let (nvx, nvy) = if speed > self.max_velocity {
                    let scale = self.max_velocity / speed;
                    (new_vx * scale, new_vy * scale)
                } else {
                    (new_vx, new_vy)
                };
                let alpha = 0.5;
                t.vx = alpha * nvx + (1.0 - alpha) * t.vx;
                t.vy = alpha * nvy + (1.0 - alpha) * t.vy;
            }
            t.x = det.x;
            t.y = det.y;
            t.last_seen = det.timestamp;
            t.detections += 1;
        }
    }

    /// Predict position of track `track_id` `dt` seconds into the future.
    pub fn predict(&self, track_id: usize, dt: f64) -> Option<(f64, f64)> {
        self.tracks.get(&track_id).map(|t| (t.x + t.vx * dt, t.y + t.vy * dt))
    }

    /// Predict positions of all tracks `dt` seconds ahead.
    pub fn predict_all(&self, dt: f64) -> HashMap<usize, (f64, f64)> {
        self.tracks.iter()
            .map(|(&id, t)| (id, (t.x + t.vx * dt, t.y + t.vy * dt)))
            .collect()
    }

    /// Return the nearest active track to (x, y).
    pub fn nearest_track(&self, x: f64, y: f64, now: f64) -> Option<&Track> {
        self.active_tracks(now).into_iter().min_by(|a, b| {
            let da = ((a.x - x).powi(2) + (a.y - y).powi(2)).sqrt();
            let db = ((b.x - x).powi(2) + (b.y - y).powi(2)).sqrt();
            da.partial_cmp(&db).unwrap()
        })
    }

    /// Return active tracks within `radius` meters of (x, y).
    pub fn tracks_in_radius(&self, x: f64, y: f64, radius: f64, now: f64) -> Vec<&Track> {
        self.active_tracks(now).into_iter().filter(|t| {
            ((t.x - x).powi(2) + (t.y - y).powi(2)).sqrt() <= radius
        }).collect()
    }

    /// Speed of a track in m/s.
    pub fn speed(&self, track_id: usize) -> f64 {
        self.tracks.get(&track_id).map_or(0.0, |t| (t.vx.powi(2) + t.vy.powi(2)).sqrt())
    }

    /// Heading of a track in degrees (0 = east, 90 = north).
    pub fn heading(&self, track_id: usize) -> f64 {
        self.tracks.get(&track_id).map_or(0.0, |t| {
            if t.vx == 0.0 && t.vy == 0.0 { return 0.0; }
            t.vy.atan2(t.vx).to_degrees().rem_euclid(360.0)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_track() {
        let mut tracker = ObjectTracker::new();
        let det = Detection { x: 1.0, y: 2.0, timestamp: 100.0, confidence: 1.0, label: String::new() };
        let ids = tracker.update(&[det]);
        assert_eq!(ids.len(), 1);
        assert_eq!(tracker.tracks().len(), 1);
    }

    #[test]
    fn test_track_update() {
        let mut tracker = ObjectTracker::new();
        let d1 = Detection { x: 0.0, y: 0.0, timestamp: 100.0, confidence: 1.0, label: String::new() };
        tracker.update(&[d1]);
        let d2 = Detection { x: 1.0, y: 0.0, timestamp: 101.0, confidence: 1.0, label: String::new() };
        tracker.update(&[d2]);
        let track = tracker.get_track(1).unwrap();
        assert!((track.x - 1.0).abs() < 1e-10);
        assert!(track.vx > 0.0);
    }

    #[test]
    fn test_predict() {
        let mut tracker = ObjectTracker::new();
        tracker.association_gate = 20.0;
        let d1 = Detection { x: 0.0, y: 0.0, timestamp: 100.0, confidence: 1.0, label: String::new() };
        tracker.update(&[d1]);
        let d2 = Detection { x: 10.0, y: 0.0, timestamp: 101.0, confidence: 1.0, label: String::new() };
        tracker.update(&[d2]);
        // After two detections, velocity is smoothed: 0.5 * 10 + 0.5 * 0 = 5
        let track = tracker.get_track(1).unwrap();
        assert!(track.vx > 0.0);
        let pos = tracker.predict(1, 1.0).unwrap();
        assert!(pos.0 > 10.0);
    }

    #[test]
    fn test_active_and_lost() {
        let mut tracker = ObjectTracker::new();
        let det = Detection { x: 0.0, y: 0.0, timestamp: 100.0, confidence: 1.0, label: String::new() };
        tracker.update(&[det]);
        assert_eq!(tracker.active_tracks(100.0).len(), 1);
        assert_eq!(tracker.lost_tracks(110.0).len(), 1);
    }

    #[test]
    fn test_speed_and_heading() {
        let mut tracker = ObjectTracker::new();
        tracker.association_gate = 20.0; // large enough to associate
        let d1 = Detection { x: 0.0, y: 0.0, timestamp: 100.0, confidence: 1.0, label: String::new() };
        tracker.update(&[d1]);
        let d2 = Detection { x: 10.0, y: 0.0, timestamp: 101.0, confidence: 1.0, label: String::new() };
        tracker.update(&[d2]);
        let speed = tracker.speed(1);
        assert!(speed > 0.0);
        let heading = tracker.heading(1);
        assert!((heading - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_prune_lost() {
        let mut tracker = ObjectTracker::new();
        let det = Detection { x: 0.0, y: 0.0, timestamp: 100.0, confidence: 1.0, label: String::new() };
        tracker.update(&[det]);
        let pruned = tracker.prune_lost(110.0);
        assert_eq!(pruned, 1);
        assert_eq!(tracker.tracks().len(), 0);
    }

    #[test]
    fn test_tracks_in_radius() {
        let mut tracker = ObjectTracker::new();
        let d1 = Detection { x: 0.0, y: 0.0, timestamp: 100.0, confidence: 1.0, label: String::new() };
        let d2 = Detection { x: 50.0, y: 50.0, timestamp: 100.0, confidence: 1.0, label: String::new() };
        tracker.update(&[d1, d2]);
        let in_range = tracker.tracks_in_radius(0.0, 0.0, 10.0, 100.0);
        assert_eq!(in_range.len(), 1);
    }
}
