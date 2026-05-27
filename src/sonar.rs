//! Sonar — ping/echo simulation and distance estimation.

use crate::signal::Signal;

/// Default speed of sound in water (m/s).
pub const SOUND_SPEED_WATER: f64 = 1500.0;
/// Default speed of sound in air (m/s).
pub const SOUND_SPEED_AIR: f64 = 343.0;

/// Result of a sonar ping.
#[derive(Debug, Clone)]
pub struct PingResult {
    /// Estimated distance in meters.
    pub distance: f64,
    /// Round-trip time in seconds.
    pub travel_time: f64,
    /// Received signal amplitude (0–1 normalised).
    pub signal_strength: f64,
    /// Signal-to-noise ratio in decibels.
    pub snr_db: f64,
    /// Ping frequency in Hz.
    pub frequency: f64,
}

/// Active sonar model with configurable medium and transducer.
#[derive(Debug, Clone)]
pub struct Sonar {
    /// Speed of sound in the medium (m/s).
    pub sound_speed: f64,
    /// Transducer centre frequency (Hz).
    pub frequency: f64,
    /// Transmit pulse length (seconds).
    pub pulse_duration: f64,
    /// Maximum detectable range (meters).
    pub max_range: f64,
    /// Transducer beam width (degrees).
    pub beam_width: f64,
    /// Transmit source level (arbitrary dB scale).
    pub source_level: f64,
    /// Ambient noise floor (arbitrary dB scale).
    pub noise_level: f64,
}

impl Default for Sonar {
    fn default() -> Self {
        Self {
            sound_speed: SOUND_SPEED_WATER,
            frequency: 50000.0,
            pulse_duration: 0.001,
            max_range: 1000.0,
            beam_width: 30.0,
            source_level: 200.0,
            noise_level: 60.0,
        }
    }
}

impl Sonar {
    /// Create a new sonar with default parameters.
    pub fn new() -> Self {
        Self::default()
    }

    /// Generate a transmit pulse (tone burst).
    pub fn generate_ping(&self, sample_rate: f64) -> Signal {
        Signal::sine(self.frequency, self.pulse_duration, sample_rate, 1.0)
    }

    /// Compute round-trip travel time for a given distance.
    pub fn round_trip_time(&self, distance: f64) -> f64 {
        2.0 * distance / self.sound_speed
    }

    /// Estimate distance from measured round-trip travel time.
    pub fn distance_from_time(&self, travel_time: f64) -> f64 {
        travel_time * self.sound_speed / 2.0
    }

    /// Cylindrical + spherical spreading loss in dB (simplified).
    pub fn spreading_loss(&self, distance: f64) -> f64 {
        if distance <= 0.0 {
            0.0
        } else {
            20.0 * distance.max(1e-12).log10()
        }
    }

    /// Frequency-dependent absorption loss in dB.
    pub fn absorption_loss(&self, distance: f64, absorption_db_km: f64) -> f64 {
        absorption_db_km * distance / 1000.0
    }

    /// Total transmission loss (spreading + absorption) in dB.
    pub fn total_loss(&self, distance: f64, absorption_db_km: f64) -> f64 {
        self.spreading_loss(distance) + self.absorption_loss(distance, absorption_db_km)
    }

    /// Simulate a ping at `target_distance` and return detection result.
    ///
    /// Note: deterministic version without random jitter.
    pub fn ping(&self, target_distance: f64, target_strength: f64) -> PingResult {
        if target_distance <= 0.0 || target_distance > self.max_range {
            return PingResult {
                distance: f64::NAN,
                travel_time: f64::NAN,
                signal_strength: 0.0,
                snr_db: f64::NEG_INFINITY,
                frequency: self.frequency,
            };
        }

        let _rtt = self.round_trip_time(target_distance);
        let loss = self.total_loss(target_distance, 10.0);
        let received_level = self.source_level - loss + target_strength;
        let snr = received_level - self.noise_level;
        let strength = (1.0 / (1.0 + (-(snr - 6.0) / 3.0).exp())).clamp(0.0, 1.0);
        let estimated_distance = target_distance;

        PingResult {
            distance: estimated_distance,
            travel_time: self.round_trip_time(estimated_distance),
            signal_strength: strength,
            snr_db: snr,
            frequency: self.frequency,
        }
    }

    /// Generate a synthetic echo return signal for a target.
    pub fn ping_return_signal(&self, target_distance: f64, target_strength: f64, sample_rate: f64) -> Signal {
        if target_distance <= 0.0 || target_distance > self.max_range {
            return Signal::empty(sample_rate);
        }

        let tx = self.generate_ping(sample_rate);
        let rtt_samples = (self.round_trip_time(target_distance) * sample_rate) as usize;

        let loss_db = self.total_loss(target_distance, 10.0) - target_strength;
        let gain = 10.0_f64.powf(-loss_db / 20.0).clamp(0.0, 1.0);
        let echo_samples: Vec<f64> = tx.samples.iter().map(|s| s * gain).collect();

        let silence_len = rtt_samples.saturating_sub(tx.samples.len());
        let mut combined = tx.samples.clone();
        combined.extend(std::iter::repeat(0.0).take(silence_len));
        combined.extend(echo_samples);

        Signal::new(combined, sample_rate)
    }

    /// Check whether a target at `bearing_offset_deg` from boresight is within the beam.
    pub fn in_beam(&self, bearing_offset_deg: f64) -> bool {
        bearing_offset_deg.abs() <= self.beam_width / 2.0
    }

    /// Return the angular coverage in degrees of the beam.
    pub fn beam_coverage(&self) -> f64 {
        self.beam_width
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_round_trip_time() {
        let sonar = Sonar::new();
        let rtt = sonar.round_trip_time(750.0);
        assert!((rtt - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_distance_from_time() {
        let sonar = Sonar::new();
        let dist = sonar.distance_from_time(1.0);
        assert!((dist - 750.0).abs() < 1e-10);
    }

    #[test]
    fn test_spreading_loss() {
        let sonar = Sonar::new();
        let loss = sonar.spreading_loss(100.0);
        assert!((loss - 40.0).abs() < 1e-10);
    }

    #[test]
    fn test_spreading_loss_zero() {
        let sonar = Sonar::new();
        assert_eq!(sonar.spreading_loss(0.0), 0.0);
    }

    #[test]
    fn test_absorption_loss() {
        let sonar = Sonar::new();
        let loss = sonar.absorption_loss(1000.0, 10.0);
        assert!((loss - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_total_loss() {
        let sonar = Sonar::new();
        let loss = sonar.total_loss(100.0, 10.0);
        let expected = sonar.spreading_loss(100.0) + sonar.absorption_loss(100.0, 10.0);
        assert!((loss - expected).abs() < 1e-10);
    }

    #[test]
    fn test_ping_in_range() {
        let sonar = Sonar::new();
        let result = sonar.ping(100.0, -20.0);
        assert!(!result.distance.is_nan());
        assert!(result.signal_strength > 0.0);
    }

    #[test]
    fn test_ping_out_of_range() {
        let sonar = Sonar::new();
        let result = sonar.ping(0.0, -20.0);
        assert!(result.distance.is_nan());
        assert_eq!(result.signal_strength, 0.0);
    }

    #[test]
    fn test_in_beam() {
        let sonar = Sonar::new();
        assert!(sonar.in_beam(10.0));
        assert!(!sonar.in_beam(20.0));
    }

    #[test]
    fn test_beam_coverage() {
        let sonar = Sonar::new();
        assert!((sonar.beam_coverage() - 30.0).abs() < 1e-10);
    }

    #[test]
    fn test_generate_ping() {
        let sonar = Sonar::new();
        let ping = sonar.generate_ping(44100.0);
        assert!(ping.n_samples() > 0);
    }

    #[test]
    fn test_ping_return_signal() {
        let sonar = Sonar::new();
        let signal = sonar.ping_return_signal(100.0, -20.0, 44100.0);
        assert!(signal.n_samples() > 0);
    }
}
