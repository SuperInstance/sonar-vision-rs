//! Sonar/ultrasonic perception pipeline — beam forming, echo detection, spatial mapping.

use std::f64::consts::PI;

/// Speed of sound in air (m/s).
pub const SPEED_OF_SOUND: f64 = 343.0;

/// A detected echo with timing and amplitude.
#[derive(Debug, Clone, Copy)]
pub struct Echo {
    /// Sample index where echo was detected.
    pub sample_idx: usize,
    /// Round-trip time in seconds.
    pub rtt: f64,
    /// Estimated distance in meters.
    pub distance: f64,
    /// Signal amplitude (0.0–1.0).
    pub amplitude: f64,
    /// Confidence of detection.
    pub confidence: f64,
}

impl Echo {
    pub fn from_rtt(rtt: f64, amplitude: f64) -> Self {
        Self {
            sample_idx: 0,
            rtt,
            distance: rtt * SPEED_OF_SOUND / 2.0,
            amplitude,
            confidence: amplitude,
        }
    }
}

/// A point in a sonar spatial map.
#[derive(Debug, Clone, Copy)]
pub struct SpatialPoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub intensity: f64,
}

/// Sonar pulse parameters.
#[derive(Debug, Clone, Copy)]
pub struct PulseConfig {
    pub frequency: f64,   // Hz
    pub duration: f64,    // seconds
    pub sample_rate: f64, // Hz
    pub amplitude: f64,   // 0.0–1.0
}

impl Default for PulseConfig {
    fn default() -> Self {
        Self {
            frequency: 40_000.0,
            duration: 0.001,
            sample_rate: 1_000_000.0,
            amplitude: 1.0,
        }
    }
}

/// Generate a sonar pulse signal.
pub fn generate_pulse(config: &PulseConfig) -> Vec<f64> {
    let n_samples = (config.duration * config.sample_rate) as usize;
    (0..n_samples)
        .map(|i| {
            let t = i as f64 / config.sample_rate;
            config.amplitude * (2.0 * PI * config.frequency * t).sin()
        })
        .collect()
}

/// Detect echoes in a return signal using threshold-based peak detection.
pub fn detect_echoes(
    signal: &[f64],
    sample_rate: f64,
    threshold: f64,
    min_separation: f64,
) -> Vec<Echo> {
    let min_sep_samples = (min_separation * sample_rate) as usize;
    let mut echoes = Vec::new();
    let mut last_peak = 0isize;

    for i in 1..signal.len().saturating_sub(1) {
        if signal[i] > threshold
            && signal[i] > signal[i - 1]
            && signal[i] > signal[i + 1]
            && (last_peak < 0 || (i as isize - last_peak) as usize >= min_sep_samples)
        {
            let rtt = i as f64 / sample_rate;
            echoes.push(Echo {
                sample_idx: i,
                rtt,
                distance: rtt * SPEED_OF_SOUND / 2.0,
                amplitude: signal[i],
                confidence: signal[i],
            });
            last_peak = i as isize;
        }
    }
    echoes
}

/// Simple delay-and-sum beamforming.
pub fn beamform(signals: &[Vec<f64>], delays: &[f64], sample_rate: f64) -> Vec<f64> {
    if signals.is_empty() || delays.len() != signals.len() {
        return vec![];
    }
    let max_len = signals.iter().map(|s| s.len()).max().unwrap_or(0);
    if max_len == 0 {
        return vec![];
    }

    let mut output = vec![0.0f64; max_len];
    let mut counts = vec![0usize; max_len];

    for (signal, &delay) in signals.iter().zip(delays.iter()) {
        let delay_samples = (delay * sample_rate).round() as isize;
        for (i, &val) in signal.iter().enumerate() {
            let out_idx = i as isize + delay_samples;
            if out_idx >= 0 && (out_idx as usize) < max_len {
                output[out_idx as usize] += val;
                counts[out_idx as usize] += 1;
            }
        }
    }

    for i in 0..max_len {
        if counts[i] > 0 {
            output[i] /= counts[i] as f64;
        }
    }
    output
}

/// Convert an echo to a spatial point given sensor angle (radians from forward).
pub fn echo_to_spatial(echo: &Echo, angle: f64) -> SpatialPoint {
    SpatialPoint {
        x: echo.distance * angle.cos(),
        y: echo.distance * angle.sin(),
        z: 0.0,
        intensity: echo.amplitude,
    }
}

/// Build a spatial map from multiple sonar sweeps at different angles.
pub fn build_spatial_map(sweeps: &[(f64, Vec<Echo>)]) -> Vec<SpatialPoint> {
    sweeps
        .iter()
        .flat_map(|(angle, echoes)| echoes.iter().map(|e| echo_to_spatial(e, *angle)))
        .collect()
}

/// Compute signal-to-noise ratio of a signal.
pub fn compute_snr(signal: &[f64]) -> f64 {
    if signal.len() < 2 {
        return 0.0;
    }
    let mean: f64 = signal.iter().sum::<f64>() / signal.len() as f64;
    let signal_power: f64 = signal.iter().map(|x| x.powi(2)).sum::<f64>() / signal.len() as f64;
    let noise_power = (signal_power - mean * mean).max(0.0);
    if noise_power == 0.0 {
        return f64::INFINITY;
    }
    10.0 * (signal_power / noise_power).log10()
}

/// Simple matched filter (cross-correlation with template).
pub fn matched_filter(signal: &[f64], template: &[f64]) -> Vec<f64> {
    if signal.len() < template.len() {
        return vec![];
    }
    let out_len = signal.len() - template.len() + 1;
    (0..out_len)
        .map(|i| {
            signal[i..i + template.len()]
                .iter()
                .zip(template.iter())
                .map(|(s, t)| s * t)
                .sum()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_pulse() {
        let config = PulseConfig::default();
        let pulse = generate_pulse(&config);
        assert!(!pulse.is_empty());
        assert!(pulse.iter().all(|&s| s.abs() <= 1.0));
    }

    #[test]
    fn test_detect_echoes() {
        // Create signal with two echoes
        let mut signal = vec![0.0; 1000];
        signal[100] = 0.8;
        signal[500] = 0.6;
        let echoes = detect_echoes(&signal, 1_000_000.0, 0.5, 0.0001);
        assert_eq!(echoes.len(), 2);
        assert!((echoes[0].distance - 0.01715).abs() < 0.001);
    }

    #[test]
    fn test_echo_from_rtt() {
        let echo = Echo::from_rtt(0.01, 0.9);
        assert!((echo.distance - 1.715).abs() < 0.01);
    }

    #[test]
    fn test_beamform() {
        let signals = vec![vec![1.0, 2.0, 3.0, 0.0], vec![0.0, 1.0, 2.0, 3.0]];
        let delays = vec![0.0, 0.000001];
        let output = beamform(&signals, &delays, 1_000_000.0);
        assert!(!output.is_empty());
    }

    #[test]
    fn test_echo_to_spatial() {
        let echo = Echo::from_rtt(0.01, 0.9);
        let point = echo_to_spatial(&echo, 0.0); // forward
        assert!((point.x - 1.715).abs() < 0.01);
        assert!(point.y.abs() < 0.01);
    }

    #[test]
    fn test_spatial_map() {
        let sweeps = vec![
            (0.0, vec![Echo::from_rtt(0.01, 0.9)]),
            (PI / 4.0, vec![Echo::from_rtt(0.02, 0.7)]),
        ];
        let map = build_spatial_map(&sweeps);
        assert_eq!(map.len(), 2);
    }

    #[test]
    fn test_matched_filter() {
        let template = vec![1.0, 1.0, 1.0];
        let signal = vec![0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0];
        let result = matched_filter(&signal, &template);
        assert_eq!(result.len(), 5);
        assert!((result[2] - 3.0).abs() < 0.001);
    }

    #[test]
    fn test_snr() {
        let signal = vec![1.0, 1.0, 1.0, 1.0];
        let snr = compute_snr(&signal);
        assert!(snr.is_infinite() || snr > 30.0); // pure signal = high SNR
    }
}
