//! Discrete-time signal processing: sampling, filtering, DFT, and analysis.

use std::f64::consts::PI;

/// A discrete-time signal with uniform sampling.
#[derive(Debug, Clone)]
pub struct Signal {
    /// Amplitude values.
    pub samples: Vec<f64>,
    /// Samples per second (Hz).
    pub sample_rate: f64,
}

impl Signal {
    /// Create a new signal with the given samples and sample rate.
    pub fn new(samples: Vec<f64>, sample_rate: f64) -> Self {
        Self { samples, sample_rate }
    }

    /// Create an empty signal.
    pub fn empty(sample_rate: f64) -> Self {
        Self { samples: Vec::new(), sample_rate }
    }

    /// Generate a sine-wave signal.
    pub fn sine(frequency: f64, duration: f64, sample_rate: f64, amplitude: f64) -> Self {
        let n = (sample_rate * duration) as usize;
        let samples: Vec<f64> = (0..n)
            .map(|i| amplitude * (2.0 * PI * frequency * i as f64 / sample_rate).sin())
            .collect();
        Self { samples, sample_rate }
    }

    /// Generate uniform random noise using a simple LCG PRNG (seeded).
    pub fn noise(duration: f64, sample_rate: f64, amplitude: f64, seed: u64) -> Self {
        let n = (sample_rate * duration) as usize;
        let mut state = seed;
        let mut samples = Vec::with_capacity(n);
        for _ in 0..n {
            // LCG: x_{n+1} = (a * x_n + c) mod m
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let val = ((state >> 33) as f64) / (1u64 << 31) as f64; // [0, 1)
            samples.push(amplitude * (val * 2.0 - 1.0));
        }
        Self { samples, sample_rate }
    }

    /// Generate a linear chirp from `f0` to `f1` Hz.
    pub fn chirp(f0: f64, f1: f64, duration: f64, sample_rate: f64, amplitude: f64) -> Self {
        let n = (sample_rate * duration) as usize;
        let rate = (f1 - f0) / duration;
        let samples: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                amplitude * (2.0 * PI * (f0 * t + 0.5 * rate * t * t)).sin()
            })
            .collect();
        Self { samples, sample_rate }
    }

    /// Signal duration in seconds.
    pub fn duration(&self) -> f64 {
        if self.sample_rate > 0.0 {
            self.samples.len() as f64 / self.sample_rate
        } else {
            0.0
        }
    }

    /// Number of samples.
    pub fn n_samples(&self) -> usize {
        self.samples.len()
    }

    /// Linear-interpolation resample to `target_rate`.
    pub fn resample(&self, target_rate: f64) -> Signal {
        if target_rate <= 0.0 || self.samples.is_empty() {
            return Signal::empty(target_rate);
        }
        let ratio = self.sample_rate / target_rate;
        let new_n = ((self.samples.len() as f64) / ratio).max(1.0) as usize;
        let mut new_samples = Vec::with_capacity(new_n);
        for i in 0..new_n {
            let pos = i as f64 * ratio;
            let idx = pos as usize;
            let frac = pos - idx as f64;
            if idx + 1 < self.samples.len() {
                new_samples.push(self.samples[idx] * (1.0 - frac) + self.samples[idx + 1] * frac);
            } else {
                new_samples.push(self.samples[idx.min(self.samples.len() - 1)]);
            }
        }
        Signal { samples: new_samples, sample_rate: target_rate }
    }

    /// Moving-average low-pass filter.
    pub fn lowpass(&self, cutoff: f64, order: usize) -> Signal {
        let nyquist = self.sample_rate / 2.0;
        if cutoff <= 0.0 || cutoff >= nyquist {
            return self.clone();
        }
        let window = ((nyquist / cutoff) as usize * order)
            .min(self.samples.len())
            .max(1);
        if window <= 1 {
            return self.clone();
        }
        let half = window / 2;
        let mut out = Vec::with_capacity(self.samples.len());
        for i in 0..self.samples.len() {
            let start = i.saturating_sub(half);
            let end = (i + half + 1).min(self.samples.len());
            let sum: f64 = self.samples[start..end].iter().sum();
            out.push(sum / (end - start) as f64);
        }
        Signal { samples: out, sample_rate: self.sample_rate }
    }

    /// High-pass filter by subtracting low-pass from original.
    pub fn highpass(&self, cutoff: f64, order: usize) -> Signal {
        let lp = self.lowpass(cutoff, order);
        let samples: Vec<f64> = self.samples.iter()
            .zip(lp.samples.iter())
            .map(|(s, l)| s - l)
            .collect();
        Signal { samples, sample_rate: self.sample_rate }
    }

    /// Band-pass filter.
    pub fn bandpass(&self, low_cutoff: f64, high_cutoff: f64, order: usize) -> Signal {
        self.lowpass(high_cutoff, order).highpass(low_cutoff, order)
    }

    /// Hard threshold — samples below `level` in absolute value become 0.
    pub fn threshold(&self, level: f64) -> Signal {
        let samples: Vec<f64> = self.samples.iter()
            .map(|&s| if s.abs() >= level { s } else { 0.0 })
            .collect();
        Signal { samples, sample_rate: self.sample_rate }
    }

    /// Simple magnitude envelope via sliding maximum of absolute value.
    pub fn envelope(&self) -> Signal {
        if self.samples.is_empty() {
            return Signal::empty(self.sample_rate);
        }
        let window = (self.sample_rate * 0.005).max(1.0) as usize;
        let mut out = Vec::with_capacity(self.samples.len());
        for i in 0..self.samples.len() {
            let start = i.saturating_sub(window);
            out.push(self.samples[start..=i].iter().map(|s| s.abs()).fold(0.0_f64, f64::max));
        }
        Signal { samples: out, sample_rate: self.sample_rate }
    }

    /// Root-mean-square amplitude.
    pub fn rms(&self) -> f64 {
        if self.samples.is_empty() {
            return 0.0;
        }
        let sum_sq: f64 = self.samples.iter().map(|s| s * s).sum();
        (sum_sq / self.samples.len() as f64).sqrt()
    }

    /// Peak (max absolute) amplitude.
    pub fn peak(&self) -> f64 {
        self.samples.iter().map(|s| s.abs()).fold(0.0, f64::max)
    }

    /// Signal-to-noise ratio in dB relative to `noise`.
    pub fn snr_db(&self, noise: &Signal) -> f64 {
        let sig_power = self.rms();
        let noise_power = noise.rms();
        if noise_power == 0.0 {
            if sig_power > 0.0 { f64::INFINITY } else { 0.0 }
        } else {
            20.0 * (sig_power / noise_power).log10()
        }
    }

    /// Total signal energy.
    pub fn energy(&self) -> f64 {
        self.samples.iter().map(|s| s * s).sum()
    }

    /// Estimate dominant frequency via zero-crossing count.
    pub fn dominant_frequency(&self) -> f64 {
        if self.samples.len() < 2 {
            return 0.0;
        }
        let crossings = self.samples.windows(2)
            .filter(|w| w[0] * w[1] < 0.0)
            .count();
        (crossings as f64 / 2.0) * self.sample_rate / self.samples.len() as f64
    }

    /// Compute DFT magnitude spectrum (naïve O(N²)).
    ///
    /// Returns magnitudes for bins 0..N/2.
    pub fn dft_magnitude(&self, n: Option<usize>) -> Vec<f64> {
        let n = n.unwrap_or(self.samples.len()).min(self.samples.len());
        let half = n / 2 + 1;
        let mut magnitudes = Vec::with_capacity(half);
        for k in 0..half {
            let mut re = 0.0;
            let mut im = 0.0;
            for n_idx in 0..n {
                let angle = 2.0 * PI * k as f64 * n_idx as f64 / n as f64;
                re += self.samples[n_idx] * angle.cos();
                im -= self.samples[n_idx] * angle.sin();
            }
            magnitudes.push((re * re + im * im).sqrt() / n as f64);
        }
        magnitudes
    }
}

impl std::ops::Add for Signal {
    type Output = Signal;

    fn add(self, other: Signal) -> Signal {
        assert!((self.sample_rate - other.sample_rate).abs() < f64::EPSILON,
            "Sample rates must match for addition");
        let n = self.samples.len().max(other.samples.len());
        let mut samples = Vec::with_capacity(n);
        for i in 0..n {
            let a = self.samples.get(i).copied().unwrap_or(0.0);
            let b = other.samples.get(i).copied().unwrap_or(0.0);
            samples.push(a + b);
        }
        Signal { samples, sample_rate: self.sample_rate }
    }
}

impl std::ops::Mul<f64> for Signal {
    type Output = Signal;

    fn mul(self, scalar: f64) -> Signal {
        let samples: Vec<f64> = self.samples.iter().map(|s| s * scalar).collect();
        Signal { samples, sample_rate: self.sample_rate }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sine_generation() {
        let sig = Signal::sine(440.0, 0.01, 44100.0, 1.0);
        assert_eq!(sig.n_samples(), 441);
        assert!((sig.duration() - 0.01).abs() < 1e-6);
    }

    #[test]
    fn test_sine_values() {
        let sig = Signal::sine(1.0, 1.0, 4.0, 1.0);
        // At 1 Hz, sample_rate=4: sin(2pi*0/4)=0, sin(2pi*1/4)=1, sin(2pi*2/4)=0, sin(2pi*3/4)=-1
        assert!((sig.samples[0]).abs() < 1e-10);
        assert!((sig.samples[1] - 1.0).abs() < 1e-10);
        assert!((sig.samples[2]).abs() < 1e-10);
        assert!((sig.samples[3] + 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_noise_generation() {
        let sig = Signal::noise(0.1, 44100.0, 1.0, 42);
        assert_eq!(sig.n_samples(), 4410);
        // All values should be within [-1, 1] range approximately
        for &s in &sig.samples {
            assert!(s.abs() <= 1.1); // small tolerance
        }
    }

    #[test]
    fn test_chirp_generation() {
        let sig = Signal::chirp(100.0, 1000.0, 0.1, 44100.0, 1.0);
        assert_eq!(sig.n_samples(), 4410);
    }

    #[test]
    fn test_resample() {
        let sig = Signal::sine(440.0, 0.01, 44100.0, 1.0);
        let resampled = sig.resample(22050.0);
        assert!((resampled.sample_rate - 22050.0).abs() < 1e-6);
        assert_eq!(resampled.n_samples(), 220);
    }

    #[test]
    fn test_lowpass() {
        // High frequency signal should be attenuated
        let sig = Signal::sine(10000.0, 0.01, 44100.0, 1.0);
        let filtered = sig.lowpass(1000.0, 5);
        assert!(filtered.rms() < sig.rms());
    }

    #[test]
    fn test_highpass() {
        let sig = Signal::sine(100.0, 0.01, 44100.0, 1.0);
        let filtered = sig.highpass(1000.0, 5);
        assert!(filtered.rms() < sig.rms());
    }

    #[test]
    fn test_threshold() {
        let sig = Signal::new(vec![0.1, -0.05, 0.3, -0.2, 0.0], 44100.0);
        let thresh = sig.threshold(0.15);
        assert_eq!(thresh.samples, vec![0.0, 0.0, 0.3, -0.2, 0.0]);
    }

    #[test]
    fn test_rms() {
        let sig = Signal::new(vec![1.0, -1.0, 1.0, -1.0], 44100.0);
        assert!((sig.rms() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_peak() {
        let sig = Signal::new(vec![0.5, -0.8, 0.3], 44100.0);
        assert!((sig.peak() - 0.8).abs() < 1e-10);
    }

    #[test]
    fn test_energy() {
        let sig = Signal::new(vec![1.0, 2.0, 3.0], 44100.0);
        assert!((sig.energy() - 14.0).abs() < 1e-10);
    }

    #[test]
    fn test_dominant_frequency() {
        let sig = Signal::sine(440.0, 0.1, 44100.0, 1.0);
        let freq = sig.dominant_frequency();
        assert!((freq - 440.0).abs() < 50.0); // within 50 Hz
    }

    #[test]
    fn test_dft_magnitude() {
        let sig = Signal::sine(1000.0, 0.01, 44100.0, 1.0);
        let mag = sig.dft_magnitude(None);
        assert!(!mag.is_empty());
        // Peak should be near bin corresponding to 1000 Hz
        let bin = 1000.0 * sig.n_samples() as f64 / sig.sample_rate;
        let peak_bin = mag.iter().enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert!((peak_bin as f64 - bin).abs() < 5.0);
    }

    #[test]
    fn test_snr_db() {
        let sig = Signal::new(vec![1.0, 1.0, 1.0, 1.0], 44100.0);
        let noise = Signal::new(vec![0.1, 0.1, 0.1, 0.1], 44100.0);
        let snr = sig.snr_db(&noise);
        assert!((snr - 20.0).abs() < 1e-10); // 20*log10(1/0.1) = 20
    }

    #[test]
    fn test_add_signals() {
        let a = Signal::new(vec![1.0, 2.0], 44100.0);
        let b = Signal::new(vec![3.0, 4.0], 44100.0);
        let c = a + b;
        assert_eq!(c.samples, vec![4.0, 6.0]);
    }

    #[test]
    fn test_mul_scalar() {
        let sig = Signal::new(vec![1.0, 2.0, 3.0], 44100.0);
        let scaled = sig * 2.0;
        assert_eq!(scaled.samples, vec![2.0, 4.0, 6.0]);
    }

    #[test]
    fn test_empty_signal() {
        let sig = Signal::empty(44100.0);
        assert_eq!(sig.n_samples(), 0);
        assert!((sig.rms()).abs() < 1e-10);
        assert!((sig.peak()).abs() < 1e-10);
    }

    #[test]
    fn test_bandpass() {
        let sig = Signal::sine(5000.0, 0.01, 44100.0, 1.0);
        let bp = sig.bandpass(4000.0, 6000.0, 5);
        // Should retain most energy since 5kHz is in the passband
        assert!(bp.rms() > 0.0);
    }

    #[test]
    fn test_envelope() {
        let sig = Signal::sine(1000.0, 0.01, 44100.0, 1.0);
        let env = sig.envelope();
        assert_eq!(env.n_samples(), sig.n_samples());
        // Envelope values should all be >= 0
        for &v in &env.samples {
            assert!(v >= 0.0);
        }
    }
}
