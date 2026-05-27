# sonar-vision-rs

Rust port of [sonar-vision](https://github.com/SuperInstance/sonar-vision) — sonar/ultrasonic perception pipeline.

## Features

- **Pulse generation**: configurable frequency, duration, sample rate
- **Echo detection**: threshold-based peak detection with minimum separation
- **Beamforming**: delay-and-sum across multiple sensors
- **Spatial mapping**: convert echoes to 2D/3D coordinates
- **Matched filter**: cross-correlation with template pulse
- **SNR computation**: signal-to-noise ratio estimation

## Usage

```rust
use sonar_vision::{generate_pulse, detect_echoes, PulseConfig, build_spatial_map, Echo};

let config = PulseConfig::default();
let pulse = generate_pulse(&config);

// Process return signal
let echoes = detect_echoes(&return_signal, 1_000_000.0, 0.5, 0.0001);
for echo in &echoes {
    println!("Object at {:.2}m (amplitude: {:.2})", echo.distance, echo.amplitude);
}

// Build spatial map from multiple angles
let sweeps = vec![
    (0.0, echoes.clone()),
    (std::f64::consts::PI / 4.0, echoes.clone()),
];
let map = build_spatial_map(&sweeps);
```

## License

MIT
