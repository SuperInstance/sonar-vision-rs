# sonar-vision-rs

Rust port of [sonar-vision](https://github.com/SuperInstance/sonar-vision) — sonar signal processing, spatial mapping, object tracking, and ASCII radar visualization.

## Features

- **`Signal`** — discrete-time signal with DFT, filtering (lowpass/highpass/bandpass), and analysis
- **`Sonar`** — active sonar ping/echo simulation with propagation loss modeling
- **`SpatialMap`** — 2D occupancy grid with ray casting and obstacle management
- **`ObjectTracker`** — multi-object tracking with nearest-neighbour association and motion prediction
- **`SonarDisplay`** — ASCII radar rendering for sweeps, maps, and tracks

## Usage

```rust
use sonar_vision::{Signal, Sonar, SpatialMap, ObjectTracker, SonarDisplay};
use sonar_vision::map::Obstacle;

// Generate and analyze a signal
let sig = Signal::sine(440.0, 0.1, 44100.0, 1.0);
println!("RMS: {:.4}", sig.rms());
println!("Dominant freq: {:.1} Hz", sig.dominant_frequency());

// Sonar ping simulation
let sonar = Sonar::new();
let result = sonar.ping(100.0, -20.0);
println!("Distance: {:.2}m, SNR: {:.1}dB", result.distance, result.snr_db);

// Spatial mapping
let mut map = SpatialMap::new(100.0, 100.0, 1.0);
map.add_obstacle(Obstacle::new(10.0, 10.0).with_radius(3.0));
let hit = map.ray_cast(0.0, 0.0, 10.0, 50.0);
println!("Ray hit: {:?}", hit);

// ASCII display
let display = SonarDisplay::default();
let sweep = display.render_sweep(&[0.0, 45.0, 90.0], &[10.0, 20.0, 30.0], None);
println!("{}", sweep);
```

## Installation

```toml
[dependencies]
sonar-vision = "0.1.0"
```

## License

MIT
