//! # sonar-vision
//!
//! Sonar signal processing, spatial mapping, object tracking, and ASCII radar visualization.
//!
//! Rust port of the Python `sonar-vision` library. Provides:
//! - `Signal` — discrete-time signal with DFT, filtering, and analysis
//! - `Sonar` — active sonar ping/echo simulation
//! - `SpatialMap` — 2D occupancy grid with ray casting
//! - `ObjectTracker` — multi-object tracking with nearest-neighbour association
//! - `SonarDisplay` — ASCII radar rendering

pub mod signal;
pub mod sonar;
pub mod map;
pub mod tracker;
pub mod display;

pub use signal::Signal;
pub use sonar::Sonar;
pub use map::SpatialMap;
pub use tracker::ObjectTracker;
pub use display::SonarDisplay;
