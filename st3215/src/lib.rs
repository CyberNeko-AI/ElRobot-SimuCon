//! Standalone ST3215 serial-bus servo driver + auto-calibration.
//!
//! Decoupled from the norma-core station framework; only depends on `tokio`,
//! `tokio-serial`, `bytes` and `log`.

pub mod auto_calibrate;
pub mod calibrate;
pub mod driver;
pub mod presets;
pub mod protocol;

pub use driver::St3215;

#[cfg(test)]
mod calibrate_test;
