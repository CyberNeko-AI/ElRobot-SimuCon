//! Standalone ST3215 auto-calibration for the ElRobot arm.
//!
//! Sweeps each joint to its mechanical limits (detecting stall) and persists
//! the travel range (offset / midpoint) into the servo EEPROM.

mod calibrator;
mod elrobot;

pub use calibrator::ST3215Calibrator;
pub use elrobot::auto_calibrate_elrobot;
