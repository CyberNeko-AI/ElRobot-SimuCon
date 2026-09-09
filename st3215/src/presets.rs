/// ST3215 motor settings presets

// ============================================================================
// Default Motor Settings
// ============================================================================

/// Default max torque
pub const DEFAULT_MAX_TORQUE: u16 = 500;

/// Default protection current
pub const DEFAULT_PROTECTION_CURRENT: u16 = 500; // 260;

/// Default overload torque
pub const DEFAULT_OVERLOAD_TORQUE: u8 = 25;

/// Default acceleration
pub const DEFAULT_ACCEL: u8 = 254;

/// Default torque limit
pub const DEFAULT_TORQUE_LIMIT: u16 = 500;

/// PID configuration for a motor bus
#[derive(Clone, Copy, Debug)]
pub struct PidConfig {
    pub p: u8,
    pub i: u8,
    pub d: u8,
}

/// PID config for ElRobot (8-motor arm)
pub const ELROBOT_PID: PidConfig = PidConfig { p: 16, i: 0, d: 0 };

/// PID config for SO101 (6-motor arm)
pub const SO101_PID: PidConfig = PidConfig { p: 16, i: 8, d: 32 };

/// Returns the appropriate PID config based on motor count
pub fn pid_config_for_motor_count(max_motors_cnt: u8) -> PidConfig {
    match max_motors_cnt {
        6 => SO101_PID,
        8 => ELROBOT_PID,
        _ => ELROBOT_PID, // default fallback
    }
}

// ============================================================================
// Calibration Motor Settings
// ============================================================================

/// Sweep speed during calibration
pub const CALIBRATION_SPEED: u16 = 365;

/// Sweep acceleration during calibration
pub const CALIBRATION_ACCEL: u8 = 50;

/// Torque limit during calibration
pub const CALIBRATION_TORQUE_LIMIT: u16 = 300;
