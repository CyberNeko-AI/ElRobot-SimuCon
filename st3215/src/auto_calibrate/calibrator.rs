//! Generic ST3215 motor calibrator (decoupled from the station framework).
//!
//! Sweeps each motor to its mechanical limits (detecting stall via velocity),
//! records the encoder positions, and computes the valid travel arc.

use std::collections::{BTreeSet, HashMap};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use log::{info, warn};

use crate::calibrate;
use crate::driver::St3215;
use crate::presets::*;
use crate::protocol::{self, EepromRegister};

const VELOCITY_THRESHOLD: u16 = 10;
const SKIP_INITIAL_SAMPLES: u32 = 3;
const MOTOR_STARTUP_STEPS: u32 = 4;
const MIN_DISPLACEMENT: u32 = 5;
const MAX_READS_WITHOUT_DISPLACEMENT: u32 = 100;
const CALIBRATION_STEP: u16 = 1020;
const SAFE_OFFSET: u16 = 60;

pub struct ST3215Calibrator<'a> {
    driver: &'a mut St3215,
    stop_requested: Arc<AtomicBool>,
    motor_positions: HashMap<u8, BTreeSet<u16>>,
    active_motors: Vec<u8>,
}

impl<'a> ST3215Calibrator<'a> {
    pub fn new(driver: &'a mut St3215, stop_requested: Arc<AtomicBool>) -> Self {
        Self {
            driver,
            stop_requested,
            motor_positions: HashMap::new(),
            active_motors: Vec::new(),
        }
    }

    pub fn set_active_motors(&mut self, motor_ids: Vec<u8>) {
        self.active_motors = motor_ids;
    }

    fn is_stopped(&self) -> bool {
        self.stop_requested.load(Ordering::Relaxed)
    }

    async fn check_stop(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        if self.is_stopped() {
            info!("Stop detected - disabling torque for all motors");
            self.disable_all_motors_torque().await?;
            return Err("Calibration stopped by user".into());
        }
        Ok(())
    }

    pub async fn disable_all_motors_torque(
        &mut self,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let ids = self.active_motors.clone();
        for motor in ids {
            self.driver.set_torque(motor, false).await?;
        }
        Ok(())
    }

    // ---- low-level helpers ------------------------------------------------

    async fn read_offset(&mut self, motor: u8) -> Result<i16, protocol::Error> {
        let data = self
            .driver
            .read_eeprom(motor, EepromRegister::Offset.address(), 2)
            .await?;
        Ok(i16::from_le_bytes([data[0], data[1]]))
    }

    async fn read_encoder_position(&mut self, motor: u8) -> Result<u16, protocol::Error> {
        let displayed = self.driver.read_position(motor).await?;
        let offset = self.read_offset(motor).await?;
        Ok(((displayed as i32 + offset as i32 + 4096) % 4096) as u16)
    }

    async fn set_position(&mut self, motor: u8, position: u16) -> Result<(), protocol::Error> {
        self.driver.set_goal_speed(motor, CALIBRATION_SPEED).await?;
        self.driver.set_accel(motor, CALIBRATION_ACCEL).await?;
        self.driver.set_position(motor, position).await
    }

    pub async fn set_torque(&mut self, motor: u8, enable: bool) -> Result<(), protocol::Error> {
        self.driver.set_torque(motor, enable).await
    }

    pub async fn send_eeprom_write_verified(
        &mut self,
        motor: u8,
        address: u8,
        value: Vec<u8>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.driver.write_eeprom_verified(motor, address, &value).await?;
        Ok(())
    }

    /// Poll position/velocity until the motor stalls against a mechanical stop.
    async fn wait_for_stall(&mut self, motor: u8) -> Result<u16, Box<dyn std::error::Error>> {
        info!("Motor {} - Waiting for stall", motor);

        let mut start_position: Option<u16> = None;
        let mut startup_steps = 0u32;
        let mut stable_count = 0u32;
        let mut total_reads = 0u32;

        loop {
            self.check_stop().await?;

            match (self.driver.read_position(motor).await, self.driver.read_velocity(motor).await) {
                (Ok(displayed), Ok(velocity)) => {
                    self.motor_positions.entry(motor).or_default().insert(displayed);

                    if start_position.is_none() {
                        start_position = Some(displayed);
                    }
                    startup_steps += 1;
                    total_reads += 1;

                    let displacement =
                        (displayed as i32 - start_position.unwrap() as i32).unsigned_abs();

                    if startup_steps >= MOTOR_STARTUP_STEPS {
                        if displacement >= MIN_DISPLACEMENT {
                            if velocity < VELOCITY_THRESHOLD {
                                stable_count += 1;
                            } else {
                                stable_count = 0;
                            }
                            if stable_count > SKIP_INITIAL_SAMPLES {
                                info!("Motor {} - Stalled at position {}", motor, displayed);
                                return Ok(displayed);
                            }
                        } else if total_reads >= MAX_READS_WITHOUT_DISPLACEMENT {
                            warn!(
                                "Motor {} - Unable to move from position {} after {} reads",
                                motor, displayed, total_reads
                            );
                            return Ok(displayed);
                        }
                    }
                }
                (Err(protocol::Error::Servo { .. }), _) | (_, Err(protocol::Error::Servo { .. })) => {
                    // recover from a transient servo error by re-enabling torque
                    warn!("Motor {} in error state, recovering", motor);
                    let _ = self.driver.set_torque(motor, false).await;
                    let _ = self.driver.set_torque(motor, true).await;
                }
                (Err(e), _) | (_, Err(e)) => {
                    return Err(Box::new(e));
                }
            }

            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    // ---- calibration primitives -------------------------------------------

    pub async fn prepare_motor(&mut self, motor: u8, motor_count: u8) -> Result<(), Box<dyn std::error::Error>> {
        info!("Motor {} - Preparing", motor);

        // reset + zero offset
        self.driver.reset_calibration(motor).await?;
        tokio::time::sleep(Duration::from_millis(100)).await;

        let pid = pid_config_for_motor_count(motor_count);
        self.send_eeprom_write_verified(motor, EepromRegister::Mode.address(), vec![0]).await?;
        self.send_eeprom_write_verified(motor, EepromRegister::PCoef.address(), vec![pid.p]).await?;
        self.send_eeprom_write_verified(motor, EepromRegister::ICoef.address(), vec![pid.i]).await?;
        self.send_eeprom_write_verified(motor, EepromRegister::DCoef.address(), vec![pid.d]).await?;
        self.send_eeprom_write_verified(motor, EepromRegister::ReturnDelay.address(), vec![0]).await?;

        self.driver.set_torque(motor, false).await?;
        self.send_eeprom_write_verified(motor, EepromRegister::MaxTorque.address(), CALIBRATION_TORQUE_LIMIT.to_le_bytes().to_vec()).await?;
        self.driver.set_goal_speed(motor, CALIBRATION_SPEED).await?;
        self.driver.set_accel(motor, CALIBRATION_ACCEL).await?;
        self.driver.set_torque_limit(motor, CALIBRATION_TORQUE_LIMIT).await?;
        Ok(())
    }

    pub async fn find_min(&mut self, motor: u8, torque_after: bool) -> Result<(), Box<dyn std::error::Error>> {
        info!("Motor {} - Finding minimum position", motor);
        self.check_stop().await?;

        let current_encoder = self.read_encoder_position(motor).await?;
        let mut new_offset = (current_encoder as i32 - (4095 - SAFE_OFFSET as i32)) as i16;
        if new_offset < -2047 {
            new_offset += 4096;
        } else if new_offset > 2047 {
            new_offset -= 4096;
        }
        new_offset = new_offset.clamp(-2047, 2047);
        self.send_eeprom_write_verified(motor, EepromRegister::Offset.address(), new_offset.to_le_bytes().to_vec()).await?;

        let mut current_target = 4095 - SAFE_OFFSET;
        while current_target > 0 {
            let next_target = if current_target >= CALIBRATION_STEP { current_target - CALIBRATION_STEP } else { 0 };
            self.set_position(motor, next_target).await?;
            let final_pos = self.wait_for_stall(motor).await?;
            if final_pos > next_target + 50 {
                info!("Motor {} - Found min at {}", motor, final_pos);
                break;
            }
            current_target = next_target;
        }

        self.set_torque(motor, torque_after).await?;
        Ok(())
    }

    pub async fn find_max(&mut self, motor: u8, torque_after: bool) -> Result<(), Box<dyn std::error::Error>> {
        info!("Motor {} - Finding maximum position", motor);
        self.check_stop().await?;

        let current_encoder = self.read_encoder_position(motor).await?;
        let mut new_offset = (current_encoder as i32 - SAFE_OFFSET as i32) as i16;
        if new_offset > 2047 {
            new_offset -= 4096;
        } else if new_offset < -2047 {
            new_offset += 4096;
        }
        new_offset = new_offset.clamp(-2047, 2047);
        self.send_eeprom_write_verified(motor, EepromRegister::Offset.address(), new_offset.to_le_bytes().to_vec()).await?;

        let mut current_target = SAFE_OFFSET;
        while current_target < 4095 {
            let next_target = if current_target + CALIBRATION_STEP <= 4095 { current_target + CALIBRATION_STEP } else { 4095 };
            self.set_position(motor, next_target).await?;
            let final_pos = self.wait_for_stall(motor).await?;
            if final_pos < next_target.saturating_sub(50) {
                info!("Motor {} - Found max at {}", motor, final_pos);
                break;
            }
            current_target = next_target;
        }

        self.set_torque(motor, torque_after).await?;
        Ok(())
    }

    pub async fn shift(&mut self, motor: u8, steps: i16, torque_after: bool) -> Result<(), Box<dyn std::error::Error>> {
        info!("Motor {} - Shifting by {} steps", motor, steps);
        self.check_stop().await?;

        let current_encoder = self.read_encoder_position(motor).await?;
        let start_position = if steps >= 0 { SAFE_OFFSET as i32 } else { 4095 - SAFE_OFFSET as i32 };
        let mut new_offset = (current_encoder as i32 - start_position) as i16;
        if new_offset > 2047 {
            new_offset -= 4096;
        } else if new_offset < -2047 {
            new_offset += 4096;
        }
        new_offset = new_offset.clamp(-2047, 2047);
        self.send_eeprom_write_verified(motor, EepromRegister::Offset.address(), new_offset.to_le_bytes().to_vec()).await?;

        let target_displayed = (start_position + steps as i32).clamp(0, 4095) as u16;
        let step_size = CALIBRATION_STEP as i32;
        let mut current_pos = start_position as u16;
        let direction: i32 = if steps > 0 { 1 } else { -1 };

        while (direction > 0 && current_pos < target_displayed) || (direction < 0 && current_pos > target_displayed) {
            let next_step = if direction > 0 {
                ((current_pos as i32 + step_size).min(target_displayed as i32)) as u16
            } else {
                ((current_pos as i32 - step_size).max(target_displayed as i32)) as u16
            };
            self.set_position(motor, next_step).await?;
            self.wait_for_stall(motor).await?;
            current_pos = next_step;
        }

        self.set_torque(motor, torque_after).await?;
        Ok(())
    }

    pub async fn go_to_float_position(&mut self, motor: u8, float_pos: f32, torque_after: bool) -> Result<(), Box<dyn std::error::Error>> {
        info!("Motor {} - Moving to float position {}", motor, float_pos);
        self.check_stop().await?;

        let float_pos = float_pos.clamp(0.0, 1.0);
        let positions = self
            .motor_positions
            .get(&motor)
            .ok_or("No recorded positions for motor")?;
        if positions.is_empty() {
            return Err("No positions recorded during calibration".into());
        }

        let arc = calibrate::calculate_arc(positions);
        let (midpoint, range) = if arc.max >= arc.min {
            let range = arc.max - arc.min;
            (arc.min + range / 2, range)
        } else {
            let range = (4096 - arc.min) + arc.max;
            (((arc.min as u32 + range as u32 / 2) & 0xFFF) as u16, range)
        };

        let offset = (midpoint as i32 - 2048).clamp(-2047, 2047) as i16;
        self.send_eeprom_write_verified(motor, EepromRegister::Offset.address(), offset.to_le_bytes().to_vec()).await?;

        let goal_offset = ((float_pos - 0.5) * range as f32) as i32;
        let goal_pos = ((2048 + goal_offset) & 0xFFF) as u16;

        self.set_position(motor, goal_pos).await?;
        self.wait_for_stall(motor).await?;
        self.set_torque(motor, torque_after).await?;
        Ok(())
    }

    // ---- save / freeze -----------------------------------------------------

    /// Compute each motor's travel arc and persist the calibration (offset,
    /// PID, mode, torque/current defaults) to EEPROM.
    pub async fn save_calibration(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let motor_ids = self.active_motors.clone();
        let motor_count = motor_ids.len() as u8;

        for motor in motor_ids {
            let positions = match self.motor_positions.get(&motor) {
                Some(p) if !p.is_empty() => p,
                _ => {
                    warn!("Motor {}: No calibration data, skipping", motor);
                    continue;
                }
            };
            let arc = calibrate::calculate_arc(positions);
            let midpoint = if arc.max >= arc.min {
                arc.min + (arc.max - arc.min) / 2
            } else {
                let range = (4096 - arc.min) + arc.max;
                ((arc.min as u32 + range as u32 / 2) & 0xFFF) as u16
            };
            info!("Motor {}: arc min={} max={} midpoint={}", motor, arc.min, arc.max, midpoint);
            self.driver.freeze_calibration(motor, midpoint, motor_count).await?;
        }
        Ok(())
    }
}
