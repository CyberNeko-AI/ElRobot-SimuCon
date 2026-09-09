//! Standalone ST3215 serial-bus servo driver.
//!
//! Wraps a [`tokio_serial::SerialStream`] with the ST3215 request/response
//! protocol and convenience helpers for RAM / EEPROM access plus the
//! calibration freeze/reset sequences. Decoupled from the norma-core station
//! framework — only `tokio`, `tokio-serial`, `bytes` and `log` are used.

use std::time::Duration;

use bytes::Bytes;
use log::warn;
use tokio::io::AsyncReadExt;
use tokio_serial::SerialPortBuilderExt;

use crate::presets::*;
use crate::protocol::{self, EepromRegister, Error, RamRegister, ST3215Request, ST3215Response};

pub const COMMAND_TIMEOUT_MS: u64 = 20;
pub const MAX_MOTORS_CNT: u8 = 8;

pub struct St3215 {
    port: tokio_serial::SerialStream,
}

impl St3215 {
    /// Open a serial port at the given baud rate.
    pub fn open(path: &str, baud_rate: u32) -> std::io::Result<Self> {
        let port = tokio_serial::new(path, baud_rate).open_native_async()?;
        Ok(Self { port })
    }

    /// Open at the default 1 Mbps bus rate.
    pub fn open_default(path: &str) -> std::io::Result<Self> {
        Self::open(path, protocol::SUPPORTED_BAUD_RATES[0])
    }

    pub fn port(&mut self) -> &mut tokio_serial::SerialStream {
        &mut self.port
    }

    // ---- raw protocol commands -------------------------------------------

    pub async fn ping(&mut self, motor: u8) -> Result<(), Error> {
        ST3215Request::Ping { motor }
            .async_readwrite(&mut self.port, COMMAND_TIMEOUT_MS)
            .await?;
        Ok(())
    }

    pub async fn read(&mut self, motor: u8, address: u8, length: u8) -> Result<Bytes, Error> {
        let req = ST3215Request::Read { motor, address, length };
        match req.async_readwrite(&mut self.port, COMMAND_TIMEOUT_MS).await? {
            ST3215Response::Read { data, .. } => Ok(data),
            _ => unreachable!("validated by protocol"),
        }
    }

    pub async fn write(&mut self, motor: u8, address: u8, data: &[u8]) -> Result<(), Error> {
        ST3215Request::Write { motor, address, data: Bytes::copy_from_slice(data) }
            .async_readwrite(&mut self.port, COMMAND_TIMEOUT_MS)
            .await?;
        Ok(())
    }

    pub async fn reg_write(&mut self, motor: u8, address: u8, data: &[u8]) -> Result<(), Error> {
        ST3215Request::RegWrite { motor, address, data: Bytes::copy_from_slice(data) }
            .async_readwrite(&mut self.port, COMMAND_TIMEOUT_MS)
            .await?;
        Ok(())
    }

    pub async fn action(&mut self, motor: u8) -> Result<(), Error> {
        ST3215Request::Action { motor }
            .async_readwrite(&mut self.port, COMMAND_TIMEOUT_MS)
            .await?;
        Ok(())
    }

    pub async fn reset(&mut self, motor: u8) -> Result<(), Error> {
        ST3215Request::Reset { motor }
            .async_readwrite(&mut self.port, COMMAND_TIMEOUT_MS)
            .await?;
        Ok(())
    }

    pub async fn sync_write(&mut self, address: u8, data: Vec<(u8, Vec<u8>)>) -> Result<(), Error> {
        let data: Vec<(u8, Bytes)> =
            data.into_iter().map(|(id, d)| (id, Bytes::from(d))).collect();
        ST3215Request::SyncWrite { address, data }
            .async_write(&mut self.port, COMMAND_TIMEOUT_MS)
            .await
    }

    // ---- RAM convenience ---------------------------------------------------

    pub async fn set_torque(&mut self, motor: u8, enable: bool) -> Result<(), Error> {
        self.write(motor, RamRegister::TorqueEnable.address(), &[enable as u8]).await
    }

    pub async fn set_position(&mut self, motor: u8, position: u16) -> Result<(), Error> {
        self.write(motor, RamRegister::GoalPosition.address(), &position.to_le_bytes()).await
    }

    pub async fn set_goal_speed(&mut self, motor: u8, speed: u16) -> Result<(), Error> {
        self.write(motor, RamRegister::GoalSpeed.address(), &speed.to_le_bytes()).await
    }

    pub async fn set_accel(&mut self, motor: u8, accel: u8) -> Result<(), Error> {
        self.write(motor, RamRegister::Acc.address(), &[accel]).await
    }

    pub async fn set_torque_limit(&mut self, motor: u8, limit: u16) -> Result<(), Error> {
        self.write(motor, RamRegister::TorqueLimit.address(), &limit.to_le_bytes()).await
    }

    pub async fn read_position(&mut self, motor: u8) -> Result<u16, Error> {
        let data = self.read(motor, RamRegister::PresentPosition.address(), 2).await?;
        Ok(protocol::get_motor_position(&data))
    }

    pub async fn read_velocity(&mut self, motor: u8) -> Result<u16, Error> {
        let data = self.read(motor, RamRegister::PresentSpeed.address(), 2).await?;
        Ok(protocol::get_motor_velocity(&data))
    }

    pub async fn read_load(&mut self, motor: u8) -> Result<u16, Error> {
        let data = self.read(motor, RamRegister::PresentLoad.address(), 2).await?;
        Ok(u16::from_le_bytes([data[0], data[1]]))
    }

    pub async fn read_current(&mut self, motor: u8) -> Result<u16, Error> {
        let data = self.read(motor, RamRegister::PresentCurrent.address(), 2).await?;
        Ok(protocol::get_motor_current(&data))
    }

    pub async fn read_voltage(&mut self, motor: u8) -> Result<u8, Error> {
        let data = self.read(motor, RamRegister::PresentVoltage.address(), 1).await?;
        Ok(data[0])
    }

    pub async fn read_temperature(&mut self, motor: u8) -> Result<u8, Error> {
        let data = self.read(motor, RamRegister::PresentTemperature.address(), 1).await?;
        Ok(data[0])
    }

    pub async fn is_moving(&mut self, motor: u8) -> Result<bool, Error> {
        let data = self.read(motor, RamRegister::Moving.address(), 1).await?;
        Ok(data[0] != 0)
    }

    // ---- EEPROM access -----------------------------------------------------

    async fn unlock_eeprom(&mut self, motor: u8) -> Result<(), Error> {
        self.write(motor, RamRegister::Lock.address(), &[0]).await
    }

    async fn lock_eeprom(&mut self, motor: u8) -> Result<(), Error> {
        self.write(motor, RamRegister::Lock.address(), &[1]).await
    }

    pub async fn read_eeprom(&mut self, motor: u8, address: u8, length: u8) -> Result<Bytes, Error> {
        self.read(motor, address, length).await
    }

    /// EEPROM write: unlock -> reg_write -> action -> lock.
    pub async fn write_eeprom(&mut self, motor: u8, address: u8, data: &[u8]) -> Result<(), Error> {
        self.unlock_eeprom(motor).await?;
        self.reg_write(motor, address, data).await?;
        self.action(motor).await?;
        self.lock_eeprom(motor).await
    }

    /// EEPROM write with read-back verification and retries.
    pub async fn write_eeprom_verified(
        &mut self,
        motor: u8,
        address: u8,
        data: &[u8],
    ) -> Result<bool, Error> {
        const MAX_RETRIES: u8 = 5;
        for attempt in 1..=MAX_RETRIES {
            self.write_eeprom(motor, address, data).await?;
            match self.read_eeprom(motor, address, data.len() as u8).await {
                Ok(readback) if readback.as_ref() == data => return Ok(true),
                Ok(readback) => {
                    warn!(
                        "EEPROM verify mismatch motor {}: 0x{:02X} expected {:02x?} got {:02x?} (attempt {}/{})",
                        motor, address, data, readback.as_ref(), attempt, MAX_RETRIES
                    );
                }
                Err(e) => {
                    warn!(
                        "EEPROM verify read failed motor {}: 0x{:02X}: {} (attempt {}/{})",
                        motor, address, e, attempt, MAX_RETRIES
                    );
                    self.drain().await;
                }
            }
        }
        Ok(false)
    }

    // ---- calibration freeze / reset ---------------------------------------

    /// Reset a motor's calibration: unlock EEPROM, reset, zero the position
    /// offset, lock EEPROM.
    pub async fn reset_calibration(&mut self, motor: u8) -> Result<bool, Error> {
        self.unlock_eeprom(motor).await?;
        self.reset(motor).await?;
        tokio::time::sleep(Duration::from_millis(100)).await;
        let verified =
            self.write_eeprom_verified(motor, EepromRegister::Offset.address(), &[0, 0]).await?;
        self.lock_eeprom(motor).await?;
        Ok(verified)
    }

    /// Persist calibration for a motor: writes the position-offset correction
    /// (`midpoint - 2048`), the PID coefficients, position-control mode and
    /// default torque/current/accel settings to EEPROM.
    pub async fn freeze_calibration(
        &mut self,
        motor: u8,
        midpoint: u16,
        motor_count: u8,
    ) -> Result<bool, Error> {
        let mut correction = midpoint as i16 - 2048;
        if correction > 2047 {
            correction -= 4096;
        } else if correction < -2047 {
            correction += 4096;
        }
        correction = correction.clamp(-2047, 2047);

        let pid = pid_config_for_motor_count(motor_count);

        self.unlock_eeprom(motor).await?;

        let mut ok = true;
        ok &= self.write_eeprom_verified(motor, EepromRegister::Mode.address(), &[0]).await?;
        ok &= self.write_eeprom_verified(motor, EepromRegister::PCoef.address(), &[pid.p]).await?;
        ok &= self.write_eeprom_verified(motor, EepromRegister::ICoef.address(), &[pid.i]).await?;
        ok &= self.write_eeprom_verified(motor, EepromRegister::DCoef.address(), &[pid.d]).await?;
        ok &= self.write_eeprom_verified(motor, EepromRegister::ReturnDelay.address(), &[0]).await?;
        ok &= self
            .write_eeprom_verified(motor, EepromRegister::MaxTorque.address(), &DEFAULT_MAX_TORQUE.to_le_bytes())
            .await?;
        ok &= self
            .write_eeprom_verified(motor, EepromRegister::ProtectionCurrent.address(), &DEFAULT_PROTECTION_CURRENT.to_le_bytes())
            .await?;
        ok &= self
            .write_eeprom_verified(motor, EepromRegister::OverloadTorque.address(), &[DEFAULT_OVERLOAD_TORQUE])
            .await?;
        ok &= self
            .write_eeprom_verified(motor, EepromRegister::Offset.address(), &correction.to_le_bytes())
            .await?;

        // Acceleration is RAM-only (some firmware clamps it); best-effort.
        let _ = self.set_accel(motor, DEFAULT_ACCEL).await;

        self.lock_eeprom(motor).await?;
        self.action(motor).await?;
        Ok(ok)
    }

    // ---- utilities ----------------------------------------------------------

    /// Drain stale bytes left in the serial buffer (e.g. after a failed request).
    pub async fn drain(&mut self) {
        let mut buf = [0u8; 256];
        let mut total = 0usize;
        loop {
            match tokio::time::timeout(Duration::from_millis(5), self.port.read(&mut buf)).await {
                Ok(Ok(n)) if n > 0 => {
                    total += n;
                    continue;
                }
                _ => break,
            }
        }
        if total > 0 {
            warn!("Drained {} stale bytes from serial port", total);
        }
    }

    /// Scan the bus for motors with IDs `1..=max_id`.
    pub async fn scan(&mut self, max_id: u8) -> Vec<u8> {
        let mut found = Vec::new();
        for motor in 1..=max_id {
            if self.ping(motor).await.is_ok() {
                found.push(motor);
            }
        }
        found
    }
}
