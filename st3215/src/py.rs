//! PyO3 bindings for the ST3215 driver.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use pyo3::exceptions::{PyIOError, PyRuntimeError};
use pyo3::prelude::*;

use crate::auto_calibrate::auto_calibrate_elrobot;
use crate::St3215 as RustSt3215;

#[pyclass(name = "St3215")]
pub struct St3215 {
    inner: RustSt3215,
    rt: tokio::runtime::Runtime,
    stop: Arc<AtomicBool>,
}

#[pymethods]
impl St3215 {
    /// Open the ST3215 serial bus. `baud` defaults to 1_000_000.
    #[new]
    #[pyo3(signature = (path, baud=None))]
    fn new(path: String, baud: Option<u32>) -> PyResult<Self> {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        let baud = baud.unwrap_or(1_000_000);
        let inner = rt
            .block_on(async move { RustSt3215::open(&path, baud) })
            .map_err(|e| PyIOError::new_err(e.to_string()))?;
        Ok(Self { inner, rt, stop: Arc::new(AtomicBool::new(false)) })
    }

    // ---- raw protocol commands -------------------------------------------

    fn ping(&mut self, motor: u8) -> PyResult<()> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.ping(motor).await }).map_err(py_err)
    }

    fn read(&mut self, motor: u8, address: u8, length: u8) -> PyResult<Vec<u8>> {
        let inner = &mut self.inner;
        self.rt
            .block_on(async move { inner.read(motor, address, length).await })
            .map(|b| b.to_vec())
            .map_err(py_err)
    }

    fn write(&mut self, motor: u8, address: u8, data: Vec<u8>) -> PyResult<()> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.write(motor, address, &data).await }).map_err(py_err)
    }

    fn reg_write(&mut self, motor: u8, address: u8, data: Vec<u8>) -> PyResult<()> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.reg_write(motor, address, &data).await }).map_err(py_err)
    }

    fn action(&mut self, motor: u8) -> PyResult<()> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.action(motor).await }).map_err(py_err)
    }

    fn reset(&mut self, motor: u8) -> PyResult<()> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.reset(motor).await }).map_err(py_err)
    }

    // ---- RAM convenience ---------------------------------------------------

    fn set_torque(&mut self, motor: u8, enable: bool) -> PyResult<()> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.set_torque(motor, enable).await }).map_err(py_err)
    }

    fn set_position(&mut self, motor: u8, position: u16) -> PyResult<()> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.set_position(motor, position).await }).map_err(py_err)
    }

    fn set_goal_speed(&mut self, motor: u8, speed: u16) -> PyResult<()> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.set_goal_speed(motor, speed).await }).map_err(py_err)
    }

    fn set_accel(&mut self, motor: u8, accel: u8) -> PyResult<()> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.set_accel(motor, accel).await }).map_err(py_err)
    }

    fn set_torque_limit(&mut self, motor: u8, limit: u16) -> PyResult<()> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.set_torque_limit(motor, limit).await }).map_err(py_err)
    }

    fn read_position(&mut self, motor: u8) -> PyResult<u16> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.read_position(motor).await }).map_err(py_err)
    }

    fn read_velocity(&mut self, motor: u8) -> PyResult<u16> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.read_velocity(motor).await }).map_err(py_err)
    }

    fn read_load(&mut self, motor: u8) -> PyResult<u16> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.read_load(motor).await }).map_err(py_err)
    }

    fn read_current(&mut self, motor: u8) -> PyResult<u16> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.read_current(motor).await }).map_err(py_err)
    }

    fn read_voltage(&mut self, motor: u8) -> PyResult<u8> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.read_voltage(motor).await }).map_err(py_err)
    }

    fn read_temperature(&mut self, motor: u8) -> PyResult<u8> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.read_temperature(motor).await }).map_err(py_err)
    }

    fn is_moving(&mut self, motor: u8) -> PyResult<bool> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.is_moving(motor).await }).map_err(py_err)
    }

    // ---- EEPROM ------------------------------------------------------------

    fn read_eeprom(&mut self, motor: u8, address: u8, length: u8) -> PyResult<Vec<u8>> {
        let inner = &mut self.inner;
        self.rt
            .block_on(async move { inner.read_eeprom(motor, address, length).await })
            .map(|b| b.to_vec())
            .map_err(py_err)
    }

    fn write_eeprom(&mut self, motor: u8, address: u8, data: Vec<u8>) -> PyResult<()> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.write_eeprom(motor, address, &data).await }).map_err(py_err)
    }

    fn write_eeprom_verified(&mut self, motor: u8, address: u8, data: Vec<u8>) -> PyResult<bool> {
        let inner = &mut self.inner;
        self.rt
            .block_on(async move { inner.write_eeprom_verified(motor, address, &data).await })
            .map_err(py_err)
    }

    // ---- calibration -------------------------------------------------------

    fn reset_calibration(&mut self, motor: u8) -> PyResult<bool> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.reset_calibration(motor).await }).map_err(py_err)
    }

    fn freeze_calibration(&mut self, motor: u8, midpoint: u16, motor_count: u8) -> PyResult<bool> {
        let inner = &mut self.inner;
        self.rt
            .block_on(async move { inner.freeze_calibration(motor, midpoint, motor_count).await })
            .map_err(py_err)
    }

    /// Scan the bus for motors with IDs `1..=max_id`.
    fn scan(&mut self, max_id: u8) -> Vec<u8> {
        let inner = &mut self.inner;
        self.rt.block_on(async move { inner.scan(max_id).await })
    }

    /// Run the full ElRobot (8-motor) auto-calibration sequence.
    /// Call `stop_calibration()` from another thread to abort it.
    fn auto_calibrate_elrobot(&mut self) -> PyResult<()> {
        self.stop.store(false, Ordering::SeqCst);
        let inner = &mut self.inner;
        let stop = self.stop.clone();
        self.rt
            .block_on(async move { auto_calibrate_elrobot(inner, stop).await })
            .map_err(py_err)
    }

    /// Request the running calibration to stop (disables torque on all motors).
    fn stop_calibration(&self) {
        self.stop.store(true, Ordering::SeqCst);
    }
}

fn py_err<E: std::fmt::Display>(e: E) -> PyErr {
    PyRuntimeError::new_err(e.to_string())
}

#[pymodule]
fn _st3215(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<St3215>()?;
    Ok(())
}
