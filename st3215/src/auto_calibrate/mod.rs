/// Auto-calibration for ST3215 motors
///
/// Calibrates motor physical range by sweeping to limits and detecting stalls.
/// Handles register boundary conditions by adjusting the offset (midpoint) value.

use crate::port::{MAX_MOTORS_CNT, ST3215_COMMAND_TIMEOUT_MS};
use crate::protocol::{self};
use bytes::Bytes;
use std::sync::atomic::Ordering;

pub mod calibrator;
mod so101;
mod elrobot;

/// Main auto-calibration entry point
///
/// Detects robot type and applies appropriate calibration strategy
/// Returns Some(stop_flag) for auto-calibration (SO101), None for default calibration
pub async fn calibrate(
    port: &mut tokio_serial::SerialStream,
    bus_info: &crate::st3215_proto::St3215Bus,
    meta: &crate::port_meta::St3215PortMeta,
) -> Result<Option<std::sync::Arc<std::sync::atomic::AtomicBool>>, protocol::Error> {
    log::info!("Starting auto-calibration sequence for bus {}", bus_info.serial_number);

    // Search for motors on the bus
    let mut found_motors = Vec::new();
    for motor_id in 1..=MAX_MOTORS_CNT {
        let ping_req = protocol::ST3215Request::Ping { motor: motor_id };
        if ping_req
            .async_readwrite(port, ST3215_COMMAND_TIMEOUT_MS)
            .await
            .is_ok()
        {
            found_motors.push(motor_id);
        }
    }

    if found_motors.is_empty() {
        log::warn!("No motors found on bus");
        return Ok(None);
    }

    log::info!("Found {} motor(s): {:?}", found_motors.len(), found_motors);

    // Detect robot type and apply appropriate calibration
    let is_so101_6 = found_motors.len() == 6
        && found_motors.iter().all(|&id| id >= 1 && id <= 6);

    let is_elrobot_8 = found_motors.len() == 8
        && found_motors.iter().all(|&id| id >= 1 && id <= 8);

    if is_so101_6 {
        let comm = meta.get_communicator().clone();

        // Check if calibration is already in progress and signal it to stop
        if let Some(existing_stop_flag) = comm.get_calibration_stop(&bus_info.serial_number) {
            log::info!("Stopping existing calibration for bus {}", bus_info.serial_number);
            existing_stop_flag.store(true, Ordering::Relaxed);
        }

        let stop_flag = so101::auto_calibrate(bus_info.serial_number.clone(), comm.clone())
            .await
            .map_err(|e| protocol::Error::InvalidData {
                msg: format!("Auto-calibration failed: {}", e),
                source_packet: Bytes::new(),
                reply_packet: Bytes::new(),
            })?;

        // Store the stop flag in the communicator
        comm.set_calibration_stop(&bus_info.serial_number, stop_flag.clone());
        Ok(Some(stop_flag))
    } else if is_elrobot_8 {
        let comm = meta.get_communicator().clone();

        if let Some(existing_stop_flag) = comm.get_calibration_stop(&bus_info.serial_number) {
            log::info!("Stopping existing calibration for bus {}", bus_info.serial_number);
            existing_stop_flag.store(true, Ordering::Relaxed);
        }

        let stop_flag = elrobot::auto_calibrate(bus_info.serial_number.clone(), found_motors, comm.clone())
            .await
            .map_err(|e| protocol::Error::InvalidData {
                msg: format!("Auto-calibration failed: {}", e),
                source_packet: Bytes::new(),
                reply_packet: Bytes::new(),
            })?;

        comm.set_calibration_stop(&bus_info.serial_number, stop_flag.clone());
        Ok(Some(stop_flag))
    } else {
        Ok(None)
    }
}