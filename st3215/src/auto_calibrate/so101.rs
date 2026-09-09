/// SO101-specific auto-calibration using the generic ST3215Calibrator

use super::calibrator::ST3215Calibrator;
use crate::protocol::{RamRegister, EepromRegister};
use crate::st3215_proto::InferenceState;
use crate::state::ST3215BusCommunicator;
use log::info;
use prost::Message;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use tokio::sync::watch;

pub async fn auto_calibrate(
    target_bus_serial: String,
    comm: Arc<ST3215BusCommunicator>,
) -> Result<Arc<AtomicBool>, Box<dyn std::error::Error>> {
    info!("Starting SO101 auto-calibration for bus {}", target_bus_serial);

    // Create watch channel with empty initial state
    let initial_state = InferenceState::default();
    let (inference_tx, inference_rx) = watch::channel(initial_state);
    let inference_queue_id = comm.normfs.resolve("st3215/inference");

    // Create stop flag
    let stop_requested = Arc::new(AtomicBool::new(false));

    // Subscribe to inference updates - exits when inference_tx.send() fails
    let normfs = comm.normfs.clone();
    tokio::spawn(async move {
        let _ = normfs.subscribe(
            &inference_queue_id,
            Box::new(move |entries: &[(normfs::UintN, bytes::Bytes)]| {
                for (_, data) in entries {
                    if let Ok(state) = InferenceState::decode(data.as_ref()) {
                        // Exit subscription if receiver was dropped
                        if inference_tx.send(state).is_err() {
                            info!("Stop auto-calibration state subscription");
                            return false;
                        }
                    }
                }
                true
            }),
        );
    });

    // Run calibration sequence - inference_rx is dropped when this completes
    let target_serial = target_bus_serial.clone();
    let target_serial_cleanup = target_bus_serial.clone();
    let stop_flag = stop_requested.clone();
    let comm_cleanup = comm.clone();
    tokio::spawn(async move {
        match run_so101_calibration(target_serial, comm, inference_rx, stop_flag).await {
            Ok(_) => {
                log::info!("SO101 calibration completed successfully");
            }
            Err(e) => {
                log::error!("SO101 calibration failed: {}", e);
            }
        }
        // inference_rx is dropped here, causing subscribe to exit on next send
        comm_cleanup.clear_calibration_stop(&target_serial_cleanup);
    });

    Ok(stop_requested)
}

async fn run_so101_calibration(
    target_bus_serial: String,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // Reset calibration bounds before starting
    info!("Resetting calibration bounds for bus {}", target_bus_serial);
    comm.reset_bounds(&target_bus_serial);

    let mut calibrator = ST3215Calibrator::new(target_bus_serial.clone(), comm.clone(), inference_rx, stop_requested);

    // Set active motors for stop handling
    calibrator.set_active_motors(vec![1, 2, 3, 4, 5, 6]);

    // Count total steps in the calibration sequence
    // 1 prepare + 25 calibration operations + 1 finalize = 27 total
    calibrator.set_total_steps(27);

    // Mark calibration as started
    comm.update_calibration_progress(
        &target_bus_serial,
        0,
        27,
        "Starting calibration",
        crate::state::CalibrationStatus::InProgress,
        None,
    );

    // Run calibration and handle errors
    let result = run_calibration_sequence(&mut calibrator).await;

    // Cleanup all motors regardless of result
    info!("Cleaning up motors after calibration");
    for motor_id in 1..=6 {
        if let Err(e) = calibrator.cleanup_motor(motor_id).await {
            log::error!("Failed to cleanup motor {}: {}", motor_id, e);
        }
    }

    match result {
        Ok(_) => {
            if let Err(e) = calibrator.save_calibration().await {
                log::error!("Failed to save calibration: {}", e);
                calibrator.mark_failed(&e.to_string());
                return Err(e);
            }
            calibrator.mark_done();
            info!("SO101 calibration sequence complete");
            Ok(())
        }
        Err(e) => {
            calibrator.mark_failed(&e.to_string());
            Err(e)
        }
    }
}

async fn run_calibration_sequence(
    calibrator: &mut ST3215Calibrator,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {

    // Prepare all 6 motors
    calibrator.next_step("Preparing all motors").await?;
    for motor_id in 1..=6 {
        calibrator.prepare_motor(motor_id, 6).await?;
        info!("Motor {} prepared", motor_id);
    }

    // Set current limits for each motor
    let current_limits: [(u8, u16); 6] = [
        (1, 50),
        (2, 500),
        (3, 500),
        (4, 50),
        (5, 15),
        (6, 10),
    ];

    for (motor_id, current_limit) in current_limits {
        calibrator.send_eeprom_write_verified(
            motor_id,
            EepromRegister::ProtectionCurrent.address(),
            current_limit.to_le_bytes().to_vec(),
        ).await?;
        info!("Motor {} current limit set to {}", motor_id, current_limit);
    }

    // Set motor 5 and 6 torque limits
    calibrator.send_write_verified(
        5,
        RamRegister::TorqueLimit.address(),
        100_i16.to_le_bytes().to_vec(),
    ).await?;
    info!("Motor 5 torque limit set to 10");

    calibrator.send_write_verified(
        6,
        RamRegister::TorqueLimit.address(),
        100_i16.to_le_bytes().to_vec(),
    ).await?;
    info!("Motor 6 torque limit set to 10");

    // SO101 calibration sequence
    info!("Starting SO101 calibration sequence");

    calibrator.next_step("Motor 1: Find maximum").await?;
    calibrator.find_max(1, 0).await?;
    calibrator.next_step("Motor 1: Find minimum").await?;
    calibrator.find_min(1, 0).await?;
    calibrator.next_step("Motor 1: Move to center").await?;
    calibrator.go_to_float_position(1, 0.5, 0).await?;

    calibrator.next_step("Motor 2: Find minimum").await?;
    calibrator.find_min(2, 0).await?;
    calibrator.next_step("Motor 3: Find maximum").await?;
    calibrator.find_max(3, 0).await?;

    calibrator.next_step("Motor 4: Find maximum").await?;
    calibrator.find_max(4, 0).await?;
    calibrator.next_step("Motor 4: Find minimum").await?;
    calibrator.find_min(4, 0).await?;
    calibrator.next_step("Motor 4: Move to center").await?;
    calibrator.go_to_float_position(4, 0.5, 1).await?;

    calibrator.next_step("Motor 5: Find maximum").await?;
    calibrator.find_max(5, 0).await?;
    calibrator.next_step("Motor 5: Find minimum").await?;
    calibrator.find_min(5, 0).await?;

    calibrator.next_step("Motor 6: Find maximum").await?;
    calibrator.find_max(6, 0).await?;
    calibrator.next_step("Motor 6: Find minimum").await?;
    calibrator.find_min(6, 0).await?;

    calibrator.send_eeprom_write_verified(
        3,
        EepromRegister::MaxTorque.address(),
        300u16.to_le_bytes().to_vec(),
    ).await?;
    calibrator.send_write_verified(3,
        RamRegister::TorqueLimit.address(),
         300_i16.to_le_bytes().to_vec(),
    ).await?;

    calibrator.next_step("Motor 3: Find minimum").await?;
    calibrator.find_min(3, 1).await?;
    calibrator.next_step("Motor 3: Shifting").await?;
    calibrator.shift(3, 1200, 1).await?;

    calibrator.next_step("Motor 4: Re-find maximum").await?;
    calibrator.find_max(4, 0).await?;
    calibrator.next_step("Motor 4: Re-find minimum").await?;
    calibrator.find_min(4, 0).await?;
    calibrator.next_step("Motor 4: Move to 0.1").await?;
    calibrator.go_to_float_position(4, 0.1, 1).await?;

    calibrator.next_step("Motor 2: Shift position").await?;
    calibrator.shift(2, 1216, 1).await?;

    calibrator.next_step("Motor 3: Move to min").await?;
    calibrator.find_min(3, 1).await?;

    calibrator.next_step("Motor 2: Find maximum").await?;
    calibrator.find_max(2, 1).await?;

    calibrator.next_step("Motor 2: Move to 0.5").await?;
    calibrator.go_to_float_position(2, 0.5, 1).await?;
    calibrator.next_step("Motor 3: Move to 0.5").await?;
    calibrator.go_to_float_position(3, 0.5, 1).await?;
    calibrator.next_step("Motor 2: Move to min").await?;
    calibrator.find_min(2, 0).await?;
    calibrator.next_step("Motor 3: Move to max").await?;
    calibrator.find_max(3, 0).await?;
    calibrator.next_step("Motor 4: Move to 0.7").await?;
    calibrator.go_to_float_position(4, 0.7, 0).await?;

    calibrator.next_step("Finalizing all motors").await?;

    Ok(())
}
