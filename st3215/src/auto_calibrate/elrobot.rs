/// ElRobot-specific auto-calibration using the generic ST3215Calibrator

use super::calibrator::ST3215Calibrator;
use crate::protocol::EepromRegister;
use crate::st3215_proto::InferenceState;
use crate::state::ST3215BusCommunicator;
use log::info;
use prost::Message;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use tokio::sync::watch;

pub async fn auto_calibrate(
    target_bus_serial: String,
    _found_motors: Vec<u8>,
    comm: Arc<ST3215BusCommunicator>,
) -> Result<Arc<AtomicBool>, Box<dyn std::error::Error>> {
    info!("Starting ElRobot auto-calibration for bus {}", target_bus_serial);

    // Create watch channel with empty initial state
    let initial_state = InferenceState::default();
    let (inference_tx, inference_rx) = watch::channel(initial_state);
    let inference_queue_id = comm.normfs.resolve("st3215/inference");

    // Create stop flag
    let stop_requested = Arc::new(AtomicBool::new(false));

    // Subscribe to inference updates
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

    // Run calibration sequence
    let target_serial = target_bus_serial.clone();
    let target_serial_cleanup = target_bus_serial.clone();
    let stop_flag = stop_requested.clone();
    let comm_cleanup = comm.clone();
    tokio::spawn(async move {
        match run_elrobot_calibration(target_serial, comm, inference_rx, stop_flag).await {
            Ok(_) => {
                log::info!("ElRobot calibration completed successfully");
            }
            Err(e) => {
                log::error!("ElRobot calibration failed: {}", e);
            }
        }
        comm_cleanup.clear_calibration_stop(&target_serial_cleanup);
    });

    Ok(stop_requested)
}

async fn run_elrobot_calibration(
    target_bus_serial: String,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // Reset calibration bounds before starting
    info!("Resetting calibration bounds for bus {}", target_bus_serial);
    comm.reset_bounds(&target_bus_serial);

    let mut calibrator = ST3215Calibrator::new(
        target_bus_serial.clone(), comm.clone(), inference_rx, stop_requested,
    );

    calibrator.set_active_motors(vec![1, 2, 3, 4, 5, 6, 7, 8]);
    // 1 prepare + 31 calibration operations + 1 finalize = 33 total
    calibrator.set_total_steps(32);

    comm.update_calibration_progress(
        &target_bus_serial,
        0,
        32,
        "Starting calibration",
        crate::state::CalibrationStatus::InProgress,
        None,
    );

    let result = run_calibration_sequence(&mut calibrator).await;

    match result {
        Ok(_) => {
            if let Err(e) = calibrator.save_calibration().await {
                log::error!("Failed to save calibration: {}", e);
                calibrator.mark_failed(&e.to_string());
                return Err(e);
            }
            calibrator.mark_done();
            info!("ElRobot calibration sequence complete");
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

    // ── Step 1: Prepare all 8 motors ─────────────────────────────────────
    calibrator.next_step("Preparing all motors").await?;
    for motor_id in 1..=8 {
        calibrator.prepare_motor(motor_id, 8).await?;
        info!("Motor {} prepared", motor_id);
    }

    // Set current limits for each motor
    let current_limits: [(u8, u16); 8] = [
        (1, 50),
        (2, 50),
        (3, 50),
        (4, 50),
        (5, 15),
        (6, 20),
        (7, 15),
        (8, 10),
    ];

    for (motor_id, current_limit) in current_limits {
        calibrator.send_eeprom_write_verified(
            motor_id,
            EepromRegister::ProtectionCurrent.address(),
            current_limit.to_le_bytes().to_vec(),
        ).await?;
        info!("Motor {} current limit set to {}", motor_id, current_limit);
    }

    // ── Steps 2-4: Calibrate motor 8 (gripper) ──────────────────────────
    calibrator.next_step("Motor 8: Find minimum").await?;
    calibrator.find_min(8, 0).await?;
    calibrator.next_step("Motor 8: Find maximum").await?;
    calibrator.find_max(8, 0).await?;
    calibrator.next_step("Motor 8: Move to center").await?;
    calibrator.shift(8, -700, 0).await?;

    // ── Steps 5-7: Calibrate motor 1 (base) ─────────────────────────────
    calibrator.next_step("Motor 1: Find minimum").await?;
    calibrator.find_min(1, 0).await?;
    calibrator.next_step("Motor 1: Find maximum").await?;
    calibrator.find_max(1, 0).await?;
    calibrator.next_step("Motor 1: Move to center").await?;
    calibrator.shift(1, -1000, 1).await?;

    // ── Steps 8-9: Motor 6 find temporary range + shift ──────────────────────────────
    calibrator.next_step("Motor 6: Find temporary minimum").await?;
    calibrator.find_min(6, 0).await?;
    calibrator.next_step("Motor 6: Find temporary maximum").await?;
    calibrator.find_max(6, 0).await?;
    calibrator.go_to_float_position(6, 0.5, 1).await?;

    // ── Steps 10-12: Motors 2, 3 find min with torque hold ───────────────
    calibrator.next_step("Motor 2: Find minimum").await?;
    calibrator.find_min(2, 1).await?;

    calibrator.next_step("Motor 4: Enable torque").await?;
    calibrator.set_torque(4, 1).await?;

    calibrator.next_step("Motor 3: Find minimum").await?;
    calibrator.find_min(3, 1).await?;

    // ── Step 13: Motor 4 find max ────────────────────────────────────────
    calibrator.next_step("Motor 4: Find maximum").await?;
    calibrator.find_max(4, 1).await?;

    // ── Steps 14-16: Calibrate motor 6 ─────────────────────────────
    calibrator.next_step("Motor 6: Find minimum").await?;
    calibrator.find_min(6, 0).await?;
    calibrator.next_step("Motor 6: Find maximum").await?;
    calibrator.find_max(6, 0).await?;
    calibrator.next_step("Motor 6: Move to center").await?;
    calibrator.shift(6, -1130, 1).await?;

    // ── Steps 17-19: Calibrate motor 7 ───────────────────────────────────
    calibrator.next_step("Motor 7: Find minimum").await?;
    calibrator.find_min(7, 0).await?;
    calibrator.next_step("Motor 7: Find maximum").await?;
    calibrator.find_max(7, 0).await?;
    calibrator.next_step("Motor 7: Move to center").await?;
    calibrator.shift(7, -1782, 0).await?;

    // ── Steps 20-22: Calibrate motor 5 ───────────────────────────────────
    calibrator.next_step("Motor 5: Find minimum").await?;
    calibrator.find_min(5, 0).await?;
    calibrator.next_step("Motor 5: Find maximum").await?;
    calibrator.find_max(5, 0).await?;
    calibrator.next_step("Motor 5: Move to center").await?;
    calibrator.shift(5, -2120, 0).await?;

    // ── Steps 23-28: Final calibration passes for motors 2, 3, 4 ────────
    calibrator.next_step("Motor 2: Shift by 1030").await?;
    calibrator.shift(2, 1030, 1).await?;

    calibrator.next_step("Motor 4: Find minimum").await?;
    calibrator.find_min(4, 1).await?;

    calibrator.next_step("Motor 3: Find maximum").await?;
    calibrator.find_max(3, 0).await?;
    calibrator.next_step("Motor 3: Position at min").await?;
    calibrator.find_min(3, 1).await?;

    calibrator.next_step("Motor 2: Find maximum").await?;
    calibrator.find_max(2, 0).await?;
    calibrator.next_step("Motor 2: Position at min").await?;
    calibrator.shift(2, -1030, 1).await?;
    calibrator.find_max(3, 1).await?;
    calibrator.find_min(2, 1).await?;

    // ── Steps 29-32: Final positioning ───────────────────────────────────
    calibrator.next_step("Motor 3: Move to 50%").await?;
    calibrator.go_to_float_position(3, 0.5, 1).await?;
    calibrator.next_step("Motor 4: Move to 50%").await?;
    calibrator.go_to_float_position(4, 0.5, 1).await?;
    calibrator.next_step("Motor 2: Move to 1%").await?;
    calibrator.go_to_float_position(2, 0.01, 0).await?;
    calibrator.next_step("Motor 4: Move to 95%").await?;
    calibrator.go_to_float_position(4, 0.95, 0).await?;

    // ── Step 33: Finalize all motors ─────────────────────────────────────
    calibrator.next_step("Finalizing all motors").await?;

    Ok(())
}
