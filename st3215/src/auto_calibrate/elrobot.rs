//! ElRobot (8-motor arm) auto-calibration sequence.

use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use log::info;

use super::calibrator::ST3215Calibrator;
use crate::driver::St3215;
use crate::protocol::EepromRegister;

/// Run the full ElRobot auto-calibration sequence.
///
/// `stop` lets the caller abort mid-run; on stop, torque is disabled for all
/// motors so the arm goes limp.
pub async fn auto_calibrate_elrobot(
    driver: &mut St3215,
    stop: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error>> {
    info!("Starting ElRobot auto-calibration");

    let mut cal = ST3215Calibrator::new(driver, stop);
    cal.set_active_motors(vec![1, 2, 3, 4, 5, 6, 7, 8]);

    // Step 1: prepare all 8 motors
    info!("Preparing all motors");
    for motor in 1..=8 {
        cal.prepare_motor(motor, 8).await?;
    }

    // Per-motor protection current limits (lower for the wrist/gripper)
    let current_limits: [(u8, u16); 8] =
        [(1, 50), (2, 50), (3, 50), (4, 50), (5, 15), (6, 20), (7, 15), (8, 10)];
    for (motor, limit) in current_limits {
        cal.send_eeprom_write_verified(
            motor,
            EepromRegister::ProtectionCurrent.address(),
            limit.to_le_bytes().to_vec(),
        )
        .await?;
    }

    // Motor 8 (gripper)
    info!("Motor 8: Find minimum");
    cal.find_min(8, false).await?;
    info!("Motor 8: Find maximum");
    cal.find_max(8, false).await?;
    info!("Motor 8: Move to center");
    cal.shift(8, -700, false).await?;

    // Motor 1 (base)
    info!("Motor 1: Find minimum");
    cal.find_min(1, false).await?;
    info!("Motor 1: Find maximum");
    cal.find_max(1, false).await?;
    info!("Motor 1: Move to center");
    cal.shift(1, -1000, true).await?;

    // Motor 6 temporary range
    info!("Motor 6: Find temporary minimum");
    cal.find_min(6, false).await?;
    info!("Motor 6: Find temporary maximum");
    cal.find_max(6, false).await?;
    cal.go_to_float_position(6, 0.5, true).await?;

    // Motors 2, 3 find min with torque hold
    info!("Motor 2: Find minimum");
    cal.find_min(2, true).await?;
    info!("Motor 4: Enable torque");
    cal.set_torque(4, true).await?;
    info!("Motor 3: Find minimum");
    cal.find_min(3, true).await?;

    // Motor 4 find max
    info!("Motor 4: Find maximum");
    cal.find_max(4, true).await?;

    // Motor 6 final
    info!("Motor 6: Find minimum");
    cal.find_min(6, false).await?;
    info!("Motor 6: Find maximum");
    cal.find_max(6, false).await?;
    info!("Motor 6: Move to center");
    cal.shift(6, -1130, true).await?;

    // Motor 7
    info!("Motor 7: Find minimum");
    cal.find_min(7, false).await?;
    info!("Motor 7: Find maximum");
    cal.find_max(7, false).await?;
    info!("Motor 7: Move to center");
    cal.shift(7, -1782, false).await?;

    // Motor 5
    info!("Motor 5: Find minimum");
    cal.find_min(5, false).await?;
    info!("Motor 5: Find maximum");
    cal.find_max(5, false).await?;
    info!("Motor 5: Move to center");
    cal.shift(5, -2120, false).await?;

    // Final passes for motors 2, 3, 4
    info!("Motor 2: Shift by 1030");
    cal.shift(2, 1030, true).await?;
    info!("Motor 4: Find minimum");
    cal.find_min(4, true).await?;
    info!("Motor 3: Find maximum");
    cal.find_max(3, false).await?;
    info!("Motor 3: Position at min");
    cal.find_min(3, true).await?;
    info!("Motor 2: Find maximum");
    cal.find_max(2, false).await?;
    info!("Motor 2: Position at min");
    cal.shift(2, -1030, true).await?;
    cal.find_max(3, true).await?;
    cal.find_min(2, true).await?;

    // Final positioning
    info!("Motor 3: Move to 50%");
    cal.go_to_float_position(3, 0.5, true).await?;
    info!("Motor 4: Move to 50%");
    cal.go_to_float_position(4, 0.5, true).await?;
    info!("Motor 2: Move to 1%");
    cal.go_to_float_position(2, 0.01, false).await?;
    info!("Motor 4: Move to 95%");
    cal.go_to_float_position(4, 0.95, false).await?;

    // Save (freeze) calibration to EEPROM
    cal.save_calibration().await?;

    info!("ElRobot calibration sequence complete");
    Ok(())
}
