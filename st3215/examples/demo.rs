//! Example: scan the ST3215 bus and read motor state, or run auto-calibration.

use std::env;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use st3215::auto_calibrate::auto_calibrate_elrobot;
use st3215::St3215;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let path = env::args().nth(1).unwrap_or_else(|| {
        eprintln!("usage: cargo run --example demo <serial_port> [--calibrate]");
        std::process::exit(1);
    });
    let calibrate = env::args().any(|a| a == "--calibrate");

    let mut driver = St3215::open_default(&path)?;
    let motors = driver.scan(8).await;
    println!("Found motors: {:?}", motors);

    for &m in &motors {
        let pos = driver.read_position(m).await?;
        let temp = driver.read_temperature(m).await?;
        let volt = driver.read_voltage(m).await?;
        println!("motor {}: pos={} temp={}°C volt={}", m, pos, temp, volt);
    }

    if calibrate {
        println!("Starting ElRobot auto-calibration...");
        let stop = Arc::new(AtomicBool::new(false));
        auto_calibrate_elrobot(&mut driver, stop).await?;
        println!("Calibration complete");
    }

    Ok(())
}
