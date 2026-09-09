# ST3215 舵机驱动 + 自动标定（独立 crate）

从 norma-core 的 `software/drivers/st3215` 解耦出来的**独立可编译** Rust crate。
只依赖 `tokio`、`tokio-serial`、`bytes`、`log`，去掉了 `normfs` / `station_iface` /
`systime` / protobuf 等框架依赖。

## 目录结构

```
src/
  lib.rs              模块入口
  driver.rs           St3215 串口驱动（RAM / EEPROM 读写、标定 freeze/reset）
  protocol/           串行总线协议（帧打包/校验、寄存器表、单位换算）
  presets.rs          预设值（PID、标定速度/力矩、默认保护值）
  calibrate.rs        行程弧段计算（含 4096 回绕处理）
  auto_calibrate/     ElRobot 8 电机自动标定
examples/demo.rs      扫描总线 + 读状态 / 运行标定
```

## 构建

```bash
cargo build                # 编译
cargo test                 # 运行协议/弧段计算的单元测试
cargo run --example demo -- /dev/tty.usbserial-XXXX
```

## 快速开始

```rust
use st3215::St3215;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut d = St3215::open_default("/dev/tty.usbserial-XXXX")?;  // 1 Mbps

    // 扫描总线
    let motors = d.scan(8).await;

    // 读状态
    let pos = d.read_position(1).await?;
    let vel = d.read_velocity(1).await?;
    let load = d.read_load(1).await?;

    // 控制
    d.set_torque(1, true).await?;       // 使能力矩
    d.set_position(1, 2048).await?;     // 移到中点
    Ok(())
}
```

## 主要 API

**原始协议**：`ping / read / write / reg_write / action / reset / sync_write`

**RAM 便捷方法**：
- 控制：`set_torque`、`set_position`、`set_goal_speed`、`set_accel`、`set_torque_limit`
- 读取：`read_position`、`read_velocity`、`read_load`、`read_current`、
  `read_voltage`、`read_temperature`、`is_moving`

**EEPROM**：`read_eeprom`、`write_eeprom`（解锁→reg_write→action→上锁）、
`write_eeprom_verified`（写后读回校验、重试 5 次）

**标定**：
- `reset_calibration(motor)` — 复位并把位置偏移归零
- `freeze_calibration(motor, midpoint, motor_count)` — 固化标定（写偏移修正、
  PID、位置模式、默认力矩/电流/加速度）
- `auto_calibrate::auto_calibrate_elrobot(&mut driver, stop)` — 全自动标定

## 自动标定（ElRobot）

`auto_calibrate_elrobot` 执行 8 电机标定流程：
1. 逐个电机 `prepare`（复位、写 PID=纯 P、限力矩/速度/加速度）；
2. 按关节设保护电流（腕部/夹爪更低：电机 5/7=15、6=20、8=10，其余 50）；
3. 逐个 `find_min/find_max`（扫到堵转，速度 < 阈值判定撞限位）记录行程；
4. 电机 2/3/4 多轮特殊处理，各电机 `shift` 居中；
5. 最后 `save_calibration`：算弧段（`calibrate::calculate_arc`，含回绕）→
   写 `Offset = 中点 − 2048` 与 PID 等参数到 EEPROM。

> 注意：标定会**写舵机 EEPROM**，首次上电前请确认机械臂可自由运动、限位不会
> 损坏结构。`stop` 标志可在中途触发，触发后所有电机关闭力矩（臂变软）。

## 与上游的差异

- 去掉了 station 框架的命令/状态通道（normfs 队列、protobuf `TxEnvelope`），
  改为**直接串口读写**；
- 进度上报改为 `log::info!`；
- 移除了 SO101（6 电机臂）标定，仅保留 ElRobot；
- 保留：协议、寄存器表、弧段计算、标定序列与 freeze/reset 逻辑。
