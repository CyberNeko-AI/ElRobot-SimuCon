# ElRobot 机械臂项目

自研 ElRobot 机械臂（3D 打印 + ST3215 C001 数字舵机）的独立仓库。内容提取自
[norma-core](https://github.com/norma-core/norma-core) 框架，只保留本机所需的
硬件设计文件、MuJoCo 仿真，以及**已解耦、可独立编译**的 ST3215 舵机驱动与标定程序。

## 目录结构

| 目录 | 内容 |
|------|------|
| `elrobot/` | 跟随臂/主臂硬件：STL/STEP 打印件、装配手册 PDF、URDF、README、许可证 |
| `elrobot/simulation/` | MuJoCo 仿真（`build_mjcf.py` 生成场景、`controller.py` 伺服模型、`control_ui.py` 浏览器控制、`demo.py` 等） |
| `pgripper/` | PGripper 夹爪（打印件、STEP/STL、手册 PDF、许可证） |
| `st3215/` | ST3215 舵机 Rust 驱动 + 自动标定（独立 crate，已解耦） |

## 来源与许可

- `elrobot/`、`pgripper/`：来自 norma-core 的 `hardware/`，许可证 **Apache-2.0**
  （见各目录内的 `LICENSE`；其中部分零件来自 SO-ARM100，署名见 `elrobot/README.md`）。
- `st3215/`：来自 norma-core 的 `software/drivers/st3215`，许可证
  **MIT OR Apache-2.0**（声明于 `st3215/Cargo.toml`）。
- 上游仓库：<https://github.com/norma-core/norma-core>

## 关于 st3215 驱动

`st3215/` 是从上游 monorepo 提取后**解耦成独立 crate** 的 Rust 驱动 + 标定程序，
只依赖 `tokio` / `tokio-serial` / `bytes` / `log`，可独立编译：

```bash
cd st3215
cargo build                 # 编译
cargo test                  # 运行协议/弧段计算单元测试
cargo run --example demo -- /dev/tty.usbserial-XXXX [--calibrate]
```

提供：串口驱动（RAM/EEPROM 读写、`set_position`/`read_position`/`read_velocity`/
`read_load` 等）、ElRobot 8 电机自动标定（扫到堵转定行程、算弧段、写 Offset/PID
固化到 EEPROM）。详见 `st3215/README.md`。与上游相比去掉了 station 框架的命令/
状态通道（改为直接串口读写），并移除了 SO101 标定，仅保留 ElRobot。

## 仿真快速开始

需要 Python 3.12 + `mujoco`（`numpy`、`pillow`）。详见 `elrobot/simulation/README.md`。

```bash
cd elrobot/simulation
python build_mjcf.py          # 从 URDF 生成 MuJoCo 场景
python control_ui.py          # 浏览器关节控制（自动打开）
python run_viewer.py          # 原生 MuJoCo 查看器
python demo.py 4.0            # 无头演示动画
```

## 待测参数

舵机 PID 带宽、位置刚度、控制器死区、接触摩擦、反射惯量等将在实物制造后实测，
记录于 `elrobot/simulation/MEASUREMENTS.md`，标定后回填该文件并同步更新
`build_mjcf.py` / `controller.py`。
