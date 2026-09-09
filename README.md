# ElRobot 机械臂项目

自研 ElRobot 机械臂（3D 打印 + ST3215 C001 数字舵机）的独立仓库。内容提取自
[norma-core](https://github.com/norma-core/norma-core) 框架，只保留本机所需的
硬件设计文件、MuJoCo 仿真，以及 ST3215 舵机驱动源码（作参考）。

## 目录结构

| 目录 | 内容 |
|------|------|
| `elrobot/` | 跟随臂/主臂硬件：STL/STEP 打印件、装配手册 PDF、URDF、README、许可证 |
| `elrobot/simulation/` | MuJoCo 仿真（`build_mjcf.py` 生成场景、`controller.py` 伺服模型、`control_ui.py` 浏览器控制、`demo.py` 等） |
| `pgripper/` | PGripper 夹爪（打印件、STEP/STL、手册 PDF、许可证） |
| `st3215/` | ST3215 舵机 Rust 驱动源码（参考，无法独立编译，见下） |

## 来源与许可

- `elrobot/`、`pgripper/`：来自 norma-core 的 `hardware/`，许可证 **Apache-2.0**
  （见各目录内的 `LICENSE`；其中部分零件来自 SO-ARM100，署名见 `elrobot/README.md`）。
- `st3215/`：来自 norma-core 的 `software/drivers/st3215`，许可证
  **MIT OR Apache-2.0**（声明于 `st3215/Cargo.toml`）。
- 上游仓库：<https://github.com/norma-core/norma-core>

## 关于 st3215 驱动

`st3215/` 是从上游 monorepo 提取的 Rust crate 源码，其 `Cargo.toml` 依赖上游的
共享 crate（`normfs`、`station_iface`、`systime-rs` 以及 protobuf 定义），因此
**单独无法编译**。此处仅作协议与标定逻辑的参考，重点可看：

- `st3215/src/protocol/packet.rs`、`units.rs`、`devices.rs` —— 串行总线协议与单位换算
- `st3215/src/calibrate.rs`、`src/auto_calibrate/elrobot.rs` —— 标定 / ElRobot 自动标定
- `st3215/src/driver.rs`、`port.rs` —— 端口与驱动逻辑

如需独立驱动，可据此改写为 Python 或嵌入式实现（总线为 1 Mbps 半双工 TTL 串行，
12 bit 位置分辨率，详见 `elrobot/simulation/MEASUREMENTS.md` §3）。

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
