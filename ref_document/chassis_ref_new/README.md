# STM32H723 强化学习轮腿底盘

本工程部署31维输入、6维动作的强化学习actor。四个DM8009J与两个M3508共用CAN2，使用板载BMI088与UART5 DR16/DT7。

**2026-11-15源码状态：已接入u100平衡候选、起身接管接口、观测质量管理和遥测缓存，主机测试及ARM构建通过。本轮未烧录、未访问硬件、未运行仿真/GPU；没有训练完成的起身actor，未完成实物平衡或自主起身验收。**

当前actor来自 `../fifty_mujoco/runs/hardware31_guarded_20261115/deployment_candidate/actor31.pt`，由 `make export-policy` 安装，来源及SHA见 [manifest](user/algorithm/model/manifest.json)。旧model_3050.pt及11月13–14日现场记录保留为历史证据。

| 配置 | 当前值/含义 |
|---|---|
| ENABLE_OUTPUT | 1，生产源码允许在满足控制条件后输出；本轮未更改授权 |
| MOTOR_CALIBRATED / IMU_FRAME_VERIFIED | 1 / 1，历史方向/参考姿态及IMU方向已有实测，重复性和动态性能仍需补充 |
| POLICY_CONTRACT_VERIFIED / ODOMETRY_VERIFIED | 0 / 0，未提升验证状态 |
| 控制/估计/策略周期 | 2 / 5 / 10 ms；板上最坏耗时待测 |
| 腿/轮电机空间上限 | 20 / 2 Nm；不是独立测得的有效输出轴力矩 |
| 轮反馈比例/半径 | 15.88 / 0.055 m；有效滚动半径待低滑移真值复核 |

## 文档与工具

| 入口 | 内容 |
|---|---|
| [算法实现报告](docs/ALGORITHM_IMPROVEMENTS_20261115.md) | 本轮改动、参数、测试结果及动态验证边界 |
| [实测计划](../../progress_real_data.md) | T0–T8具体测试步骤、字段、标准和训练用途；原始JSON/OpenOCD日志链接 |
| [总进度](../../progress.md) | §5.12训练规划、§5.13离线起身准备、§5.14固件实现 |
| [部署规范](docs/DEPLOYMENT.md) | 六路映射、31维观测、耦合与力矩换算、当前遥控语义 |
| [协作约束](AGENTS.md) | 构建和硬件操作边界 |
| [历史联调](docs/COMMISSIONING_LOG_20261113.md) | 当时的现场操作、测量、决定与原始数据 |
| [历史上电记录](docs/BOARD_BRINGUP_20261113.md) | CAN、遥控、IMU初始证据 |
| [本轮离线验证快照](docs/validation/algorithm_20261115.json) | 主机/ARM结果及源码、构建产物SHA |
| [配置](user/robot_config.h) | 周期、映射、参考位、上限与超时 |
| [遥测工具](tools/telemetry_log.py) | capture读取RAM、decode离线解码；不会烧录或发电机命令 |

历史记录曾出现六路均增长，之后也出现右轮0x203无反馈。它们不是当前现场状态；本轮没有重新采样。历史支撑测量姿态不是已经确认的自然失能停位。现有训练模型含弹簧项，但450 N恒力仍待真实曲线修正。历史DM上报/请求比例约0.79不能替代外部力矩测量。

## 构建与验证

```powershell
make -j4
make test
python tools/verify_build.py --fresh
make flash-check
```

以上不连接硬件。唯一活动镜像为 `build/rl/wheel_leg_rl.elf`（另有hex/bin）。`make flash`是另行执行的真实烧录/校验/复位命令，使用Horco CMSIS-DAP；源码实现不自动烧录。旧Keil/CubeMX/VS Code配置保存在archive。

左腿DM1髋/DM2膝，右腿DM4髋/DM3膝；左轮ID2、右轮ID3。策略与遥测顺序始终为**右髋、右膝、右轮、左髋、左膝、左轮**。

## 当前遥控与起身行为

| 操作/条件 | 行为 |
|---|---|
| S1下档 | 零电流停车；上电/故障/断联后需一次新鲜下档才可再运行 |
| S1中档/上档、摇杆回中且反馈健康 | 按既有使能流程进入ACTIVE，力矩200 ms渐入 |
| 已处于近站立窗口 | 接入BALANCE |
| 非近站姿、未注册起身actor | fault=5并零输出；当前缺起身权重 |
| 有有效起身actor | GETUP→稳定驻留1 s→300 ms HANDOFF→BALANCE |
| S2 | 不参与使能 |
| 右摇杆竖直 | BALANCE且里程计可信时前进0…0.15 m/s，反向拨为0请求 |
| 右摇杆水平 | 同上，yaw±0.30 rad/s |
| 左摇杆/高度 | 暂不使用；固定底部净空命令0.10 m |

反馈、遥控、IMU超时及总线/机构/推理/时序故障沿停车路径处理；不因轮反馈丢失就擅自保留腿部主动输出。组控就绪并非确认收到FC应答，CAN提交成功也不等于实际出力。

## 下一步

按[实测计划](../../progress_real_data.md)先做零主动输出的通信、自然停位、参考位重复性，再做带外部测量的力矩/弹簧与IMU/里程计测试。专用低力矩台架模式和连续遥测通道尚待实现，不能用整车RL的20 Nm上限代替标定程序。

新遥测每帧636字节、32帧缓存，约64 ms；退出运行时冻结，下一次运行清空。故障后先保存再重启。它适合故障短窗，不能代替完整起身记录。调试器读取可能扰动控制，先在支撑停车状态使用，不以Live Watch进行自由平衡动态记录。

新IMU/里程计/参考管理还需接入仿真并重新回归。旧203/203与500/500不构成本轮新闭环验收。主机测试与交叉构建也不代表板上时序或物理起身通过。
