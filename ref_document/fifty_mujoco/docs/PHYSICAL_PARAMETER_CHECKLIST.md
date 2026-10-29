# 物理参数清单与获取记录（P0 前置）

> 建立：2026-10-29。依据 `progress.md` §9、§10、§16.2。
> **本文件是提需求的清单与记录格式，不是测量结果。**表中已有的数值全部是
> `provisional` / `cad_extracted`，没有一个来自称重、台架或厂商资料。

## 0. 三条硬规则

1. **空白不等于 0。** 未知项写 `待填`，不写 `0`，也不要在代码里给未知量填默认 0 再当已知。
2. **`cad_extracted` 不自动升级为 `measured`。** 状态只能因为出现了新的来源而改变，不能因为
   "看起来合理"而改字符串；`mechanism.yaml` 的加载器会拒绝没有 `method` + `date` 的 `measured`。
3. **参数变更要升模型版本**，并在下表登记"需重跑的关卡"。改动总质量、轮半径、腿段长度、
   弹簧力或电机限幅，都会让已有权重与已有评估报告失效。

## 1. 状态词表

| status | 含义 | 谁可以写 |
|---|---|---|
| `provisional` | 占位/估算，来源不可靠 | 任何人 |
| `cad_extracted` | 从 STEP 几何算出，可复算 | `tools/step_audit_standalone.py` |
| `computed_from_points` | 由上一条中的坐标派生（距离、夹角） | `fifty_mujoco.mechanism` |
| `candidate_inferred` | 几何支持但需 CAD 配合确认的推断（如固连归属） | `configs/mechanism.yaml` |
| `datasheet` | 厂商资料给出的数字 | 需要记录型号 + 文档版本 |
| `measured` | 在真机上测过 | 需要 `method` + `date` + 仪器 |
| `validated` | 在真机上被独立复现过 | 需要两次独立测量 |

## 2. 已登记参数（当前值，全部未实测）

位置均相对于 `ref_document/fifty_mujoco/`。

| # | 参数 | 当前值 | 单位 | 实现位置 | status | 备注 |
|---|---|---:|---|---|---|---|
| 1 | 轮半径 | 0.06 | m | `configs/robot.yaml: geometry.wheel_radius_m` | `provisional` | 网格绕轴半径，可能缺胎面 |
| 2 | 腿关节力矩上限 | 30 | N·m | `configs/robot.yaml: joints.effort_nm.hip/knee` | `provisional` | 占位值，无厂商数据 |
| 3 | 轮关节力矩上限 | 5 | N·m | `configs/robot.yaml: joints.effort_nm.wheel` | `provisional` | 由复旦资产改来 |
| 4 | 轮关节 armature | 0.01 | kg·m² | `configs/robot.yaml: joints.armature.wheel` | `provisional` | 缺此值时轮速环 5 ms 发散 |
| 5 | 腿关节 armature | 0.0 | kg·m² | `configs/robot.yaml: joints.armature.hip/knee` | `provisional` | 明确为"未知"，不是"无惯量" |
| 6 | 关节摩擦损失（腿/轮） | 0.015 / 0.001 | N·m | `configs/robot.yaml: joints.frictionloss` | `provisional` | 广义关节干摩擦 |
| 7 | 关节阻尼（腿/轮） | 0.03 / 0.002 | N·m·s/rad | `configs/robot.yaml: joints.damping` | `provisional` | |
| 8 | 接触摩擦三元组 | [1.0, 0.005, 0.0001] | 1 | `configs/robot.yaml: contact.friction` | `provisional` | 滑动/扭转/滚动，量纲不同 |
| 9 | 总质量 | 23.2596305 | kg | `reports/model_validation.json`（MJCF 统计） | `cad_extracted` | **不是**实车称重值 |
| 10 | 气弹簧参考力 | 300 | N | `configs/gas_spring.yaml: force_ref_n` | `provisional` | 借自复旦；刚度/阻尼/摩擦全为 0 |
| 11 | 腿 PD 增益 Kp / Kd | 20 / 1.0 | N·m/rad、N·m·s/rad | `configs/robot.yaml: control` | `provisional` | 静态站立需 `\|a\|≈1.4–2.4` |
| 12 | 轮速环 Kd | 0.2 | N·m·s/rad | `configs/robot.yaml: control` | `provisional` | `wheel_kp = 0` |
| 13 | 策略周期 / 物理步长 | 0.01 / 0.005 | s | `configs/robot.yaml: control`、`simulation` | `configured` | 10 ms 策略 = 2 个 5 ms 子步 |
| 14 | 髋/膝/轮平面坐标 | 见 `configs/mechanism.yaml: planar_nodes` | mm | STEP 审计 | `cad_extracted` | 装配姿态 = 腿收起姿态 |
| 15 | 腿段 210 / 250 mm | 210.000000 / 250.000000 | mm | `configs/mechanism.yaml: link_lengths` | `cad_extracted` | 与现有 MJCF 的 0.21/0.25 m 一致 |
| 16 | 传动杆长 135 / 96.6 / 135-67-200.51 | 见 `link_lengths` | mm | 同上 | `cad_extracted` | 闭链连杆 |
| 17 | 固连归属（三孔件/末端连杆销） | 两个候选 | — | `configs/mechanism.yaml: topology.chain.variants` | `candidate_inferred` | **必须 CAD 确认** |

## 3. 待获取数据，按优先级

来源：`progress.md` §10.2。

| 优先级 | 数据 | 为什么需要 | 获取方式 |
|---|---|---|---|
| P0 | 6 路电机型号、安装位置、控制模式 | 决定动作最终作用在哪 | 实车照片、BOM、驱动器说明 |
| P0 | 总减速比、齿数、反馈在转子侧还是输出侧 | 防止角度/速度/力矩倍率错误 | 图纸与慢速核对 |
| P0 | 连续/峰值力矩、峰值持续时间、最大转速、供电电压 | 判断站立是否物理可行 | 官方规格 + 台架 |
| P0 | 编码器零位、正方向、机械限位、装配分支 | 决定 FK/IK 与控制符号 | 固定姿态记录角度 |
| P0 | 弹簧两端所属刚体、安装长度、有效行程、型号 | 决定支撑力臂与可行域 | STEP/CAD + 实物 |
| P1 | 实车总质量、关键部件质量、载荷状态 | 决定重力与支撑需求 | 称重（含云台/电池/弹药状态） |
| P1 | 轮胎外径、受载滚动半径、轮距 | 轮速到车速的换算 | 几何测量 + 低速滚动距离 |
| P1 | 电流到力矩换算、限流/限速规则 | 不能把电流当 N·m | 厂商常数 + 已知力臂验证 |
| P1 | 控制周期、反馈周期、滤波、通信/执行延迟 | 教师与学生面对的动态 | 带时间戳日志 |
| P2 | 关节摩擦、减速器效率、折算转子惯量 | 改善模型与稳态误差 | §5 的辨识流程 |
| P2 | 胎面与场地摩擦、滚动阻力 | 改善接触与防打滑 | §6 的测力流程 |
| P2 | 间隙、弹性、温升、批次差异 | 后续鲁棒性与域随机化 | 多次重复实验 |

## 4. 电机通道表（待填）

`progress.md` §10.6。每行记录一份来源；同一通道有多次测量时分行记录，不要取平均后只留一个数。

| 通道 | 型号/控制模式 | 总减速比 | 输出角零位/方向 | 连续/峰值 N·m | 速度限制 rad/s | 数据来源 |
|---|---|---|---|---|---|---|
| 右主动腿电机 A | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 |
| 右主动腿电机 B | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 |
| 右轮 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 |
| 左主动腿电机 A | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 |
| 左主动腿电机 B | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 |
| 左轮 | 待填 | 待填 | 待填 | 待填 | 待填 | 待填 |

## 5. 气弹簧表（待填）

`progress.md` §9.1。左右两侧分开填，因为它们可能装在不同姿态下。

| 字段 | 左 | 右 |
|---|---|---|
| endpoint_A_body / endpoint_B_body | 待填 | 待填 |
| endpoint_A_local_m / endpoint_B_local_m | 待填 | 待填 |
| 销轴方向、是否球头 | 待填 | 待填 |
| 型号 | 待填 | 待填 |
| 安装长度 m（对应某个明确姿态） | 待填 | 待填 |
| min / max 长度 m | 待填 | 待填 |
| 参考力 N（含参考长度、温度） | 待填 | 待填 |
| 压缩/伸展是否不同、是否有迟滞 | 待填 | 待填 |
| 力曲线来源（手册/测量表/假设） | 待填 | 待填 |
| CPU/GPU 是否同一模型 | 待填 | 待填 |

> 型号名 `1BKB0.45-75` **不能**当作 450 N / 75 mm 直接使用。

## 6. 摩擦辨识流程（需要授权与台架）

`progress.md` §10.4–§10.5。要点：

1. 固定机身，选已知姿态，对单通道低速正反转，至少重复 3 次，正反向都测。
2. 一起记录：`time_s, motor_id, control_mode, q_motor_rad, q_output_rad, qd_output_rad_s,
   current_A, torque_estimate_Nm, command, bus_voltage_V, temperature_C, pose_id,
   spring_enabled, payload_kg`。
3. 只有扣除重力与弹簧作用后剩余阻力才能归为摩擦；不能把支撑体重所需电流都算作摩擦。
4. 稳态区拟合 `tau_f ≈ tau_c·sign(q̇) + b·q̇`，静摩擦启动阈值单独记录。
5. 轮地摩擦与滚动阻力分开测：自由滚动的牵引力主要反映滚动阻力与传动阻力。
6. 参数位于转子侧还是输出侧必须写明；转子惯量折算通常含 n²，闭链坐标下还要再经映射。

## 7. 记录格式

每条参数至少包含：

```text
value / unit / coordinate_frame_or_actuator_side
source_file_or_log / source_hash / measurement_date / method
uncertainty_or_valid_range
status: provisional | cad_extracted | computed_from_points | candidate_inferred
        datasheet | measured | validated
valid_model_version
gates_to_rerun: [P2, P4, P5, ...]
```

## 8. 与代码的连接点

| 参数类别 | 落地文件 | 校验位置 |
|---|---|---|
| 模型/控制参数 | `configs/robot.yaml` | `tests/test_model_release.py`、`tests/test_spawn_contract.py` |
| 弹簧 | `configs/gas_spring.yaml` | `tests/test_gas_spring.py` |
| 平面几何与固连候选 | `configs/mechanism.yaml` | `tests/test_mechanism_config.py`、`tests/test_planar_mechanism.py` |
| 课程/随机化/门槛 | `configs/training_stages.yaml` | `tests/test_training_stages.py` |
| 站立门槛唯一来源 | `src/fifty_mujoco/evaluation/stand_eval.py::THRESHOLDS` | 本表与 `training_stages.yaml` 只引用键名 |
