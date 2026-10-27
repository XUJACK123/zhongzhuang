# 带真实网格版本：关节修正

本次针对该目录独立修复，不替换成之前的简化几何模型。

- right_large_leg、right_small_leg、left_large_leg、left_small_link：fixed 改为 revolute，局部轴 0 0 1。
- right_wheel、left_wheel：fixed 改为 continuous，删除角度上下限，保留 effort、velocity。
- 右轮局部轴改为 0.0687283918 0 -0.997635408；其原坐标系有偏转，轴向由父小腿 Z 轴转换到原轮关节坐标系。STL 最大共面面积组的法向与此轴平行，交叉核对通过。左轮局部轴为 0 0 1。
- 原关节位置、RPY、link 名、joint 名、质量、质心、惯量和所有网格文件保持不变，零角度时的装配外形保持不变。
- 同步 CSV 和 controller_joint_names，补齐 RViz 配置并移除 display.launch 中未使用且没有默认值的 model 参数。

## 已检查与限制

检查了 7 个网格存在且为完整二进制 STL、6 个非固定关节、单位长度轴、运动树、正质量和有效惯量；验证每个关节施加非零角度能使对应网格运动。备份及 mesh SHA256 存于 backups 和 config/joint_fix_report.json。未运行 ROS/RViz/Isaac Gym。

四腿关节仍沿用原 [-100,100] rad 范围，所有关节沿用 effort=30 N·m、velocity=1000 rad/s。这些是原有宽泛参数，不能当成真实机械限位或已校准驱动能力；本次没有猜测并改写实机参数。RViz 滑块跨度因此较大，建议输入小角度，例如 0.1 或 0.2 rad 观察运动。

本版仍保留 SW 的机身坐标与关节零位，未执行前一个简化版本的复旦训练坐标转换或命名转换。不能直接用前一个简化版训练入口替换其 URDF；接入复旦时仍需匹配名称、顺序、坐标和零位。

云台仍并在 base_link，未新增 yaw/pitch；已经合并进 link 的连杆和气弹簧仍随该 link 刚性运动。网格较大且使用原 STL 碰撞，本次修复旋转行为不等于已优化碰撞或训练性能。

## 查看

ROS 1 环境将该包加入工作空间并 source 后，可运行：

```text
roslaunch fourth_version_with_meshes display.launch
```

在 joint_state_publisher_gui 输入小角度分别检查六个关节。输入零角度会回到原 SW 导出的装配姿态。CSV 已同步修改，但 SW 内原有 URDF Export Configuration 未被改写；若再次从 SW 导出，应检查插件内的关节类型和轴向，避免覆盖修正。
