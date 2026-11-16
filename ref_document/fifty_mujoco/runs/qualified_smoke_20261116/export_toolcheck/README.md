# 平衡仿真候选（未烧录）

来源：update_000003.pt，update 3。这是平衡actor，不是起身actor。

轮Kd=0.14，膝目标余量=0.03 rad；31维输入，PD/估计器/actor为2/5/10 ms。
仿真使用当前固件C里程计质量/参考管理，传感器仍为理想输入；不包含实机IMU滤波、CAN时序、失能后使能或起身。

`model.c/.h`及`actor31.pt`可复核，512组主机C/Torch输入对拍通过。`c_runtime/`保存本次编译源码，`simulation_physics.json`保存物理参数。
不得把本包的验证视作实车通过；原固件未修改。
