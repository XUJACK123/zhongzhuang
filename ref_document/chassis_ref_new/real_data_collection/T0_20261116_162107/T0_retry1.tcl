source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
echo INITIAL_CPU_STATE
echo [stm32h723.cpu0 curstate]
if {[string equal [stm32h723.cpu0 curstate] halted]} { echo RESUME_FOR_ZERO_OUTPUT_SAMPLING; resume }
sleep 2000
echo SNAPSHOT_T0_retry1_000
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_000_before_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_000_g_can_monitor.bin} 0x20000000 160
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_000_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_000_g_remote.bin} 0x20000220 28
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_000_g_imu.bin} 0x20000360 128
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_000_g_measurement.bin} 0x20000b40 92
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_000_g_motor_snapshot.bin} 0x20000a48 248
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_000_g_board_debug.bin} 0x200001f8 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_000_g_can_tx_fail.bin} 0x20000120 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_000_g_can_rx_overflow.bin} 0x2000011c 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_000_g_rl.bin} 0x20000b9c 1164
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_000_after_g_chassis.bin} 0x20001028 40
sleep 2000
echo SNAPSHOT_T0_retry1_001
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_001_before_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_001_g_can_monitor.bin} 0x20000000 160
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_001_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_001_g_remote.bin} 0x20000220 28
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_001_g_imu.bin} 0x20000360 128
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_001_g_measurement.bin} 0x20000b40 92
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_001_g_motor_snapshot.bin} 0x20000a48 248
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_001_g_board_debug.bin} 0x200001f8 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_001_g_can_tx_fail.bin} 0x20000120 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_001_g_can_rx_overflow.bin} 0x2000011c 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_001_g_rl.bin} 0x20000b9c 1164
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_001_after_g_chassis.bin} 0x20001028 40
sleep 2000
echo SNAPSHOT_T0_retry1_002
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_002_before_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_002_g_can_monitor.bin} 0x20000000 160
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_002_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_002_g_remote.bin} 0x20000220 28
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_002_g_imu.bin} 0x20000360 128
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_002_g_measurement.bin} 0x20000b40 92
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_002_g_motor_snapshot.bin} 0x20000a48 248
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_002_g_board_debug.bin} 0x200001f8 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_002_g_can_tx_fail.bin} 0x20000120 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_002_g_can_rx_overflow.bin} 0x2000011c 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_002_g_rl.bin} 0x20000b9c 1164
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_002_after_g_chassis.bin} 0x20001028 40
sleep 2000
echo SNAPSHOT_T0_retry1_003
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_003_before_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_003_g_can_monitor.bin} 0x20000000 160
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_003_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_003_g_remote.bin} 0x20000220 28
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_003_g_imu.bin} 0x20000360 128
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_003_g_measurement.bin} 0x20000b40 92
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_003_g_motor_snapshot.bin} 0x20000a48 248
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_003_g_board_debug.bin} 0x200001f8 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_003_g_can_tx_fail.bin} 0x20000120 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_003_g_can_rx_overflow.bin} 0x2000011c 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_003_g_rl.bin} 0x20000b9c 1164
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_003_after_g_chassis.bin} 0x20001028 40
sleep 2000
echo SNAPSHOT_T0_retry1_004
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_004_before_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_004_g_can_monitor.bin} 0x20000000 160
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_004_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_004_g_remote.bin} 0x20000220 28
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_004_g_imu.bin} 0x20000360 128
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_004_g_measurement.bin} 0x20000b40 92
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_004_g_motor_snapshot.bin} 0x20000a48 248
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_004_g_board_debug.bin} 0x200001f8 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_004_g_can_tx_fail.bin} 0x20000120 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_004_g_can_rx_overflow.bin} 0x2000011c 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_004_g_rl.bin} 0x20000b9c 1164
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_004_after_g_chassis.bin} 0x20001028 40
sleep 2000
echo SNAPSHOT_T0_retry1_005
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_005_before_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_005_g_can_monitor.bin} 0x20000000 160
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_005_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_005_g_remote.bin} 0x20000220 28
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_005_g_imu.bin} 0x20000360 128
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_005_g_measurement.bin} 0x20000b40 92
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_005_g_motor_snapshot.bin} 0x20000a48 248
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_005_g_board_debug.bin} 0x200001f8 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_005_g_can_tx_fail.bin} 0x20000120 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_005_g_can_rx_overflow.bin} 0x2000011c 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_005_g_rl.bin} 0x20000b9c 1164
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/T0_20261116_162107/raw/T0_retry1_005_after_g_chassis.bin} 0x20001028 40
echo FINAL_CPU_STATE
echo [stm32h723.cpu0 curstate]
shutdown
