source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
set phase [lindex [read_memory 0x2001447c 32 1] 0]
if {[lsearch {0 1 4} $phase] < 0} {error "Diagnostic may be active; refuse long SWD sampling"}
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_183956/raw/status_184950_g_ident.bin} 0x2001447c 48
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_183956/raw/status_184950_g_ident_request_token.bin} 0x200144b4 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_183956/raw/status_184950_g_ident_log.bin} 0x20000660 28
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_183956/raw/status_184950_g_ident_tick.bin} 0x200144ac 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_183956/raw/status_184950_g_ident_healthy.bin} 0x200144b0 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_183956/raw/status_184950_g_motor_snapshot.bin} 0x20000568 248
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_183956/raw/status_184950_g_remote.bin} 0x20000220 28
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_183956/raw/status_184950_g_imu.bin} 0x20000360 128
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_183956/raw/status_184950_g_can_monitor.bin} 0x20000000 160
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_183956/raw/status_184950_g_can_tx_fail.bin} 0x20000120 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_183956/raw/status_184950_g_can_rx_overflow.bin} 0x2000011c 4
shutdown
