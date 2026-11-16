source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
set phase [lindex [read_memory 0x200143d4 32 1] 0]
echo PHASE_BEFORE_SWD_DUMPS_$phase
if {$phase != 0 && $phase != 1 && $phase != 4} {error "Diagnostic may be active; refuse long SWD sampling"}
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/zero_freshness_a_g_ident.bin} 0x200143d4 44
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/zero_freshness_a_g_ident_request_token.bin} 0x20014408 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/zero_freshness_a_g_ident_log.bin} 0x200005b8 28
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/zero_freshness_a_g_ident_tick.bin} 0x20014400 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/zero_freshness_a_g_ident_healthy.bin} 0x20014404 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/zero_freshness_a_g_motor_snapshot.bin} 0x200004c0 248
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/zero_freshness_a_g_remote.bin} 0x20000178 28
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/zero_freshness_a_g_imu.bin} 0x200002b8 128
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/zero_freshness_a_g_can_monitor.bin} 0x20000010 160
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/zero_freshness_a_g_can_tx_fail.bin} 0x2001441c 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/zero_freshness_a_g_can_rx_overflow.bin} 0x20014418 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/zero_freshness_a_g_diag_can.bin} 0x20014544 60
shutdown
