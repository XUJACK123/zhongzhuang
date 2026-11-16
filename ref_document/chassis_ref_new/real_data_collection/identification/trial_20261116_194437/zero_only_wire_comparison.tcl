source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_194437/raw/zero_a_g_ident_tick.bin} 0x20014400 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_194437/raw/zero_a_g_diag_can.bin} 0x20014544 60
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_194437/raw/zero_a_g_can_monitor.bin} 0x20000010 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_194437/raw/zero_a_g_ident_request_token.bin} 0x20014408 4
sleep 5000
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_194437/raw/zero_b_g_ident_tick.bin} 0x20014400 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_194437/raw/zero_b_g_diag_can.bin} 0x20014544 60
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_194437/raw/zero_b_g_can_monitor.bin} 0x20000010 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_194437/raw/zero_b_g_ident_request_token.bin} 0x20014408 4
shutdown
